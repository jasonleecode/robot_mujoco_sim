#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <opencv2/opencv.hpp>
#include <dds/dds.h>

#include "ControlActions.hpp"
#include "RobotSim.hpp"
#include "planner.h"
#include "dds_generated/ControlMsg.h"

namespace {

constexpr char kDefaultModel[] = "robot/boston_dynamics_spot/scene.xml";
// 物理循环频率：1000Hz (1ms)
constexpr double kSimulationDt = 0.001; 
constexpr int kControlMsgSlots = 16;

struct ScopedParticipant {
    ~ScopedParticipant() {
        if (handle > 0) {
            dds_delete(handle);
        }
    }
    dds_entity_t handle = DDS_ENTITY_NIL;
};

// 线程间共享数据结构
struct SharedControlData {
    std::mutex mutex;
    
    // 当前控制模式：true=使用Planner, false=使用DDS原始关节数据
    bool use_planner = true; 
    
    // Planner 模式下的目标动作
    control::BasicMotion current_motion = control::BasicMotion::kStand;
    
    // Raw 模式下的关节数据缓存
    std::vector<double> raw_values;

    SharedControlData() {
        raw_values.resize(kControlMsgSlots, 0.0);
    }
};

// 从 DDS 消息更新共享数据 (主线程调用)
void CopyMessageToShared(const MujocoDDS_ControlMsg& msg,
                         SharedControlData& shared_data,
                         bool& warned_clamp) {
    std::lock_guard<std::mutex> lock(shared_data.mutex);
    
    // 如果是 Raw 模式
    if (msg.mode == static_cast<uint32_t>(control::CommandMode::kRaw)) {
        shared_data.use_planner = false;
        const int copy_count = std::min<int>(shared_data.raw_values.size(), kControlMsgSlots);
        for (int i = 0; i < copy_count; ++i) {
            shared_data.raw_values[i] = msg.values[i];
        }
        if (static_cast<int>(shared_data.raw_values.size()) > kControlMsgSlots && !warned_clamp) {
            std::cerr << "Warning: DDS ControlMsg supports only " << kControlMsgSlots
                      << " actuators.\n";
            warned_clamp = true;
        }
    } 
    // 如果是 Basic Motion 模式
    else if (msg.mode == static_cast<uint32_t>(control::CommandMode::kBasic)) {
        control::BasicMotion motion;
        if (control::TryParseBasicMotion(msg.action, motion)) {
            shared_data.use_planner = true;
            shared_data.current_motion = motion;
        } else {
            std::cerr << "Unknown basic motion id: " << msg.action << '\n';
        }
    }
}

bool PollDDSCommands(dds_entity_t reader, MujocoDDS_ControlMsg& latest_command) {
    constexpr int kSamplesPerTake = 4;
    void* samples[kSamplesPerTake] = {};
    dds_sample_info_t infos[kSamplesPerTake];
    bool updated = false;

    while (true) {
        const dds_return_t rc = dds_take(reader, samples, infos, kSamplesPerTake, kSamplesPerTake);
        if (rc == 0 || rc == DDS_RETCODE_NO_DATA) {
            break;
        }
        if (rc < 0) {
            throw std::runtime_error(std::string("dds_take: ") + dds_strretcode(-rc));
        }

        for (int i = 0; i < rc; ++i) {
            if (infos[i].valid_data) {
                latest_command = *static_cast<MujocoDDS_ControlMsg*>(samples[i]);
                updated = true;
            }
        }
        dds_return_loan(reader, samples, rc);
    }
    return updated;
}

void PrintUsage(const char* binary_name) {
    std::cout << "Usage:\n  " << binary_name << " [path/to/model.xml]\n\n"
              << "When no path is provided the default model '" << kDefaultModel
              << "' is used.\n";
}

std::string ResolveModelPath(int argc, char** argv) {
    if (argc > 1) return argv[1];
    return kDefaultModel;
}

void SanityCheckRuntime() {
    std::cout << "MuJoCo version " << mj_versionString() << '\n';
    if (mjVERSION_HEADER != mj_version()) {
        throw std::runtime_error("MuJoCo header/library version mismatch");
    }
}

int RequireOk(int value, const char* label) {
    if (value < 0) {
        throw std::runtime_error(std::string(label) + ": " + dds_strretcode(-value));
    }
    return value;
}

}  // namespace

int main(int argc, char** argv) {
    std::thread physics_thread;
    std::atomic<bool> sim_running{false};

    try {
        PrintUsage(argv[0]);
        SanityCheckRuntime();

        const std::string model_path = ResolveModelPath(argc, argv);
        if (!std::filesystem::exists(model_path)) {
            throw std::runtime_error("Model file not found: " + model_path);
        }

        // 初始化 RobotSim
        RobotSim robot(model_path);
        const int num_actuators = robot.getNumActuators();
        std::cout << "Loaded model '" << model_path << "' with " << num_actuators << " actuators\n";

        // DDS 初始化
        ScopedParticipant participant_guard;
        participant_guard.handle = RequireOk(dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr), "dds_create_participant");
        const dds_entity_t topic = RequireOk(dds_create_topic(participant_guard.handle, &MujocoDDS_ControlMsg_desc, "RobotControl", nullptr, nullptr), "dds_create_topic");
        const dds_entity_t reader = RequireOk(dds_create_reader(participant_guard.handle, topic, nullptr, nullptr), "dds_create_reader");

        // 共享数据与 Planner
        SharedControlData shared_data;
        shared_data.raw_values.assign(num_actuators, 0.0);
        
        // 注意：SpotPlanner 现在完全属于物理线程，不需要锁
        SpotPlanner planner;
        planner.setControlFrequency(kSimulationDt);
        
        bool warned_clamp = false;

        // --- 物理线程逻辑 (Fixed Time Step Loop) ---
        auto physics_loop = [&]() {
            using namespace std::chrono;
            
            // 本地缓存，减少锁竞争和内存分配
            RobotState current_state;
            std::vector<double> control_target(num_actuators, 0.0);
            
            bool local_use_planner = true;
            control::BasicMotion local_motion = control::BasicMotion::kStand;
            std::vector<double> local_raw_values(num_actuators, 0.0);

            bool is_control_active = false;

            // 计时器
            auto next_tick = steady_clock::now();
            const auto tick_interval = microseconds(static_cast<int>(kSimulationDt * 1e6));

            while (sim_running.load(std::memory_order_acquire)) {
                // 1. 同步外部指令 (最小化临界区)
                {
                    std::lock_guard<std::mutex> lock(shared_data.mutex);
                    local_use_planner = shared_data.use_planner;
                    local_motion = shared_data.current_motion;
                    if (!local_use_planner) {
                        local_raw_values = shared_data.raw_values;
                    }
                }

                // 2. 获取机器人状态
                robot.getState(current_state);

                // 3. 计算控制输出
                if (local_use_planner) {
                    if (!is_control_active && local_motion != control::BasicMotion::kStand) {
                std::cout << "Motion Command Received. Activating Planner..." << std::endl;
                is_control_active = true;
                
                // [关键] 激活瞬间，强制同步 Planner 内部状态到当前真实状态
                // 防止激活瞬间再次发生跳变
                planner.setCurrentState(current_state); 
            }

            if (is_control_active) {
                // === 正常规划逻辑 ===
                if (planner.mode() != local_motion) {
                    planner.setMode(local_motion);
                }
                planner.update(current_state);
                planner.getJointTargets(control_target);
                robot.updatePlotData(control_target);
            } else {
                // === [核心] 静止保持逻辑 (Servo Hold) ===
                // 还没有激活时，目标 = 当前位置
                // 这利用了 PD 控制器的特性，将电机锁死在当前角度，保持绝对静止
                if (current_state.qpos.size() >= num_actuators) {
                    // 注意：MuJoCo 的 qpos 前7位是基座位置姿态，关节角度从第7位开始 (offset=7)
                    // 需要根据你的 RobotSim 实现确认 qpos 的结构。
                    // 假如 RobotSim::getState 已经处理好了只返回关节角度，则直接拷贝。
                    // 但通常 getState 返回的是完整 qpos。
                    
                    // 这里需要特别注意 LegIndexHelper 的转换，或者直接透传
                    // 简单做法：假设 control_target 对应 qpos 的后 12 位
                    // 更加稳健的做法是：
                    
                    // 临时方案：直接让所有电机维持当前读数
                    // 假设 num_actuators = 12, qpos 包含 floating base (7) + 12 joints
                    // 请务必确认 RobotSim::getState 中 qpos 的定义
                    
                    // 如果 RobotSim::getState 返回的是纯关节角度：
                    // control_target = current_state.qpos;
                    
                    // 如果 RobotSim::getState 返回的是完整 MuJoCo qpos (7 + 12)：
                    for(int i=0; i<num_actuators; ++i) {
                         // MuJoCo qpos offset = 7 (3 pos + 4 quat)
                         control_target[i] = current_state.qpos[7 + i]; 
                    }
                }
            }
                
                
                } else {
                    // Raw 模式直接透传
                    control_target = local_raw_values;
                }

                // 4. 应用控制并执行物理步进
                robot.applyControlVector(control_target);
                robot.stepPhysics();

                // 5. 休眠直到下一个时间片 (保持 500Hz)
                next_tick += tick_interval;
                std::this_thread::sleep_until(next_tick);
            }
        };

        // 启动物理线程
        sim_running.store(true, std::memory_order_release);
        physics_thread = std::thread(physics_loop);

        // --- 主线程逻辑 (Rendering & DDS Input) ---
        MujocoDDS_ControlMsg latest_command{};

        while (true) {
            // 1. 处理 DDS 消息 (非阻塞)
            if (PollDDSCommands(reader, latest_command)) {
                CopyMessageToShared(latest_command, shared_data, warned_clamp);
            }

            // 2. 渲染帧
            // renderFrame 内部已经优化了锁的使用，不会长时间阻塞物理线程
            robot.renderFrame();

            if (robot.windowShouldClose()) {
                break;
            }
        }

        // 清理退出
        sim_running.store(false, std::memory_order_release);
        if (physics_thread.joinable()) {
            physics_thread.join();
        }

        return EXIT_SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        sim_running.store(false, std::memory_order_release);
        if (physics_thread.joinable()) {
            physics_thread.join();
        }
        return EXIT_FAILURE;
    }
}