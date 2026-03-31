#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ControlActions.hpp"
#include "RobotSim.hpp"
#include "planner.h"

#ifdef USE_DDS
#include <dds/dds.h>
#include "dds_generated/ControlMsg.h"
#endif

namespace {

constexpr char kDefaultModel[] = "robot/boston_dynamics_spot/scene.xml";
// 物理循环频率：1000Hz (1ms)
constexpr double kSimulationDt = 0.001;
constexpr int kControlMsgSlots = 16;

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

#ifdef USE_DDS
struct ScopedParticipant {
  ~ScopedParticipant() {
    if (handle > 0) {
      dds_delete(handle);
    }
  }
  dds_entity_t handle = DDS_ENTITY_NIL;
};

// 从 DDS 消息更新共享数据 (主线程调用)
void CopyMessageToShared(const MujocoDDS_ControlMsg& msg, SharedControlData& shared_data,
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
      std::cerr << "Warning: DDS ControlMsg supports only " << kControlMsgSlots << " actuators.\n";
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

int RequireOk(int value, const char* label) {
  if (value < 0) {
    throw std::runtime_error(std::string(label) + ": " + dds_strretcode(-value));
  }
  return value;
}
#endif  // USE_DDS

void PrintUsage(const char* binary_name) {
  std::cout << "Usage:\n  " << binary_name << " [path/to/model.xml]\n\n"
            << "When no path is provided the default model '" << kDefaultModel << "' is used.\n";
}

std::string ResolveModelPath(int argc, char** argv) {
  if (argc > 1)
    return argv[1];
  return kDefaultModel;
}

void SanityCheckRuntime() {
  std::cout << "MuJoCo version " << mj_versionString() << '\n';
  if (mjVERSION_HEADER != mj_version()) {
    throw std::runtime_error("MuJoCo header/library version mismatch");
  }
}

// 加载 MuJoCo 插件（OBJ/STL 解码器等），在加载模型前调用
void LoadMuJoCoPlugins() {
  // 优先尝试与可执行文件同目录的插件（适合自定义部署）
  // 然后回退到 ~/.mujoco/mujoco-X.X.X/bin/mujoco_plugin
  const char* plugin_dirs[] = {
    "mujoco_plugin",         // 相对路径（与二进制同目录时）
    nullptr,                 // 用环境变量 MUJOCO_PLUGIN_DIR 覆盖
  };

  // 优先检查环境变量
  if (const char* env_dir = std::getenv("MUJOCO_PLUGIN_DIR")) {
    mj_loadAllPluginLibraries(env_dir, nullptr);
    std::cout << "Loaded MuJoCo plugins from: " << env_dir << '\n';
    return;
  }

  // 否则从 MuJoCo 安装目录加载
  const char* home = std::getenv("HOME");
  if (home) {
    // 尝试常见安装路径
    const std::string candidates[] = {
      std::string(home) + "/.mujoco/mujoco-3.6.0/bin/mujoco_plugin",
      std::string(home) + "/.mujoco/mujoco-3.5.0/bin/mujoco_plugin",
      std::string(home) + "/.mujoco/mujoco-3.4.0/bin/mujoco_plugin",
    };
    for (const auto& dir : candidates) {
      if (std::filesystem::exists(dir)) {
        mj_loadAllPluginLibraries(dir.c_str(), nullptr);
        std::cout << "Loaded MuJoCo plugins from: " << dir << '\n';
        return;
      }
    }
  }
  std::cerr << "Warning: MuJoCo plugin directory not found. OBJ meshes may fail to load.\n"
            << "  Set MUJOCO_PLUGIN_DIR to your MuJoCo bin/mujoco_plugin directory.\n";
}

}  // namespace

template <typename T>
T lerp(T a, T b, double t) {
  return a + (b - a) * t;
}

int main(int argc, char** argv) {
  std::thread physics_thread;
  std::atomic<bool> sim_running{false};

  try {
    PrintUsage(argv[0]);
    SanityCheckRuntime();
    LoadMuJoCoPlugins();

    const std::string model_path = ResolveModelPath(argc, argv);
    if (!std::filesystem::exists(model_path)) {
      throw std::runtime_error("Model file not found: " + model_path);
    }

    // 初始化 RobotSim
    RobotSim robot(model_path);
    const int num_actuators = robot.getNumActuators();
    std::cout << "Loaded model '" << model_path << "' with " << num_actuators << " actuators\n";

#ifdef USE_DDS
    // DDS 初始化
    ScopedParticipant participant_guard;
    participant_guard.handle = RequireOk(
        dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr), "dds_create_participant");
    const dds_entity_t topic =
        RequireOk(dds_create_topic(participant_guard.handle, &MujocoDDS_ControlMsg_desc,
                                   "RobotControl", nullptr, nullptr),
                  "dds_create_topic");
    const dds_entity_t reader = RequireOk(
        dds_create_reader(participant_guard.handle, topic, nullptr, nullptr), "dds_create_reader");
#else
    std::cout << "DDS disabled. Using Planner-only control mode.\n";
#endif

    // 共享数据与 Planner
    SharedControlData shared_data;
    shared_data.raw_values.assign(num_actuators, 0.0);

    // 设置运动控制回调
    robot.motion_callback = [&shared_data](int motion_type) {
      std::lock_guard<std::mutex> lock(shared_data.mutex);
      if (motion_type == 0) {
        // 停止
        shared_data.current_motion = control::BasicMotion::kStand;
        std::cout << "Motion: Stop (Stand)" << std::endl;
      } else if (motion_type == 1) {
        // 前进
        shared_data.current_motion = control::BasicMotion::kForward;
        std::cout << "Motion: Forward" << std::endl;
      }
    };

    // 注意：SpotPlanner 现在完全属于物理线程，不需要锁
    SpotPlanner planner;
    planner.setControlFrequency(kSimulationDt);

#ifdef USE_DDS
    bool warned_clamp = false;
#endif

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
      // 归位相关变量
      bool is_homing_complete = false;
      double homing_duration = 2.0;
      double current_sim_time = 0.0;
      std::vector<double> spawn_qpos;

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

        // 记录第一帧的出生姿态
        if (spawn_qpos.empty() && current_state.qpos.size() >= static_cast<size_t>(num_actuators)) {
          spawn_qpos.resize(num_actuators);
          // 注意 MuJoCo qpos offset = 7
          for (int i = 0; i < num_actuators; ++i)
            spawn_qpos[i] = current_state.qpos[7 + i];
        }

        // 3. 计算控制输出
        if (local_use_planner) {
          // === 阶段一：软启动归位 (Homing) ===
          if (!is_homing_complete) {
            std::vector<double> stand_target;
            planner.getJointTargets(stand_target);

            double progress = current_sim_time / homing_duration;

            if (progress < 1.0) {
              double smooth_t = progress * progress * (3 - 2 * progress);
              for (int i = 0; i < num_actuators; ++i) {
                control_target[i] = lerp(spawn_qpos[i], stand_target[i], smooth_t);
              }
            } else {
              is_homing_complete = true;
              planner.setCurrentState(current_state);
              std::cout << "Homing Complete. Robot Standing." << std::endl;
            }

            current_sim_time += kSimulationDt;
          }
          // === 阶段二：正常控制逻辑 ===
          else {
            if (!is_control_active && local_motion != control::BasicMotion::kStand) {
              std::cout << "Motion Command Received. Activating Planner..." << std::endl;
              is_control_active = true;
              planner.setCurrentState(current_state);
            }

            if (is_control_active) {
              if (planner.mode() != local_motion) {
                planner.setMode(local_motion);
              }
              planner.update(current_state);
              planner.getJointTargets(control_target);
              robot.updatePlotData(control_target);
            } else {
              planner.getJointTargets(control_target);
            }
          }

        } else {
          // Raw 模式直接透传
          control_target = local_raw_values;
        }

        // 4. 应用控制并执行物理步进
        robot.applyControlVector(control_target);
        robot.stepPhysics();

        // === 诊断：每 500ms 打印一次物理层身体状态 ===
        static int phys_log_counter = 0;
        if (++phys_log_counter >= 500) {
          phys_log_counter = 0;
          if (current_state.qpos.size() >= 7) {
            double bz   = current_state.qpos[2];
            double q_w  = current_state.qpos[3];
            double q_x  = current_state.qpos[4];
            double q_y  = current_state.qpos[5];
            double q_z  = current_state.qpos[6];
            double zproj = 1.0 - 2.0*(q_x*q_x + q_y*q_y);
            // 欧拉角：roll/pitch
            double sinr  = 2*(q_w*q_x + q_y*q_z);
            double cosr  = 1 - 2*(q_x*q_x + q_y*q_y);
            double roll  = std::atan2(sinr, cosr) * 57.3;
            double sinp  = 2*(q_w*q_y - q_z*q_x);
            double pitch = (std::abs(sinp) >= 1) ? std::copysign(90.0, sinp)
                                                  : std::asin(sinp) * 57.3;
            printf("[PHYS t=%.2f] body_z=%.3f  zproj=%.3f  roll=%.1fdeg  pitch=%.1fdeg"
                   "  homing=%s  ctrl_active=%s\n",
                   current_state.time, bz, zproj, roll, pitch,
                   is_homing_complete ? "done" : "...",
                   is_control_active ? "yes" : "no");
            fflush(stdout);
          }
        }

        // 5. 休眠直到下一个时间片
        next_tick += tick_interval;
        std::this_thread::sleep_until(next_tick);
      }
    };

    // 启动物理线程
    sim_running.store(true, std::memory_order_release);
    physics_thread = std::thread(physics_loop);

    // --- 主线程逻辑 (Rendering & Input) ---
#ifdef USE_DDS
    MujocoDDS_ControlMsg latest_command{};
#endif

    while (true) {
#ifdef USE_DDS
      // 处理 DDS 消息 (非阻塞)
      if (PollDDSCommands(reader, latest_command)) {
        CopyMessageToShared(latest_command, shared_data, warned_clamp);
      }
#endif

      // 渲染帧
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
