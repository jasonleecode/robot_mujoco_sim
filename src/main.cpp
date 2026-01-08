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
constexpr std::chrono::milliseconds kStepDelay(2);
constexpr int kControlMsgSlots = 16;

struct ScopedParticipant {
    ~ScopedParticipant() {
        if (handle > 0) {
            dds_delete(handle);
        }
    }

    dds_entity_t handle = DDS_ENTITY_NIL;
};

struct SharedControlBuffer {
    std::vector<double> values;
    std::mutex mutex;
};

void CopyMessageToControl(const MujocoDDS_ControlMsg& msg,
                          std::vector<double>& control,
                          bool& warned_clamp) {
    const int copy_count = std::min<int>(control.size(), kControlMsgSlots);
    for (int i = 0; i < copy_count; ++i) {
        control[i] = msg.values[i];
    }
    if (static_cast<int>(control.size()) > kControlMsgSlots && !warned_clamp) {
        std::cerr << "Warning: DDS ControlMsg supports only " << kControlMsgSlots
                  << " actuators, clamping from " << control.size() << '\n';
        warned_clamp = true;
    }
}

bool PollDDSCommands(dds_entity_t reader, MujocoDDS_ControlMsg& latest_command) {
    constexpr int kSamplesPerTake = 4;
    void* samples[kSamplesPerTake] = {};
    dds_sample_info_t infos[kSamplesPerTake];
    bool updated = false;

    while (true) {
        const dds_return_t rc =
            dds_take(reader, samples, infos, kSamplesPerTake, kSamplesPerTake);
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
              << "' is used relative to the repository root.\n";
}

std::string ResolveModelPath(int argc, char** argv) {
    if (argc > 1) {
        return argv[1];
    }

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
    bool physics_started = false;
    std::atomic<bool> sim_running{false};

    try {
        PrintUsage(argv[0]);
        SanityCheckRuntime();

        const std::string model_path = ResolveModelPath(argc, argv);
        if (!std::filesystem::exists(model_path)) {
            throw std::runtime_error("Model file not found: " + model_path);
        }

        RobotSim robot(model_path);
        const int num_actuators = robot.getNumActuators();
        std::cout << "Loaded model '" << model_path << "' with "
                  << num_actuators << " actuators\n";

        ScopedParticipant participant_guard;
        participant_guard.handle =
            RequireOk(dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr),
                      "dds_create_participant");
        const dds_entity_t topic =
            RequireOk(dds_create_topic(participant_guard.handle, &MujocoDDS_ControlMsg_desc,
                                       "RobotControl", nullptr, nullptr),
                      "dds_create_topic");
        const dds_entity_t reader =
            RequireOk(dds_create_reader(participant_guard.handle, topic, nullptr, nullptr),
                      "dds_create_reader");

        SpotPlanner planner;
        planner.reset();
        RobotState robot_state;
        SharedControlBuffer shared_control;
        shared_control.values.assign(num_actuators, 0.0);
        
        bool warned_clamp = false;
        MujocoDDS_ControlMsg latest_command{};
        bool planner_drive = true;   // start with planner standing
        bool received_remote = false;
        auto physics_loop = [&]() {
            std::vector<double> local = shared_control.values;
            while (sim_running.load(std::memory_order_acquire)) {
                {
                    std::lock_guard<std::mutex> lock(shared_control.mutex);
                    local = shared_control.values;
                }
                robot.applyControlVector(local);
                robot.stepPhysics();
            }
        };

        sim_running.store(true, std::memory_order_release);
        physics_thread = std::thread(physics_loop);
        physics_started = true;

        while (true) {
            const bool has_command = PollDDSCommands(reader, latest_command);
            if (has_command) {
                std::lock_guard<std::mutex> lock(shared_control.mutex);
                if (latest_command.mode ==
                    static_cast<uint32_t>(control::CommandMode::kBasic)) {
                    control::BasicMotion motion;
                    if (control::TryParseBasicMotion(latest_command.action, motion)) {
                        if (motion == control::BasicMotion::kForward ||
                            motion == control::BasicMotion::kStand) {
                            planner.setMode(motion);
                            planner_drive = true;
                        } else {
                            robot.applyBasicMotion(motion, shared_control.values);
                            planner_drive = false;
                        }
                        received_remote = true;
                    } else {
                        std::cerr << "Unknown basic motion id: "
                                  << latest_command.action << '\n';
                    }
                } else {
                    CopyMessageToControl(latest_command, shared_control.values, warned_clamp);
                    planner_drive = false;
                    received_remote = true;
                }
            } 

            if (planner_drive) {
                robot.getState(robot_state);
                planner.update(robot_state);
                std::vector<double> qref;
                planner.getJointTargets(qref);
                std::lock_guard<std::mutex> lock(shared_control.mutex);
                shared_control.values.assign(qref.begin(), qref.end());


                if (qref.size() >= 12) {
                    int p_idx = robot.plot_idx % robot.kPlotPoints;
                    // 1. 记录四条腿的膝关节目标值 (Target)
                    robot.plot_data[0][p_idx] = (float)qref[2];  // FR
                    robot.plot_data[1][p_idx] = (float)qref[5];  // FL
                    robot.plot_data[2][p_idx] = (float)qref[8];  // RR
                    robot.plot_data[3][p_idx] = (float)qref[11]; // RL
                    robot.plot_idx++;

                    // 2. 将数据填入 mjvFigure 的线性缓冲区
                    for (int k = 0; k < robot.kPlotPoints; k++) {
                        int history_idx = (robot.plot_idx + k) % robot.kPlotPoints; 
                        
                        // 遍历 4 条线
                        for (int line = 0; line < 4; ++line) {
                            // 偶数位存 x (时间/索引)，奇数位存 y (数值)
                            robot.fig.linedata[line][2*k]   = (float)k;
                            robot.fig.linedata[line][2*k+1] = robot.plot_data[line][history_idx];
                        }
                    }
                }
            }

            robot.renderFrame();

            if (robot.windowShouldClose()) {
                break;
            }
        }

        // 清理退出
        sim_running.store(false, std::memory_order_release);
        if (physics_started && physics_thread.joinable()) {
            physics_thread.join();
        }

        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        sim_running.store(false, std::memory_order_release);
        if (physics_started && physics_thread.joinable()) {
            physics_thread.join();
        }
        return EXIT_FAILURE;
    }
}