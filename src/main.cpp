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

constexpr char kDefaultModel[] = "model/boston_dynamics_spot/scene.xml";
constexpr std::chrono::milliseconds kStepDelay(2);
constexpr int kControlMsgSlots = 16;

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

}  // namespace

int main(int argc, char** argv) {
    std::thread physics_thread;
    bool physics_started = false;
    std::atomic<bool> sim_running{false};

    // // 定义 ToF 分辨率
    // const int tof_width = 320;
    // const int tof_height = 240;
    // 定义相机分辨率
    const int cam_w = 640; // 稍微调大一点看得清楚，或者保持 320
    const int cam_h = 480;
    // 数据容器
    std::vector<unsigned char> rgb_data;
    std::vector<float> depth_data;

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
                //std::this_thread::sleep_for(kStepDelay);
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
            }

            // 1. 获取数据
            robot.getCameraImages("tof_cam", cam_w, cam_h, rgb_data, depth_data);

            // 2. OpenCV 处理与显示
            if (!rgb_data.empty() && !depth_data.empty()) {
                // --- 处理 RGB 部分 ---
                // MuJoCo 的 RGB 是紧凑排列的 R,G,B
                cv::Mat img_rgb_raw(cam_h, cam_w, CV_8UC3, rgb_data.data());
                cv::Mat img_rgb_flipped;
                
                // MuJoCo 图像原点在左下角，OpenCV 在左上角，所以要沿 X 轴翻转 (code 0)
                cv::flip(img_rgb_raw, img_rgb_flipped, 0);
                
                // MuJoCo 是 RGB 顺序，OpenCV 显示通常默认 BGR，转换一下颜色正常点
                cv::Mat img_bgr;
                cv::cvtColor(img_rgb_flipped, img_bgr, cv::COLOR_RGB2BGR);

                // --- 处理 Depth 部分 ---
                cv::Mat img_depth_raw(cam_h, cam_w, CV_32F, depth_data.data());
                cv::Mat img_depth_flipped;
                cv::flip(img_depth_raw, img_depth_flipped, 0);

                // 归一化：将 0~10米 映射到 0~255，便于伪彩色显示
                cv::Mat img_depth_norm;
                double max_dist = 10.0; // 超过10米就是最亮
                // convertTo(输出, 类型, 缩放因子)
                img_depth_flipped.convertTo(img_depth_norm, CV_8U, 255.0 / max_dist);

                // 伪彩色处理 (把黑白深度图变成彩虹色热力图，方便观察)
                // 结果也是一个 3 通道的图 (CV_8UC3)
                cv::Mat img_depth_color;
                cv::applyColorMap(img_depth_norm, img_depth_color, cv::COLORMAP_JET);

                // --- 拼接 ---
                // 左边显示深度 (img_depth_color)，右边显示 RGB (img_bgr)
                cv::Mat combined_view;
                std::vector<cv::Mat> matrices = {img_depth_color, img_bgr};
                cv::hconcat(matrices, combined_view);

                // --- 显示 ---
                cv::imshow("Spot Robot Camera (Left: Depth, Right: RGB)", combined_view);
                cv::waitKey(1); // 刷新窗口
            }

            robot.renderFrame();

            if (robot.windowShouldClose()) {
                break;
            }
        }

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
