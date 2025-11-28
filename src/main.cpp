#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include <dds/dds.h>

#include "ControlActions.hpp"
#include "RobotSim.hpp"
#include "dds_generated/ControlMsg.h"

namespace {

constexpr char kDefaultModel[] = "boston_dynamics_spot/scene.xml";
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

// Basic sinusoidal control, enough to show actuator wiring works.
void FillDemoControls(std::vector<double>& ctrl_buffer, double sim_time_s) {
    constexpr double kAmp = 0.25;
    constexpr double kFreq = 0.5;  // Hz
    for (int i = 0; i < static_cast<int>(ctrl_buffer.size()); ++i) {
        double phase = sim_time_s * kFreq * 2.0 * mjPI + (i * 0.15);
        ctrl_buffer[i] = kAmp * std::sin(phase);
    }
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

}  // namespace

int main(int argc, char** argv) {
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

        std::vector<double> control(num_actuators, 0.0);
        auto t0 = std::chrono::steady_clock::now();
        bool warned_clamp = false;
        bool remote_override = false;
        MujocoDDS_ControlMsg latest_command{};


        while (robot.isWindowOpen()) {
            const auto now = std::chrono::steady_clock::now();
            const double sim_time =
                std::chrono::duration<double>(now - t0).count();
            const bool has_command = PollDDSCommands(reader, latest_command);
            if (has_command) {
                if (latest_command.mode ==
                    static_cast<uint32_t>(control::CommandMode::kBasic)) {
                    control::BasicMotion motion;
                    if (control::TryParseBasicMotion(latest_command.action, motion)) {
                        robot.applyBasicMotion(motion, control);
                        remote_override = true;
                    } else {
                        std::cerr << "Unknown basic motion id: "
                                  << latest_command.action << '\n';
                    }
                } else {
                    CopyMessageToControl(latest_command, control, warned_clamp);
                    remote_override = true;
                }
            }

            if (!remote_override) {
                FillDemoControls(control, sim_time);
            }

            for (int i = 0; i < num_actuators; ++i) {
                robot.setControl(i, control[i]);
            }

            robot.step();

            // std::this_thread::sleep_for(kStepDelay);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}