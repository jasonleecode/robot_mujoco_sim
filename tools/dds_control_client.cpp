#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <dds/dds.h>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "ControlActions.hpp"
#include "ControlMsg.h"

namespace {

enum class ClientMode { Raw, Basic };

int RequireOk(int value, const char* label) {
    if (value < 0) {
        throw std::runtime_error(std::string(label) + ": " + dds_strretcode(-value));
    }
    return value;
}

std::string Trim(const std::string& input) {
    const auto first = input.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = input.find_last_not_of(" \t\r\n");
    return input.substr(first, last - first + 1);
}

std::string NormalizeToken(const std::string& token) {
    std::string result = token;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return result;
}

ClientMode ParseMode(int argc, char** argv) {
    if (argc < 2) {
        throw std::runtime_error("Usage: dds_control_client <raw|basic>");
    }

    std::string arg = argv[1];
    std::string mode = arg;

    if (arg == "--mode" || arg == "-m") {
        if (argc < 3) {
            throw std::runtime_error("Missing value for --mode");
        }
        mode = argv[2];
    } else {
        auto pos = arg.find('=');
        if (pos != std::string::npos) {
            mode = arg.substr(pos + 1);
        }
    }

    std::string normalized = NormalizeToken(mode);
    if (normalized == "raw") {
        return ClientMode::Raw;
    }
    if (normalized == "basic") {
        return ClientMode::Basic;
    }

    throw std::runtime_error("Unknown control mode: " + mode);
}

std::vector<double> ParseLine(const std::string& line) {
    std::istringstream iss(line);
    std::vector<double> values;
    double val;
    while (iss >> val) {
        values.push_back(val);
        if (values.size() == 16) {
            break;
        }
    }
    return values;
}

void FillMessage(const std::vector<double>& values, MujocoDDS_ControlMsg& msg) {
    for (int i = 0; i < 16; ++i) {
        msg.values[i] = (i < static_cast<int>(values.size())) ? values[i] : 0.0;
    }
}

bool LookupBasicMotion(const std::string& token, control::BasicMotion& motion) {
    const std::vector<std::pair<std::string, control::BasicMotion>> mapping = {
        {"forward", control::BasicMotion::kForward},
        {"f", control::BasicMotion::kForward},
        {"前进", control::BasicMotion::kForward},
        {"backward", control::BasicMotion::kBackward},
        {"b", control::BasicMotion::kBackward},
        {"后退", control::BasicMotion::kBackward},
        {"left", control::BasicMotion::kTurnLeft},
        {"turn_left", control::BasicMotion::kTurnLeft},
        {"l", control::BasicMotion::kTurnLeft},
        {"左转", control::BasicMotion::kTurnLeft},
        {"right", control::BasicMotion::kTurnRight},
        {"turn_right", control::BasicMotion::kTurnRight},
        {"r", control::BasicMotion::kTurnRight},
        {"右转", control::BasicMotion::kTurnRight},
        {"crouch", control::BasicMotion::kCrouch},
        {"prone", control::BasicMotion::kCrouch},
        {"趴下", control::BasicMotion::kCrouch},
        {"stand", control::BasicMotion::kStand},
        {"standup", control::BasicMotion::kStand},
        {"站起", control::BasicMotion::kStand},
        {"jump", control::BasicMotion::kJump},
        {"跃", control::BasicMotion::kJump},
        {"跳跃", control::BasicMotion::kJump},
    };

    const std::string normalized = NormalizeToken(token);
    for (const auto& [key, value] : mapping) {
        if (normalized == key || token == key) {
            motion = value;
            return true;
        }
    }
    return false;
}

void PrintBasicMenu() {
    std::cout << "Available basic motions:\n"
                 "  forward / 前进\n"
                 "  backward / 后退\n"
                 "  left / turn_left / 左转\n"
                 "  right / turn_right / 右转\n"
                 "  crouch / prone / 趴下\n"
                 "  stand / 站起\n"
                 "  jump / 跳跃\n"
                 "Type 'list' to show this menu again.\n";
}

int RunRawMode(dds_entity_t writer) {
    std::cout << "Raw mode: enter up to 16 actuator values separated by spaces per line.\n"
                 "Press Ctrl+D to exit.\n";
    MujocoDDS_ControlMsg msg{};
    msg.mode = static_cast<uint32_t>(control::CommandMode::kRaw);
    msg.action = 0;

    std::string line;
    while (std::cout << "> " && std::getline(std::cin, line)) {
        line = Trim(line);
        if (line.empty()) {
            continue;
        }

        auto values = ParseLine(line);
        if (values.empty()) {
            std::cout << "No valid numbers detected.\n";
            continue;
        }
        FillMessage(values, msg);
        RequireOk(dds_write(writer, &msg), "dds_write");
        std::cout << "Sent " << values.size() << " actuator values.\n";
    }

    return EXIT_SUCCESS;
}

int RunBasicMode(dds_entity_t writer) {
    std::cout << "Basic mode: type a motion keyword (e.g., forward, left, jump) and press Enter.\n";
    PrintBasicMenu();

    MujocoDDS_ControlMsg msg{};
    msg.mode = static_cast<uint32_t>(control::CommandMode::kBasic);
    msg.action = 0;
    std::fill(std::begin(msg.values), std::end(msg.values), 0.0);

    std::string line;
    while (std::cout << "basic> " && std::getline(std::cin, line)) {
        line = Trim(line);
        if (line.empty()) {
            continue;
        }

        if (line == "list" || line == "help") {
            PrintBasicMenu();
            continue;
        }

        control::BasicMotion motion;
        if (!LookupBasicMotion(line, motion)) {
            std::cout << "Unknown motion '" << line << "'. Type 'list' to view options.\n";
            continue;
        }

        msg.action = static_cast<uint32_t>(motion);
        RequireOk(dds_write(writer, &msg), "dds_write");
        std::cout << "Sent basic action: " << control::ToString(motion) << '\n';
    }

    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const ClientMode mode = ParseMode(argc, argv);

        dds_entity_t participant = RequireOk(
            dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr),
            "dds_create_participant");
        const dds_entity_t topic = RequireOk(
            dds_create_topic(participant, &MujocoDDS_ControlMsg_desc, "RobotControl", nullptr, nullptr),
            "dds_create_topic");
        const dds_entity_t writer =
            RequireOk(dds_create_writer(participant, topic, nullptr, nullptr),
                      "dds_create_writer");

        int result = EXIT_SUCCESS;
        if (mode == ClientMode::Basic) {
            result = RunBasicMode(writer);
        } else {
            result = RunRawMode(writer);
        }

        dds_delete(participant);
        return result;
    } catch (const std::exception& e) {
        std::cerr << "Client error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}

