#pragma once

#include <cstdint>

namespace control {

enum class CommandMode : uint32_t {
    kRaw = 0,
    kBasic = 1,
};

enum class BasicMotion : uint32_t {
    kForward = 0,
    kBackward,
    kTurnLeft,
    kTurnRight,
    kCrouch,
    kStand,
    kJump,
    kCount
};

inline bool TryParseBasicMotion(uint32_t value, BasicMotion& motion) {
    if (value >= static_cast<uint32_t>(BasicMotion::kCount)) {
        return false;
    }
    motion = static_cast<BasicMotion>(value);
    return true;
}

inline const char* ToString(BasicMotion motion) {
    switch (motion) {
        case BasicMotion::kForward:
            return "forward";
        case BasicMotion::kBackward:
            return "backward";
        case BasicMotion::kTurnLeft:
            return "turn_left";
        case BasicMotion::kTurnRight:
            return "turn_right";
        case BasicMotion::kCrouch:
            return "crouch";
        case BasicMotion::kStand:
            return "stand";
        case BasicMotion::kJump:
            return "jump";
        case BasicMotion::kCount:
        default:
            return "unknown";
    }
}

}  // namespace control

