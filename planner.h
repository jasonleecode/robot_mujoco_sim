#pragma once

#include <vector>

#include "ControlActions.hpp"

struct RobotState {
    std::vector<double> qpos;
    std::vector<double> qvel;
    double time = 0.0;
};

class SpotPlanner {
public:
    SpotPlanner();

    void reset();
    void setMode(control::BasicMotion motion);
    control::BasicMotion mode() const { return mode_; }
    void update(const RobotState& state);
    void getJointTargets(std::vector<double>& qref);

private:
    double phase_;
    double last_time_;
    control::BasicMotion mode_;
    double current_yaw_rate_ = 0.0; // 用于平滑转向过渡
    double speed_multiplier_ = 0.0;
};