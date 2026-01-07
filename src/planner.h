#pragma once

#include <vector>
#include <Eigen/Dense>

#include "ControlActions.hpp"

// 前向声明planner模块的Robot类
class Robot;
class Leg;

struct RobotState {
    std::vector<double> qpos;
    std::vector<double> qvel;
    double time = 0.0;
};

/**
 * @class SpotPlanner
 * @brief 封装planner模块的接口类，使用Robot类和运动学模型进行步态规划
 * 
 * 这个类是对planner模块的封装，内部使用planner模块的Robot类和运动学模型
 * 来生成机器人的步态控制指令。
 */
class SpotPlanner {
public:
    SpotPlanner();
    ~SpotPlanner();

    void reset();
    void setMode(control::BasicMotion motion);
    control::BasicMotion mode() const { return mode_; }
    void update(const RobotState& state);
    void getJointTargets(std::vector<double>& qref);
    
    // 设置实际机器人状态（用于步态调整）
    void setCurrentState(const RobotState& state);

private:
    // planner模块的Robot类实例
    Robot* robot_model_;
    
    // 步态参数
    double phase_;
    double last_time_;
    control::BasicMotion mode_;
    double current_yaw_rate_ = 0.0; // 用于平滑转向过渡
    double speed_multiplier_ = 0.0;
    
    // 标准站立位置的关节角度（用于初始化）
    std::vector<double> stand_angles_;
    
    // 标准站立位置的足端位置（在机器人坐标系中）
    std::vector<Eigen::Vector3d> stand_foot_positions_;
    
    // 当前实际机器人状态（用于步态调整）
    RobotState current_state_;
    bool has_current_state_ = false;
    
    // 辅助函数：将SpotPlanner的腿索引映射到Robot的legs数组索引
    // SpotPlanner: 0=FL, 1=FR, 2=RL, 3=RR
    // Robot legs:  0=FR, 1=FL, 2=RR, 3=RL
    int mapLegIndex(int spot_leg_index) const;
};