#include "planner.h"
#include "Quadruped/Robot.h"
#include "Quadruped/Leg.h"
#include "spot_robot_params.h"
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

static constexpr int kNumJoints = 12;

// 默认站立姿态的关节角度（FL, FR, RL, RR顺序，每个腿3个关节）
// 从spot.xml的keyframe "home"提取：qpos="0 1.04 -1.8" (每个腿)
static const double kStandPose[kNumJoints] = {
    spot_params::STAND_HX, spot_params::STAND_HY, spot_params::STAND_KN,   // FL (HX, HY, KN)
    spot_params::STAND_HX, spot_params::STAND_HY, spot_params::STAND_KN,   // FR
    spot_params::STAND_HX, spot_params::STAND_HY, spot_params::STAND_KN,   // RL
    spot_params::STAND_HX, spot_params::STAND_HY, spot_params::STAND_KN    // RR
};

// 注意：planner模块的Robot类使用硬编码的宏定义参数（LEN_BASE, LEN_HIP等），
// 这些参数是为Go2机器人设计的。Spot机器人的实际参数记录在spot_robot_params.h中。
// 由于无法直接修改planner模块的参数，当前实现使用实际机器人状态进行步态调整。

// 腿索引映射：SpotPlanner索引 -> Robot legs数组索引
// SpotPlanner内部顺序: 0=FL, 1=FR, 2=RL, 3=RR
// Robot legs数组顺序:  0=FR, 1=FL, 2=RR, 3=RL
static const int kLegIndexMap[4] = {1, 0, 3, 2}; // FL->1, FR->0, RL->3, RR->2

// MuJoCo关节顺序: FL, FR, HL(RL), HR(RR) - 但qpos前7个是body位置和四元数，关节从索引7开始
// SpotPlanner内部顺序: FL, FR, RL, RR
// Robot类期望顺序: FR, FL, RR, RL

// 将SpotPlanner内部顺序(FL,FR,RL,RR)转换为Robot类期望顺序(FR,FL,RR,RL)
static std::vector<double> convertToRobotOrder(const std::vector<double>& spot_angles) {
    std::vector<double> robot_angles(12);
    // SpotPlanner: FL(0), FR(1), RL(2), RR(3) -> Robot: FR(1), FL(0), RR(3), RL(2)
    for (int leg = 0; leg < 4; ++leg) {
        int robot_leg = kLegIndexMap[leg];
        for (int joint = 0; joint < 3; ++joint) {
            robot_angles[robot_leg * 3 + joint] = spot_angles[leg * 3 + joint];
        }
    }
    return robot_angles;
}

// MuJoCo qpos中关节角度的起始索引（前7个是body位置和四元数）
static constexpr int kMuJoCoJointStartIdx = 7;

SpotPlanner::SpotPlanner() 
    : robot_model_(new Robot())
    , phase_(0.0)
    , mode_(control::BasicMotion::kStand)
    , last_time_(-1.0)
{
    // 初始化标准站立角度（SpotPlanner内部顺序：FL, FR, RL, RR）
    stand_angles_.assign(kStandPose, kStandPose + kNumJoints);
    
    // 转换为Robot类期望的顺序并设置
    std::vector<double> robot_angles = convertToRobotOrder(stand_angles_);
    robot_model_->setAngles(robot_angles);
    
    // 获取标准站立位置的足端位置（相对于机器人基座）
    // getPosition_*方法返回的是实际的FL/FR/RL/RR位置，不依赖于legs数组索引
    stand_foot_positions_.resize(4);
    stand_foot_positions_[0] = robot_model_->getPosition_FL(); // FL
    stand_foot_positions_[1] = robot_model_->getPosition_FR(); // FR
    stand_foot_positions_[2] = robot_model_->getPosition_RL(); // RL
    stand_foot_positions_[3] = robot_model_->getPosition_RR(); // RR
}

SpotPlanner::~SpotPlanner() {
    if (robot_model_) {
        delete robot_model_;
        robot_model_ = nullptr;
    }
}

int SpotPlanner::mapLegIndex(int spot_leg_index) const {
    if (spot_leg_index >= 0 && spot_leg_index < 4) {
        return kLegIndexMap[spot_leg_index];
    }
    return spot_leg_index;
}

void SpotPlanner::reset() {
    phase_ = 0.0;
    mode_ = control::BasicMotion::kStand;
    last_time_ = -1.0;
    speed_multiplier_ = 0.0;
    current_yaw_rate_ = 0.0;
    has_current_state_ = false;
    
    // 重置Robot到标准站立位置（需要转换为Robot类期望的顺序）
    std::vector<double> robot_angles = convertToRobotOrder(stand_angles_);
    robot_model_->setAngles(robot_angles);
}

void SpotPlanner::setCurrentState(const RobotState& state) {
    current_state_ = state;
    has_current_state_ = true;
}

void SpotPlanner::setMode(control::BasicMotion mode) {
    mode_ = mode;
}

void SpotPlanner::update(const RobotState& state) {
    // 保存当前状态用于步态调整
    current_state_ = state;
    has_current_state_ = true;
    
    if (last_time_ < 0) {
        last_time_ = state.time;
        return;
    }
    double dt = state.time - last_time_;
    last_time_ = state.time;

    // 简单的转向平滑滤波
    double target_yaw = 0.0;
    if (mode_ == control::BasicMotion::kTurnLeft) target_yaw = 0.4;
    else if (mode_ == control::BasicMotion::kTurnRight) target_yaw = -0.4;

    // 线性插值，让转向动作不那么突兀
    current_yaw_rate_ += (target_yaw - current_yaw_rate_) * 0.1;

    // 速度斜坡 (Speed Ramp) - 平滑加速/减速
    double target_multiplier = (mode_ == control::BasicMotion::kForward) ? 1.0 : 0.0;
    
    // 每一帧平滑靠近目标速度，0.05 是平滑系数（可根据需要调整）
    speed_multiplier_ += (target_multiplier - speed_multiplier_) * 0.05;

    // 更新步态相位：根据速度倍率动态调整频率
    double freq = 1.5 * speed_multiplier_; // 步态频率 (Hz)
    
    if (mode_ == control::BasicMotion::kForward || phase_ > 0.01) {
        // 只有在前进模式或相位未回到 0 时继续推进（确保停步时动作完整）
        phase_ += freq * dt;
        if (phase_ > 1.0) phase_ -= 1.0;
    }
    
    // 停止逻辑：如果模式是 Stand 且速度倍率接近0，强制归零实现平滑停止
    if (mode_ == control::BasicMotion::kStand && speed_multiplier_ < 0.01) {
        phase_ = 0.0;
    }
}

void SpotPlanner::getJointTargets(std::vector<double>& qref) {
    qref.resize(kNumJoints);

    // 如果是站立模式且相位为0，直接返回标准站立角度
    if (mode_ == control::BasicMotion::kStand && phase_ < 0.001) {
        qref = stand_angles_;
        return;
    }

    // 是否处于前进模式
    bool is_forward = (mode_ == control::BasicMotion::kForward);
    
    // 转向因子
    double turn_factor = 0.0;
    if (mode_ == control::BasicMotion::kTurnLeft) turn_factor = 0.4;
    if (mode_ == control::BasicMotion::kTurnRight) turn_factor = -0.4;

    // Trot步态相位偏移 [FL, FR, RL, RR]
    const double offsets[4] = {0.0, 0.5, 0.5, 0.0};

    // 步态参数
    double base_stride_x = is_forward ? 0.08 : 0.0; // 基础前进步幅（米）
    double stride_x = base_stride_x * speed_multiplier_;
    double lift_h = 0.05 * speed_multiplier_; // 抬腿高度（米）

    // 对每条腿计算足端轨迹并使用逆运动学
    for (int leg = 0; leg < 4; ++leg) {
        int idx = leg * 3;
        double leg_phase = std::fmod(phase_ + offsets[leg], 1.0);
        
        // 获取标准站立位置的足端位置
        Eigen::Vector3d base_foot_pos = stand_foot_positions_[leg];
        
        // 计算足端轨迹偏移
        Eigen::Vector3d foot_offset(0, 0, 0);
        
        // 前后摆动（X方向）
        double swing_x = std::cos(2 * M_PI * leg_phase); // -1到1
        foot_offset.x() = stride_x * swing_x;
        
        // 抬腿（Z方向，向上为正）
        double lift = (leg_phase < 0.5) ? std::sin(M_PI * leg_phase / 0.5) : 0;
        foot_offset.z() = lift_h * lift;
        
        // 转向修正（Y方向）
        double side_sign = (leg == 0 || leg == 2) ? 1.0 : -1.0; // 左侧为正
        foot_offset.y() = turn_factor * side_sign * 0.02;
        
        // 目标足端位置
        Eigen::Vector3d target_foot_pos = base_foot_pos + foot_offset;
        
        // 获取当前关节角度：优先使用实际机器人状态，否则使用标准站立位置
        // 注意：MuJoCo的qpos前7个是body位置和四元数，关节角度从索引7开始
        // MuJoCo顺序：FL, FR, HL(RL), HR(RR) - 与SpotPlanner内部顺序一致（HL=RL, HR=RR）
        Eigen::Vector3d current_angles;
        if (has_current_state_ && current_state_.qpos.size() >= kMuJoCoJointStartIdx + kNumJoints) {
            // 使用实际机器人状态（MuJoCo顺序与SpotPlanner内部顺序一致）
            int mujoco_idx = kMuJoCoJointStartIdx + idx;
            current_angles[0] = current_state_.qpos[mujoco_idx];     // HX
            current_angles[1] = current_state_.qpos[mujoco_idx + 1]; // HY
            current_angles[2] = current_state_.qpos[mujoco_idx + 2]; // KN
        } else {
            // 使用标准站立位置
            current_angles[0] = stand_angles_[idx];
            current_angles[1] = stand_angles_[idx + 1];
            current_angles[2] = stand_angles_[idx + 2];
        }
        
        // 计算足端位置变化量
        Eigen::Vector3d delta_pos = target_foot_pos - base_foot_pos;
        
        // 使用逆运动学计算新的关节角度
        // 注意：需要映射到Robot类的legs数组索引
        int robot_leg_idx = mapLegIndex(leg);
        Leg* robot_leg = robot_model_->legs[robot_leg_idx];
        
        // 使用运动学模型的逆运动学求解（基于当前实际角度）
        Eigen::Vector3d new_angles = robot_leg->getKinematics()->jointAngleCompute(
            current_angles, delta_pos
        );
        
        // 注意：jointAngleCompute内部已经应用了关节限制，所以这里不需要再次调用
        
        // 设置到输出向量
        qref[idx] = new_angles[0];     // HX
        qref[idx + 1] = new_angles[1]; // HY
        qref[idx + 2] = new_angles[2]; // KN
    }
}
