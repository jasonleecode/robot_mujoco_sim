#include "planner.h"
#include <cmath>
#include <algorithm>

static constexpr int kNumJoints = 12;

// 默认站立姿态
static const double kStandPose[kNumJoints] = {
    0, 1.04, -1.8,   // FL (HX, HY, KN)
    0, 1.04, -1.8,   // FR
    0, 1.04, -1.8,   // HL
    0, 1.04, -1.8    // HR
};

SpotPlanner::SpotPlanner() : phase_(0.0), mode_(control::BasicMotion::kStand), last_time_(-1.0) {}

void SpotPlanner::reset() {
    phase_ = 0.0;
    mode_ = control::BasicMotion::kStand;
    last_time_ = -1.0;
}

void SpotPlanner::setMode(control::BasicMotion mode) {
    // 如果从前进切换到停止，可以在这里记录状态以做平滑处理
    mode_ = mode;
}

void SpotPlanner::update(const RobotState& state) {
    if (last_time_ < 0) {
        last_time_ = state.time;
        return;
    }
    double dt = state.time - last_time_;
    last_time_ = state.time;

    // 1. 设置步态频率
    double target_freq = (mode_ == control::BasicMotion::kForward) ? 1.5 : 0.0;

    // 简单的转向平滑滤波
    double target_yaw = 0.0;
    if (mode_ == control::BasicMotion::kTurnLeft) target_yaw = 0.4;
    else if (mode_ == control::BasicMotion::kTurnRight) target_yaw = -0.4;

    // 线性插值，让转向动作不那么突兀
    current_yaw_rate_ += (target_yaw - current_yaw_rate_) * 0.1;

    // 1. 实现速度斜坡 (Speed Ramp)
    double target_multiplier = (mode_ == control::BasicMotion::kForward) ? 1.0 : 0.0;
    
    // 每一帧平滑靠近目标速度，0.05 是平滑系数（可根据需要调整）
    // 这会让机器人从静止到全速大约需要 0.5 - 1 秒的过渡期
    speed_multiplier_ += (target_multiplier - speed_multiplier_) * 0.05;

    // 2. 根据速度倍率动态调整频率
    // 静止时频率为 0，起步瞬间频率低，随后变快
    double freq = 1.5 * speed_multiplier_; 
    
    phase_ += freq * dt;
    if (phase_ > 1.0) phase_ -= 1.0;
    
    // 如果处于 Stand 模式且动作基本回到原点，强制归零
    if (mode_ == control::BasicMotion::kStand && speed_multiplier_ < 0.01) {
        phase_ = 0.0;
    }
    
    // 2. 更新相位：只有在前进模式或相位未回到 0 时继续推进（确保停步时动作完整）
    if (mode_ == control::BasicMotion::kForward || phase_ > 0.01) {
        double freq = 1.5; // 固定频率确保动作节奏一致
        phase_ += freq * dt;
        if (phase_ > 1.0) phase_ -= 1.0;
        
        // 3. 停止逻辑：如果模式是 Stand 且相位接近周期结束，强制归零实现平滑停止
        if (mode_ == control::BasicMotion::kStand && phase_ > 0.95) {
            phase_ = 0.0;
        }
    }
}

void SpotPlanner::getJointTargets(std::vector<double>& qref) {
    qref.resize(kNumJoints);

    if (mode_ == control::BasicMotion::kStand && phase_ < 0.001) {
        for (int i = 0; i < kNumJoints; i++) qref[i] = kStandPose[i];
        return;
    }

    // 1. 定义转向强度 (Yaw Rate)
    // turn_factor > 0 向左转, < 0 向右转
    double turn_factor = 0.0;
    if (mode_ == control::BasicMotion::kTurnLeft) turn_factor = 0.4;
    if (mode_ == control::BasicMotion::kTurnRight) turn_factor = -0.4;
    
    // 是否处于前进模式
    bool is_forward = (mode_ == control::BasicMotion::kForward);

    const double offsets[4] = {0.0, 0.5, 0.5, 0.0}; // FL, FR, HL, HR

    // 3. 应用 speed_multiplier_ 到步幅参数
    double stride_x = 0.25 * speed_multiplier_; // 前后跨度随速度增加
    double lift_h = 0.4 * speed_multiplier_;   // 抬腿高度随速度增加
    
    for (int leg = 0; leg < 4; ++leg) {
        int idx = leg * 3;
        double leg_phase = std::fmod(phase_ + offsets[leg], 1.0);
        
        // 核心步态曲线
        double swing_x = std::cos(2 * M_PI * leg_phase); 
        double lift = (leg_phase < 0.5) ? std::sin(M_PI * leg_phase / 0.5) : 0;

        // 2. 转向逻辑实现：
        // 对于左侧腿 (leg 0, 2) 和右侧腿 (leg 1, 3) 给定相反的步幅修正
        double side_sign = (leg == 0 || leg == 2) ? 1.0 : -1.0;
        
        // 步幅增益计算
        double stride_x = 0.2; // 默认前进步幅
        if (!is_forward) stride_x = 0.0; // 纯原地转弯时前进步幅为0
        
        // 转向是通过改变左右两侧腿的前后移动方向实现的
        // 迈步时：一侧向前迈，另一侧向后迈（或迈得更小）
        double final_stride = stride_x + (turn_factor * side_sign);

        // 3. 关节映射
    /*
        // HX (侧摆): 转向时可以稍微增加侧向外摆，增加稳定性
        qref[idx + 0] = kStandPose[idx + 0] + (0.1 * turn_factor * side_sign * lift); 
        
        // HY (大腿前后): 受 final_stride 影响
        qref[idx + 1] = kStandPose[idx + 1] - (final_stride * swing_x) - (0.1 * lift);
        
        // KN (膝盖): 保持抬腿高度
        qref[idx + 2] = kStandPose[idx + 2] + (0.4 * lift);
    */    
        

        qref[idx + 0] = kStandPose[idx + 0]; 
        qref[idx + 1] = kStandPose[idx + 1] - (stride_x * swing_x) - (0.1 * lift * speed_multiplier_);
        qref[idx + 2] = kStandPose[idx + 2] + (lift_h * lift);
    }
}