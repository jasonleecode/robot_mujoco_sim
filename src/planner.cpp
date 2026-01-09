#include "planner.h"
#include "LegIndexHelper.hpp"
#include <iostream>

// 构造函数：初始化 planner 库的对象
SpotPlanner::SpotPlanner() 
    : last_time_(0.0)
    , mode_(control::BasicMotion::kStand)
{
    // 1. 初始化算法模型
    // 注意：TrotGait 的构造函数需要 Robot 指针和 nodeName
    // 在非 ROS 模式下，nodeName 可以为空
    trot_gait_ = std::make_unique<TrotGait>(&planner_robot_, "");

    // 2. 初始化一些默认参数 (例如步态频率、高度等)
    trot_gait_->setStanceDuration(250);
    
    // 3. 在非ROS模式下，设置standing为true，以便步态算法可以执行
    // standing标志表示机器人已经处于站立状态，可以开始执行步态动作
    trot_gait_->standing = false;
}

SpotPlanner::~SpotPlanner() {
    // unique_ptr 会自动释放
}

void SpotPlanner::reset() {
    mode_ = control::BasicMotion::kStand;
    trot_gait_->setGaitMotion(GaitMotion::STOP);
    trot_gait_->standing = true;  // 确保standing标志为true
    // 重置 robot 状态...
}

void SpotPlanner::setMode(control::BasicMotion motion) {
    mode_ = motion;
    
    // 将外部的 ControlAction 映射到 TrotGait 的枚举
    switch (motion) {
        case control::BasicMotion::kStand:
            trot_gait_->setGaitMotion(GaitMotion::STOP);
            break;
        case control::BasicMotion::kForward:
            trot_gait_->setGaitMotion(GaitMotion::FORWARD);
            break;
        case control::BasicMotion::kBackward:
            trot_gait_->setGaitMotion(GaitMotion::BACKWARD);
            break;
        case control::BasicMotion::kTurnLeft:
            trot_gait_->setGaitMotion(GaitMotion::LEFT);
            break;
        case control::BasicMotion::kTurnRight:
            trot_gait_->setGaitMotion(GaitMotion::RIGHT);
            break;
        case control::BasicMotion::kJump:
            trot_gait_->setGaitMotion(GaitMotion::JUMP);
            break;
        default:
            trot_gait_->setGaitMotion(GaitMotion::STOP);
            break;
    }
}

// 核心：状态同步 -> 步态计算
void SpotPlanner::update(const RobotState& state) {
    // 1. 控制调用频率
    // TrotGait 内部并没有频率控制，它依赖外部调用的间隔。
    // 如果仿真步长是 2ms (500Hz)，而步态逻辑希望 1000Hz 或 500Hz 运行，直接调用即可。
    // 如果需要降频 (比如步态只跑 100Hz)，这里加一个计时器判断。
    
    if (state.time - last_time_ < 0.002) { 
        // 防止过于频繁调用（如果仿真步长极小）
        return; 
    }
    last_time_ = state.time;

    // 2. 将 MuJoCo 的真实状态 (反馈) 填入 planner_robot_
    mapMujocoToPlanner(state);

    // 3. 在非ROS模式下，需要手动调用legMovers的mover()来更新straightPhase和swingPhase
    // 在ROS模式下，这些由定时器调用，但在非ROS模式下需要手动调用
    for (int i = 0; i < 4; ++i) {
        trot_gait_->legMovers[i]->mover();
    }

    // 4. 运行一步步态规划
    // 这会更新 planner_robot_.legs[i]->q_target
    trot_gait_->runStep(); 
}

void SpotPlanner::getJointTargets(std::vector<double>& qref) {
    // 将 planner_robot_ 计算出的目标角度填回 qref
    mapPlannerToRef(qref);
}

void SpotPlanner::setCurrentState(const RobotState& state) {
    // 兼容接口，直接调用 map
    mapMujocoToPlanner(state);
}

// --- 辅助映射函数 ---

// MuJoCo (qpos) -> Planner Robot (q, qd)
void SpotPlanner::mapMujocoToPlanner(const RobotState& state) {
    // MuJoCo qpos 结构: [0-6] 基座位置姿态, [7-18] 关节角度
    // MuJoCo qvel 结构: [0-5] 基座速度角速度, [6-17] 关节角速度
    
    // 1. 映射基座状态 (如果有 IMU 数据，填入 planner_robot_.body)
    // planner_robot_.body.pos = ...
    // planner_robot_.body.rpy = ... (通常 TrotGait 只需要 rpy)

    // 2. 映射关节状态
    // 假设 MuJoCo 顺序: FL, FR, RL, RR (根据你的 spot_robot_params.h)
    // 假设 Planner 顺序: FR, FL, RR, RL (TrotGait 的常见顺序)
    
    // 我们遍历 MuJoCo 的腿 (i=0:FL, 1:FR...)
    for (int i = 0; i < 4; ++i) {
        // 使用你的 Helper 找到对应的 Planner 腿索引
        // 假设 LegIndexHelper::toPlannerIndex(i) 返回对应的 FR/FL...
        // 这里为了演示，假设简单的一一对应，你需要根据实际情况修改！
        
        // 假设:
        // MuJoCo: 0:FL, 1:FR, 2:RL, 3:RR
        // Planner: 0:FR, 1:FL, 2:RR, 3:RL
        int planner_leg_idx = -1;
        if(i==0) planner_leg_idx = 1; // FL -> FL(1)
        if(i==1) planner_leg_idx = 0; // FR -> FR(0)
        if(i==2) planner_leg_idx = 3; // RL -> RL(3)
        if(i==3) planner_leg_idx = 2; // RR -> RR(2)

        int mujoco_q_offset = 7 + i * 3;
        int mujoco_v_offset = 6 + i * 3;

        // 填入 Robot 模型
        // 使用 setAngles 和 setJointVels 方法
        planner_robot_.legs[planner_leg_idx]->setAngles(
            state.qpos[mujoco_q_offset + 0],
            state.qpos[mujoco_q_offset + 1],
            state.qpos[mujoco_q_offset + 2]
        );
        
        planner_robot_.legs[planner_leg_idx]->setJointVels(Eigen::Vector3d(
            state.qvel[mujoco_v_offset + 0],
            state.qvel[mujoco_v_offset + 1],
            state.qvel[mujoco_v_offset + 2]
        ));
    }
}

// Planner Robot (q_target) -> qref (发送给 MuJoCo)
void SpotPlanner::mapPlannerToRef(std::vector<double>& qref) {
    qref.resize(12);
    
    // TrotGait 继承自 BaseGait -> BodyMover -> StateMonitor
    // StateMonitor 有 qTarg 成员，存储目标关节角度
    // qTarg 的顺序是 Robot 类的腿顺序: FR(0), FL(1), RR(2), RL(3)
    
    // 遍历 MuJoCo 的腿顺序 (FL, FR, RL, RR)
    for (int i = 0; i < 4; ++i) {
        int planner_leg_idx = LegIndexHelper::toRobotIndex(i); // 转换为Robot顺序
        
        // 从 qTarg 中读取目标角度（qTarg 的顺序是 Robot 腿顺序）
        int qTarg_offset = planner_leg_idx * 3;
        qref[i * 3 + 0] = trot_gait_->qTarg[qTarg_offset + 0];
        qref[i * 3 + 1] = trot_gait_->qTarg[qTarg_offset + 1];
        qref[i * 3 + 2] = trot_gait_->qTarg[qTarg_offset + 2];
    }
}