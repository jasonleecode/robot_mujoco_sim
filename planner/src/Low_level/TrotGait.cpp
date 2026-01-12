/*
 * TrotGait.cpp
 *
 *  Created on: 16 Aug 2024
 *      Author: Felix
 */

#include "Low_level/TrotGait.h"

TrotGait::TrotGait(Robot* robotModel, std::string nodeName)
    : BaseGait(robotModel, nodeName), robotModel(robotModel) {
}

void TrotGait::gaitCallback() {
  if (delay > 0) {
    --delay;
    return;
  } else {
    if (legMovers[FR]->straightPhase || legMovers[FL]->straightPhase ||
        legMovers[RR]->straightPhase || legMovers[RL]->straightPhase)
      return;

    if (active) {
      switch (gaitMotion) {
        case STOP:
          stand();
          if ((curTime - gaitStartTime) > 1000)
            gaitMotion = (GAIT_MOTION_NUM);  // Make it stop after 5 secss
          else
            writeFile = false;
          break;

        case STANDUP:
          stand();
          break;

        case FORWARD:
          forward();
          break;

        case BACKWARD:
          backward();
          break;

        case LEFT:
          left();
          break;

        case RIGHT:
          right();
          break;

        case JUMP:
          jump();
          break;

        default:
          break;
      }
    } else {
      // 如果不活跃,保持机器人默认姿态
      // std::cout << "TrotGait inactive" << std::endl;
    }
  }
}

// 辅助函数：根据当前状态计算XYZ三个方向的修正量
// robot: 机器人模型指针
// legID: 腿的索引 (0:FR, 1:FL, 2:RR, 3:RL)
static Eigen::Vector3d calculateBalanceAdjustment(Robot* robot, int legID) {
  // 1. 获取当前姿态 (Roll, Pitch) 和 速度 (Vx, Vy)
  Eigen::Vector3d rpy = robot->getOrientation();
  Eigen::Vector3d vel = robot->getLinearVelocity();

  double roll = rpy[0];
  double pitch = rpy[1];

  // 2. 姿态平衡控制 (Z轴修正)
  // 目的：身体倾斜时，通过伸缩腿长来保持身体水平
  // 符号定义：
  // FR(0): 右前 | FL(1): 左前 | RR(2): 右后 | RL(3): 左后
  int sign_y = (legID == 0 || legID == 2) ? -1 : 1;  // 右侧为负(-1)，左侧为正(+1)
  int sign_x = (legID == 0 || legID == 1) ? 1 : -1;  // 前侧为正(+1)，后侧为负(-1)

  // 参数增益 (需根据实际仿真效果微调)
  double k_roll = 0.2;   // Roll 增益
  double k_pitch = 0.2;  // Pitch 增益

  // 计算 Z 轴修正量
  // 例如：向右倾斜(Roll>0)，右腿需要伸长(Z更负)，所以 z_correction 应为负值
  // sign_y(-1) * roll(+) * k -> 负值 -> 正确
  double z_correction = sign_y * roll * k_roll - sign_x * pitch * k_pitch;

  // 3. Raibert Heuristic (X/Y轴速度修正)
  // 目的：防止摔倒，根据当前速度调整下一个落足点
  // Target = Nominal + Vel * T_stance / 2 + k * (Vel - Vel_cmd)
  // 这里使用简化版：Offset = Vel * Gain
  double k_vel = 0.15;  // 速度增益，速度越快步幅修正越大
  double x_correction = vel[0] * k_vel;
  double y_correction = vel[1] * k_vel;

  // 返回总修正向量
  return Eigen::Vector3d(x_correction, y_correction, z_correction);
}

// 辅助工具：将宏定义的数组转换为 Eigen::Vector3d
// 解决不能直接对宏 {x,y,z} 做加法的问题
static Eigen::Vector3d macroToVec(const double arr[3]) {
  return Eigen::Vector3d(arr[0], arr[1], arr[2]);
}

void TrotGait::stand() {
  switch (phase) {
    case 0:
      if (legMovers[FL]->swingPhase || legMovers[RR]->swingPhase)
        return;
      if (!legMovers[FR]->swingPhase)
        legMovers[FR]->moveLegPosition(FR_STAND, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RL]->swingPhase)
        legMovers[RL]->moveLegPosition(RL_STAND, stance_duration, SWING, swingHeight, 0);

      phase = 1;
      delay = DELAY_TIME;

      break;

    case 1:
      if (legMovers[FR]->swingPhase || legMovers[RL]->swingPhase)
        return;
      if (!legMovers[FL]->swingPhase)
        legMovers[FL]->moveLegPosition(FL_STAND, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RR]->swingPhase)
        legMovers[RR]->moveLegPosition(RR_STAND, stance_duration, SWING, swingHeight, 0);

      phase = 0;
      delay = DELAY_TIME;
      break;
  }
}

void TrotGait::forward() {
  // --- 1. 计算所有腿的平衡修正量 ---
  Eigen::Vector3d adj_FR = calculateBalanceAdjustment(robotModel, FR);
  Eigen::Vector3d adj_FL = calculateBalanceAdjustment(robotModel, FL);
  Eigen::Vector3d adj_RR = calculateBalanceAdjustment(robotModel, RR);
  Eigen::Vector3d adj_RL = calculateBalanceAdjustment(robotModel, RL);

  // --- 2. 准备基础目标位置 (从宏定义转换) ---
  // Swing 目标 (迈步去哪里)
  const double fr_front_arr[] = FR_FRONT;
  Eigen::Vector3d target_FR_Swing = macroToVec(fr_front_arr);
  const double fl_front_arr[] = FL_FRONT;
  Eigen::Vector3d target_FL_Swing = macroToVec(fl_front_arr);
  const double rr_front_arr[] = RR_FRONT;
  Eigen::Vector3d target_RR_Swing = macroToVec(rr_front_arr);
  const double rl_front_arr[] = RL_FRONT;
  Eigen::Vector3d target_RL_Swing = macroToVec(rl_front_arr);

  // Stance 目标 (向后划到哪里)
  const double fr_back_arr[] = FR_BACK;
  Eigen::Vector3d target_FR_Stance = macroToVec(fr_back_arr);
  const double fl_back_arr[] = FL_BACK;
  Eigen::Vector3d target_FL_Stance = macroToVec(fl_back_arr);
  const double rr_back_arr[] = RR_BACK;
  Eigen::Vector3d target_RR_Stance = macroToVec(rr_back_arr);
  const double rl_back_arr[] = RL_BACK;
  Eigen::Vector3d target_RL_Stance = macroToVec(rl_back_arr);

  // --- 3. 应用修正量 ---
  // 策略：
  // Swing 腿：应用 X/Y (速度修正) + Z (姿态修正)
  // Stance 腿：仅应用 Z (姿态修正)，X/Y 保持标称轨迹以稳定推进

  // Swing Targets (加全量修正)
  target_FR_Swing += adj_FR;
  target_FL_Swing += adj_FL;
  target_RR_Swing += adj_RR;
  target_RL_Swing += adj_RL;

  // Stance Targets (只加 Z 轴姿态修正，保持 X/Y 轨迹稳定)
  target_FR_Stance[2] += adj_FR[2];
  target_FL_Stance[2] += adj_FL[2];
  target_RR_Stance[2] += adj_RR[2];
  target_RL_Stance[2] += adj_RL[2];

  // --- 4. 执行状态机 ---
  switch (phase) {
    case 0:
      // Group 1: FR & RL (Swing 摆动)
      if (!legMovers[FR]->swingPhase)
        legMovers[FR]->moveLegPosition(target_FR_Swing, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RL]->swingPhase)
        legMovers[RL]->moveLegPosition(target_RL_Swing, stance_duration, SWING, swingHeight, 0);

      // Group 2: FL & RR (Stance 支撑)
      if (!legMovers[FL]->straightPhase)
        legMovers[FL]->moveLegPosition(target_FL_Stance, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RR]->straightPhase)
        legMovers[RR]->moveLegPosition(target_RR_Stance, stance_duration, STRAIGHT, stanceDepth, 0);

      phase = 1;
      delay = DELAY_TIME;
      break;

    case 1:
      // Group 2: FL & RR (Swing 摆动)
      if (!legMovers[FL]->swingPhase)
        legMovers[FL]->moveLegPosition(target_FL_Swing, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RR]->swingPhase)
        legMovers[RR]->moveLegPosition(target_RR_Swing, stance_duration, SWING, swingHeight, 0);

      // Group 1: FR & RL (Stance 支撑)
      if (!legMovers[FR]->straightPhase)
        legMovers[FR]->moveLegPosition(target_FR_Stance, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RL]->straightPhase)
        legMovers[RL]->moveLegPosition(target_RL_Stance, stance_duration, STRAIGHT, stanceDepth, 0);

      phase = 0;
      delay = DELAY_TIME;
      break;
  }
}

void TrotGait::backward() {
  switch (phase) {
    case 0:
      if (!legMovers[FR]->swingPhase)
        legMovers[FR]->moveLegPosition(FR_BACK, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RL]->swingPhase)
        legMovers[RL]->moveLegPosition(RL_BACK, stance_duration, SWING, swingHeight, 0);

      if (!legMovers[FL]->straightPhase)
        legMovers[FL]->moveLegPosition(FL_FRONT, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RR]->straightPhase)
        legMovers[RR]->moveLegPosition(RR_FRONT, stance_duration, STRAIGHT, stanceDepth, 0);

      // standing = false;

      phase = 1;
      delay = DELAY_TIME;

      break;

    case 1:
      if (!legMovers[FL]->swingPhase)
        legMovers[FL]->moveLegPosition(FL_BACK, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RR]->swingPhase)
        legMovers[RR]->moveLegPosition(RR_BACK, stance_duration, SWING, swingHeight, 0);

      if (!legMovers[FR]->straightPhase)
        legMovers[FR]->moveLegPosition(FR_FRONT, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RL]->straightPhase)
        legMovers[RL]->moveLegPosition(RL_FRONT, stance_duration, STRAIGHT, stanceDepth, 0);

      // standing = false;

      phase = 0;
      delay = DELAY_TIME;
      break;
  }
}

void TrotGait::right() {
  switch (phase) {
    case 0:
      if (!legMovers[FR]->swingPhase)
        legMovers[FR]->moveLegPosition(FR_SIDE_RIGHT, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RL]->swingPhase)
        legMovers[RL]->moveLegPosition(RL_SIDE_RIGHT, stance_duration, SWING, swingHeight, 0);

      if (!legMovers[FL]->straightPhase)
        legMovers[FL]->moveLegPosition(FL_SIDE_LEFT, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RR]->straightPhase)
        legMovers[RR]->moveLegPosition(RR_SIDE_LEFT, stance_duration, STRAIGHT, stanceDepth, 0);

      // standing = false;

      phase = 1;
      delay = DELAY_TIME;

      break;

    case 1:
      if (!legMovers[FL]->swingPhase)
        legMovers[FL]->moveLegPosition(FL_SIDE_RIGHT, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RR]->swingPhase)
        legMovers[RR]->moveLegPosition(RR_SIDE_RIGHT, stance_duration, SWING, swingHeight, 0);

      if (!legMovers[FR]->straightPhase)
        legMovers[FR]->moveLegPosition(FR_SIDE_LEFT, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RL]->straightPhase)
        legMovers[RL]->moveLegPosition(RL_SIDE_LEFT, stance_duration, STRAIGHT, stanceDepth, 0);

      // standing = false;

      phase = 0;
      delay = DELAY_TIME;
      break;
  }
}

void TrotGait::jump() {
  switch (phase) {
    case 0:
      if (!legMovers[FR]->straightPhase)
        legMovers[FR]->moveLegPosition(FR_BENT, 1000, STRAIGHT, stanceDepth, 0);
      if (!legMovers[FL]->straightPhase)
        legMovers[FL]->moveLegPosition(FL_BENT, 1000, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RR]->straightPhase)
        legMovers[RR]->moveLegPosition(RR_BENT, 1000, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RL]->straightPhase)
        legMovers[RL]->moveLegPosition(RL_BENT, 1000, STRAIGHT, stanceDepth, 0);
      // standing = false;
      // std::cout<<"I Bent"<<std::endl;
      phase = 1;

      break;

    case 1:
      if (!legMovers[FR]->straightPhase)
        legMovers[FR]->moveLegPosition(FR_JUMP, stance_duration, STRAIGHT, swingHeight, 0);
      if (!legMovers[FL]->straightPhase)
        legMovers[FL]->moveLegPosition(FL_JUMP, stance_duration, STRAIGHT, swingHeight, 0);
      if (!legMovers[RR]->straightPhase)
        legMovers[RR]->moveLegPosition(RR_JUMP, stance_duration, STRAIGHT, swingHeight, 0);
      if (!legMovers[RL]->straightPhase)
        legMovers[RL]->moveLegPosition(RL_JUMP, stance_duration, STRAIGHT, swingHeight, 0);
      // std::cout<<"I jumped"<<std::endl;
      phase = 2;
      break;

    case 2:
      if (!legMovers[FR]->straightPhase)
        legMovers[FR]->moveLegPosition(FR_MIDDLE, 200, STRAIGHT, swingHeight, 0);
      if (!legMovers[FL]->straightPhase)
        legMovers[FL]->moveLegPosition(FL_MIDDLE, 200, STRAIGHT, swingHeight, 0);
      if (!legMovers[RR]->straightPhase)
        legMovers[RR]->moveLegPosition(RR_MIDDLE, 200, STRAIGHT, swingHeight, 0);
      if (!legMovers[RL]->straightPhase)
        legMovers[RL]->moveLegPosition(RL_MIDDLE, 200, STRAIGHT, swingHeight, 0);
      // std::cout<<"I jumped"<<std::endl;
      phase = 3;
      break;

    case 3:
      if (!legMovers[FR]->straightPhase)
        legMovers[FR]->moveLegPosition(FR_STAND, 1000, STRAIGHT, swingHeight, 0);
      if (!legMovers[FL]->straightPhase)
        legMovers[FL]->moveLegPosition(FL_STAND, 1000, STRAIGHT, swingHeight, 0);
      if (!legMovers[RR]->straightPhase)
        legMovers[RR]->moveLegPosition(RR_STAND, 1000, STRAIGHT, swingHeight, 0);
      if (!legMovers[RL]->straightPhase)
        legMovers[RL]->moveLegPosition(RL_STAND, 1000, STRAIGHT, swingHeight, 0);

      // std::cout<<"I stood"<<std::endl;
      phase = 0;
      gaitMotion = GAIT_MOTION_NUM;
      break;
  }
}

void TrotGait::left() {
  switch (phase) {
    case 0:
      if (!legMovers[FR]->swingPhase)
        legMovers[FR]->moveLegPosition(FR_SIDE_LEFT, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RL]->swingPhase)
        legMovers[RL]->moveLegPosition(RL_SIDE_LEFT, stance_duration, SWING, swingHeight, 0);

      if (!legMovers[FL]->straightPhase)
        legMovers[FL]->moveLegPosition(FL_SIDE_RIGHT, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RR]->straightPhase)
        legMovers[RR]->moveLegPosition(RR_SIDE_RIGHT, stance_duration, STRAIGHT, stanceDepth, 0);

      // standing = false;

      phase = 1;
      delay = DELAY_TIME;
      break;

    case 1:
      if (!legMovers[FL]->swingPhase)
        legMovers[FL]->moveLegPosition(FL_SIDE_LEFT, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RR]->swingPhase)
        legMovers[RR]->moveLegPosition(RR_SIDE_LEFT, stance_duration, SWING, swingHeight, 0);

      if (!legMovers[FR]->straightPhase)
        legMovers[FR]->moveLegPosition(FR_SIDE_RIGHT, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RL]->straightPhase)
        legMovers[RL]->moveLegPosition(RL_SIDE_RIGHT, stance_duration, STRAIGHT, stanceDepth, 0);

      // standing = false;

      phase = 0;
      delay = DELAY_TIME;
      break;
  }
}

TrotGait::~TrotGait() {
}