/*
 * TrotGait.cpp
 *
 *  Created on: 16 Aug 2024
 *      Author: Felix
 */

#include "Low_level/TrotGait.h"
#include <algorithm>

TrotGait::TrotGait(Robot* robotModel, std::string nodeName)
    : BaseGait(robotModel, nodeName), robotModel(robotModel) {
}

void TrotGait::gaitCallback() {
  if (delay > 0) {
    --delay;
    return;
  } else {
    if (legMovers[FR]->straightPhase || legMovers[FL]->straightPhase ||
        legMovers[RR]->straightPhase || legMovers[RL]->straightPhase ||
        legMovers[FR]->swingPhase   || legMovers[FL]->swingPhase   ||
        legMovers[RR]->swingPhase   || legMovers[RL]->swingPhase)
      return;

    if (active) {
      // === 早期倾斜安全保护 ===
      // 在执行步态前检测倾斜程度，防止大角度时继续行走加剧倾倒
      {
        Eigen::Vector3d rpy = robotModel->getOrientation();
        double roll_abs  = std::abs(rpy[0]);
        double pitch_abs = std::abs(rpy[1]);
        // 超过 22°：强制切换到站立（避免过早停止干扰修正步态）
        if ((roll_abs > 0.384 || pitch_abs > 0.384) &&
            (gaitMotion == FORWARD || gaitMotion == BACKWARD ||
             gaitMotion == LEFT    || gaitMotion == RIGHT)) {
          printf("[SAFETY] Large tilt (roll=%.1fdeg pitch=%.1fdeg), forcing STOP\n",
                 rpy[0] * 57.3, rpy[1] * 57.3);
          fflush(stdout);
          gaitMotion = STOP;
        }
      }

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
static Eigen::Vector3d calculateBalanceAdjustment(Robot* robot, int legID, double heading) {
  Eigen::Vector3d rpy    = robot->getOrientation();
  Eigen::Vector3d vel    = robot->getLinearVelocity();
  Eigen::Vector3d angvel = robot->getAngularVelocity();
  Eigen::Vector3d bpos   = robot->getBodyPosition();

  double roll       = rpy[0];
  double pitch      = rpy[1];
  double yaw        = rpy[2];
  double roll_rate  = angvel[0];
  double pitch_rate = angvel[1];
  double yaw_rate   = angvel[2];

  // sign_y: FR(0),RR(2)=-1(右侧腿); FL(1),RL(3)=+1(左侧腿)
  // sign_x: FR(0),FL(1)=+1(前腿);   RR(2),RL(3)=-1(后腿)
  int sign_y = (legID == 0 || legID == 2) ? -1 : 1;
  int sign_x = (legID == 0 || legID == 1) ?  1 : -1;

  // === Y轴（横向）: Raibert capture point + yaw阻尼 ===
  // bpos[1] 是世界坐标系 Y，不能加到体坐标系落脚目标上，已移除该项
  double k_vy  = 0.08;
  double y_correction = vel[1] * k_vy;

  // Roll capture point: shift all feet toward falling side (roll<0=left down → +y)
  double k_roll_lat = 0.12;
  y_correction += -roll * k_roll_lat;

  // Yaw: empirically the original sign caused CW drift to amplify; keep zero
  // until the forward/heading coupling is properly characterized.

  y_correction = std::max(-0.10, std::min(0.10, y_correction));

  double k_roll_p  = 0.15;
  double k_pitch_p = 0.15;
  // pitch>0=nose-down (front lower in body frame), -sign_x corrects:
  //   positive pitch (nose-down) → negative z_adj on front → longer front leg → front higher ✓
  //   positive pitch (nose-down) → positive z_adj on rear → shorter rear leg → rear lower ✓
  double z_correction = sign_y * (roll * k_roll_p)
                      - sign_x * (pitch * k_pitch_p);
  z_correction = std::max(-0.05, std::min(0.05, z_correction));

  // === X轴: 前进速度修正 ===
  double x_correction = std::max(-0.03, std::min(0.03, vel[0] * 0.05));

  return Eigen::Vector3d(x_correction, y_correction, z_correction);
}

// 辅助工具：将宏定义的数组转换为 Eigen::Vector3d
// 解决不能直接对宏 {x,y,z} 做加法的问题
static Eigen::Vector3d macroToVec(const double arr[3]) {
  return Eigen::Vector3d(arr[0], arr[1], arr[2]);
}

void TrotGait::stand() {
  // 计算姿态补偿后的站立目标位置
  // 当机体已倾斜时（如 Stop 命令到来时 roll 较大），使用与前进步态相同的
  // 平衡修正量调整各脚落点，避免以固定坐标迈步导致倾倒加剧
  auto adjustedTarget = [&](const double arr[3], int legID) -> Eigen::Vector3d {
    Eigen::Vector3d tgt = macroToVec(arr);
    Eigen::Vector3d adj = calculateBalanceAdjustment(robotModel, legID, heading_target_);
    tgt[1] += adj[1];  // 横向(y)补偿：大roll/大横速时脚向倾斜侧偏移
    tgt[2] += adj[2];  // 腿长(z)补偿：差动腿长抵抗倾斜
    return tgt;
  };

  // Use STRAIGHT (ground-contact slide) to avoid airborne phase during stop transition.
  // 300ms gives slower, more controlled repositioning when body still has momentum.
  const int stand_duration = 300;

  switch (phase) {
    case 0:
      {
        const double fr_arr[] = FR_STAND;
        const double rl_arr[] = RL_STAND;
        if (!legMovers[FR]->straightPhase)
          legMovers[FR]->moveLegPosition(adjustedTarget(fr_arr, FR), stand_duration, STRAIGHT, stanceDepth, 0);
        if (!legMovers[RL]->straightPhase)
          legMovers[RL]->moveLegPosition(adjustedTarget(rl_arr, RL), stand_duration, STRAIGHT, stanceDepth, 0);
      }
      phase = 1;
      delay = DELAY_TIME;
      break;

    case 1:
      {
        const double fl_arr[] = FL_STAND;
        const double rr_arr[] = RR_STAND;
        if (!legMovers[FL]->straightPhase)
          legMovers[FL]->moveLegPosition(adjustedTarget(fl_arr, FL), stand_duration, STRAIGHT, stanceDepth, 0);
        if (!legMovers[RR]->straightPhase)
          legMovers[RR]->moveLegPosition(adjustedTarget(rr_arr, RR), stand_duration, STRAIGHT, stanceDepth, 0);
      }
      phase = 0;
      delay = DELAY_TIME;
      break;
  }
}

Eigen::Vector3d TrotGait::headingAdjustment(int leg) const {
  const auto rpy = robotModel->getOrientation();
  double error = std::atan2(std::sin(heading_target_-rpy[2]), std::cos(heading_target_-rpy[2]));
  double angle = std::clamp(0.20*error - 0.04*robotModel->getAngularVelocity()[2], -0.08, 0.08);
  double x = (leg == FR || leg == FL) ? 0.250 : -0.346;
  double y = (leg == FR || leg == RR) ? -0.170 : 0.170;
  return Eigen::Vector3d(-angle*y, angle*x, 0.0);
}

void TrotGait::applyTurnOffset(Eigen::Vector3d& target, bool swing) const {
  if (turn_scale_ == 0.0) return;
  // 角步幅与原地转向 turn() 一致：摆动腿 +、支撑腿 -，绕机体中心旋转
  const double a = turn_scale_ * 0.05 * (swing ? 1.0 : -1.0);
  const double nx = std::cos(a) * target[0] - std::sin(a) * target[1];
  const double ny = std::sin(a) * target[0] + std::cos(a) * target[1];
  target[0] = nx;
  target[1] = ny;
}

void TrotGait::forward() {
  // --- 1. 计算所有腿的平衡修正量 ---
  Eigen::Vector3d adj_FR = calculateBalanceAdjustment(robotModel, FR, heading_target_);
  Eigen::Vector3d adj_FL = calculateBalanceAdjustment(robotModel, FL, heading_target_);
  Eigen::Vector3d adj_RR = calculateBalanceAdjustment(robotModel, RR, heading_target_);
  Eigen::Vector3d adj_RL = calculateBalanceAdjustment(robotModel, RL, heading_target_);

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

  target_FR_Swing[0] = 0.250 + 0.04 * stride_scale_;
  target_FR_Stance[0] = 0.250 - 0.04 * stride_scale_;
  target_FL_Swing[0] = 0.250 + 0.04 * stride_scale_;
  target_FL_Stance[0] = 0.250 - 0.04 * stride_scale_;
  target_RR_Swing[0] = -0.346 + 0.04 * stride_scale_;
  target_RR_Stance[0] = -0.346 - 0.04 * stride_scale_;
  target_RL_Swing[0] = -0.346 + 0.04 * stride_scale_;
  target_RL_Stance[0] = -0.346 - 0.04 * stride_scale_;

  // --- 3. 应用修正量 ---
  // 策略：
  // Swing 腿：应用 X/Y (速度修正) + Z (姿态修正)
  // Stance 腿：仅应用 Z (姿态修正)，X/Y 保持标称轨迹以稳定推进

  // Swing Targets (加全量修正)
  target_FR_Swing += adj_FR + headingAdjustment(FR);
  target_FR_Stance -= headingAdjustment(FR);
  target_FL_Swing += adj_FL + headingAdjustment(FL);
  target_FL_Stance -= headingAdjustment(FL);
  target_RR_Swing += adj_RR + headingAdjustment(RR);
  target_RR_Stance -= headingAdjustment(RR);
  target_RL_Swing += adj_RL + headingAdjustment(RL);
  target_RL_Stance -= headingAdjustment(RL);

  // Stance Targets: apply Z (attitude) + Y (roll capture point, no velocity term)
  // Y correction on stance keeps support polygon under CoM when rolling
  Eigen::Vector3d rpy_now = robotModel->getOrientation();
  double roll_now  = rpy_now[0];
  double pitch_now = rpy_now[1];
  double stance_y_adj = -roll_now * 0.15;
  stance_y_adj = std::max(-0.06, std::min(0.06, stance_y_adj));

  target_FR_Stance[1] += stance_y_adj;
  target_FL_Stance[1] += stance_y_adj;
  target_RR_Stance[1] += stance_y_adj;
  target_RL_Stance[1] += stance_y_adj;

  target_FR_Stance[2] = -NOMINAL_HEIGHT;
  target_FL_Stance[2] = -NOMINAL_HEIGHT;
  target_RR_Stance[2] = -NOMINAL_HEIGHT;
  target_RL_Stance[2] = -NOMINAL_HEIGHT;

  // --- 4. 行进中转向：叠加弧线步态的角步幅 ---
  applyTurnOffset(target_FR_Swing, true);
  applyTurnOffset(target_FL_Swing, true);
  applyTurnOffset(target_RR_Swing, true);
  applyTurnOffset(target_RL_Swing, true);
  applyTurnOffset(target_FR_Stance, false);
  applyTurnOffset(target_FL_Stance, false);
  applyTurnOffset(target_RR_Stance, false);
  applyTurnOffset(target_RL_Stance, false);

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
  Eigen::Vector3d adj_FR = calculateBalanceAdjustment(robotModel, FR, heading_target_);
  Eigen::Vector3d adj_FL = calculateBalanceAdjustment(robotModel, FL, heading_target_);
  Eigen::Vector3d adj_RR = calculateBalanceAdjustment(robotModel, RR, heading_target_);
  Eigen::Vector3d adj_RL = calculateBalanceAdjustment(robotModel, RL, heading_target_);

  const double fr_back_arr[] = FR_BACK;   Eigen::Vector3d t_FR_Sw = macroToVec(fr_back_arr) + adj_FR;
  const double rl_back_arr[] = RL_BACK;   Eigen::Vector3d t_RL_Sw = macroToVec(rl_back_arr) + adj_RL;
  const double fl_back_arr[] = FL_BACK;   Eigen::Vector3d t_FL_Sw = macroToVec(fl_back_arr) + adj_FL;
  const double rr_back_arr[] = RR_BACK;   Eigen::Vector3d t_RR_Sw = macroToVec(rr_back_arr) + adj_RR;

  const double fl_front_arr[] = FL_FRONT; Eigen::Vector3d t_FL_St = macroToVec(fl_front_arr); t_FL_St[2] += adj_FL[2];
  const double rr_front_arr[] = RR_FRONT; Eigen::Vector3d t_RR_St = macroToVec(rr_front_arr); t_RR_St[2] += adj_RR[2];
  const double fr_front_arr[] = FR_FRONT; Eigen::Vector3d t_FR_St = macroToVec(fr_front_arr); t_FR_St[2] += adj_FR[2];
  const double rl_front_arr[] = RL_FRONT; Eigen::Vector3d t_RL_St = macroToVec(rl_front_arr); t_RL_St[2] += adj_RL[2];

  t_FR_Sw[0] = 0.250 - 0.04 * stride_scale_ + adj_FR[0];
  t_FR_St[0] = 0.250 + 0.04 * stride_scale_;
  t_FL_Sw[0] = 0.250 - 0.04 * stride_scale_ + adj_FL[0];
  t_FL_St[0] = 0.250 + 0.04 * stride_scale_;
  t_RR_Sw[0] = -0.346 - 0.04 * stride_scale_ + adj_RR[0];
  t_RR_St[0] = -0.346 + 0.04 * stride_scale_;
  t_RL_Sw[0] = -0.346 - 0.04 * stride_scale_ + adj_RL[0];
  t_RL_St[0] = -0.346 + 0.04 * stride_scale_;

  t_FR_Sw += headingAdjustment(FR);
  t_FR_St -= headingAdjustment(FR);
  t_FL_Sw += headingAdjustment(FL);
  t_FL_St -= headingAdjustment(FL);
  t_RR_Sw += headingAdjustment(RR);
  t_RR_St -= headingAdjustment(RR);
  t_RL_Sw += headingAdjustment(RL);
  t_RL_St -= headingAdjustment(RL);

  // 行进中转向：叠加弧线步态的角步幅
  applyTurnOffset(t_FR_Sw, true);
  applyTurnOffset(t_FL_Sw, true);
  applyTurnOffset(t_RR_Sw, true);
  applyTurnOffset(t_RL_Sw, true);
  applyTurnOffset(t_FR_St, false);
  applyTurnOffset(t_FL_St, false);
  applyTurnOffset(t_RR_St, false);
  applyTurnOffset(t_RL_St, false);
  switch (phase) {
    case 0:
      if (!legMovers[FR]->swingPhase)
        legMovers[FR]->moveLegPosition(t_FR_Sw, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RL]->swingPhase)
        legMovers[RL]->moveLegPosition(t_RL_Sw, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[FL]->straightPhase)
        legMovers[FL]->moveLegPosition(t_FL_St, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RR]->straightPhase)
        legMovers[RR]->moveLegPosition(t_RR_St, stance_duration, STRAIGHT, stanceDepth, 0);
      phase = 1;
      delay = DELAY_TIME;
      break;

    case 1:
      if (!legMovers[FL]->swingPhase)
        legMovers[FL]->moveLegPosition(t_FL_Sw, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[RR]->swingPhase)
        legMovers[RR]->moveLegPosition(t_RR_Sw, stance_duration, SWING, swingHeight, 0);
      if (!legMovers[FR]->straightPhase)
        legMovers[FR]->moveLegPosition(t_FR_St, stance_duration, STRAIGHT, stanceDepth, 0);
      if (!legMovers[RL]->straightPhase)
        legMovers[RL]->moveLegPosition(t_RL_St, stance_duration, STRAIGHT, stanceDepth, 0);
      phase = 0;
      delay = DELAY_TIME;
      break;
  }
}

void TrotGait::right() { turn(-1.0); }

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

void TrotGait::left() { turn(1.0); }

void TrotGait::turn(double direction) {
  const Eigen::Vector3d neutral[4] = {Eigen::Vector3d FR_STAND,
    Eigen::Vector3d FL_STAND, Eigen::Vector3d RR_STAND, Eigen::Vector3d RL_STAND};
  for (int i = 0; i < 4; ++i) {
    bool swing = (phase == 0) ? (i == FR || i == RL) : (i == FL || i == RR);
    double angle = direction * stride_scale_ * (swing ? 0.05 : -0.05);
    Eigen::Vector3d target = neutral[i];
    target[0] = std::cos(angle)*neutral[i][0] - std::sin(angle)*neutral[i][1];
    target[1] = std::sin(angle)*neutral[i][0] + std::cos(angle)*neutral[i][1];
    Eigen::Vector3d balance = calculateBalanceAdjustment(robotModel, i, robotModel->getOrientation()[2]);
    if (swing) target += balance;
    else target[2] += balance[2];
    legMovers[i]->moveLegPosition(target, stance_duration, swing ? SWING : STRAIGHT,
                                  swing ? swingHeight : stanceDepth, 0);
  }
  phase = 1 - phase;
  delay = DELAY_TIME;
}

TrotGait::~TrotGait() {
}