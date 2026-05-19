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

      // 步态相位切换日志
      static int last_logged_phase = -1;
      static GaitMotion last_logged_motion = GAIT_MOTION_NUM;
      static long long step_count = 0;
      if (phase != last_logged_phase || gaitMotion != last_logged_motion) {
        const char* motion_names[] = {"STOP","STANDUP","FORWARD","BACKWARD","LEFT","RIGHT","JUMP","DEFAULT","NONE"};
        const char* mname = (gaitMotion < GAIT_MOTION_NUM) ? motion_names[gaitMotion] : "NONE";
        // 腿状态：S=swing, T=stance(straight), I=idle
        auto leg_state = [&](int idx) -> char {
          if (legMovers[idx]->swingPhase) return 'S';
          if (legMovers[idx]->straightPhase) return 'T';
          return 'I';
        };
        printf("[STEP #%lld] motion=%-8s  phase: %d->%d  "
               "legs(FR,FL,RR,RL)=(%c,%c,%c,%c)  delay=%lld\n",
               step_count++, mname,
               last_logged_phase, phase,
               leg_state(FR), leg_state(FL), leg_state(RR), leg_state(RL),
               delay);
        fflush(stdout);
        last_logged_phase = phase;
        last_logged_motion = gaitMotion;
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
static Eigen::Vector3d calculateBalanceAdjustment(Robot* robot, int legID) {
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
  double k_vy  = 0.18;
  double y_correction = vel[1] * k_vy;

  // Roll capture point: shift all feet toward falling side (roll<0=left down → +y)
  double k_roll_lat = 0.15;
  y_correction += -roll * k_roll_lat;

  // Yaw damping — keep gains low to avoid yaw oscillation
  double k_yaw_p = 0.04;
  double k_yaw_d = 0.04;
  y_correction += -sign_x * (yaw * k_yaw_p + yaw_rate * k_yaw_d);

  y_correction = std::max(-0.10, std::min(0.10, y_correction));

  double k_roll_p  = 0.15;
  double k_pitch_p = 0.06;
  double k_roll_d  = 0.08;
  double k_pitch_d = 0.06;
  // roll<0=left side down: FL(sign_y=+1) extends → sign_y*negative=negative ✓
  // pitch<0=nose down:     FR(sign_x=+1) extends → +sign_x*negative=negative ✓
  double z_correction = sign_y * (roll * k_roll_p + roll_rate * k_roll_d)
                      + sign_x * (pitch * k_pitch_p + pitch_rate * k_pitch_d);
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
    Eigen::Vector3d adj = calculateBalanceAdjustment(robotModel, legID);
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

  // Stance Targets: apply Z (attitude) + Y (roll capture point, no velocity term)
  // Y correction on stance keeps support polygon under CoM when rolling
  Eigen::Vector3d rpy_now = robotModel->getOrientation();
  double roll_now = rpy_now[0];
  double stance_y_adj = -roll_now * 0.15;  // same roll→y as in calculateBalanceAdjustment
  stance_y_adj = std::max(-0.06, std::min(0.06, stance_y_adj));

  target_FR_Stance[1] += stance_y_adj;
  target_FL_Stance[1] += stance_y_adj;
  target_RR_Stance[1] += stance_y_adj;
  target_RL_Stance[1] += stance_y_adj;

  target_FR_Stance[2] += adj_FR[2];
  target_FL_Stance[2] += adj_FL[2];
  target_RR_Stance[2] += adj_RR[2];
  target_RL_Stance[2] += adj_RL[2];

  // --- 4. 执行状态机 ---
  // 打印本次落脚目标及平衡修正量
  {
    Eigen::Vector3d rpy = robotModel->getOrientation();
    Eigen::Vector3d vel = robotModel->getLinearVelocity();
    printf("[FORWARD ph=%d] rpy=(%.1f,%.1f,%.1f)deg  vel=(%.3f,%.3f)\n"
           "  adj: FR=(%.3f,%.3f,%.3f) FL=(%.3f,%.3f,%.3f)"
           " RR=(%.3f,%.3f,%.3f) RL=(%.3f,%.3f,%.3f)\n"
           "  swing_tgt: FR=(%.3f,%.3f,%.3f) FL=(%.3f,%.3f,%.3f)"
           " RR=(%.3f,%.3f,%.3f) RL=(%.3f,%.3f,%.3f)\n"
           "  stance_tgt:FR=(%.3f,%.3f,%.3f) FL=(%.3f,%.3f,%.3f)"
           " RR=(%.3f,%.3f,%.3f) RL=(%.3f,%.3f,%.3f)\n",
           phase, rpy[0]*57.3, rpy[1]*57.3, rpy[2]*57.3, vel[0], vel[1],
           adj_FR[0],adj_FR[1],adj_FR[2], adj_FL[0],adj_FL[1],adj_FL[2],
           adj_RR[0],adj_RR[1],adj_RR[2], adj_RL[0],adj_RL[1],adj_RL[2],
           target_FR_Swing[0],target_FR_Swing[1],target_FR_Swing[2],
           target_FL_Swing[0],target_FL_Swing[1],target_FL_Swing[2],
           target_RR_Swing[0],target_RR_Swing[1],target_RR_Swing[2],
           target_RL_Swing[0],target_RL_Swing[1],target_RL_Swing[2],
           target_FR_Stance[0],target_FR_Stance[1],target_FR_Stance[2],
           target_FL_Stance[0],target_FL_Stance[1],target_FL_Stance[2],
           target_RR_Stance[0],target_RR_Stance[1],target_RR_Stance[2],
           target_RL_Stance[0],target_RL_Stance[1],target_RL_Stance[2]);
    fflush(stdout);
  }

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
  Eigen::Vector3d adj_FR = calculateBalanceAdjustment(robotModel, FR);
  Eigen::Vector3d adj_FL = calculateBalanceAdjustment(robotModel, FL);
  Eigen::Vector3d adj_RR = calculateBalanceAdjustment(robotModel, RR);
  Eigen::Vector3d adj_RL = calculateBalanceAdjustment(robotModel, RL);

  const double fr_back_arr[] = FR_BACK;   Eigen::Vector3d t_FR_Sw = macroToVec(fr_back_arr) + adj_FR;
  const double rl_back_arr[] = RL_BACK;   Eigen::Vector3d t_RL_Sw = macroToVec(rl_back_arr) + adj_RL;
  const double fl_back_arr[] = FL_BACK;   Eigen::Vector3d t_FL_Sw = macroToVec(fl_back_arr) + adj_FL;
  const double rr_back_arr[] = RR_BACK;   Eigen::Vector3d t_RR_Sw = macroToVec(rr_back_arr) + adj_RR;

  const double fl_front_arr[] = FL_FRONT; Eigen::Vector3d t_FL_St = macroToVec(fl_front_arr); t_FL_St[2] += adj_FL[2];
  const double rr_front_arr[] = RR_FRONT; Eigen::Vector3d t_RR_St = macroToVec(rr_front_arr); t_RR_St[2] += adj_RR[2];
  const double fr_front_arr[] = FR_FRONT; Eigen::Vector3d t_FR_St = macroToVec(fr_front_arr); t_FR_St[2] += adj_FR[2];
  const double rl_front_arr[] = RL_FRONT; Eigen::Vector3d t_RL_St = macroToVec(rl_front_arr); t_RL_St[2] += adj_RL[2];

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

void TrotGait::right() {
  Eigen::Vector3d adj_FR = calculateBalanceAdjustment(robotModel, FR);
  Eigen::Vector3d adj_FL = calculateBalanceAdjustment(robotModel, FL);
  Eigen::Vector3d adj_RR = calculateBalanceAdjustment(robotModel, RR);
  Eigen::Vector3d adj_RL = calculateBalanceAdjustment(robotModel, RL);

  const double fr_sr[] = FR_SIDE_RIGHT; Eigen::Vector3d t_FR_Sw = macroToVec(fr_sr) + adj_FR;
  const double rl_sr[] = RL_SIDE_RIGHT; Eigen::Vector3d t_RL_Sw = macroToVec(rl_sr) + adj_RL;
  const double fl_sr[] = FL_SIDE_RIGHT; Eigen::Vector3d t_FL_Sw = macroToVec(fl_sr) + adj_FL;
  const double rr_sr[] = RR_SIDE_RIGHT; Eigen::Vector3d t_RR_Sw = macroToVec(rr_sr) + adj_RR;

  const double fl_sl[] = FL_SIDE_LEFT; Eigen::Vector3d t_FL_St = macroToVec(fl_sl); t_FL_St[2] += adj_FL[2];
  const double rr_sl[] = RR_SIDE_LEFT; Eigen::Vector3d t_RR_St = macroToVec(rr_sl); t_RR_St[2] += adj_RR[2];
  const double fr_sl[] = FR_SIDE_LEFT; Eigen::Vector3d t_FR_St = macroToVec(fr_sl); t_FR_St[2] += adj_FR[2];
  const double rl_sl[] = RL_SIDE_LEFT; Eigen::Vector3d t_RL_St = macroToVec(rl_sl); t_RL_St[2] += adj_RL[2];

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
  Eigen::Vector3d adj_FR = calculateBalanceAdjustment(robotModel, FR);
  Eigen::Vector3d adj_FL = calculateBalanceAdjustment(robotModel, FL);
  Eigen::Vector3d adj_RR = calculateBalanceAdjustment(robotModel, RR);
  Eigen::Vector3d adj_RL = calculateBalanceAdjustment(robotModel, RL);

  const double fr_sl[] = FR_SIDE_LEFT; Eigen::Vector3d t_FR_Sw = macroToVec(fr_sl) + adj_FR;
  const double rl_sl[] = RL_SIDE_LEFT; Eigen::Vector3d t_RL_Sw = macroToVec(rl_sl) + adj_RL;
  const double fl_sl[] = FL_SIDE_LEFT; Eigen::Vector3d t_FL_Sw = macroToVec(fl_sl) + adj_FL;
  const double rr_sl[] = RR_SIDE_LEFT; Eigen::Vector3d t_RR_Sw = macroToVec(rr_sl) + adj_RR;

  const double fl_sr[] = FL_SIDE_RIGHT; Eigen::Vector3d t_FL_St = macroToVec(fl_sr); t_FL_St[2] += adj_FL[2];
  const double rr_sr[] = RR_SIDE_RIGHT; Eigen::Vector3d t_RR_St = macroToVec(rr_sr); t_RR_St[2] += adj_RR[2];
  const double fr_sr[] = FR_SIDE_RIGHT; Eigen::Vector3d t_FR_St = macroToVec(fr_sr); t_FR_St[2] += adj_FR[2];
  const double rl_sr[] = RL_SIDE_RIGHT; Eigen::Vector3d t_RL_St = macroToVec(rl_sr); t_RL_St[2] += adj_RL[2];

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

TrotGait::~TrotGait() {
}