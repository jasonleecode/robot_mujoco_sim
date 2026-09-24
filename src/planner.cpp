#include "planner.h"

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>

#include "LegIndexHelper.hpp"

// 构造函数：初始化 planner 库的对象
SpotPlanner::SpotPlanner() : mode_(control::BasicMotion::kStand), last_time_(0.0) {
  reset();
}

SpotPlanner::~SpotPlanner() {
  // unique_ptr 会自动释放
}

void SpotPlanner::reset() {
  mode_ = control::BasicMotion::kStand;
  trot_gait_ = std::make_unique<TrotGait>(&planner_robot_, "");
  trot_gait_->setStanceDuration(kBaseStanceDuration);
  trot_gait_->setSwingHeight(0.035f);
  setSpeedScale(speed_scale_);
  is_fallen_ = false;
  tilt_stopped_ = false;
  initialized_ = false;
  last_time_ = 0.0;
  last_tilt_warn_time_ = -1.0;
}

void SpotPlanner::setControlFrequency(double dt) {
  if (!std::isfinite(dt) || dt < 0.001 || dt > 0.1)
    throw std::invalid_argument("control period must be between 1 and 100 ms");
  control_dt_ = dt;
}

void SpotPlanner::setSpeedScale(double scale) {
  if (!std::isfinite(scale)) return;
  speed_scale_ = std::clamp(scale, 0.1, 2.0);
  // Below half speed a tiny stride is lost to compliant contact and joint
  // tracking error. Keep a 4 cm foot sweep and slow the cadence instead.
  constexpr double min_stride_scale = 0.5;
  trot_gait_->setStrideScale(std::max(min_stride_scale, speed_scale_));
  trot_gait_->setStanceDuration(static_cast<int>(std::lround(
      kBaseStanceDuration * std::max(1.0, min_stride_scale / speed_scale_))));
  trot_gait_->setSwingHeight(0.035f * std::clamp(speed_scale_, 0.6, 1.0));
}

void SpotPlanner::setMode(control::BasicMotion motion) {
  mode_ = motion;
  tilt_stopped_ = false;
  trot_gait_->setHeadingReference(planner_robot_.getOrientation()[2]);

  // 注意：不在此处重置 is_fallen_，跌倒状态只能通过 reset() 清除。
  // 否则 main 循环检测到 mode 不匹配时会反复调用 setMode，导致警告日志刷屏。
  if (is_fallen_) return;

  if (trot_gait_) {
    trot_gait_->active = true;
  }
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
  if (state.qpos.size() < 19 || state.qvel.size() < 18 || state.imu_quat.size() < 4)
    throw std::invalid_argument("SpotPlanner requires 12 joints and a floating base");
  if (!initialized_) {
    auto requested = mode_;
    setCurrentState(state);
    setMode(requested);
    return;
  }
  if (state.time < last_time_ - 1e-9) {
    reset();
    setCurrentState(state);
    return;
  }
  if (state.time - last_time_ + 1e-9 < control_dt_) return;
  // LegMover counters are milliseconds. Advance by simulation time, including
  // at 500 Hz; floating point roundoff must not skip every other physics tick.
  int ticks = static_cast<int>(std::floor((state.time - last_time_ + 1e-9) * 1000.0));
  if (ticks <= 0) return;
  if (ticks > 100) last_time_ = state.time;
  else last_time_ += ticks * 0.001;
  ticks = std::min(ticks, 100);

  // 2. 状态映射 (保持不变)
  mapMujocoToPlanner(state);

  // --- [修改] 跌倒检测逻辑 (基于重力投影) ---
  // 目标：检查机器人本体 Z 轴在世界坐标系 Z 轴上的投影分量
  // 这是一个纯几何计算，比欧拉角更稳定

  // 获取当前四元数 (w, x, y, z)
  // 注意：state.imu_quat 顺序是 [w, x, y, z]
  double x = state.imu_quat[1];
  double y = state.imu_quat[2];

  // 计算旋转矩阵的 R33 元素 (即 Body-Z 在 World-Z 上的投影)
  // 公式：R33 = 1 - 2*(x^2 + y^2)
  double z_projection = 1.0 - 2.0 * (x * x + y * y);

  // 阈值设定：
  // 1.0  = 直立
  // 0.5  = 倾斜 60度
  // 0.0  = 侧躺 (90度)
  // -1.0 = 肚子朝上 (180度)
  const double FALL_THRESHOLD = 0.5;    // 60°: 判定摔倒，停止所有控制
  const double TILT_THRESHOLD = 0.906;  // 25°: 预警阈值，强制切换到站立模式

  // 中间倾斜预警：强制停止前进/后退/转向，切换为站立
  if (z_projection < TILT_THRESHOLD && !is_fallen_ && !tilt_stopped_) {
    if (mode_ == control::BasicMotion::kForward  ||
        mode_ == control::BasicMotion::kBackward ||
        mode_ == control::BasicMotion::kTurnLeft ||
        mode_ == control::BasicMotion::kTurnRight) {
      if (state.time - last_tilt_warn_time_ > 1.0) {
        printf("[WARNING] Tilt pre-alarm (z_proj=%.3f < %.3f), switching to STAND\n",
               z_projection, TILT_THRESHOLD);
        fflush(stdout);
        last_tilt_warn_time_ = state.time;
      }
      tilt_stopped_ = true;
      trot_gait_->setGaitMotion(GaitMotion::STOP);
    }
  }

  if (z_projection < FALL_THRESHOLD) {
    if (!is_fallen_) {
      std::cout << "[WARNING] Fall Detected! (Z-Projection: " << z_projection << ")" << std::endl;
      if (z_projection < 0)
        std::cout << "Status: Upside Down (肚子朝上)" << std::endl;
      else
        std::cout << "Status: Tilted/Side Lying (侧身/倾斜)" << std::endl;

      std::cout << "Gait Planner Stopped." << std::endl;

      is_fallen_ = true;

      if (trot_gait_) {
        trot_gait_->active = false;
      }
    }
  }

  if (is_fallen_) {
    return;
  }
  // ------------------------------------------

  // 3. 正常步态更新 (保持不变)
  for (int tick = 0; tick < ticks; ++tick) {
    for (int i = 0; i < 4; ++i) trot_gait_->legMovers[i]->mover();
    trot_gait_->runStep();
  }

  // 4. Continuous stance height correction (runs every control tick, 1kHz).
  // During the stance (STRAIGHT) phase the leg tracks a fixed targPos. While the
  // body tilts between phase transitions, we update that target's z component in
  // real-time so the closed-loop straightMover continuously compensates roll.
  {
    double roll_cont, pitch_cont, yaw_cont;
    toEulerAngle(state.imu_quat, roll_cont, pitch_cont, yaw_cont);
    double roll_rate_cont = (state.qvel.size() > 3) ? state.qvel[3] : 0.0;

    // sign convention: FR=0,RR=2 are right-side legs (sign_y=-1); FL=1,RL=3 left (sign_y=+1)
    const int sign_y[4] = {-1, 1, -1, 1};
    // Nominal target z for each leg (from planner macros via NOMINAL_HEIGHT=0.41)
    const double nominal_z = -0.41;

    const double k_cont_roll_p = 0.15;
    const double k_cont_roll_d = 0.08;
    for (int i = 0; i < 4; ++i) {
      auto& lm = trot_gait_->legMovers[i];
      if (lm->straightPhase > 0) {
        // roll<0=left down: FL(sign_y=+1) extends → sign_y*negative=negative z ✓
        double z_corr = sign_y[i] * (roll_cont * k_cont_roll_p + roll_rate_cont * k_cont_roll_d);
        z_corr = std::max(-0.05, std::min(0.05, z_corr));
        lm->setTargetZ(nominal_z + z_corr);
      }
    }
  }
}

void SpotPlanner::getJointTargets(std::vector<double>& qref) {
  // 将 planner_robot_ 计算出的目标角度填回 qref
  mapPlannerToRef(qref);
}

void SpotPlanner::setCurrentState(const RobotState& state,
                                  const std::vector<double>& commanded_targets) {
  if (!commanded_targets.empty() &&
      (commanded_targets.size() != 12 ||
       !std::all_of(commanded_targets.begin(), commanded_targets.end(),
                    [](double value) { return std::isfinite(value); })))
    throw std::invalid_argument("initial joint targets must contain 12 finite values");
  mapMujocoToPlanner(state);
  last_time_ = state.time;
  initialized_ = true;
  // Continue the last actuator command, or use measured joints when the
  // caller has no command history.
  for (int planner_idx = 0; planner_idx < 4; ++planner_idx) {
    Eigen::Vector3d actual = planner_robot_.legs[planner_idx]->getAngles();
    int offset = planner_idx * 3;
    trot_gait_->qTarg[offset + 0] = actual[0];
    trot_gait_->qTarg[offset + 1] = actual[1];
    trot_gait_->qTarg[offset + 2] = actual[2];
  }
  if (!commanded_targets.empty()) {
    for (int leg = 0; leg < 4; ++leg) {
      const int planner_leg = LegIndexHelper::toRobotIndex(leg);
      for (int joint = 0; joint < 3; ++joint)
        trot_gait_->qTarg[3*planner_leg+joint] = commanded_targets[3*leg+joint];
    }
  }
}

// --- 辅助映射函数 ---

// MuJoCo (qpos) -> Planner Robot (q, qd)
void SpotPlanner::mapMujocoToPlanner(const RobotState& state) {
  if (state.qpos.size() < 19 || state.qvel.size() < 18 || state.imu_quat.size() < 4)
    throw std::invalid_argument("SpotPlanner requires 12 joints and a floating base");
  for (int i = 0; i < 4; ++i) {
    int planner_leg_idx = -1;
    if (i == 0)
      planner_leg_idx = 1;  // FL -> FL(1)
    if (i == 1)
      planner_leg_idx = 0;  // FR -> FR(0)
    if (i == 2)
      planner_leg_idx = 3;  // RL -> RL(3)
    if (i == 3)
      planner_leg_idx = 2;  // RR -> RR(2)

    int mujoco_q_offset = 7 + i * 3;
    int mujoco_v_offset = 6 + i * 3;

    planner_robot_.legs[planner_leg_idx]->setAngles(
        state.qpos[mujoco_q_offset + 0],
        state.qpos[mujoco_q_offset + 1],
        state.qpos[mujoco_q_offset + 2]);

    planner_robot_.legs[planner_leg_idx]->setJointVels(
        Eigen::Vector3d(state.qvel[mujoco_v_offset + 0],
                        state.qvel[mujoco_v_offset + 1],
                        state.qvel[mujoco_v_offset + 2]));
  }

  double r, p, y;
  if (state.imu_quat.size() >= 4) {
    toEulerAngle(state.imu_quat, r, p, y);
  }

  double world_vx = state.qvel[0];
  double world_vy = state.qvel[1];

  // 旋转到机身坐标系 (简化版，仅考虑 Yaw)
  double body_vx = cos(y) * world_vx + sin(y) * world_vy;
  double body_vy = -sin(y) * world_vx + cos(y) * world_vy;

  planner_robot_.setLinearVelocity(body_vx, body_vy, 0);
  // 同步姿态角
  planner_robot_.setOrientation(static_cast<float>(r), static_cast<float>(p), static_cast<float>(y));

  // 同步角速度 (freejoint qvel[3..5] 是机身坐标系角速度)
  double wx = (state.qvel.size() > 3) ? state.qvel[3] : 0.0;
  double wy = (state.qvel.size() > 4) ? state.qvel[4] : 0.0;
  double wz = (state.qvel.size() > 5) ? state.qvel[5] : 0.0;
  planner_robot_.setAngularVelocity(wx, wy, wz);

  // 同步机身世界坐标 (用于横向位置修正)
  double bx = (state.qpos.size() > 0) ? state.qpos[0] : 0.0;
  double by = (state.qpos.size() > 1) ? state.qpos[1] : 0.0;
  double bz = (state.qpos.size() > 2) ? state.qpos[2] : 0.0;
  planner_robot_.setBodyPosition(bx, by, bz);
}

// Planner Robot (q_target) -> qref (发送给 MuJoCo)
void SpotPlanner::mapPlannerToRef(std::vector<double>& qref) {
  qref.resize(12);

  // TrotGait 继承自 BaseGait -> BodyMover -> StateMonitor
  // StateMonitor 有 qTarg 成员，存储目标关节角度
  // qTarg 的顺序是 Robot 类的腿顺序: FR(0), FL(1), RR(2), RL(3)

  // 遍历 MuJoCo 的腿顺序 (FL, FR, RL, RR)
  for (int i = 0; i < 4; ++i) {
    int planner_leg_idx = LegIndexHelper::toRobotIndex(i);  // 转换为Robot顺序

    int qTarg_offset = planner_leg_idx * 3;
    qref[i * 3 + 0] = trot_gait_->qTarg[qTarg_offset + 0];
    qref[i * 3 + 1] = trot_gait_->qTarg[qTarg_offset + 1];
    qref[i * 3 + 2] = trot_gait_->qTarg[qTarg_offset + 2];
  }
}

// 辅助函数：四元数转欧拉角 (Roll, Pitch, Yaw)
// quat: [w, x, y, z]
void SpotPlanner::toEulerAngle(const std::vector<double>& q, double& roll, double& pitch,
                               double& yaw) {
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
  double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
  double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}
