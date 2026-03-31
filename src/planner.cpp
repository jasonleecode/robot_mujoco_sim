#include "planner.h"

#include <cmath>
#include <iostream>

#include "LegIndexHelper.hpp"

// 构造函数：初始化 planner 库的对象
SpotPlanner::SpotPlanner() : last_time_(0.0), mode_(control::BasicMotion::kDefault) {
  // 1. 初始化算法模型
  // 注意：TrotGait 的构造函数需要 Robot 指针和 nodeName
  // 在非 ROS 模式下，nodeName 可以为空
  trot_gait_ = std::make_unique<TrotGait>(&planner_robot_, "");

  // 停止后台定时器：物理线程通过 runStep() 直接驱动步态，不需要定时器线程
  // 不停止会导致定时器线程与物理线程同时调用 gaitCallback()，产生数据竞争
  trot_gait_->stopGaitTimer();

  // 2. 初始化一些默认参数 (例如步态频率、高度等)
  trot_gait_->setStanceDuration(250);
}

SpotPlanner::~SpotPlanner() {
  // unique_ptr 会自动释放
}

void SpotPlanner::reset() {
  mode_ = control::BasicMotion::kDefault;
  trot_gait_->setGaitMotion(GaitMotion::DEFAULT);
  trot_gait_->active = false;
  // 重置跌倒状态
  is_fallen_ = false;
}

void SpotPlanner::setMode(control::BasicMotion motion) {
  mode_ = motion;

  is_fallen_ = false;

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
  // 1. 频率控制 (保持不变)
  if (state.time - last_time_ < control_dt_)
    return;
  last_time_ = state.time;

  // ===================================================================
  // 诊断日志：每 100ms 打印一次完整姿态+步态状态
  // ===================================================================
  static double last_log_time = -1.0;
  if (state.time - last_log_time >= 0.1) {
    last_log_time = state.time;

    // --- 身体位姿 ---
    double bx = (state.qpos.size() > 0) ? state.qpos[0] : 0.0;
    double by = (state.qpos.size() > 1) ? state.qpos[1] : 0.0;
    double bz = (state.qpos.size() > 2) ? state.qpos[2] : 0.0;

    // 四元数
    double qw = state.imu_quat[0], qx = state.imu_quat[1];
    double qy = state.imu_quat[2], qz = state.imu_quat[3];
    double zproj = 1.0 - 2.0 * (qx * qx + qy * qy);  // Body-Z 直立度

    double r_log = 0, p_log = 0, y_log = 0;
    if (state.imu_quat.size() >= 4) toEulerAngle(state.imu_quat, r_log, p_log, y_log);

    // --- 身体速度 ---
    double vx = (state.qvel.size() > 0) ? state.qvel[0] : 0.0;
    double vy = (state.qvel.size() > 1) ? state.qvel[1] : 0.0;
    double vz = (state.qvel.size() > 2) ? state.qvel[2] : 0.0;
    double wx = (state.qvel.size() > 3) ? state.qvel[3] : 0.0;  // 角速度
    double wy = (state.qvel.size() > 4) ? state.qvel[4] : 0.0;
    double wz = (state.qvel.size() > 5) ? state.qvel[5] : 0.0;

    // --- 实际关节角（MuJoCo qpos 偏移7开始，FL/FR/RL/RR 顺序）---
    // MuJoCo 顺序: 0=FL, 1=FR, 2=RL, 3=RR (每腿3关节: hip,thigh,calf)
    auto qget = [&](int leg, int joint) -> double {
      int idx = 7 + leg * 3 + joint;
      return (idx < (int)state.qpos.size()) ? state.qpos[idx] : 0.0;
    };

    // --- 关节目标 (planner qTarg 顺序: FR=0, FL=1, RR=2, RL=3) ---
    const auto& qt = trot_gait_->qTarg;

    // --- 步态状态 ---
    bool fr_sw = trot_gait_->legMovers[FR]->swingPhase;
    bool fl_sw = trot_gait_->legMovers[FL]->swingPhase;
    bool rr_sw = trot_gait_->legMovers[RR]->swingPhase;
    bool rl_sw = trot_gait_->legMovers[RL]->swingPhase;
    bool fr_st = trot_gait_->legMovers[FR]->straightPhase;
    bool fl_st = trot_gait_->legMovers[FL]->straightPhase;
    bool rr_st = trot_gait_->legMovers[RR]->straightPhase;
    bool rl_st = trot_gait_->legMovers[RL]->straightPhase;

    // --- 脚的实际位置（由 FK 计算）---
    // planner_robot_ 的关节已在 mapMujocoToPlanner 中设置
    Eigen::Vector3d pos_FR = planner_robot_.getPosition_FR();
    Eigen::Vector3d pos_FL = planner_robot_.getPosition_FL();
    Eigen::Vector3d pos_RR = planner_robot_.getPosition_RR();
    Eigen::Vector3d pos_RL = planner_robot_.getPosition_RL();

    printf("\n======== T=%.3f ========\n", state.time);
    printf("[BODY]  pos=(%.3f, %.3f, %.3f)  zproj=%.3f  fallen=%d\n",
           bx, by, bz, zproj, (int)is_fallen_);
    printf("[RPY ]  roll=%.2fdeg  pitch=%.2fdeg  yaw=%.2fdeg\n",
           r_log * 57.3, p_log * 57.3, y_log * 57.3);
    printf("[QUAT]  w=%.3f x=%.3f y=%.3f z=%.3f\n", qw, qx, qy, qz);
    printf("[VEL ]  lin=(%.3f,%.3f,%.3f)  ang=(%.3f,%.3f,%.3f)\n",
           vx, vy, vz, wx, wy, wz);
    printf("[GAIT]  active=%d  mode=%d  leg_sw(FR,FL,RR,RL)=(%d,%d,%d,%d)  st=(%d,%d,%d,%d)\n",
           (int)trot_gait_->active, (int)mode_,
           (int)fr_sw, (int)fl_sw, (int)rr_sw, (int)rl_sw,
           (int)fr_st, (int)fl_st, (int)rr_st, (int)rl_st);
    // 实际关节角 (MuJoCo qpos: leg0=FL, leg1=FR, leg2=RL, leg3=RR)
    printf("[QACT]  FL=(%.2f,%.2f,%.2f)  FR=(%.2f,%.2f,%.2f)"
           "  RL=(%.2f,%.2f,%.2f)  RR=(%.2f,%.2f,%.2f)\n",
           qget(0,0), qget(0,1), qget(0,2),
           qget(1,0), qget(1,1), qget(1,2),
           qget(2,0), qget(2,1), qget(2,2),
           qget(3,0), qget(3,1), qget(3,2));
    // 关节目标 (planner qTarg: FR=0, FL=1, RR=2, RL=3)
    printf("[QTGT]  FR=(%.2f,%.2f,%.2f)  FL=(%.2f,%.2f,%.2f)"
           "  RR=(%.2f,%.2f,%.2f)  RL=(%.2f,%.2f,%.2f)\n",
           qt[0], qt[1], qt[2],
           qt[3], qt[4], qt[5],
           qt[6], qt[7], qt[8],
           qt[9], qt[10], qt[11]);
    // 脚的 FK 位置（body frame）
    printf("[FOOT]  FR=(%.3f,%.3f,%.3f)  FL=(%.3f,%.3f,%.3f)"
           "  RR=(%.3f,%.3f,%.3f)  RL=(%.3f,%.3f,%.3f)\n",
           pos_FR[0], pos_FR[1], pos_FR[2],
           pos_FL[0], pos_FL[1], pos_FL[2],
           pos_RR[0], pos_RR[1], pos_RR[2],
           pos_RL[0], pos_RL[1], pos_RL[2]);
    fflush(stdout);
  }
  // ===================================================================

  // 2. 状态映射 (保持不变)
  mapMujocoToPlanner(state);

  // --- [修改] 跌倒检测逻辑 (基于重力投影) ---
  // 目标：检查机器人本体 Z 轴在世界坐标系 Z 轴上的投影分量
  // 这是一个纯几何计算，比欧拉角更稳定

  // 获取当前四元数 (w, x, y, z)
  // 注意：state.imu_quat 顺序是 [w, x, y, z]
  double w = state.imu_quat[0];
  double x = state.imu_quat[1];
  double y = state.imu_quat[2];
  double z = state.imu_quat[3];

  // 计算旋转矩阵的 R33 元素 (即 Body-Z 在 World-Z 上的投影)
  // 公式：R33 = 1 - 2*(x^2 + y^2)
  double z_projection = 1.0 - 2.0 * (x * x + y * y);

  // 阈值设定：
  // 1.0  = 直立
  // 0.5  = 倾斜 60度
  // 0.0  = 侧躺 (90度)
  // -1.0 = 肚子朝上 (180度)
  const double FALL_THRESHOLD = 0.5;

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
  for (int i = 0; i < 4; ++i) {
    trot_gait_->legMovers[i]->mover();
  }
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

    // 填入 Robot 模型
    // 使用 setAngles 和 setJointVels 方法
    planner_robot_.legs[planner_leg_idx]->setAngles(state.qpos[mujoco_q_offset + 0],
                                                    state.qpos[mujoco_q_offset + 1],
                                                    state.qpos[mujoco_q_offset + 2]);

    planner_robot_.legs[planner_leg_idx]->setJointVels(
        Eigen::Vector3d(state.qvel[mujoco_v_offset + 0], state.qvel[mujoco_v_offset + 1],
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

  // 同步角速度 (world frame qvel[3..5] = wx,wy,wz，近似等于 body frame 角速度)
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

    // 从 qTarg 中读取目标角度（qTarg 的顺序是 Robot 腿顺序）
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