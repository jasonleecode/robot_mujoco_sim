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