#pragma once

#include <Eigen/Dense>
#include <memory>  // [新增]
#include <string>
#include <vector>

#include "ControlActions.hpp"

// [新增] 引入 planner 库的头文件
#include "Low_level/TrotGait.h"
#include "Quadruped/Robot.h"

// 定义 RobotState (保持不变)
struct RobotState {
  std::vector<double> qpos;
  std::vector<double> qvel;
  double time = 0.0;

  // 默认为单位四元数 (1, 0, 0, 0) 代表无旋转
  std::vector<double> imu_quat = {1.0, 0.0, 0.0, 0.0};

  // 陀螺仪角速度 (x, y, z)
  std::vector<double> imu_gyro = {0.0, 0.0, 0.0};
};

/**
 * @class SpotPlanner
 * @brief 适配器类：将 MuJoCo 的 RobotState 转换为 TrotGait 需要的数据，
 * 并驱动 TrotGait 算法运行。
 */
class SpotPlanner {
 public:
  SpotPlanner();
  ~SpotPlanner();

  void reset();

  // 设置运动模式 (Forward, Turn, Stand...)
  void setMode(control::BasicMotion motion);
  control::BasicMotion mode() const {
    return mode_;
  }

  // 核心更新函数：传入 MuJoCo 状态，运行一步规划
  void update(const RobotState& state);

  // 获取规划出的关节角度 (12维向量)
  void getJointTargets(std::vector<double>& qref);

  // (可选) 兼容旧接口，虽然现在 update 里已经包含了状态设置
  void setCurrentState(const RobotState& state);

  // 设置控制频率
  void setControlFrequency(double dt) {
    control_dt_ = dt;
  }

  // 检查是否处于跌倒保护状态
  bool isFallen() const {
    return is_fallen_;
  }

 private:
  // [关键] 内部持有一个 planner 库定义的 Robot 实例
  // 注意：这个 Robot 是纯算法模型，不是 MuJoCo 的 mjModel
  Robot planner_robot_;

  // [关键] 持有步态算法实例
  std::unique_ptr<TrotGait> trot_gait_;
  double control_dt_ = 0.002;  // 默认控制频率为 500Hz

  control::BasicMotion mode_;
  double last_time_;
  // 跌倒标志位
  bool is_fallen_ = false;

  // 辅助：状态映射
  void mapMujocoToPlanner(const RobotState& state);
  void mapPlannerToRef(std::vector<double>& qref);

  void toEulerAngle(const std::vector<double>& q, double& roll, double& pitch, double& yaw);
};