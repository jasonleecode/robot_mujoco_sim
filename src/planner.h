#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ControlActions.hpp"

// Standalone gait and kinematics.
#include "Low_level/TrotGait.h"
#include "Quadruped/Robot.h"

// Snapshot from the physics thread.
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
  // Seed from the actuator's previous command when handing over control so
  // tracking error is not sent back as an instantaneous target jump.
  void setCurrentState(const RobotState& state,
                       const std::vector<double>& commanded_targets = {});

  // 设置控制频率
  void setControlFrequency(double dt);

  // 设置目标行进速度缩放（常速调步幅，低速保留步幅并降低步频）。
  // 静止起步时实际速度从 1.0 平滑爬升到该目标值，不会立即生效。
  void setSpeedScale(double scale);

  // 设置地面摩擦倍率（与 UI 的 Ground friction 滑块一致）。
  // 决定步幅封顶值：超过封顶的速度由提高步频补足。
  void setGroundFriction(double friction);

  // 设置行进中转向分量：-1=右满舵, 0=直行, +1=左满舵。
  // 叠加在前进/后退上，实现边走边转向；与 mode 相互独立。
  void setTurnRate(double rate);

  // 当前支撑相时长 (ms)，用于诊断和测试阈值标定
  int currentStanceDuration() const;

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
  double control_dt_ = 0.001;  // 默认控制频率为 1000Hz
  static constexpr int kBaseStanceDuration = 350;  // 基准支撑相时长 (ms)

  control::BasicMotion mode_;
  double last_time_;
  double speed_scale_ = 1.0;             // 滑块设定的目标速度倍率
  double effective_speed_scale_ = 1.0;   // 实际生效的速度倍率（起步时向目标值爬升）
  double ground_friction_ = 1.0;         // 地面摩擦倍率（决定步幅封顶）
  double turn_rate_ = 0.0;               // 行进中转向分量 (-1..1)
  // 起步加速速率：有效速度倍率每秒的变化量上限
  static constexpr double kSpeedRampRate = 2.0;
  // 支撑相最短时长 (ms)：摆动腿完成"抬-跨-落"所需的最小时间
  static constexpr int kMinStanceDuration = 200;
  double last_tilt_warn_time_ = -1.0;
  bool initialized_ = false;
  bool tilt_stopped_ = false;
  // 跌倒标志位
  bool is_fallen_ = false;

  // 辅助：状态映射
  void mapMujocoToPlanner(const RobotState& state);
  void mapPlannerToRef(std::vector<double>& qref);

  // 把速度倍率实际写入步态参数（步幅/支撑相时长/抬脚高度）
  void applySpeedScale(double scale);

  // 当前摩擦下的步幅封顶值（实测边界插值，见 docs/SPEED_LIMIT_ANALYSIS.md）
  double strideCapForFriction() const;

  void toEulerAngle(const std::vector<double>& q, double& roll, double& pitch, double& yaw);
};
