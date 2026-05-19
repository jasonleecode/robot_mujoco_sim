#pragma once

/**
 * RobotConfig — 机器人通用参数描述
 *
 * 通过 detectRobotConfig() 从已加载的 mjModel 自动提取，
 * 供 RobotSim / PolicyController / main.cpp 使用。
 * 不依赖任何机器人专有头文件。
 */

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

struct RobotConfig {
  // ----- 基本信息 -----------------------------------------------------------
  std::string name;          // 机器人型号名，如 "spot" / "go1" / "go2"
  int num_actuators = 0;     // 关节驱动器数量（= MuJoCo m->nu）

  // ----- qpos / qvel 布局 --------------------------------------------------
  // 所有当前支持的四足机器人均为单 freejoint：
  //   qpos = [x,y,z, qw,qx,qy,qz, joint_0..joint_N-1]
  //   qvel = [vx,vy,vz, wx,wy,wz, dq_0..dq_N-1]
  int qpos_joint_offset = 7;
  int qvel_joint_offset = 6;

  // ----- 初始站立姿态 -------------------------------------------------------
  double stand_height = 0.40;           // "home" 关键帧时机身 Z（m）
  std::vector<double> stand_angles;     // 各关节角（num_actuators 个，rad）

  // ----- IMU 传感器 ---------------------------------------------------------
  // MuJoCo sensor 名称。若为空，表示模型中无显式传感器定义，
  // 将自动回退到从 freejoint qpos/qvel 读取。
  std::string imu_quat_sensor;   // 如 "imu_quat"；空 = 无
  std::string imu_gyro_sensor;   // 如 "imu_gyro"；空 = 无
  bool has_hardware_imu = false; // true = 找到了 MuJoCo sensor

  // ----- 关节限位 -----------------------------------------------------------
  // 与 stand_angles 同顺序（MuJoCo actuator 顺序）
  std::vector<double> joint_limits_low;
  std::vector<double> joint_limits_high;

  // ----- 能力标志 -----------------------------------------------------------
  // 是否支持基于规则的 SpotPlanner（仅 Spot 支持）
  bool supports_rule_gait = false;
};

/**
 * 从已加载的 mjModel 自动检测并填充 RobotConfig。
 * @param m        MuJoCo 模型指针
 * @param xml_path 加载模型的路径（用于辅助命名，可为空）
 */
RobotConfig detectRobotConfig(const mjModel* m, const std::string& xml_path = "");
