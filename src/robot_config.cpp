#include "robot_config.h"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iostream>

// =========================================================================
// 内部辅助：在 MuJoCo sensor 列表中搜索第一个匹配的名称
// =========================================================================
static std::string findSensor(const mjModel* m,
                               std::initializer_list<const char*> candidates) {
  for (const char* name : candidates) {
    if (mj_name2id(m, mjOBJ_SENSOR, name) >= 0)
      return std::string(name);
  }
  return "";
}

// =========================================================================
// 内部辅助：从路径推断机器人名称
// 例: "robot/boston_dynamics_spot/scene.xml" → "spot"
//     "robot/unitree_go1/scene.xml"           → "go1"
// =========================================================================
static std::string nameFromPath(const std::string& xml_path) {
  if (xml_path.empty()) return "";
  // 取父目录名（最后一级目录）
  std::filesystem::path p(xml_path);
  std::string dir = p.parent_path().filename().string();
  // 转小写方便匹配
  std::transform(dir.begin(), dir.end(), dir.begin(), ::tolower);

  if (dir.find("spot") != std::string::npos) return "spot";
  if (dir.find("go2")  != std::string::npos) return "go2";
  if (dir.find("go1")  != std::string::npos) return "go1";
  return dir;  // 未知型号，直接返回目录名
}

// =========================================================================
// 内部辅助：从 MuJoCo 模型中读取 keyframe "home" 的关节角和高度
// =========================================================================
static bool readHomeKeyframe(const mjModel* m, int nu,
                              double& out_height,
                              std::vector<double>& out_angles) {
  const int key_id = mj_name2id(m, mjOBJ_KEY, "home");
  if (key_id < 0) return false;

  const mjtNum* kqpos = m->key_qpos + key_id * m->nq;
  out_height = kqpos[2];                          // body Z（freejoint 第3分量）
  out_angles.resize(nu);
  for (int i = 0; i < nu; ++i)
    out_angles[i] = kqpos[7 + i];                // 跳过 freejoint 7 个分量
  return true;
}

// =========================================================================
// 内部辅助：从 mjModel 读取各关节限位
// =========================================================================
static void readJointLimits(const mjModel* m, int nu,
                             std::vector<double>& low,
                             std::vector<double>& high) {
  low.resize(nu);
  high.resize(nu);
  for (int i = 0; i < nu; ++i) {
    // 只处理 joint-transmission 驱动器
    if (m->actuator_trntype[i] != mjTRN_JOINT) {
      low[i]  = -3.14159;
      high[i] =  3.14159;
      continue;
    }
    const int jnt_id = m->actuator_trnid[i * 2];
    if (jnt_id < 0 || jnt_id >= m->njnt || !m->jnt_limited[jnt_id]) {
      low[i]  = -3.14159;
      high[i] =  3.14159;
    } else {
      low[i]  = m->jnt_range[jnt_id * 2];
      high[i] = m->jnt_range[jnt_id * 2 + 1];
    }
  }
}

// =========================================================================
// detectRobotConfig — 主函数
// =========================================================================
RobotConfig detectRobotConfig(const mjModel* m, const std::string& xml_path) {
  RobotConfig cfg;
  if (!m) return cfg;

  // --- 基本信息 -------------------------------------------------------------
  cfg.num_actuators = m->nu;
  cfg.name = nameFromPath(xml_path);

  // --- 站立关键帧 -----------------------------------------------------------
  if (!readHomeKeyframe(m, m->nu, cfg.stand_height, cfg.stand_angles)) {
    // 无 "home" 关键帧：使用零角度作为默认值
    cfg.stand_height = 0.40;
    cfg.stand_angles.assign(m->nu, 0.0);
    std::cerr << "[RobotConfig] Warning: keyframe 'home' not found in model. "
                 "Using zero stand angles.\n";
  }

  // --- IMU 传感器搜索 -------------------------------------------------------
  cfg.imu_quat_sensor = findSensor(m, {"imu_quat", "imu_orientation",
                                        "orientation_sensor"});
  cfg.imu_gyro_sensor = findSensor(m, {"imu_gyro", "gyro_sensor", "gyro"});
  cfg.has_hardware_imu = !cfg.imu_quat_sensor.empty();

  // --- 关节限位 -------------------------------------------------------------
  readJointLimits(m, m->nu, cfg.joint_limits_low, cfg.joint_limits_high);

  // --- 能力标志 -------------------------------------------------------------
  // SpotPlanner 仅适用于 Spot：通过 "imu_quat" 传感器存在或名称判断
  cfg.supports_rule_gait = (cfg.name == "spot") || cfg.has_hardware_imu;

  // --- 打印检测结果 ---------------------------------------------------------
  std::cout << "[RobotConfig] Detected robot: \"" << cfg.name << "\"\n"
            << "  actuators=" << cfg.num_actuators
            << "  stand_height=" << cfg.stand_height << "m\n"
            << "  imu_quat_sensor=\"" << cfg.imu_quat_sensor << "\""
            << "  imu_gyro_sensor=\"" << cfg.imu_gyro_sensor << "\"\n"
            << "  has_hardware_imu=" << cfg.has_hardware_imu
            << "  supports_rule_gait=" << cfg.supports_rule_gait << "\n";

  if (!cfg.stand_angles.empty()) {
    std::cout << "  stand_angles[0..2]=[" << cfg.stand_angles[0]
              << ", " << cfg.stand_angles[1]
              << ", " << cfg.stand_angles[2] << "] (first leg)\n";
  }

  return cfg;
}
