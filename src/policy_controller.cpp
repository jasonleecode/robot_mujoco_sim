#include "policy_controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>

// =========================================================================
// Spot 默认标称关节角（MuJoCo 顺序: FL, FR, RL, RR；每腿: hip_x, hip_y, knee）
// 切换到其他机器人时，setRobotConfig() 会用 RobotConfig::stand_angles 覆盖。
// =========================================================================
static const double kSpotNominalJoints[12] = {
    0.0, 1.04, -1.80,   // FL
    0.0, 1.04, -1.80,   // FR
    0.0, 1.04, -1.80,   // RL
    0.0, 1.04, -1.80,   // RR
};

// =========================================================================
// StubBackend — 模型未加载时的占位实现
// forward() 始终输出全零 → action = 0 → q_target = q_nominal（标称站立）
// =========================================================================
class StubBackend : public PolicyBackend {
 public:
  bool load(const std::string& path) override {
    std::cout << "[PolicyBackend] Stub mode: no real model loaded.\n"
              << "  Path was: \"" << path << "\"\n"
              << "  Robot will hold nominal stance until a real backend is set.\n"
              << "  To use a real model, implement PolicyBackend and call\n"
              << "  policy_controller.setBackend(std::make_unique<YourBackend>()).\n";
    return false;
  }

  bool forward(const float* /*input*/, int /*input_dim*/,
               float* output, int output_dim) override {
    std::fill(output, output + output_dim, 0.0f);
    return true;
  }

  bool isLoaded() const override { return false; }
};

// =========================================================================
// PolicyController 实现
// =========================================================================

PolicyController::PolicyController(const Config& cfg)
    : cfg_(cfg),
      mode_(control::BasicMotion::kDefault),
      backend_(std::make_unique<StubBackend>()),
      last_action_(Eigen::VectorXf::Zero(cfg.action_dim)),
      last_obs_(Eigen::VectorXf::Zero(cfg.obs_dim)),
      joint_targets_(12),
      nominal_joints_(kSpotNominalJoints, kSpotNominalJoints + 12),
      joint_limits_low_( {-0.785398, -0.898845, -2.792527,
                          -0.785398, -0.898845, -2.792527,
                          -0.785398, -0.898845, -2.792527,
                          -0.785398, -0.898845, -2.792527}),
      joint_limits_high_({ 0.785398,  2.295116, -0.254402,
                           0.785398,  2.295116, -0.254402,
                           0.785398,  2.295116, -0.254402,
                           0.785398,  2.295116, -0.254402}) {
  // 初始化关节目标为标称站立
  joint_targets_ = nominal_joints_;

  // 若配置中指定了路径，尝试立即加载
  if (!cfg_.model_path.empty())
    loadModel(cfg_.model_path);
}

PolicyController::~PolicyController() = default;

bool PolicyController::loadModel(const std::string& path) {
  cfg_.model_path = path;
  const bool ok = backend_->load(path);
  if (ok)
    std::cout << "[PolicyController] Model loaded: " << path << "\n";
  else
    std::cout << "[PolicyController] Running in stub mode (nominal stance output)\n";
  return ok;
}

void PolicyController::setBackend(std::unique_ptr<PolicyBackend> backend) {
  backend_ = std::move(backend);
  if (!cfg_.model_path.empty() && !backend_->isLoaded())
    backend_->load(cfg_.model_path);
}

void PolicyController::setRobotConfig(const RobotConfig& robot_cfg) {
  const int n = std::min(static_cast<int>(robot_cfg.stand_angles.size()), 12);

  // 更新标称关节角
  nominal_joints_.assign(12, 0.0);
  for (int i = 0; i < n; ++i)
    nominal_joints_[i] = robot_cfg.stand_angles[i];

  // 更新关节限位
  if (robot_cfg.joint_limits_low.size() >= 12 &&
      robot_cfg.joint_limits_high.size() >= 12) {
    joint_limits_low_  = std::vector<double>(
        robot_cfg.joint_limits_low.begin(),
        robot_cfg.joint_limits_low.begin() + 12);
    joint_limits_high_ = std::vector<double>(
        robot_cfg.joint_limits_high.begin(),
        robot_cfg.joint_limits_high.begin() + 12);
  }

  // 同步关节目标到新的标称角（仅当未激活时）
  if (!is_fallen_ && last_time_ < 0.0)
    joint_targets_ = nominal_joints_;

  std::cout << "[PolicyController] Robot config applied: " << robot_cfg.name
            << "  nominal[0..2]=[" << nominal_joints_[0]
            << ", " << nominal_joints_[1]
            << ", " << nominal_joints_[2] << "]\n";
}

bool PolicyController::isModelLoaded() const {
  return backend_->isLoaded();
}

// =========================================================================
// 速度指令 / 模式设置
// =========================================================================

void PolicyController::setCommand(double vx, double vy, double wyaw) {
  cmd_vx_   = vx;
  cmd_vy_   = vy;
  cmd_wyaw_ = wyaw;
}

void PolicyController::setMode(control::BasicMotion motion) {
  mode_ = motion;
  switch (motion) {
    case control::BasicMotion::kForward:
      setCommand(kForwardVx, 0.0, 0.0);
      break;
    case control::BasicMotion::kBackward:
      setCommand(kBackwardVx, 0.0, 0.0);
      break;
    case control::BasicMotion::kTurnLeft:
      setCommand(0.0, 0.0,  kTurnWyaw);
      break;
    case control::BasicMotion::kTurnRight:
      setCommand(0.0, 0.0, -kTurnWyaw);
      break;
    case control::BasicMotion::kStand:
    default:
      setCommand(0.0, 0.0, 0.0);
      break;
  }
}

// =========================================================================
// 重力投影辅助函数
//
// 将世界坐标系重力方向 [0, 0, -1] 旋转到 body frame。
// 公式推导（body-from-world 旋转 q = [w, x, y, z]）：
//   g_body = q^{-1} ⊗ [0,0,-1] ⊗ q
// 化简后直接结果：
//   gx = 2(wy - xz)
//   gy = -2(wx + yz)
//   gz = -(1 - 2x² - 2y²)
// 验证：q=[1,0,0,0] 时 g_body=[0,0,-1] ✓（直立时重力朝 -z）
// =========================================================================
Eigen::Vector3f PolicyController::projectedGravity(const std::vector<double>& q) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return Eigen::Vector3f(
      static_cast<float>( 2.0 * (w*y - x*z)),
      static_cast<float>(-2.0 * (w*x + y*z)),
      static_cast<float>(-(1.0 - 2.0*(x*x + y*y)))
  );
}

// =========================================================================
// 观测向量构建（48 维）
// =========================================================================
Eigen::VectorXf PolicyController::buildObservation(const RobotState& state) const {
  Eigen::VectorXf obs(cfg_.obs_dim);
  int idx = 0;

  // ---- [0-2] body 线速度（body frame）------------------------------------
  // MuJoCo qvel[0-2] = 世界坐标系线速度，通过 yaw 旋转到机身坐标系
  const double vx_w = (state.qvel.size() > 0) ? state.qvel[0] : 0.0;
  const double vy_w = (state.qvel.size() > 1) ? state.qvel[1] : 0.0;
  const double vz_w = (state.qvel.size() > 2) ? state.qvel[2] : 0.0;

  const double qw = state.imu_quat[0], qx = state.imu_quat[1];
  const double qy = state.imu_quat[2], qz_q = state.imu_quat[3];
  const double yaw = std::atan2(2.0*(qw*qz_q + qx*qy),
                                1.0 - 2.0*(qy*qy + qz_q*qz_q));
  const double cy = std::cos(yaw), sy = std::sin(yaw);

  obs[idx++] = static_cast<float>( cy*vx_w + sy*vy_w);   // vx_body
  obs[idx++] = static_cast<float>(-sy*vx_w + cy*vy_w);   // vy_body
  obs[idx++] = static_cast<float>(vz_w);                  // vz_body

  // ---- [3-5] body 角速度（body frame ≈ MuJoCo qvel[3-5]）----------------
  obs[idx++] = (state.qvel.size() > 3) ? static_cast<float>(state.qvel[3]) : 0.0f;
  obs[idx++] = (state.qvel.size() > 4) ? static_cast<float>(state.qvel[4]) : 0.0f;
  obs[idx++] = (state.qvel.size() > 5) ? static_cast<float>(state.qvel[5]) : 0.0f;

  // ---- [6-8] 投影重力（body frame）---------------------------------------
  const Eigen::Vector3f pg = projectedGravity(state.imu_quat);
  obs[idx++] = pg[0];
  obs[idx++] = pg[1];
  obs[idx++] = pg[2];

  // ---- [9-11] 速度指令 (vx, vy, wyaw)------------------------------------
  obs[idx++] = static_cast<float>(cmd_vx_);
  obs[idx++] = static_cast<float>(cmd_vy_);
  obs[idx++] = static_cast<float>(cmd_wyaw_);

  // ---- [12-23] 关节角 - 标称角（MuJoCo 顺序: FL,FR,RL,RR）--------------
  for (int i = 0; i < 12; ++i) {
    const int qi = 7 + i;  // MuJoCo qpos 偏移 7
    const double q = (state.qpos.size() > static_cast<size_t>(qi))
                         ? state.qpos[qi] : 0.0;
    obs[idx++] = static_cast<float>(q - nominal_joints_[i]);
  }

  // ---- [24-35] 关节速度（MuJoCo 顺序）-----------------------------------
  for (int i = 0; i < 12; ++i) {
    const int vi = 6 + i;  // MuJoCo qvel 偏移 6
    obs[idx++] = (state.qvel.size() > static_cast<size_t>(vi))
                     ? static_cast<float>(state.qvel[vi]) : 0.0f;
  }

  // ---- [36-47] 上一时刻 action -------------------------------------------
  for (int i = 0; i < cfg_.action_dim; ++i)
    obs[idx++] = (i < last_action_.size()) ? last_action_[i] : 0.0f;

  return obs;
}

// =========================================================================
// 推理
// =========================================================================
Eigen::VectorXf PolicyController::runInference(const Eigen::VectorXf& obs) {
  std::vector<float> raw_out(cfg_.action_dim, 0.0f);
  backend_->forward(obs.data(), static_cast<int>(obs.size()),
                    raw_out.data(), cfg_.action_dim);

  Eigen::VectorXf action(cfg_.action_dim);
  for (int i = 0; i < cfg_.action_dim; ++i)
    action[i] = raw_out[i];
  return action;
}

// =========================================================================
// 主更新循环
// =========================================================================
void PolicyController::update(const RobotState& state) {
  // --- 频率控制（policy 以 control_dt 运行，仿真以 1kHz 运行）-------------
  if (last_time_ >= 0.0 && state.time - last_time_ < cfg_.control_dt)
    return;
  last_time_ = state.time;

  // --- 跌倒检测（与 SpotPlanner 逻辑一致）--------------------------------
  if (state.imu_quat.size() >= 4) {
    const double x = state.imu_quat[1], y = state.imu_quat[2];
    const double z_proj = 1.0 - 2.0*(x*x + y*y);
    if (z_proj < 0.5 && !is_fallen_) {
      printf("[PolicyController] Fall detected! z_proj=%.3f\n", z_proj);
      fflush(stdout);
      is_fallen_ = true;
    }
  }
  if (is_fallen_) return;

  // --- 构建观测 + 推理 ----------------------------------------------------
  last_obs_ = buildObservation(state);
  const Eigen::VectorXf action = runInference(last_obs_);
  last_action_ = action;

  // --- 动作 → 关节目标 ---------------------------------------------------
  // q_target[i] = q_nominal[i] + action_scale * action[i]
  // 关节限位 (from spot_robot_params.h):
  // --- 动作 → 关节目标（使用从 RobotConfig 获取的标称角和限位）----------
  for (int i = 0; i < 12; ++i) {
    const double target = nominal_joints_[i]
                        + cfg_.action_scale * static_cast<double>(action[i]);
    joint_targets_[i] = std::max(joint_limits_low_[i],
                                 std::min(joint_limits_high_[i], target));
  }

  // --- 诊断日志（每秒一次）-----------------------------------------------
  static double last_diag_time = -1.0;
  if (state.time - last_diag_time >= 1.0) {
    last_diag_time = state.time;
    printf("[PolicyCtrl T=%.2f] model=%s  cmd=(%.2f,%.2f,%.2f)  "
           "action_norm=%.3f\n",
           state.time,
           backend_->isLoaded() ? "LOADED" : "STUB",
           cmd_vx_, cmd_vy_, cmd_wyaw_,
           static_cast<double>(action.norm()));
    fflush(stdout);
  }
}

void PolicyController::getJointTargets(std::vector<double>& qref) {
  qref = joint_targets_;
}

void PolicyController::reset() {
  mode_     = control::BasicMotion::kDefault;
  is_fallen_ = false;
  last_time_ = -1.0;
  last_action_.setZero();
  last_obs_.setZero();
  setCommand(0.0, 0.0, 0.0);
  for (int i = 0; i < 12; ++i)
    joint_targets_[i] = nominal_joints_[i];
}

bool PolicyController::isFallen() const { return is_fallen_; }
