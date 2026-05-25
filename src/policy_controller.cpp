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
      // Policy was trained with vx ∈ (0.3, 1.5); cmd=0 is out-of-distribution.
      // Use minimum training velocity to keep behaviour stable.
      setCommand(0.3, 0.0, 0.0);
      break;
  }
}

// =========================================================================
// 将世界坐标系向量旋转到机体坐标系
// MuJoCo freejoint q=[w,x,y,z] 表示机体相对世界的朝向（body-from-world）
// v_body = R(q)^T * v_world，等价于用 q^{-1}=[w,-x,-y,-z] 旋转
// 旋转矩阵展开：
//   vb_x = (1-2y²-2z²)*vx + 2(xy+wz)*vy + 2(xz-wy)*vz
//   vb_y = 2(xy-wz)*vx  + (1-2x²-2z²)*vy + 2(yz+wx)*vz
//   vb_z = 2(xz+wy)*vx  + 2(yz-wx)*vy  + (1-2x²-2y²)*vz
// 验证：q=[1,0,0,0] 时 v_body=v_world ✓
// =========================================================================
static Eigen::Vector3f worldToBody(const std::vector<double>& q,
                                   double vx, double vy, double vz) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return Eigen::Vector3f(
    static_cast<float>((1-2*y*y-2*z*z)*vx + 2*(x*y+w*z)*vy  + 2*(x*z-w*y)*vz),
    static_cast<float>(2*(x*y-w*z)*vx   + (1-2*x*x-2*z*z)*vy + 2*(y*z+w*x)*vz),
    static_cast<float>(2*(x*z+w*y)*vx   + 2*(y*z-w*x)*vy  + (1-2*x*x-2*y*y)*vz)
  );
}

// =========================================================================
// 重力投影辅助函数
// g_body = worldToBody(q, 0, 0, -1)，直立时结果为 [0,0,-1] ✓
// =========================================================================
Eigen::Vector3f PolicyController::projectedGravity(const std::vector<double>& q) {
  return worldToBody(q, 0.0, 0.0, -1.0);
}

// =========================================================================
// 观测向量构建（52 维：48 dim Isaac Lab standard + 4 dim gait clock）
// =========================================================================
Eigen::VectorXf PolicyController::buildObservation(const RobotState& state) const {
  Eigen::VectorXf obs(cfg_.obs_dim);
  int idx = 0;

  // ---- [0-2] body 线速度（世界→机体完整旋转，乘以 scale_lin_vel）----------
  const double vx_w = (state.qvel.size() > 0) ? state.qvel[0] : 0.0;
  const double vy_w = (state.qvel.size() > 1) ? state.qvel[1] : 0.0;
  const double vz_w = (state.qvel.size() > 2) ? state.qvel[2] : 0.0;
  const Eigen::Vector3f lin_vel_b = worldToBody(state.imu_quat, vx_w, vy_w, vz_w);

  const float sv = static_cast<float>(cfg_.scale_lin_vel);
  obs[idx++] = sv * lin_vel_b[0];
  obs[idx++] = sv * lin_vel_b[1];
  obs[idx++] = sv * lin_vel_b[2];

  // ---- [3-5] body 角速度（MuJoCo qvel[3-5] 为世界系，旋转到机体系）--------
  // legged_gym 训练时用的是 base_ang_vel（机体系），需保持一致
  const double wx_w = (state.qvel.size() > 3) ? state.qvel[3] : 0.0;
  const double wy_w = (state.qvel.size() > 4) ? state.qvel[4] : 0.0;
  const double wz_w = (state.qvel.size() > 5) ? state.qvel[5] : 0.0;
  const Eigen::Vector3f ang_vel_b = worldToBody(state.imu_quat, wx_w, wy_w, wz_w);

  const float sw = static_cast<float>(cfg_.scale_ang_vel);
  obs[idx++] = sw * ang_vel_b[0];
  obs[idx++] = sw * ang_vel_b[1];
  obs[idx++] = sw * ang_vel_b[2];

  // ---- [6-8] 投影重力（body frame）---------------------------------------
  const Eigen::Vector3f pg = projectedGravity(state.imu_quat);
  obs[idx++] = pg[0];
  obs[idx++] = pg[1];
  obs[idx++] = pg[2];

  // ---- [9-11] 速度指令（与 legged_gym 一致：linear cmd × scale_lin_vel，
  //            yaw cmd × scale_ang_vel）-------------------------------------
  obs[idx++] = sv * static_cast<float>(cmd_vx_);
  obs[idx++] = sv * static_cast<float>(cmd_vy_);
  obs[idx++] = sw * static_cast<float>(cmd_wyaw_);

  // ---- [12-23] 关节角 - 标称角（乘以 scale_dof_pos）---------------------
  const float sp = static_cast<float>(cfg_.scale_dof_pos);
  for (int i = 0; i < 12; ++i) {
    const int qi = 7 + i;
    const double q = (state.qpos.size() > static_cast<size_t>(qi))
                         ? state.qpos[qi] : 0.0;
    obs[idx++] = sp * static_cast<float>(q - nominal_joints_[i]);
  }

  // ---- [24-35] 关节速度（乘以 scale_dof_vel）----------------------------
  const float sdv = static_cast<float>(cfg_.scale_dof_vel);
  for (int i = 0; i < 12; ++i) {
    const int vi = 6 + i;
    obs[idx++] = sdv * ((state.qvel.size() > static_cast<size_t>(vi))
                            ? static_cast<float>(state.qvel[vi]) : 0.0f);
  }

  // ---- [36-47] 上一时刻 action -------------------------------------------
  for (int i = 0; i < cfg_.action_dim; ++i)
    obs[idx++] = (i < last_action_.size()) ? last_action_[i] : 0.0f;

  // ---- [48-51] 步态时钟（gait phase clock，周期 0.5 s）-------------------
  // 必须与训练环境 go2_env.py 中的 GAIT_PERIOD 和 CTRL_DT 保持一致。
  // phase = 2π × t / GAIT_PERIOD，训练时初相随机化，部署时从 t=0 开始（均在分布内）。
  static constexpr float kGaitPeriod = 0.5f;
  const float phase = static_cast<float>(
      std::fmod(state.time * 2.0 * M_PI / kGaitPeriod, 2.0 * M_PI));
  obs[idx++] = std::sin(phase);
  obs[idx++] = std::cos(phase);
  obs[idx++] = std::sin(phase + static_cast<float>(M_PI));
  obs[idx++] = std::cos(phase + static_cast<float>(M_PI));

  // 观测截断：防止极端值干扰网络（legged_gym 训练时通常有 clip_observations=100，
  // 这里收紧到 ±5 以提高鲁棒性）
  return obs.cwiseMax(-5.0f).cwiseMin(5.0f);
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
  Eigen::VectorXf action = runInference(last_obs_);

  // 动作截断：将网络原始输出限制在 [-clip, clip]
  // legged_gym 默认 clip_actions=100（几乎无截断），这里收紧以防止关节暴走
  const float clip = 1.0f;
  action = action.cwiseMax(-clip).cwiseMin(clip);
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
