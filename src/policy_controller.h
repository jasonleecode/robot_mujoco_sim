#pragma once

/**
 * PolicyController — Model-based gait controller interface
 *
 * 与 SpotPlanner 接口完全对齐，可在物理循环中无缝替换。
 *
 * 工作流程：
 *   1. 构造时自动创建 StubBackend（无模型时输出 0 → 保持标称站立姿态）
 *   2. 训练完成后调用 loadModel(path) 加载真实模型文件
 *   3. PolicyBackend 是纯虚抽象类，实现它来对接 ONNX/LibTorch 等推理框架
 *
 * 观测向量 (52维，顺序与训练环境 go2_env.py 一致):
 *   [0-2]   body 线速度（body frame, m/s）
 *   [3-5]   body 角速度（body frame, rad/s）
 *   [6-8]   投影重力向量（body frame，单位向量）
 *   [9-11]  速度指令 (vx, vy, wyaw)
 *   [12-23] 关节角 - 标称角（MuJoCo 顺序: FL,FR,RL,RR 各 3 关节, rad）
 *   [24-35] 关节速度（MuJoCo 顺序, rad/s）
 *   [36-47] 上一时刻输出的 action
 *   [48-51] 步态时钟 [sin(φ), cos(φ), sin(φ+π), cos(φ+π)]，周期 0.5 s
 *
 * 动作向量 (12维):
 *   q_target[i] = q_nominal[i] + action_scale * action[i]
 *   顺序与观测中的关节角一致（MuJoCo: FL,FR,RL,RR）
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ControlActions.hpp"
#include "planner.h"       // RobotState
#include "robot_config.h"  // RobotConfig

// =========================================================================
// PolicyBackend — 推理引擎抽象接口
// =========================================================================

/**
 * @brief 推理引擎抽象接口。
 *
 * 实现此类以对接具体的推理框架（ONNX Runtime / LibTorch / 自定义）。
 * 训练完成后在此类的子类中实现 load() 和 forward()，
 * 然后通过 PolicyController::setBackend() 替换默认的 StubBackend。
 *
 * 示例子类参考文件末尾的 ONNX Runtime 注释模板。
 */
class PolicyBackend {
 public:
  virtual ~PolicyBackend() = default;

  /**
   * @brief 从文件加载模型。
   * @param path 模型文件路径（.onnx / .pt / 自定义格式）
   * @return true = 加载成功，false = 失败（将自动回退到零输出）
   */
  virtual bool load(const std::string& path) = 0;

  /**
   * @brief 执行一次前向推理。
   * @param input      输入向量数据指针（obs_dim 个 float）
   * @param input_dim  输入维度，应与 PolicyController::Config::obs_dim 一致
   * @param output     输出向量数据指针（action_dim 个 float，调用方负责分配）
   * @param output_dim 输出维度，应与 PolicyController::Config::action_dim 一致
   * @return true = 推理成功
   */
  virtual bool forward(const float* input, int input_dim,
                       float* output, int output_dim) = 0;

  virtual bool isLoaded() const = 0;
};

// =========================================================================
// PolicyController
// =========================================================================

// PolicyController 配置（必须在类声明之前定义，避免 GCC 默认参数限制）
struct PolicyControllerConfig {
  int    obs_dim      = 52;    // 观测维度（须与训练时一致）
  int    action_dim   = 12;    // 动作维度（12个关节）
  double control_dt   = 0.02;  // 控制频率 50Hz（MuJoCo 仍以 1kHz 运行）
  double action_scale = 0.25;  // 动作缩放系数 (rad/unit)，Go2 legged_gym 典型值 0.25
  std::string model_path;      // 模型文件路径（空 = Stub 模式）

  // Isaac Lab / legged_gym 标准观测缩放系数
  // （训练时对原始值乘以这些系数后输入网络）
  double scale_lin_vel  = 2.0;   // 线速度缩放
  double scale_ang_vel  = 0.25;  // 角速度缩放
  double scale_dof_pos  = 1.0;   // 关节角偏差缩放
  double scale_dof_vel  = 0.05;  // 关节角速度缩放
};

class PolicyController {
 public:
  using Config = PolicyControllerConfig;

  explicit PolicyController(const Config& cfg = Config{});
  ~PolicyController();

  // ----- 模型管理 --------------------------------------------------------

  /** 加载模型文件。可在运行时随时调用。 */
  bool loadModel(const std::string& path);

  /** 替换推理后端（用于注入 OnnxBackend 等自定义后端）。 */
  void setBackend(std::unique_ptr<PolicyBackend> backend);

  bool isModelLoaded() const;

  // ----- 机器人适配 -------------------------------------------------------

  /**
   * 从 RobotConfig 更新标称关节角和关节限位。
   * 在 loadModel 之前或之后调用均可。
   * PolicyController 构造时若传入 Config::model_path，
   * 会使用默认的 Spot 参数；切换到其他机器人后需调用此函数。
   */
  void setRobotConfig(const RobotConfig& robot_cfg);

  // ----- 控制接口（与 SpotPlanner 对齐）----------------------------------

  /** 设置运动模式，自动映射到速度指令。 */
  void setMode(control::BasicMotion motion);

  /** 直接设置速度指令（比 setMode 更精细）。 */
  void setCommand(double vx, double vy, double wyaw);

  /**
   * 主更新函数，在物理线程每个 tick 调用。
   * 内部按 control_dt 进行降频，只有满足时间间隔时才执行推理。
   */
  void update(const RobotState& state);

  /** 获取最新的 12 个关节目标角（MuJoCo 顺序：FL,FR,RL,RR）。 */
  void getJointTargets(std::vector<double>& qref);

  /** 重置所有状态（摔倒标志、上次动作、速度指令等）。 */
  void reset();

  bool isFallen() const;

  control::BasicMotion mode() const { return mode_; }

  // ----- 运行时诊断 -------------------------------------------------------

  /** 获取最近一次构建的观测向量（用于调试/可视化）。 */
  const Eigen::VectorXf& lastObservation() const { return last_obs_; }

  /** 获取最近一次网络输出的 action（缩放前）。 */
  const Eigen::VectorXf& lastAction() const { return last_action_; }

  // ----- 标称站立关节角（运行时可通过 setRobotConfig 更新）--------------
  // 默认使用 Spot 参数；切换机器人后会被 setRobotConfig 覆盖
  std::vector<double> nominal_joints_;
  std::vector<double> joint_limits_low_;
  std::vector<double> joint_limits_high_;

 private:
  // 构建 48 维观测向量
  Eigen::VectorXf buildObservation(const RobotState& state) const;

  // 执行推理，返回原始 action（未乘 action_scale）
  Eigen::VectorXf runInference(const Eigen::VectorXf& obs);

  // 将世界坐标系重力 [0,0,-1] 投影到 body frame
  // q: IMU 四元数 [w,x,y,z]（body-from-world 方向）
  static Eigen::Vector3f projectedGravity(const std::vector<double>& q);

  Config cfg_;
  control::BasicMotion mode_;
  std::unique_ptr<PolicyBackend> backend_;

  // 速度指令
  double cmd_vx_   = 0.0;
  double cmd_vy_   = 0.0;
  double cmd_wyaw_ = 0.0;

  // 历史状态
  Eigen::VectorXf last_action_;   // 上一时刻的 action（作为下一步观测）
  Eigen::VectorXf last_obs_;      // 上一时刻的观测（用于调试）

  // 关节目标（MuJoCo 顺序，12 维）
  std::vector<double> joint_targets_;

  // 控制频率
  double last_time_ = -1.0;

  // 跌倒检测
  bool is_fallen_ = false;

  // 速度指令大小（与 TrotGait 保持量级一致）
  static constexpr double kForwardVx  =  0.5;   // m/s
  static constexpr double kBackwardVx = -0.5;   // m/s
  static constexpr double kStrafeVy   =  0.5;   // m/s
  static constexpr double kTurnWyaw   =  0.5;   // rad/s
};

// =========================================================================
// LibTorch 后端 — 加载 TorchScript (.pt) 模型进行 CPU 推理
//
// 使用前需先将 legged_gym checkpoint 导出为 TorchScript：
//   python3 tools/export_policy.py models/legged_gym/model_300.pt
// 这会生成 models/legged_gym/actor_300.pt（TorchScript actor-only 模型）
//
// CMakeLists.txt 中需添加（USE_TORCH=ON 时自动启用）：
//   find_package(Torch REQUIRED)
//   target_link_libraries(RobotSim PRIVATE ${TORCH_LIBRARIES})
// =========================================================================

#ifdef USE_TORCH
#include <torch/script.h>

class TorchBackend : public PolicyBackend {
 public:
  bool load(const std::string& path) override {
    try {
      module_ = torch::jit::load(path, torch::kCPU);
      module_.eval();
      // 禁用梯度计算以加速推理
      for (auto param : module_.parameters())
        param.set_requires_grad(false);
      loaded_ = true;
      std::cout << "[TorchBackend] Loaded: " << path << "\n";
      return true;
    } catch (const c10::Error& e) {
      std::cerr << "[TorchBackend] Load failed: " << e.what() << "\n";
      return false;
    }
  }

  bool forward(const float* input, int input_dim,
               float* output, int output_dim) override {
    if (!loaded_) return false;
    try {
      torch::NoGradGuard no_grad;
      auto in_tensor = torch::from_blob(
          const_cast<float*>(input),
          {1, input_dim},
          torch::kFloat32).clone();
      auto result = module_.forward({in_tensor}).toTensor();
      result = result.squeeze(0).contiguous();
      const int n = std::min(output_dim, static_cast<int>(result.numel()));
      std::memcpy(output, result.data_ptr<float>(), n * sizeof(float));
      return true;
    } catch (const c10::Error& e) {
      std::cerr << "[TorchBackend] Inference error: " << e.what() << "\n";
      return false;
    }
  }

  bool isLoaded() const override { return loaded_; }

 private:
  torch::jit::script::Module module_;
  bool loaded_ = false;
};
#endif  // USE_TORCH
