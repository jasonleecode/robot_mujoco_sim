#pragma once

// 必须放在最前面，屏蔽 macOS 的 OpenGL 废弃警告
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "ControlActions.hpp"
#include "planner.h"

// IMU 数据结构
struct IMUData {
  double orientation[4] = {1.0, 0.0, 0.0, 0.0};  // w, x, y, z
  double gyro[3] = {0.0, 0.0, 0.0};
  double accel[3] = {0.0, 0.0, 0.0};
};

class RobotSim {
 public:
  mjrContext con;

  // --- 核心仿真数据 ---
  mjModel* m = nullptr;
  mjData* d = nullptr;

  // --- UI 系统 ---
  mjUI ui0;           // 左侧面板
  mjUI ui1;           // 右侧面板
  mjuiState uistate;  // UI 交互状态

  // UI 开关与状态
  int ui0_enable = 1;       // 默认开启左侧
  int ui1_enable = 1;       // 默认开启右侧
  int info = 1;             // 左上角信息覆盖层
  int run = 1;              // 1: 运行, 0: 暂停
  double time_scale = 1.0;  // 时间缩放

  int check_gravity = 1;

  // 运动控制按钮状态（用于UI按钮）
  int motion_forward = 0;   // 前进按钮
  int motion_stop = 0;      // 停止按钮

  // 运动控制回调（由main.cpp设置）
  std::function<void(int)> motion_callback;  // 参数：0=stop, 1=forward

  // 图表数据
  mjvFigure fig;
  mjrRect fig_rect = {0, 0, 0, 0};
  float plot_data[4][1000];
  int plot_idx = 0;
  const int kPlotPoints = 1000;

  // 截图请求
  std::atomic<int> screenshotrequest{0};

  // UI 脏标记：只在需要时才调用 uiModify
  bool ui_dirty = true;
  int prev_viewport_w = 0;
  int prev_viewport_h = 0;

  // 相机采集帧计数器（降低采集频率以减少锁竞争）
  int cam_frame_counter = 0;
  static constexpr int kCamCaptureInterval = 3;  // 每3帧采集一次

  // 构造与析构
  RobotSim(const std::string& xml_path);
  ~RobotSim();

  // --- 物理与控制接口 ---
  void stepPhysics();
  void applyControlVector(const std::vector<double>& control);
  void getState(RobotState& state) const;
  void setControl(int i, double val);
  int getNumActuators() const;

  void updatePlotData(const std::vector<double>& qref);
  void getIMUData(IMUData& data, const std::string& prefix);  // 外部调用（带锁）

  // --- 渲染与窗口接口 ---
  void renderFrame();
  bool isWindowOpen() const;
  bool windowShouldClose() const;
  GLFWwindow* getWindow() const;

  // 辅助功能
  void getCameraImages(const std::string& camera_name, int width, int height,
                       std::vector<unsigned char>& rgb_output, std::vector<float>& depth_output);

  void initializeUI();
  void updateInfoText();
  void takeScreenshot();

 private:
  GLFWwindow* window = nullptr;
  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjvScene scn_sensor;
  mjvPerturb pert;

  IMUData current_imu;  // 内部缓存
  const int cam_w = 640;
  const int cam_h = 480;
  std::vector<unsigned char> cam_rgb_vec;
  std::vector<float> cam_depth_vec;

  // 交互状态缓存
  double lastx = 0;
  double lasty = 0;
  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;

  // 核心互斥锁
  mutable std::mutex sim_mutex;

  // 内部无锁函数
  void getIMUDataInternal(IMUData& data, const std::string& sensor_name_prefix);

  void uiModify(mjUI* ui, mjuiState* state, mjrContext* con);

  // 静态回调
  static void mouse_button(GLFWwindow* window, int button, int action, int mods);
  static void mouse_move(GLFWwindow* window, double xpos, double ypos);
  static void scroll(GLFWwindow* window, double xoffset, double yoffset);
  static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

  // 内部处理
  void handle_mouse_button(int button, int action, int mods);
  void handle_mouse_move(double xpos, double ypos);
  void handle_scroll(double xoffset, double yoffset);
  void handle_keyboard(int key, int scancode, int act, int mods);

  // Info 文本缓存
  char info_title[1024] = {0};
  char info_content[1024] = {0};

  // 速度选项
  int real_time_index = 0;
  static const float percentRealTime[];
};