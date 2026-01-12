#include "RobotSim.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

// 速度选项定义
const float RobotSim::percentRealTime[] = {-1, 100, 80,  66,  50,   40,   33,   25,  20, 16,
                                           13, 10,  8,   6,   5,    4,    3,    2.5, 2,  1.5,
                                           1,  0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0};

RobotSim::RobotSim(const std::string& xml_path) {
  // --- 1. 图表初始化 ---
  mjv_defaultFigure(&fig);
  strcpy(fig.title, "Gait Analysis");
  strcpy(fig.xlabel, "Time (ticks)");
  fig.flg_extend = 1;
  fig.flg_barplot = 0;
  fig.flg_symmetric = 0;
  fig.linewidth = 2.0f;
  fig.gridsize[0] = 5;
  fig.gridsize[1] = 5;

  strcpy(fig.linename[0], "FR Knee");
  strcpy(fig.linename[1], "FL Knee");
  strcpy(fig.linename[2], "RR Knee");
  strcpy(fig.linename[3], "RL Knee");

  fig.linergb[0][0] = 1.0f;
  fig.linergb[0][1] = 0.1f;
  fig.linergb[0][2] = 0.1f;
  fig.linergb[1][0] = 0.1f;
  fig.linergb[1][1] = 1.0f;
  fig.linergb[1][2] = 0.1f;
  fig.linergb[2][0] = 0.1f;
  fig.linergb[2][1] = 0.1f;
  fig.linergb[2][2] = 1.0f;
  fig.linergb[3][0] = 1.0f;
  fig.linergb[3][1] = 0.8f;
  fig.linergb[3][2] = 0.0f;

  for (int i = 0; i < 4; ++i)
    fig.linepnt[i] = kPlotPoints;

  // --- 2. 加载模型 ---
  char error[1000];
  m = mj_loadXML(xml_path.c_str(), 0, error, 1000);
  if (!m)
    throw std::runtime_error(error);
  d = mj_makeData(m);

  // --- 3. 初始化 GLFW ---
  if (!glfwInit())
    throw std::runtime_error("GLFW Init Failed");
  glfwWindowHint(GLFW_SAMPLES, 4);
  window = glfwCreateWindow(1200, 900, "MuJoCo Sim (RobotSim)", NULL, NULL);
  glfwMakeContextCurrent(window);

  // --- 4. 初始化 MuJoCo 渲染 ---
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjv_defaultScene(&scn_sensor);
  mjr_defaultContext(&con);
  mjv_makeScene(m, &scn, 2000);
  mjv_makeScene(m, &scn_sensor, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  glfwSetWindowUserPointer(window, this);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetScrollCallback(window, scroll);
  glfwSetKeyCallback(window, keyboard);

  // --- 5. 初始化 UI ---
  initializeUI();
}

RobotSim::~RobotSim() {
  mjr_freeContext(&con);
  mjv_freeScene(&scn);
  mjv_freeScene(&scn_sensor);
  mj_deleteData(d);
  mj_deleteModel(m);
  if (window)
    glfwDestroyWindow(window);
  glfwTerminate();
}

// 线程安全的设置控制输入
void RobotSim::setControl(int i, double val) {
  std::lock_guard<std::mutex> lock(sim_mutex);
  if (m && i >= 0 && i < m->nu)
    d->ctrl[i] = val;
}

// 物理步进
void RobotSim::stepPhysics() {
  std::lock_guard<std::mutex> lock(sim_mutex);
  if (!m || !d)
    return;
  mj_step(m, d);

  // 更新当前 IMU 数据
  getIMUDataInternal(this->current_imu, "imu_");
}

// 应用完整的控制向量
void RobotSim::applyControlVector(const std::vector<double>& control) {
  std::lock_guard<std::mutex> lock(sim_mutex);
  if (!m || !d)
    return;
  const int count = std::min(static_cast<int>(control.size()), m->nu);
  for (int i = 0; i < count; ++i)
    d->ctrl[i] = control[i];
  // 其余补0
  for (int i = count; i < m->nu; ++i)
    d->ctrl[i] = 0.0;
}

// 获取状态
void RobotSim::getState(RobotState& state) const {
  std::lock_guard<std::mutex> lock(sim_mutex);
  state.time = d ? d->time : 0.0;

  if (d && m) {
    // 1. 拷贝关节数据 (原有逻辑)
    state.qpos.assign(d->qpos, d->qpos + m->nq);
    state.qvel.assign(d->qvel, d->qvel + m->nv);

    // 2. [修复] 拷贝 IMU 数据 (从内部缓存 current_imu 拷贝到 state)
    // 这一步之前漏掉了！
    state.imu_quat.resize(4);
    state.imu_quat[0] = current_imu.orientation[0];  // w
    state.imu_quat[1] = current_imu.orientation[1];  // x
    state.imu_quat[2] = current_imu.orientation[2];  // y
    state.imu_quat[3] = current_imu.orientation[3];  // z

    state.imu_gyro.resize(3);
    state.imu_gyro[0] = current_imu.gyro[0];
    state.imu_gyro[1] = current_imu.gyro[1];
    state.imu_gyro[2] = current_imu.gyro[2];

  } else {
    state.qpos.clear();
    state.qvel.clear();
    // 可选：重置 IMU 为默认值
    state.imu_quat = {1.0, 0.0, 0.0, 0.0};
    state.imu_gyro = {0.0, 0.0, 0.0};
  }
}

// [核心重构] 渲染帧：锁粒度最小化
void RobotSim::renderFrame() {
  if (!window || glfwWindowShouldClose(window))
    return;

  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  // --- 临界区 1: 访问 mjData 和更新 Scene ---
  {
    std::lock_guard<std::mutex> lock(sim_mutex);

    // 根据IMU数据做姿态解算

    // 获取相机图像 (这也需要 mjData 来更新 scene_sensor)
    getCameraImages("tof_cam", cam_w, cam_h, cam_rgb_vec, cam_depth_vec);

    // 更新主场景 (这是 CPU 操作，拷贝 mjData -> mjvScene)
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    // 更新 info 文本
    updateInfoText();
  }
  // --- 锁释放：物理线程现在可以继续运行了 ---

  // --- 计算 UI 布局 ---
  mjrRect rect_ui0 = {0, 0, 0, 0};
  mjrRect rect_ui1 = {0, 0, 0, 0};
  mjrRect rect_scene = viewport;

  if (ui0_enable) {
    rect_ui0.left = 0;
    rect_ui0.bottom = 0;
    rect_ui0.width = ui0.width;
    rect_ui0.height = viewport.height;
    rect_scene.left = ui0.width;
    rect_scene.width -= ui0.width;
  }

  if (ui1_enable) {
    rect_ui1.width = ui1.width;
    rect_ui1.left = viewport.width - ui1.width;
    rect_ui1.bottom = 0;
    rect_ui1.height = viewport.height;
    rect_scene.width -= ui1.width;
  }

  // --- 渲染阶段 (GPU 操作，耗时但无需锁住 mjData) ---
  mjr_render(rect_scene, &scn, &con);

  // 绘制图表 (在场景区域内)
  fig_rect.left = rect_scene.left + 10;
  fig_rect.bottom = 10;
  fig_rect.width = rect_scene.width / 2 - 20;
  fig_rect.height = viewport.height / 3;

  // 注意：fig 也在物理线程被写入，理论上需要锁。
  // 但为了不阻塞渲染，且 mjvFigure 的数据通常容忍一帧的撕裂，这里不加重锁。
  mjr_figure(fig_rect, &fig, &con);

  // 绘制传感器覆盖层
  drawSensorOverlay(viewport);

  // --- 渲染 UI 面板 ---
  renderUI(viewport);

  // 刷新屏幕
  glfwSwapBuffers(window);
  glfwPollEvents();
}

// [新增] 线程安全的更新图表数据
void RobotSim::updatePlotData(const std::vector<double>& qref) {
  if (qref.size() < 12)
    return;

  std::lock_guard<std::mutex> lock(sim_mutex);

  int p_idx = plot_idx % kPlotPoints;
  // 记录四个关节的目标值 (Target)
  // 假设 qref 顺序: 0-2 FR, 3-5 FL, 6-8 RR, 9-11 RL
  plot_data[0][p_idx] = (float)qref[2];   // FR Knee
  plot_data[1][p_idx] = (float)qref[5];   // FL Knee
  plot_data[2][p_idx] = (float)qref[8];   // RR Knee
  plot_data[3][p_idx] = (float)qref[11];  // RL Knee
  plot_idx++;

  // 更新 mjvFigure 的线性缓冲区
  for (int k = 0; k < kPlotPoints; k++) {
    int history_idx = (plot_idx + k) % kPlotPoints;
    for (int line = 0; line < 4; ++line) {
      fig.linedata[line][2 * k] = (float)k;
      fig.linedata[line][2 * k + 1] = plot_data[line][history_idx];
    }
  }
}

// 绘制左上角的传感器画面
void RobotSim::drawSensorOverlay(const mjrRect& viewport) {
  // 检查是否有数据
  if (cam_rgb_vec.empty() || cam_depth_vec.empty()) {
    // 画一个无信号提示
    int disp_w = viewport.width / 2 - 20;
    int disp_h = viewport.height / 3;
    mjrRect img_rect = {viewport.width - disp_w - 10, 10, disp_w, disp_h};

    // 简单画个黑框，不做复杂 OpenCV 操作以防异常
    return;
  }

  int disp_w = viewport.width / 2 - 20;
  int disp_h = viewport.height / 3;
  mjrRect img_rect;
  img_rect.width = disp_w;
  img_rect.height = disp_h;
  img_rect.left = viewport.width - disp_w - 10;
  img_rect.bottom = 10;

  // OpenCV 处理
  cv::Mat img_rgb(cam_h, cam_w, CV_8UC3, cam_rgb_vec.data());
  cv::Mat img_depth(cam_h, cam_w, CV_32F, cam_depth_vec.data());

  cv::Mat rgb_upright, depth_upright;
  cv::flip(img_rgb, rgb_upright, 0);
  cv::flip(img_depth, depth_upright, 0);

  cv::Mat depth_norm, depth_color;
  double max_dist = 10.0;
  depth_upright.convertTo(depth_norm, CV_8U, 255.0 / max_dist);
  cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);

  cv::Mat rgb_bgr;
  cv::cvtColor(rgb_upright, rgb_bgr, cv::COLOR_RGB2BGR);

  cv::Mat combined;
  cv::hconcat(std::vector<cv::Mat>{depth_color, rgb_bgr}, combined);

  cv::Mat resized_overlay;
  cv::resize(combined, resized_overlay, cv::Size(disp_w, disp_h));

  cv::Mat final_overlay;
  cv::cvtColor(resized_overlay, final_overlay, cv::COLOR_BGR2RGB);
  cv::flip(final_overlay, final_overlay, 0);

  mjr_drawPixels(final_overlay.data, nullptr, img_rect, &con);
}

// 辅助函数: 深度缓冲转换
float zbuffer_to_meters(float depth_val, float znear, float zfar) {
  return znear / (1.0f - depth_val * (1.0f - znear / zfar));
}

// 获取IMU数据
void RobotSim::getIMUData(IMUData& data, const std::string& prefix) {
  // 必须加锁，因为 physics_loop 和 main 线程可能会同时访问 mjData
  std::lock_guard<std::mutex> lock(sim_mutex);
  getIMUDataInternal(data, prefix);
}

void RobotSim::getIMUDataInternal(IMUData& data, const std::string& prefix) {
  if (!m || !d)
    return;

  // 1. 读取四元数 (Frame Quaternion)
  int id_quat = mj_name2id(m, mjOBJ_SENSOR, (prefix + "quat").c_str());
  if (id_quat != -1) {
    int adr = m->sensor_adr[id_quat];
    // MuJoCo 四元数通常是 [w, x, y, z]
    data.orientation[0] = d->sensordata[adr + 0];
    data.orientation[1] = d->sensordata[adr + 1];
    data.orientation[2] = d->sensordata[adr + 2];
    data.orientation[3] = d->sensordata[adr + 3];
  }

  // 2. 读取角速度 (Gyro)
  int id_gyro = mj_name2id(m, mjOBJ_SENSOR, (prefix + "gyro").c_str());
  if (id_gyro != -1) {
    int adr = m->sensor_adr[id_gyro];
    data.gyro[0] = d->sensordata[adr + 0];
    data.gyro[1] = d->sensordata[adr + 1];
    data.gyro[2] = d->sensordata[adr + 2];
  }

  // 3. 读取加速度 (Accelerometer)
  int id_acc = mj_name2id(m, mjOBJ_SENSOR, (prefix + "acc").c_str());
  if (id_acc != -1) {
    int adr = m->sensor_adr[id_acc];
    data.accel[0] = d->sensordata[adr + 0];
    data.accel[1] = d->sensordata[adr + 1];
    data.accel[2] = d->sensordata[adr + 2];
  }
}

// 获取相机数据
void RobotSim::getCameraImages(const std::string& camera_name, int width, int height,
                               std::vector<unsigned char>& rgb_output,
                               std::vector<float>& depth_output) {
  // 此函数在 renderFrame 的锁内调用，安全访问 m
  int cam_id = mj_name2id(m, mjOBJ_CAMERA, camera_name.c_str());
  if (cam_id == -1) {
    static bool warned = false;
    if (!warned) {
      std::cerr << "Warning: Camera '" << camera_name << "' not found.\n";
      warned = true;
    }
    return;
  }

  mjvCamera cam_sensor;
  mjv_defaultCamera(&cam_sensor);
  cam_sensor.type = mjCAMERA_FIXED;
  cam_sensor.fixedcamid = cam_id;

  // 更新传感器场景
  mjv_updateScene(m, d, &opt, NULL, &cam_sensor, mjCAT_ALL, &scn_sensor);

  // 渲染到 Offscreen Buffer
  mjrRect viewport = {0, 0, width, height};
  mjr_render(viewport, &scn_sensor, &con);

  // 读取像素
  rgb_output.resize(width * height * 3);
  depth_output.resize(width * height);
  mjr_readPixels(rgb_output.data(), depth_output.data(), viewport, &con);

  // 深度线性化
  float znear = m->vis.map.znear;
  float zfar = m->vis.map.zfar;
  for (size_t i = 0; i < depth_output.size(); ++i) {
    depth_output[i] = zbuffer_to_meters(depth_output[i], znear, zfar);
  }
}

// 窗口状态查询
bool RobotSim::isWindowOpen() const {
  // 窗口指针是 GLFW 管理的，不涉及竞争，但也加上锁比较保险，或者假定主线程调用
  return window && !glfwWindowShouldClose(window);
}
bool RobotSim::windowShouldClose() const {
  return !window || glfwWindowShouldClose(window);
}
GLFWwindow* RobotSim::getWindow() const {
  return window;
}

// 静态回调
void RobotSim::mouse_button(GLFWwindow* window, int button, int action, int mods) {
  RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
  if (sim)
    sim->handle_mouse_button(button, action, mods);
}
void RobotSim::mouse_move(GLFWwindow* window, double xpos, double ypos) {
  RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
  if (sim)
    sim->handle_mouse_move(xpos, ypos);
}
void RobotSim::scroll(GLFWwindow* window, double xoffset, double yoffset) {
  RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
  if (sim)
    sim->handle_scroll(xoffset, yoffset);
}

// 鼠标交互辅助：判断是否在图表上
static bool is_mouse_over_figure(double x, double y, const mjrRect& r, int win_h) {
  double gl_y = win_h - y;
  return x >= r.left && x <= r.left + r.width && gl_y >= r.bottom && gl_y <= r.bottom + r.height;
}

void RobotSim::handle_mouse_button(int button, int action, int mods) {
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  double x, y;
  glfwGetCursorPos(window, &x, &y);
  int w, h;
  glfwGetWindowSize(window, &w, &h);

  if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_MIDDLE) {
    if (is_mouse_over_figure(x, y, fig_rect, h)) {
      fig.flg_extend = 1;
    }
  }
}

void RobotSim::handle_mouse_move(double xpos, double ypos) {
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  if (is_mouse_over_figure(xpos, ypos, fig_rect, height)) {
    float rel_x = (float)(xpos - fig_rect.left) / fig_rect.width;
    fig.highlight[0] = rel_x;
    fig.highlight[1] = rel_x;

    if (button_left || button_right) {
      double dx = xpos - lastx;
      double dy = ypos - lasty;
      fig.flg_extend = 0;
      float range_x = fig.range[1][0] - fig.range[0][0];
      float range_y = fig.range[1][1] - fig.range[0][1];
      float pan_x = -dx / fig_rect.width * range_x;
      float pan_y = dy / fig_rect.height * range_y;
      fig.range[0][0] += pan_x;
      fig.range[1][0] += pan_x;
      fig.range[0][1] += pan_y;
      fig.range[1][1] += pan_y;
    }
    lastx = xpos;
    lasty = ypos;
    return;
  } else {
    fig.highlight[0] = -1;
  }

  if (!button_left && !button_middle && !button_right) {
    lastx = xpos;
    lasty = ypos;
    return;
  }
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
  mjtMouse action;
  if (button_right)
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if (button_left)
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void RobotSim::handle_scroll(double xoffset, double yoffset) {
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  double x, y;
  glfwGetCursorPos(window, &x, &y);

  if (is_mouse_over_figure(x, y, fig_rect, height)) {
    fig.flg_extend = 0;
    float zoom_factor = (yoffset > 0) ? 0.9f : 1.1f;
    float center_x = (fig.range[0][0] + fig.range[1][0]) / 2.0f;
    float center_y = (fig.range[0][1] + fig.range[1][1]) / 2.0f;
    float span_x = (fig.range[1][0] - fig.range[0][0]) * zoom_factor;
    float span_y = (fig.range[1][1] - fig.range[0][1]) * zoom_factor;
    fig.range[0][0] = center_x - span_x / 2.0f;
    fig.range[1][0] = center_x + span_x / 2.0f;
    fig.range[0][1] = center_y - span_y / 2.0f;
    fig.range[1][1] = center_y + span_y / 2.0f;
    return;
  }
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -yoffset * 0.05, &scn, &cam);
}

void RobotSim::applyBasicMotion(control::BasicMotion motion, std::vector<double>& control) const {
  control.assign(getNumActuators(), 0.0);
  const int total = static_cast<int>(control.size());
  if (total == 0)
    return;

  auto apply_range = [&](int start, int end, double value) {
    start = std::max(0, start);
    end = std::min(total, end);
    for (int i = start; i < end; ++i) {
      control[i] = value;
    }
  };
  const int half = std::max(1, total / 2);
  const int quarter = std::max(1, total / 4);
  constexpr double kStride = 0.45;
  constexpr double kTurn = 0.35;
  constexpr double kCrouch = -0.25;
  constexpr double kJump = 0.85;

  switch (motion) {
    case control::BasicMotion::kForward:
      apply_range(0, half, kStride);
      apply_range(half, total, -kStride);
      break;
    case control::BasicMotion::kBackward:
      apply_range(0, half, -kStride);
      apply_range(half, total, kStride);
      break;
    case control::BasicMotion::kTurnLeft:
      apply_range(0, half, -kTurn);
      apply_range(half, total, kTurn);
      break;
    case control::BasicMotion::kTurnRight:
      apply_range(0, half, kTurn);
      apply_range(half, total, -kTurn);
      break;
    case control::BasicMotion::kCrouch:
      std::fill(control.begin(), control.end(), kCrouch);
      break;
    case control::BasicMotion::kStand:
      std::fill(control.begin(), control.end(), 0.0);
      break;
    case control::BasicMotion::kJump:
      apply_range(0, quarter, kJump);
      apply_range(quarter, total - quarter, kJump * 0.8);
      apply_range(total - quarter, total, kJump);
      break;
    case control::BasicMotion::kCount:
    default:
      break;
  }
}

int RobotSim::getNumActuators() const {
  return m ? m->nu : 0;
}

// 初始化 UI
void RobotSim::initializeUI() {
  // 清空 UI 结构
  std::memset(&ui0, 0, sizeof(ui0));
  std::memset(&ui1, 0, sizeof(ui1));
  std::memset(&uistate, 0, sizeof(uistate));

  // 设置 UI 主题
  ui0.spacing = mjui_themeSpacing(spacing);
  ui0.color = mjui_themeColor(color);
  ui1.spacing = mjui_themeSpacing(spacing);
  ui1.color = mjui_themeColor(color);

  // UI0: 左侧面板 - Option 和 Simulation 部分
  mjuiDef defOption[] = {
      {mjITEM_SECTION, "Option", mjPRESERVE, nullptr, "AO"},
      {mjITEM_CHECKINT, "Help", 2, &help, " #290"},
      {mjITEM_CHECKINT, "Info", 2, &info, " #291"},
      {mjITEM_CHECKINT, "Profiler", 2, &profiler, " #292"},
      {mjITEM_CHECKINT, "VSync", 1, &vsync, ""},
      {mjITEM_SELECT, "Spacing", 1, &spacing, "Tight\nWide"},
      {mjITEM_SELECT, "Color", 1, &color, "Default\nOrange\nWhite\nBlack"},
      {mjITEM_SELECT, "Font", 1, &font, "50 %\n100 %\n150 %\n200 %\n250 %\n300 %"},
      {mjITEM_END}};

  mjuiDef defSimulation[] = {{mjITEM_SECTION, "Simulation", mjPRESERVE, nullptr, "AS"},
                             {mjITEM_RADIO, "", 5, &run, "Pause\nRun"},
                             {mjITEM_BUTTON, "Reset", 2, nullptr, " #259"},
                             {mjITEM_BUTTON, "Screenshot", 2, nullptr, "CP"},
                             {mjITEM_SEPARATOR, "Speed", 1},
                             {mjITEM_SLIDERINT, "RT Index", 2, &real_time_index, "0 26"},
                             {mjITEM_END}};

  mjuiDef defWatch[] = {{mjITEM_SECTION, "Watch", mjPRESERVE, nullptr, "AW"},
                        {mjITEM_STATIC, "Time", 2, nullptr, " "},
                        {mjITEM_STATIC, "FPS", 2, nullptr, " "},
                        {mjITEM_END}};

  // 添加到 ui0
  mjui_add(&ui0, defOption);
  mjui_add(&ui0, defSimulation);
  mjui_add(&ui0, defWatch);
  ui0.sect[0].state = 1;  // Option section expanded
  ui0.sect[1].state = 1;  // Simulation section expanded

  // UI1: 右侧面板 - 关节控制
  mjuiDef defJoint[] = {{mjITEM_SECTION, "Joint Control", mjPRESERVE, nullptr, "AJ"}, {mjITEM_END}};
  mjui_add(&ui1, defJoint);
  ui1.sect[0].state = 1;

  // 添加关节滑块
  if (m) {
    mjuiDef defSlider[] = {{mjITEM_SLIDERNUM, "", 2, nullptr, "0 1"}, {mjITEM_END}};
    defSlider[0].state = 4;

    for (int i = 0; i < m->njnt && i < mjMAXUIITEM - 2; i++) {
      if (m->jnt_type[i] == mjJNT_HINGE || m->jnt_type[i] == mjJNT_SLIDE) {
        defSlider[0].pdata = &d->qpos[m->jnt_qposadr[i]];
        const char* jnt_name = mj_id2name(m, mjOBJ_JOINT, i);
        if (jnt_name) {
          std::strncpy(defSlider[0].name, jnt_name, mjMAXUINAME - 1);
        } else {
          std::snprintf(defSlider[0].name, mjMAXUINAME, "joint %d", i);
        }
        // 设置范围
        if (m->jnt_limited[i]) {
          std::snprintf(defSlider[0].other, mjMAXUITEXT, "%.4g %.4g", m->jnt_range[2 * i],
                        m->jnt_range[2 * i + 1]);
        } else {
          std::strncpy(defSlider[0].other, "-3.14 3.14", mjMAXUITEXT - 1);
        }
        mjui_add(&ui1, defSlider);
      }
    }
  }

  // 调整 UI 大小
  mjui_resize(&ui0, &con);
  mjui_resize(&ui1, &con);
}

void RobotSim::updateInfoText() {
  // 1. 设置标题
  std::snprintf(info_title, sizeof(info_title), "RobotSim Info");

  // 2. 获取基础数据
  double time_val = d ? d->time : 0.0;

  // 3. [新增] 计算欧拉角 (Roll, Pitch, Yaw)
  // 从 current_imu 读取四元数 [w, x, y, z]
  double w = current_imu.orientation[0];
  double x = current_imu.orientation[1];
  double y = current_imu.orientation[2];
  double z = current_imu.orientation[3];

  double roll, pitch, yaw;

  // Roll (x-axis rotation)
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);

  // 4. [修改] 格式化输出内容，增加 RPY 显示
  std::snprintf(info_content, sizeof(info_content),
                "Time: %.3f s\n"
                "Speed: %.1f%%\n"
                "Actuators: %d\n"
                "-----------------\n"
                "Roll:  %6.3f\n"
                "Pitch: %6.3f\n"
                "Yaw:   %6.3f",
                time_val,
                real_time_index >= 0 && real_time_index < kNumSpeedOptions
                    ? percentRealTime[real_time_index]
                    : 100.0f,
                m ? m->nu : 0, roll, pitch, yaw);
}

void RobotSim::takeScreenshot() {
  // 请求截图
  screenshotrequest.store(1);
}

// 静态键盘回调
void RobotSim::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
  if (sim)
    sim->handle_keyboard(key, scancode, act, mods);
}

void RobotSim::handle_keyboard(int key, int scancode, int act, int mods) {
  if (act != GLFW_PRESS && act != GLFW_REPEAT)
    return;

  // UI 面板切换快捷键
  switch (key) {
    case GLFW_KEY_F1:  // Toggle left UI
      ui0_enable = !ui0_enable;
      break;
    case GLFW_KEY_F2:  // Toggle right UI
      ui1_enable = !ui1_enable;
      break;
    case GLFW_KEY_F5:  // Toggle info
      info = !info;
      break;
    case GLFW_KEY_F6:  // Toggle profiler
      profiler = !profiler;
      break;
    case GLFW_KEY_SPACE:  // Pause/Run
      run = !run;
      break;
    case GLFW_KEY_BACKSPACE:  // Reset
      if (m && d) {
        std::lock_guard<std::mutex> lock(sim_mutex);
        mj_resetData(m, d);
        mj_forward(m, d);
      }
      break;
    case GLFW_KEY_P:  // Screenshot (Ctrl+P)
      if (mods & GLFW_MOD_CONTROL) {
        takeScreenshot();
      }
      break;
    case GLFW_KEY_EQUAL:  // Speed up (+)
      if (real_time_index > 0)
        real_time_index--;
      break;
    case GLFW_KEY_MINUS:  // Speed down (-)
      if (real_time_index < kNumSpeedOptions - 1)
        real_time_index++;
      break;
    default:
      break;
  }
}

// 渲染 UI 面板
void RobotSim::renderUI(const mjrRect& viewport) {
  // UI0: 左侧面板
  if (ui0_enable) {
    mjrRect rect_ui0 = {0, 0, ui0.width, viewport.height};
    mjui_render(&ui0, &uistate, &con);
  }

  // UI1: 右侧面板
  if (ui1_enable) {
    mjrRect rect_ui1 = {viewport.width - ui1.width, 0, ui1.width, viewport.height};
    mjui_render(&ui1, &uistate, &con);
  }

  // 绘制 Info 覆盖
  if (info) {
    mjrRect rect_info = {20, viewport.height - 200, 300, 180};
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, info_title, info_content, &con);
  }

  // 绘制 Help 信息
  if (help) {
    const char* help_title = "Keyboard Shortcuts";
    const char* help_content =
        "F1: Toggle left UI\n"
        "F2: Toggle right UI\n"
        "F5: Toggle info\n"
        "F6: Toggle profiler\n"
        "Space: Pause/Run\n"
        "Backspace: Reset\n"
        "Ctrl+P: Screenshot\n"
        "+/-: Speed up/down";
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, help_title, help_content, &con);
  }
}