#include "RobotSim.hpp"
#include "SimulateUI.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>

// #include "lodepng.h"  // 如果你有这个文件用于截图

// 速度选项定义
const float RobotSim::percentRealTime[] = {100, 80, 66,  50,  40,  33,   25,   20,   16,
                                           13,  10, 8,   6,   5,   4,    3,    2.5,  2,
                                           1.5, 1,  0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0};

RobotSim::RobotSim(const std::string& xml_path) {
  // --- 1. 图表初始化 ---
  mjv_defaultFigure(&fig);
  strcpy(fig.title, "Gait Analysis");
  strcpy(fig.xlabel, "Time");
  fig.flg_extend = 1;
  fig.flg_barplot = 0;
  fig.flg_symmetric = 0;
  fig.linewidth = 2.0f;
  fig.gridsize[0] = 5;
  fig.gridsize[1] = 5;

  strcpy(fig.linename[0], "FR");
  strcpy(fig.linename[1], "FL");
  strcpy(fig.linename[2], "RR");
  strcpy(fig.linename[3], "RL");

  // 设置颜色 (红绿蓝黄)
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

  // 修改地面摩擦力
  int geom_id = mj_name2id(m, mjOBJ_GEOM, "floor");
  if (geom_id != -1) {
    m->geom_friction[geom_id * 3 + 0] = 1.0;
  }

  // --- 3. 初始化 GLFW ---
  if (!glfwInit())
    throw std::runtime_error("GLFW Init Failed");
  glfwWindowHint(GLFW_SAMPLES, 4);
  window = glfwCreateWindow(1800, 900, "MuJoCo Robot Sim", NULL, NULL);
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

void RobotSim::uiModify(mjUI* ui, mjuiState* state, mjrContext* con) {
  // 使用simulate库的标准UI修改函数
  SimulateUI::UiModify(ui, state, con);
}

// ---------------- UI 初始化 ----------------
void RobotSim::initializeUI() {
  // 清空结构体
  std::memset(&uistate, 0, sizeof(uistate));

  // 使用simulate库的标准主题初始化
  SimulateUI::InitializeTheme(&ui0, 1, 0);  // UI0: rectid=1, auxid=0
  SimulateUI::InitializeTheme(&ui1, 2, 1);  // UI1: rectid=2, auxid=1

  // ================= 左侧面板 (UI0) =================
  // 使用simulate库的标准UI定义

  // Section 1: Simulation - 使用标准定义（带运动控制按钮）
  mjui_add(&ui0, SimulateUI::SimulationSection::GetDefinition(&run, &time_scale));

  // Section 2: Physics - 使用标准定义
  mjui_add(&ui0, SimulateUI::PhysicsSection::GetDefinition(&check_gravity));

  // Section 3: Rendering - 使用标准定义（增强版）
  mjui_add(&ui0, SimulateUI::RenderingSection::GetDefinition(&opt));

  // Section 4: Visualization - 使用标准定义
  mjui_add(&ui0, SimulateUI::VisualizationSection::GetDefinition(&opt));

  // Section 5: Gait Analysis - 自定义图表区域（每个占位符约6px高度）
  SimulateUI::AddPlaceholders(&ui0, "Gait Analysis", 35);

  // 默认展开所有section
  for (int i = 0; i < ui0.nsect; i++) {
    ui0.sect[i].state = 1;
  }

  // ================= 右侧面板 (UI1) =================

  // Section 1: Joint Control - 使用工具函数添加滑块
  static mjuiDef defJoints[] = {{mjITEM_SECTION, "Joint Control", 2, nullptr, "AJ"}, {mjITEM_END}};
  mjui_add(&ui1, defJoints);
  SimulateUI::AddJointSliders(&ui1, m, d);

  // Section 2: Camera View - 相机预览区域（每个占位符约6px高度）
  SimulateUI::AddPlaceholders(&ui1, "Camera View", 45);

  // 默认展开所有section
  for (int i = 0; i < ui1.nsect; i++) {
    ui1.sect[i].state = 1;
  }

  // 应用UI修改
  uiModify(&ui0, &uistate, &con);
  uiModify(&ui1, &uistate, &con);
}

// ---------------- 物理步进 ----------------
void RobotSim::stepPhysics() {
  std::lock_guard<std::mutex> lock(sim_mutex);

  // 如果暂停且没有单步请求，则返回
  if (!run)
    return;

  if (!m || !d)
    return;

  // 应用重力开关逻辑
  // MuJoCo 通过 disableflags 控制特性，设置 mjDSBL_GRAVITY 位表示"禁用重力"
  if (check_gravity) {
    m->opt.disableflags &= ~mjDSBL_GRAVITY;  // 开启重力 (清除禁用位)
  } else {
    m->opt.disableflags |= mjDSBL_GRAVITY;  // 关闭重力 (设置禁用位)
  }

  // 执行多次步进以匹配 time_scale (简单实现)
  // 更好的做法是在 main loop 里控制
  mj_step(m, d);

  // 更新 IMU 缓存
  getIMUDataInternal(this->current_imu, "imu_");
}

void RobotSim::applyControlVector(const std::vector<double>& control) {
  std::lock_guard<std::mutex> lock(sim_mutex);
  if (!m || !d)
    return;
  const int count = std::min(static_cast<int>(control.size()), m->nu);
  for (int i = 0; i < count; ++i)
    d->ctrl[i] = control[i];
}

void RobotSim::getState(RobotState& state) const {
  std::lock_guard<std::mutex> lock(sim_mutex);
  state.time = d ? d->time : 0.0;
  if (d && m) {
    state.qpos.assign(d->qpos, d->qpos + m->nq);
    state.qvel.assign(d->qvel, d->qvel + m->nv);

    // 填充 IMU
    state.imu_quat.resize(4);
    state.imu_quat[0] = current_imu.orientation[0];
    state.imu_quat[1] = current_imu.orientation[1];
    state.imu_quat[2] = current_imu.orientation[2];
    state.imu_quat[3] = current_imu.orientation[3];

    state.imu_gyro.resize(3);
    state.imu_gyro[0] = current_imu.gyro[0];
    state.imu_gyro[1] = current_imu.gyro[1];
    state.imu_gyro[2] = current_imu.gyro[2];
  } else {
    state.qpos.clear();
    state.qvel.clear();
    state.imu_quat = {1, 0, 0, 0};
  }
}

// ---------------- 渲染帧 ----------------

void RobotSim::renderFrame() {
  if (!window || glfwWindowShouldClose(window))
    return;

  // 1. 物理同步与数据获取
  {
    std::lock_guard<std::mutex> lock(sim_mutex);
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    // 降低相机采集频率，减少锁持有时间
    if (++cam_frame_counter >= kCamCaptureInterval) {
      cam_frame_counter = 0;
      getCameraImages("tof_cam", cam_w, cam_h, cam_rgb_vec, cam_depth_vec);
    }
  }

  // 2. 布局更新
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  // 检测窗口大小变化
  if (viewport.width != prev_viewport_w || viewport.height != prev_viewport_h) {
    prev_viewport_w = viewport.width;
    prev_viewport_h = viewport.height;
    ui_dirty = true;
  }

  mjrRect rect_ui0 = {0, 0, 0, 0};
  mjrRect rect_ui1 = {0, 0, 0, 0};
  mjrRect rect_scene = viewport;

  if (ui0_enable) {
    rect_ui0.width = ui0.width;
    rect_ui0.height = viewport.height;
    rect_scene.left += ui0.width;
    rect_scene.width -= ui0.width;
  }
  if (ui1_enable) {
    rect_ui1.width = ui1.width;
    rect_ui1.left = viewport.width - ui1.width;
    rect_ui1.height = viewport.height;
    rect_scene.width -= ui1.width;
  }

  // rect[0] 必须是整个窗口大小（MuJoCo UI 系统依赖此作为参考）
  uistate.nrect = 4;
  uistate.rect[0] = viewport;
  uistate.rect[1] = rect_ui0;
  uistate.rect[2] = rect_ui1;
  uistate.rect[3] = rect_scene;

  // 布局重建只在需要时执行，mjui_update 每帧调用以刷新 aux buffer
  if (ui_dirty) {
    uiModify(&ui0, &uistate, &con);
    uiModify(&ui1, &uistate, &con);
    ui_dirty = false;
  } else {
    mjui_update(-1, -1, &ui0, &uistate, &con);
    mjui_update(-1, -1, &ui1, &uistate, &con);
  }

  // 3. 渲染场景
  mjr_render(rect_scene, &scn, &con);

  // 4. 渲染 UI 与嵌入内容
  // 重置 OpenGL viewport 到全窗口（mjr_render 会把 viewport 设为 rect_scene 区域）
  glViewport(0, 0, viewport.width, viewport.height);
  glScissor(0, 0, viewport.width, viewport.height);

  // --- UI0: Left panel with Gait Analysis graph ---
  if (ui0_enable) {
    mjui_render(&ui0, &uistate, &con);
    // 重置 OpenGL 状态（mjui_render 可能改变 viewport）
    glViewport(0, 0, viewport.width, viewport.height);
    glScissor(0, 0, viewport.width, viewport.height);

    // 在 section 内容区域绘制 Gait Analysis 图表
    int gait_sect = SimulateUI::SECT_GAIT_ANALYSIS;
    if (gait_sect < ui0.nsect && ui0.sect[gait_sect].state == 1) {
      mjrRect rc = ui0.sect[gait_sect].rcontent;
      int aux_to_screen = viewport.height - ui0.maxheight + ui0.scroll;
      int margin = 4;

      mjrRect graph_rect;
      graph_rect.left = rect_ui0.left + rc.left + margin;
      graph_rect.width = rc.width - 2 * margin;
      graph_rect.bottom = rc.bottom + aux_to_screen + margin;
      graph_rect.height = rc.height - 2 * margin;

      // 裁剪到面板可见区域
      int panel_bottom = rect_ui0.bottom;
      int panel_top = rect_ui0.bottom + rect_ui0.height;
      if (graph_rect.bottom < panel_bottom) {
        int clip = panel_bottom - graph_rect.bottom;
        graph_rect.bottom = panel_bottom;
        graph_rect.height -= clip;
      }
      if (graph_rect.bottom + graph_rect.height > panel_top) {
        graph_rect.height = panel_top - graph_rect.bottom;
      }

      if (graph_rect.width > 0 && graph_rect.height > 20) {
        fig.figurergba[0] = 0.0f;
        fig.figurergba[1] = 0.0f;
        fig.figurergba[2] = 0.0f;
        fig.figurergba[3] = 1.0f;
        fig.flg_extend = 1;
        fig.flg_barplot = 0;
        mjr_figure(graph_rect, &fig, &con);
      }
    }
  }

  // --- UI1: Right panel with Camera View ---
  if (ui1_enable) {
    mjui_render(&ui1, &uistate, &con);
    // 重置 OpenGL 状态
    glViewport(0, 0, viewport.width, viewport.height);
    glScissor(0, 0, viewport.width, viewport.height);

    // 在 section 内容区域绘制 Camera View
    int cam_sect = SimulateUI::SECT_CAMERA_VIEW;
    if (cam_sect < ui1.nsect && ui1.sect[cam_sect].state == 1) {
      mjrRect rc = ui1.sect[cam_sect].rcontent;
      int aux_to_screen = viewport.height - ui1.maxheight + ui1.scroll;
      int margin = 4;

      mjrRect img_rect;
      img_rect.left = rect_ui1.left + rc.left + margin;
      img_rect.width = rc.width - 2 * margin;
      img_rect.bottom = rc.bottom + aux_to_screen + margin;
      img_rect.height = rc.height - 2 * margin;

      // 裁剪到面板可见区域
      int panel_bottom = rect_ui1.bottom;
      int panel_top = rect_ui1.bottom + rect_ui1.height;
      if (img_rect.bottom < panel_bottom) {
        int clip = panel_bottom - img_rect.bottom;
        img_rect.bottom = panel_bottom;
        img_rect.height -= clip;
      }
      if (img_rect.bottom + img_rect.height > panel_top) {
        img_rect.height = panel_top - img_rect.bottom;
      }

      if (img_rect.width > 0 && img_rect.height > 20) {
        if (!cam_rgb_vec.empty() && !cam_depth_vec.empty()) {
          cv::Mat src_rgb(cam_h, cam_w, CV_8UC3, cam_rgb_vec.data());
          cv::Mat src_depth(cam_h, cam_w, CV_32F, cam_depth_vec.data());

          cv::Mat rgb_u, depth_u;
          cv::flip(src_rgb, rgb_u, 0);
          cv::flip(src_depth, depth_u, 0);

          // MuJoCo 深度缓冲区是非线性的 [0,1]，需要先转换为线性距离
          float znear = m->vis.map.znear * m->stat.extent;
          float zfar = m->vis.map.zfar * m->stat.extent;
          float depth_ratio = 1.0f - znear / zfar;

          cv::Mat depth_linear(depth_u.size(), CV_32F);
          for (int i = 0; i < (int)depth_u.total(); i++) {
            float d = depth_u.ptr<float>()[i];
            // d >= 1.0 表示背景/无穷远，设为 0 以便在伪彩色中显示为深蓝
            depth_linear.ptr<float>()[i] =
                (d >= 1.0f) ? 0.0f : znear / (1.0f - d * depth_ratio);
          }

          cv::Mat depth_n, depth_c;
          double max_depth;
          cv::minMaxLoc(depth_linear, nullptr, &max_depth);

          if (max_depth > 0.01) {
            depth_linear.convertTo(depth_n, CV_8U, 255.0 / max_depth);
            cv::applyColorMap(depth_n, depth_c, cv::COLORMAP_JET);
          } else {
            depth_c = cv::Mat::zeros(cam_h, cam_w, CV_8UC3);
          }

          cv::Mat rgb_bgr;
          cv::cvtColor(rgb_u, rgb_bgr, cv::COLOR_RGB2BGR);

          cv::Mat combo;
          cv::hconcat(std::vector<cv::Mat>{depth_c, rgb_bgr}, combo);

          cv::Mat final_img;
          cv::resize(combo, final_img, cv::Size(img_rect.width, img_rect.height));
          cv::cvtColor(final_img, final_img, cv::COLOR_BGR2RGB);
          cv::flip(final_img, final_img, 0);

          mjr_drawPixels(final_img.data, nullptr, img_rect, &con);
        } else {
          mjr_rectangle(img_rect, 0.3f, 0.0f, 0.0f, 1.0f);
        }
      }
    }
  }

  // 5. Info 覆盖
  if (info) {
    updateInfoText();
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect_scene, info_title, info_content, &con);
  }

  glfwSwapBuffers(window);
  glfwPollEvents();
}

void RobotSim::updateInfoText() {
  std::snprintf(info_title, sizeof(info_title), "RobotSim Info");
  double time_val = d ? d->time : 0.0;

  // 计算 RPY
  double w = current_imu.orientation[0];
  double x = current_imu.orientation[1];
  double y = current_imu.orientation[2];
  double z = current_imu.orientation[3];
  double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  double pitch = std::asin(std::max(-1.0, std::min(1.0, 2 * (w * y - z * x))));
  double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

  std::snprintf(info_content, sizeof(info_content), "Time: %.3f s\nRPY: %.2f %.2f %.2f\nRun: %d",
                time_val, roll, pitch, yaw, run);
}

// ---------------- 事件处理 ----------------

void RobotSim::handle_mouse_button(int button, int action, int mods) {
  // 1. 更新按键状态
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // 2. 更新 uistate（需要缩放到 framebuffer 坐标并翻转 Y 轴）
  double x, y;
  glfwGetCursorPos(window, &x, &y);

  int fb_w, fb_h, win_w, win_h;
  glfwGetFramebufferSize(window, &fb_w, &fb_h);
  glfwGetWindowSize(window, &win_w, &win_h);
  double ratio = (win_w > 0) ? static_cast<double>(fb_w) / win_w : 1.0;
  x *= ratio;
  y *= ratio;
  y = fb_h - y;  // 翻转 Y 轴匹配 OpenGL 坐标系

  uistate.x = x;
  uistate.y = y;

  // 更新鼠标所在区域
  uistate.mouserect = mjr_findRect(static_cast<int>(x), static_cast<int>(y),
                                    uistate.nrect - 1, uistate.rect + 1) + 1;

  uistate.type = (action == GLFW_PRESS) ? mjEVENT_PRESS : mjEVENT_RELEASE;
  // 映射按键
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    uistate.button = mjBUTTON_LEFT;
  else if (button == GLFW_MOUSE_BUTTON_RIGHT)
    uistate.button = mjBUTTON_RIGHT;
  else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    uistate.button = mjBUTTON_MIDDLE;

  // 3. 尝试传递给 UI0
  if (ui0_enable) {
    mjuiItem* it = mjui_event(&ui0, &uistate, &con);
    if (it) {
      ui_dirty = true;
      // 处理特殊按钮事件
      if (strcmp(it->name, "Reset") == 0) {
        std::lock_guard<std::mutex> lock(sim_mutex);
        mj_resetData(m, d);
        mj_forward(m, d);
        // 重置时间
        plot_idx = 0;
      } else if (strcmp(it->name, "Forward") == 0) {
        // 前进按钮
        if (motion_callback) {
          motion_callback(1);  // 1 = forward
        }
      } else if (strcmp(it->name, "Stop") == 0) {
        // 停止按钮
        if (motion_callback) {
          motion_callback(0);  // 0 = stop
        }
      }
      return;  // UI 处理了事件，不传给 3D
    }
  }

  // 4. 尝试传递给 UI1
  if (ui1_enable) {
    mjuiItem* it = mjui_event(&ui1, &uistate, &con);
    if (it) {
      ui_dirty = true;
      return;
    }
  }

  // 5. 3D 场景交互 (相机控制)
  if (action == GLFW_PRESS) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      // 点击图表扩展（x, y 已经是 OpenGL 坐标，无需再翻转）
      mjrRect r = fig_rect;
      if (x >= r.left && x <= r.left + r.width &&
          y >= r.bottom && y <= r.bottom + r.height) {
        if (button_middle)
          fig.flg_extend = !fig.flg_extend;
      }
    }
    lastx = x;
    lasty = y;
  }
}

void RobotSim::handle_mouse_move(double xpos, double ypos) {
  // 缩放到 framebuffer 坐标并翻转 Y 轴
  int fb_w, fb_h, win_w, win_h;
  glfwGetFramebufferSize(window, &fb_w, &fb_h);
  glfwGetWindowSize(window, &win_w, &win_h);
  double ratio = (win_w > 0) ? static_cast<double>(fb_w) / win_w : 1.0;
  double x = xpos * ratio;
  double y = fb_h - ypos * ratio;  // 翻转 Y 轴

  // 更新 uistate
  uistate.dx = x - uistate.x;
  uistate.dy = y - uistate.y;
  uistate.x = x;
  uistate.y = y;

  // 更新鼠标所在区域
  uistate.mouserect = mjr_findRect(static_cast<int>(x), static_cast<int>(y),
                                    uistate.nrect - 1, uistate.rect + 1) + 1;

  uistate.type = mjEVENT_MOVE;

  // 传递给 UI (处理拖拽滑块等)
  if (ui0_enable)
    mjui_event(&ui0, &uistate, &con);
  if (ui1_enable)
    mjui_event(&ui1, &uistate, &con);

  // 如果鼠标在 UI 面板上，不处理 3D 移动
  if (uistate.mouserect == 1 || uistate.mouserect == 2)
    return;

  // 只在 scene 区域（rect[3]）或默认区域（0）时处理 3D 交互
  if (uistate.mouserect != 0 && uistate.mouserect != 3)
    return;

  // 3D 相机移动
  if (!button_left && !button_right && !button_middle)
    return;

  // 使用 framebuffer 坐标计算 delta（与 handle_mouse_button 中 lastx/lasty 一致）
  double dx = x - lastx;
  double dy = y - lasty;
  lastx = x;
  lasty = y;

  int height = fb_h;

  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS);
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
  uistate.type = mjEVENT_SCROLL;
  uistate.sx = xoffset;
  uistate.sy = yoffset;

  if (ui0_enable)
    mjui_event(&ui0, &uistate, &con);
  if (ui1_enable)
    mjui_event(&ui1, &uistate, &con);

  if (uistate.mouserect == 1 || uistate.mouserect == 2)
    return;

  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void RobotSim::handle_keyboard(int key, int scancode, int act, int mods) {
  if (act != GLFW_PRESS)
    return;

  switch (key) {
    case GLFW_KEY_F1:
      ui0_enable = !ui0_enable;
      ui_dirty = true;
      break;
    case GLFW_KEY_F2:
      ui1_enable = !ui1_enable;
      ui_dirty = true;
      break;
    case GLFW_KEY_F5:
      info = !info;
      break;
    case GLFW_KEY_SPACE:
      run = !run;
      ui_dirty = true;
      break;
    case GLFW_KEY_BACKSPACE: {
      std::lock_guard<std::mutex> lock(sim_mutex);
      mj_resetData(m, d);
      mj_forward(m, d);
    } break;
  }
}

// 辅助函数
void RobotSim::updatePlotData(const std::vector<double>& qref) {
  if (qref.size() < 12)
    return;
  std::lock_guard<std::mutex> lock(sim_mutex);
  int p_idx = plot_idx % kPlotPoints;
  plot_data[0][p_idx] = (float)qref[2];
  plot_data[1][p_idx] = (float)qref[5];
  plot_data[2][p_idx] = (float)qref[8];
  plot_data[3][p_idx] = (float)qref[11];
  plot_idx++;
  for (int k = 0; k < kPlotPoints; k++) {
    int history_idx = (plot_idx + k) % kPlotPoints;
    for (int line = 0; line < 4; ++line) {
      fig.linedata[line][2 * k] = (float)k;
      fig.linedata[line][2 * k + 1] = plot_data[line][history_idx];
    }
  }
}

void RobotSim::getIMUDataInternal(IMUData& data, const std::string& prefix) {
  if (!m || !d)
    return;
  std::string name_quat = prefix + "quat";
  int id_quat = mj_name2id(m, mjOBJ_SENSOR, name_quat.c_str());
  if (id_quat != -1) {
    int adr = m->sensor_adr[id_quat];
    data.orientation[0] = d->sensordata[adr + 0];
    data.orientation[1] = d->sensordata[adr + 1];
    data.orientation[2] = d->sensordata[adr + 2];
    data.orientation[3] = d->sensordata[adr + 3];
  }
  // ... gyro, accel logic ...
}

void RobotSim::getIMUData(IMUData& data, const std::string& prefix) {
  std::lock_guard<std::mutex> lock(sim_mutex);
  getIMUDataInternal(data, prefix);
}

// 静态回调转接
void RobotSim::mouse_button(GLFWwindow* w, int b, int a, int m) {
  static_cast<RobotSim*>(glfwGetWindowUserPointer(w))->handle_mouse_button(b, a, m);
}
void RobotSim::mouse_move(GLFWwindow* w, double x, double y) {
  static_cast<RobotSim*>(glfwGetWindowUserPointer(w))->handle_mouse_move(x, y);
}
void RobotSim::scroll(GLFWwindow* w, double x, double y) {
  static_cast<RobotSim*>(glfwGetWindowUserPointer(w))->handle_scroll(x, y);
}
void RobotSim::keyboard(GLFWwindow* w, int k, int s, int a, int m) {
  static_cast<RobotSim*>(glfwGetWindowUserPointer(w))->handle_keyboard(k, s, a, m);
}

// ... IsWindowOpen 等其他函数保持不变 ...
bool RobotSim::isWindowOpen() const {
  return window && !glfwWindowShouldClose(window);
}
bool RobotSim::windowShouldClose() const {
  return !window || glfwWindowShouldClose(window);
}
GLFWwindow* RobotSim::getWindow() const {
  return window;
}
void RobotSim::setControl(int i, double val) {
  std::lock_guard<std::mutex> lock(sim_mutex);
  if (m && i >= 0 && i < m->nu)
    d->ctrl[i] = val;
}
int RobotSim::getNumActuators() const {
  return m ? m->nu : 0;
}
void RobotSim::takeScreenshot() {
  screenshotrequest.store(1);
}

void RobotSim::getCameraImages(const std::string& camera_name, int width, int height,
                               std::vector<unsigned char>& rgb_output,
                               std::vector<float>& depth_output) {
  // 1. 查找相机 ID
  int cam_id = mj_name2id(m, mjOBJ_CAMERA, camera_name.c_str());
  if (cam_id == -1)
    return;  // 没找到相机，直接返回 (UI会显示红块)

  // 2. 检查离屏缓冲区大小
  // 如果默认的离屏缓冲区比请求的小，可能会出问题。
  // MuJoCo 默认创建的 offscreen buffer 通常足够大 (如 800x600 或更大)，但为了稳健，
  // 我们这里仅在尺寸不匹配时打印警告或简单使用现有大小。
  // 更好的做法是在初始化时就分配足够大的 Context，或者在这里不做检查直接画。

  // 3. 设置相机参数
  mjvCamera sensor_cam;
  mjv_defaultCamera(&sensor_cam);
  sensor_cam.type = mjCAMERA_FIXED;
  sensor_cam.fixedcamid = cam_id;

  // 4. 更新传感器场景 (使用独立的 scn_sensor，不影响主视图)
  mjv_updateScene(m, d, &opt, NULL, &sensor_cam, mjCAT_ALL, &scn_sensor);

  // 5. 渲染到离屏缓冲区
  // 切换 OpenGL 目标到 Offscreen Buffer
  mjr_setBuffer(mjFB_OFFSCREEN, &con);

  mjrRect viewport = {0, 0, width, height};
  mjr_render(viewport, &scn_sensor, &con);

  // 6. 读取像素数据
  rgb_output.resize(width * height * 3);
  depth_output.resize(width * height);

  // MuJoCo 读取的是底部优先 (Bottom-Left)，需要在显示时翻转 (RenderFrame里已经做了)
  mjr_readPixels(rgb_output.data(), depth_output.data(), viewport, &con);

  // 7. 恢复到窗口缓冲区 (关键！否则主界面会黑屏)
  mjr_setBuffer(mjFB_WINDOW, &con);
}