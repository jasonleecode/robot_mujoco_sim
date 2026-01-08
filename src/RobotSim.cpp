#include "RobotSim.hpp"

#include <algorithm>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

RobotSim::RobotSim(const std::string &xml_path) {
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

    fig.linergb[0][0]=1.0f; fig.linergb[0][1]=0.1f; fig.linergb[0][2]=0.1f; // 红
    fig.linergb[1][0]=0.1f; fig.linergb[1][1]=1.0f; fig.linergb[1][2]=0.1f; // 绿
    fig.linergb[2][0]=0.1f; fig.linergb[2][1]=0.1f; fig.linergb[2][2]=1.0f; // 蓝
    fig.linergb[3][0]=1.0f; fig.linergb[3][1]=0.8f; fig.linergb[3][2]=0.0f; // 黄

    for(int i=0; i<4; ++i) fig.linepnt[i] = kPlotPoints;

    // --- 2. 加载模型 ---
    char error[1000];
    m = mj_loadXML(xml_path.c_str(), 0, error, 1000);
    if (!m) throw std::runtime_error(error);
    d = mj_makeData(m);

    // --- 3. 初始化 GLFW ---
    if (!glfwInit()) throw std::runtime_error("GLFW Init Failed");
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
}

RobotSim::~RobotSim() {
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mjv_freeScene(&scn_sensor);
    mj_deleteData(d);
    mj_deleteModel(m);
    if (window) glfwDestroyWindow(window);
    glfwTerminate();
}

void RobotSim::setControl(int i, double val) {
    std::lock_guard<std::mutex> lock(sim_mutex);
    if (m && i >= 0 && i < m->nu) d->ctrl[i] = val;
}

void RobotSim::stepPhysics() {
    std::lock_guard<std::mutex> lock(sim_mutex);
    if (!m || !d) return;
    mj_step(m, d);
}

GLFWwindow* RobotSim::getWindow() const {
    return window;
}

// [核心逻辑] 渲染一帧：获取相机 -> 画主场景 -> 画UI
void RobotSim::renderFrame() {
    std::lock_guard<std::mutex> lock(sim_mutex);
    if (!window || glfwWindowShouldClose(window)) return;

    // 1. 获取窗口大小
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // 2. 【关键】先获取相机数据
    // 这会在后台渲染一次 ToF 相机视角，并读取像素到 buffer
    // 注意：这会改变 scn 的状态，所以必须在画主场景之前做
    getCameraImages("tof_cam", cam_w, cam_h, cam_rgb_vec, cam_depth_vec);

    // 3. 渲染主场景 (Main View)
    // 恢复主相机的场景状态
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    
    // 5. 绘制波形图表 (Bottom-Left)
    fig_rect.left = 10;
    fig_rect.bottom = 10;
    fig_rect.width = viewport.width / 2 - 20; 
    fig_rect.height = viewport.height / 3;
    mjr_figure(fig_rect, &fig, &con);

    drawSensorOverlay(viewport);

    // 6. 刷新屏幕
    glfwSwapBuffers(window);
    glfwPollEvents();
}

// [src/RobotSim.cpp]

void RobotSim::drawSensorOverlay(const mjrRect& viewport) {
    // 定义显示大小和位置
    int disp_w = viewport.width / 2 - 20;
    int disp_h = viewport.height / 3; 
    
    mjrRect img_rect;
    img_rect.width = disp_w;
    img_rect.height = disp_h;
    img_rect.left = viewport.width - disp_w - 10; 
    img_rect.bottom = 10; 

    // --- 情况 A: 没有数据 (相机未找到或初始化失败) ---
    if (cam_rgb_vec.empty() || cam_depth_vec.empty()) {
        // 创建一个灰色的“无信号”图像 (BGR格式)
        cv::Mat empty_img(disp_h, disp_w, CV_8UC3, cv::Scalar(50, 50, 50)); // 深灰色背景
        
        // 在上面写字 "NO SIGNAL"
        // 这里的文字是画在图片里的，不是 MuJoCo 的 Overlay 文字
        cv::putText(empty_img, "NO SIGNAL (Check Camera Name)", 
                    cv::Point(disp_w/4, disp_h/2), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);

        // 转换为 MuJoCo 格式 (BGR -> RGB, Flip)
        cv::Mat final_overlay;
        cv::cvtColor(empty_img, final_overlay, cv::COLOR_BGR2RGB); 
        cv::flip(final_overlay, final_overlay, 0); 

        // 绘制占位图
        mjr_drawPixels(final_overlay.data, nullptr, img_rect, &con);
        return;
    }
    
    // 1. 准备原始数据
    cv::Mat img_rgb(cam_h, cam_w, CV_8UC3, cam_rgb_vec.data());
    cv::Mat img_depth(cam_h, cam_w, CV_32F, cam_depth_vec.data());
    
    cv::Mat rgb_upright, depth_upright;
    cv::flip(img_rgb, rgb_upright, 0);   
    cv::flip(img_depth, depth_upright, 0);

    // 2. 深度图处理
    cv::Mat depth_norm, depth_color;
    double max_dist = 10.0;
    depth_upright.convertTo(depth_norm, CV_8U, 255.0 / max_dist);
    cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET); 

    // 3. RGB图处理
    cv::Mat rgb_bgr;
    cv::cvtColor(rgb_upright, rgb_bgr, cv::COLOR_RGB2BGR);

    // 4. 拼接
    cv::Mat combined;
    cv::hconcat(std::vector<cv::Mat>{depth_color, rgb_bgr}, combined);

    // 5. 缩放
    cv::Mat resized_overlay;
    cv::resize(combined, resized_overlay, cv::Size(disp_w, disp_h));

    // 6. 最终转换
    cv::Mat final_overlay;
    cv::cvtColor(resized_overlay, final_overlay, cv::COLOR_BGR2RGB); 
    cv::flip(final_overlay, final_overlay, 0); 

    // 7. 绘制
    mjr_drawPixels(final_overlay.data, nullptr, img_rect, &con);
    
    // (可选) 画一个白色边框让它更明显
    mjr_rectangle(img_rect, 1.0f, 1.0f, 1.0f, 0.0f); // 实心矩形覆盖了... MuJoCo没有画空心矩形的简单API，mjr_drawPixels最直接
}

void RobotSim::applyControlVector(const std::vector<double>& control) {
    std::lock_guard<std::mutex> lock(sim_mutex);
    if (!m || !d) return;
    const int count = std::min(static_cast<int>(control.size()), m->nu);
    for (int i = 0; i < count; ++i) d->ctrl[i] = control[i];
    for (int i = count; i < m->nu; ++i) d->ctrl[i] = 0.0;
}

void RobotSim::getState(RobotState& state) const {
    std::lock_guard<std::mutex> lock(sim_mutex);
    state.time = d ? d->time : 0.0;
    if (d && m) {
        state.qpos.assign(d->qpos, d->qpos + m->nq);
        state.qvel.assign(d->qvel, d->qvel + m->nv);
    } else {
        state.qpos.clear();
        state.qvel.clear();
    }
}

bool RobotSim::isWindowOpen() const {
    std::lock_guard<std::mutex> lock(sim_mutex);
    return window && !glfwWindowShouldClose(window);
}

bool RobotSim::windowShouldClose() const {
    std::lock_guard<std::mutex> lock(sim_mutex);
    return !window || glfwWindowShouldClose(window);
}

void RobotSim::mouse_button(GLFWwindow* window, int button, int action, int mods) {
    RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
    if (sim) sim->handle_mouse_button(button, action, mods);
}
void RobotSim::mouse_move(GLFWwindow* window, double xpos, double ypos) {
    RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
    if (sim) sim->handle_mouse_move(xpos, ypos);
}
void RobotSim::scroll(GLFWwindow* window, double xoffset, double yoffset) {
    RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
    if (sim) sim->handle_scroll(xoffset, yoffset);
}

// 辅助函数：判断鼠标是否在图表上
static bool is_mouse_over_figure(double x, double y, const mjrRect& r, int win_h) {
    double gl_y = win_h - y; 
    return x >= r.left && x <= r.left + r.width &&
           gl_y >= r.bottom && gl_y <= r.bottom + r.height;
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
            fig.range[0][0] += pan_x; fig.range[1][0] += pan_x;
            fig.range[0][1] += pan_y; fig.range[1][1] += pan_y;
        }
        lastx = xpos; lasty = ypos;
        return; 
    } else {
        fig.highlight[0] = -1;
    }

    if (!button_left && !button_middle && !button_right) {
        lastx = xpos; lasty = ypos; return;
    }
    double dx = xpos - lastx; double dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if (button_right) action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left) action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else action = mjMOUSE_ZOOM;
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
    if (total == 0) return;

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
            apply_range(0, half, kStride); apply_range(half, total, -kStride); break;
        case control::BasicMotion::kBackward:
            apply_range(0, half, -kStride); apply_range(half, total, kStride); break;
        case control::BasicMotion::kTurnLeft:
            apply_range(0, half, -kTurn); apply_range(half, total, kTurn); break;
        case control::BasicMotion::kTurnRight:
            apply_range(0, half, kTurn); apply_range(half, total, -kTurn); break;
        case control::BasicMotion::kCrouch:
            std::fill(control.begin(), control.end(), kCrouch); break;
        case control::BasicMotion::kStand:
            std::fill(control.begin(), control.end(), 0.0); break;
        case control::BasicMotion::kJump:
            apply_range(0, quarter, kJump);
            apply_range(quarter, total - quarter, kJump * 0.8);
            apply_range(total - quarter, total, kJump);
            break;
        case control::BasicMotion::kCount: default: break;
    }
}

float zbuffer_to_meters(float depth_val, float znear, float zfar) {
    return znear / (1.0f - depth_val * (1.0f - znear / zfar));
}

void RobotSim::getCameraImages(const std::string& camera_name, int width, int height, 
                               std::vector<unsigned char>& rgb_output, 
                               std::vector<float>& depth_output) {
    int cam_id = mj_name2id(m, mjOBJ_CAMERA, camera_name.c_str());
    if (cam_id == -1) {
        static bool warned = false;
        if (!warned) {
            std::cerr << "Warning: Camera '" << camera_name << "' not found in XML! Overlay will not show.\n";
            warned = true;
        }
        return; 
    }


    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = cam_id;

    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn_sensor);

    mjrRect viewport = {0, 0, width, height};
    mjr_render(viewport, &scn_sensor, &con);

    rgb_output.resize(width * height * 3);
    depth_output.resize(width * height);

    mjr_readPixels(rgb_output.data(), depth_output.data(), viewport, &con);

    float znear = m->vis.map.znear;
    float zfar = m->vis.map.zfar;
    for (size_t i = 0; i < depth_output.size(); ++i) {
        depth_output[i] = zbuffer_to_meters(depth_output[i], znear, zfar);
    }
}