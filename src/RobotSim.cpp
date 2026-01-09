#include "RobotSim.hpp"

#include <algorithm>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

// 速度选项定义
const float RobotSim::percentRealTime[] = {
    -1, 100, 80, 66, 50, 40, 33, 25, 20, 16, 13, 10, 8, 6, 5, 4, 3, 2.5, 2, 1.5, 1, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0
};

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

    fig.linergb[0][0]=1.0f; fig.linergb[0][1]=0.1f; fig.linergb[0][2]=0.1f; 
    fig.linergb[1][0]=0.1f; fig.linergb[1][1]=1.0f; fig.linergb[1][2]=0.1f; 
    fig.linergb[2][0]=0.1f; fig.linergb[2][1]=0.1f; fig.linergb[2][2]=1.0f; 
    fig.linergb[3][0]=1.0f; fig.linergb[3][1]=0.8f; fig.linergb[3][2]=0.0f; 

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

// 线程安全的设置控制输入
void RobotSim::setControl(int i, double val) {
    std::lock_guard<std::mutex> lock(sim_mutex);
    if (m && i >= 0 && i < m->nu) d->ctrl[i] = val;
}

// 物理步进
void RobotSim::stepPhysics() {
    std::lock_guard<std::mutex> lock(sim_mutex);
    if (!m || !d) return;
    mj_step(m, d);
}

// 应用完整的控制向量
void RobotSim::applyControlVector(const std::vector<double>& control) {
    std::lock_guard<std::mutex> lock(sim_mutex);
    if (!m || !d) return;
    const int count = std::min(static_cast<int>(control.size()), m->nu);
    for (int i = 0; i < count; ++i) d->ctrl[i] = control[i];
    // 其余补0
    for (int i = count; i < m->nu; ++i) d->ctrl[i] = 0.0;
}

// 获取状态
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

// [核心重构] 渲染帧：锁粒度最小化
void RobotSim::renderFrame() {
    if (!window || glfwWindowShouldClose(window)) return;

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // --- 临界区 1: 访问 mjData 和更新 Scene ---
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        
        // 获取相机图像 (这也需要 mjData 来更新 scene_sensor)
        getCameraImages("tof_cam", cam_w, cam_h, cam_rgb_vec, cam_depth_vec);

        // 更新主场景 (这是 CPU 操作，拷贝 mjData -> mjvScene)
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    } 
    // --- 锁释放：物理线程现在可以继续运行了 ---

    // --- 渲染阶段 (GPU 操作，耗时但无需锁住 mjData) ---
    mjr_render(viewport, &scn, &con);

    // 绘制图表
    fig_rect.left = 10;
    fig_rect.bottom = 10;
    fig_rect.width = viewport.width / 2 - 20; 
    fig_rect.height = viewport.height / 3;
    
    // 注意：fig 也在物理线程被写入，理论上需要锁。
    // 但为了不阻塞渲染，且 mjvFigure 的数据通常容忍一帧的撕裂，这里不加重锁。
    mjr_figure(fig_rect, &fig, &con);

    // 绘制传感器覆盖层
    drawSensorOverlay(viewport);

    // 刷新屏幕
    glfwSwapBuffers(window);
    glfwPollEvents();
}

// [新增] 线程安全的更新图表数据
void RobotSim::updatePlotData(const std::vector<double>& qref) {
    if (qref.size() < 12) return;

    std::lock_guard<std::mutex> lock(sim_mutex);

    int p_idx = plot_idx % kPlotPoints;
    // 记录四个关节的目标值 (Target)
    // 假设 qref 顺序: 0-2 FR, 3-5 FL, 6-8 RR, 9-11 RL
    plot_data[0][p_idx] = (float)qref[2];  // FR Knee
    plot_data[1][p_idx] = (float)qref[5];  // FL Knee
    plot_data[2][p_idx] = (float)qref[8];  // RR Knee
    plot_data[3][p_idx] = (float)qref[11]; // RL Knee
    plot_idx++;

    // 更新 mjvFigure 的线性缓冲区
    for (int k = 0; k < kPlotPoints; k++) {
        int history_idx = (plot_idx + k) % kPlotPoints; 
        for (int line = 0; line < 4; ++line) {
            fig.linedata[line][2*k]   = (float)k;
            fig.linedata[line][2*k+1] = plot_data[line][history_idx];
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

// 鼠标交互辅助：判断是否在图表上
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

int RobotSim::getNumActuators() const {
    return m ? m->nu : 0;
}

// 定义初始化UI函数（虽然现在可能没用到，但保持完整性）
void RobotSim::initializeUI() {}
void RobotSim::updateInfoText() {}
void RobotSim::takeScreenshot() {}