#include "RobotSim.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>

RobotSim::RobotSim(const std::string &xml_path) {
    // 1. 加载模型 (这里需要处理路径，建议用绝对路径测试)
    char error[1000];
    m = mj_loadXML(xml_path.c_str(), 0, error, 1000);
    if (!m) throw std::runtime_error(error);
    d = mj_makeData(m);

    // 2. 初始化 GLFW (MuJoCo 不会自动做这个)
    if (!glfwInit()) throw std::runtime_error("GLFW Init Failed");
    window = glfwCreateWindow(1200, 900, "MuJoCo Sim", NULL, NULL);
    glfwMakeContextCurrent(window);

    // 3. 初始化 MuJoCo 渲染上下文
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 1. 将 'this' 指针存储到 GLFW 窗口的用户数据中 (Trampoline pattern)
    glfwSetWindowUserPointer(window, this);
    
    // 2. 注册鼠标和滚轮回调
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);
}

RobotSim::~RobotSim() {
    // 清理资源
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mj_deleteData(d);
    mj_deleteModel(m);
    
    // 关闭图形界面
    if (window) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

void RobotSim::setControl(int i, double val) {
    if (m && i >= 0 && i < m->nu) {
        d->ctrl[i] = val;
    }
}

void RobotSim::step() {
    mj_step(m, d);

    if (window && !glfwWindowShouldClose(window)) {
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

bool RobotSim::isWindowOpen() const {
    return window && !glfwWindowShouldClose(window);
}

// 鼠标按键回调
void RobotSim::mouse_button(GLFWwindow* window, int button, int action, int mods) {
    RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
    if (sim) sim->handle_mouse_button(button, action, mods);
}

// 鼠标移动回调
void RobotSim::mouse_move(GLFWwindow* window, double xpos, double ypos) {
    RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
    if (sim) sim->handle_mouse_move(xpos, ypos);
}

// 滚轮回调
void RobotSim::scroll(GLFWwindow* window, double xoffset, double yoffset) {
    RobotSim* sim = static_cast<RobotSim*>(glfwGetWindowUserPointer(window));
    if (sim) sim->handle_scroll(xoffset, yoffset);
}

// 记录鼠标按键状态
void RobotSim::handle_mouse_button(int button, int action, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

// 鼠标移动时计算相机移动
void RobotSim::handle_mouse_move(double xpos, double ypos) {
    // 仅在鼠标按住时才执行
    if (!button_left && !button_middle && !button_right) {
        lastx = xpos;
        lasty = ypos;
        return;
    }

    // 计算鼠标移动增量 (dx, dy)
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // 获取当前窗口大小 (用于归一化鼠标移动量)
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // MuJoCo 相机控制逻辑
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// 滚轮处理 (用于缩放)
void RobotSim::handle_scroll(double xoffset, double yoffset) {
    // 滚轮缩放 (Zoom)
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -yoffset * 0.05, &scn, &cam);
}

void RobotSim::applyBasicMotion(control::BasicMotion motion, std::vector<double>& control) const {
    control.assign(getNumActuators(), 0.0);
    const int total = static_cast<int>(control.size());
    if (total == 0) {
        return;
    }

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