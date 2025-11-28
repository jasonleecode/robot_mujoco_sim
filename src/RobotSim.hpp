#pragma once
#include <GLFW/glfw3.h> // 必须引用
#include <mujoco/mujoco.h>
#include <string>
#include <vector>

#include "ControlActions.hpp"

class RobotSim {
public:
    RobotSim(const std::string &xml_path);
    ~RobotSim();
    void step();
    void setControl(int i, double val);
    int getNumActuators() const { return m ? m->nu : 0; }
    bool isWindowOpen() const; // 新增：判断窗口是否开启
    void applyBasicMotion(control::BasicMotion motion, std::vector<double>& control) const;

private:
    mjModel* m = nullptr;
    mjData* d = nullptr;
    
    // 手动管理的渲染变量
    GLFWwindow* window = nullptr;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0;
    double lasty = 0;

    // 静态回调函数 (GLFW 需要静态函数)
    static void mouse_button(GLFWwindow* window, int button, int action, int mods);
    static void mouse_move(GLFWwindow* window, double xpos, double ypos);
    static void scroll(GLFWwindow* window, double xoffset, double yoffset);
    
    // 内部处理函数 (访问类成员变量)
    void handle_mouse_button(int button, int action, int mods);
    void handle_mouse_move(double xpos, double ypos);
    void handle_scroll(double xoffset, double yoffset);
};