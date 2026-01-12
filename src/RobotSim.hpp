#pragma once
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <mutex>
#include <string>
#include <vector>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "ControlActions.hpp"
#include "planner.h"

class RobotSim {
public:
    mjrContext con;
    
    // 图表相关数据
    mjvFigure fig;            
    mjrRect fig_rect = {0, 0, 0, 0};
    
    // 数据缓存: [0-3]对应四条腿的关节数据
    float plot_data[4][1000];      
    int plot_idx = 0;              
    const int kPlotPoints = 1000;  
    
    // UI 状态
    mjUI ui0;
    mjUI ui1;
    mjuiState uistate;
    int ui0_enable = 0;
    int ui1_enable = 0;
    int help = 0;
    int info = 0;
    int profiler = 0;
    int run = 1;
    int vsync = 1;
    int spacing = 0;
    int color = 0;
    int font = 0;
    char info_title[512] = {0};
    char info_content[512] = {0};
    
    // 速度控制 (UI用)
    float speed_multiplier = 1.0f;
    int real_time_index = 15;
    static constexpr int kNumSpeedOptions = 31;
    static const float percentRealTime[kNumSpeedOptions];
    
    // 截图请求
    std::atomic<int> screenshotrequest{0};
    
    RobotSim(const std::string &xml_path);
    ~RobotSim();

    // --- 物理与控制接口 (主要由物理线程调用) ---
    void stepPhysics();
    void applyControlVector(const std::vector<double>& control);
    void getState(RobotState& state) const;
    void setControl(int i, double val);
    int getNumActuators() const;
    
    // [新增] 线程安全的绘图数据更新
    void updatePlotData(const std::vector<double>& qref);

    // --- 渲染与窗口接口 (主要由主线程调用) ---
    void renderFrame(); 
    bool isWindowOpen() const;
    bool windowShouldClose() const;
    GLFWwindow* getWindow() const;
    
    // 辅助功能
    void applyBasicMotion(control::BasicMotion motion, std::vector<double>& control) const;
    void getCameraImages(const std::string& camera_name, int width, int height, 
                        std::vector<unsigned char>& rgb_output, 
                        std::vector<float>& depth_output);
    
    void initializeUI();
    void updateInfoText();
    void takeScreenshot();

    // 回调封装
    void mouse_button(int button, int action, int mods);
    void mouse_move(double xpos, double ypos);
    void scroll(double xoffset, double yoffset);

private:
    mjModel* m = nullptr;
    mjData* d = nullptr;
    
    GLFWwindow* window = nullptr;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjvScene scn_sensor;
    mjvPerturb pert;

    const int cam_w = 640;
    const int cam_h = 480;
    std::vector<unsigned char> cam_rgb_vec;
    std::vector<float> cam_depth_vec;
    
    // 交互状态
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0;
    double lasty = 0;
    
    // 核心互斥锁：保护 mjData 和 plot_data
    mutable std::mutex sim_mutex;
    
    // 静态回调
    static void mouse_button(GLFWwindow* window, int button, int action, int mods);
    static void mouse_move(GLFWwindow* window, double xpos, double ypos);
    static void scroll(GLFWwindow* window, double xoffset, double yoffset);
    static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

    // 内部实现
    void drawSensorOverlay(const mjrRect& viewport);
    void handle_mouse_button(int button, int action, int mods);
    void handle_mouse_move(double xpos, double ypos);
    void handle_scroll(double xoffset, double yoffset);
    void handle_keyboard(int key, int scancode, int act, int mods);
    void renderUI(const mjrRect& viewport);
};