#pragma once
#include <GLFW/glfw3.h> // 必须引用
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
    
    // Multiple profiler figures - use pointers to avoid stack overflow
    mjvFigure fig;            // Main gait analysis figure
    mjvFigure* figconstraint;  // Constraint profiler (heap allocated)
    mjvFigure* figcost;        // Cost profiler (heap allocated)
    mjvFigure* figtimer;       // Timer profiler (heap allocated)
    mjvFigure* figsize;        // Size profiler (heap allocated)
    mjrRect fig_rect = {0, 0, 0, 0};
    
    float plot_data[4][1000];      // 数据缓存: [0]是目标值, [1]是实际值
    int plot_idx = 0;              // 当前数据索引
    const int kPlotPoints = 1000;  // 窗口显示的采样点数
    
    // UI panels
    mjUI ui0;  // Left UI panel
    mjUI ui1;  // Right UI panel (for future joint/control sliders)
    mjuiState uistate;
    
    // UI state flags
    int ui0_enable = 1;
    int ui1_enable = 0;  // Right panel disabled by default
    int help = 0;        // F1 help overlay
    int info = 0;        // F2 info overlay
    int profiler = 0;    // F3 profiler figures
    int run = 1;         // Simulation running state (Space to pause)
    int vsync = 1;
    int spacing = 0;
    int color = 0;
    int font = 0;
    char info_title[512] = {0};
    char info_content[512] = {0};
    
    // Speed control
    float speed_multiplier = 1.0f;
    int real_time_index = 15;  // Middle of speed array
    static constexpr int kNumSpeedOptions = 31;
    // Speed percentages defined in .cpp file
    static const float percentRealTime[kNumSpeedOptions];
    
    // Screenshot
    std::atomic<int> screenshotrequest{0};
    
    RobotSim(const std::string &xml_path);
    ~RobotSim();
    void stepPhysics();
    void renderFrame();
    void applyControlVector(const std::vector<double>& control);
    void getState(RobotState& state) const;
    void setControl(int i, double val);
    int getNumActuators() const { return m ? m->nu : 0; }
    bool isWindowOpen() const;
    bool windowShouldClose() const;
    GLFWwindow* getWindow() const;
    void applyBasicMotion(control::BasicMotion motion, std::vector<double>& control) const;
    
    // 获取指定相机的 RGB 和 深度数据
    // rgb_output: 存储 RGB 数据 (大小 w * h * 3)
    // depth_output: 存储深度数据 (大小 w * h)
    void getCameraImages(const std::string& camera_name, int width, int height, 
                        std::vector<unsigned char>& rgb_output, 
                        std::vector<float>& depth_output);
    
    // UI initialization and update
    void initializeUI();
    void initializeProfilerFigures();
    void updateProfiler();
    void updateInfoText();
    
    // Input callbacks
    void keyCallback(int key, int scancode, int action, int mods);
    void handleUIInteraction();
    void takeScreenshot();


private:
    mjModel* m = nullptr;
    mjData* d = nullptr;
    
    // 手动管理的渲染变量
    GLFWwindow* window = nullptr;
    mjvCamera cam;
    mjvOption opt;
    mjvPerturb pert;  // For perturbations
    mjvScene scn;
    mjvScene scn_sensor;

    const int cam_w = 640;
    const int cam_h = 480;
    std::vector<unsigned char> cam_rgb_vec;
    std::vector<float> cam_depth_vec;
    
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0;
    double lasty = 0;
    mutable std::mutex sim_mutex;
    
    // Frame counter for profiler
    int frame_count = 0;

    // 静态回调函数 (GLFW 需要静态函数)
    static void mouse_button(GLFWwindow* window, int button, int action, int mods);
    static void mouse_move(GLFWwindow* window, double xpos, double ypos);
    static void scroll(GLFWwindow* window, double xoffset, double yoffset);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    void drawSensorOverlay(const mjrRect& viewport);
    
    // 内部处理函数 (访问类成员变量)
    void handle_mouse_button(int button, int action, int mods);
    void handle_mouse_move(double xpos, double ypos);
    void handle_scroll(double xoffset, double yoffset);
};