cmake_minimum_required(VERSION 3.22)
project(planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. 选项控制 ---
# 默认关闭 ROS，除非在 ROS 环境中检测到了 ament_cmake
option(ENABLE_ROS "Enable ROS 2 support" OFF)

# 尝试自动检测 ROS 环境
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    set(ENABLE_ROS ON)
    message(STATUS "[Planner] ROS 2 environment detected. Building with ROS support.")
else()
    message(STATUS "[Planner] Building in Standalone Mode (No ROS dependencies).")
endif()

# --- 2. 查找通用依赖 ---
find_package(Eigen3 REQUIRED NO_MODULE)

# --- 3. 定义源文件 ---
# 这里我们把 Low_level 也加进来了！
# 因为我们在 C++ 代码里用了 #ifdef ENABLE_ROS，所以即使没有 ROS 也能编译
set(PLANNER_SOURCES
    # Quadruped 核心 (纯算法)
    src/Quadruped/Link.cpp
    src/Quadruped/Joint.cpp
    src/Quadruped/Leg.cpp
    src/Quadruped/Robot.cpp
    src/Quadruped/Kinematics.cpp
    
    # Low_level 步态逻辑 (现在兼容 Standalone 了)
    src/Low_level/StateMonitor.cpp
    src/Low_level/BaseGait.cpp
    src/Low_level/TrotGait.cpp
    src/Low_level/BodyMover.cpp
    src/Low_level/LegMover.cpp
    # 注意：不要包含带 main() 的 node 文件 (如 trot_gait_node.cpp)
)

# --- 4. 创建通用库 ---
# 无论是否有 ROS，这个库都会被创建
add_library(${PROJECT_NAME} STATIC ${PLANNER_SOURCES})

# 包含头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

# 链接 Eigen3
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

# --- 5. ROS 模式特有配置 ---
if(ENABLE_ROS)
    # 查找 ROS 包
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(go2_gait_planner_interfaces REQUIRED) # 你的消息包

    # 定义宏 ENABLE_ROS，让 C++ 代码知道现在要开启 ROS 功能
    target_compile_definitions(${PROJECT_NAME} PUBLIC ENABLE_ROS)

    # 链接 ROS 库
    ament_target_dependencies(${PROJECT_NAME} 
        rclcpp 
        std_msgs 
        go2_gait_planner_interfaces
    )

    # --- 构建 ROS 节点可执行文件 (仅在 ROS 模式下) ---
    # 这些是真正运行 ROS 节点的程序
    add_executable(trot_gait_node src/Low_level/trot_gait_node.cpp)
    target_link_libraries(trot_gait_node ${PROJECT_NAME})
    ament_target_dependencies(trot_gait_node rclcpp)

    # 安装规则 (ROS workspace 需要)
    install(TARGETS ${PROJECT_NAME} trot_gait_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
else()
    # --- Standalone 模式特有配置 ---
    # 只需要安装库文件
    install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
    )
endif()

# 安装头文件 (通用)
install(DIRECTORY include/ DESTINATION include)

# ROS 包导出配置 (仅 ROS 模式)
if(ENABLE_ROS)
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME})
    ament_package()
endif()