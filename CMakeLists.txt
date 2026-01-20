cmake_minimum_required(VERSION 3.22)
project(RobotSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OSX_ARCHITECTURES arm64)
set(ENABLE_ROS OFF CACHE BOOL "Disable ROS for standalone build" FORCE)

option(BUILD_MUJOCO_SAMPLES "Build MuJoCo sample console apps in /sample" OFF)
option(BUILD_MUJOCO_SIMULATE "Build the mujoco_simulate executable (library is always built)" OFF)

# === 关键修复 1: 强制指定 Homebrew 路径 (Mac M1/M2/M3 必备) ===
# 这会让 find_package 优先去这里找 glfw3, CycloneDDS 和 OpenCV
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

# === MuJoCo 设置 (DMG 版本) ===
set(MUJOCO_DIR "/Users/jason/mujoco-3.3.7")
set(MUJOCO_FRAMEWORK "${MUJOCO_DIR}/mujoco.framework")
# 告诉 CMake 在构建阶段就把库的路径加到 RPATH 里
set(CMAKE_BUILD_RPATH "${MUJOCO_DIR}")
set(CMAKE_INSTALL_RPATH "${MUJOCO_DIR}")

# 这一行确保在 build 目录下运行时也使用 RPATH
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

# 自动处理 macOS 的 @rpath 机制
set(CMAKE_MACOSX_RPATH TRUE)

# 检查路径是否存在
if(NOT EXISTS "${MUJOCO_FRAMEWORK}")
    message(FATAL_ERROR "MuJoCo framework not found at: ${MUJOCO_FRAMEWORK}. Please check the path.")
endif()

# === 查找依赖 ===
# 1. 查找 OpenGL 包 (通常在文件前面位置)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(CycloneDDS REQUIRED)

# [修改点 1] 查找 OpenCV
find_package(OpenCV REQUIRED)
# 这一行其实在 find_package 后不是必须的，但在某些旧版 CMake 中有帮助，保留也可以，
# 但最好的做法是放在下面的 target_include_directories 中。
# include_directories(${OpenCV_INCLUDE_DIRS}) 

# 查找 ZeroMQ (用于 simulate 模块)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# 手动查找 ZeroMQ 库文件
find_library(ZMQ_LIBRARY zmq HINTS ${ZMQ_LIBRARY_DIRS})

# === 添加planner子模块 ===
add_subdirectory(planner)

# === 添加simulate子模块 ===
add_subdirectory(simulate)

# === 添加plugins子模块 ===
add_subdirectory(plugins)

# === 添加可执行文件 ===
add_executable(RobotSim
    src/main.cpp
    src/RobotSim.cpp
    src/RobotSim.hpp
    src/planner.cpp
    src/dds_generated/ControlMsg.c
)

# === 包含头文件 ===
target_include_directories(RobotSim PRIVATE 
    "${MUJOCO_FRAMEWORK}/Headers"           # MuJoCo Headers
    ${CycloneDDS_INCLUDE_DIRS}              # CycloneDDS Headers
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dds_generated"
    ${OpenCV_INCLUDE_DIRS}                  # [修改点 2] 将 OpenCV 头文件路径加入目标
    "${CMAKE_CURRENT_SOURCE_DIR}/simulate"  # Simulate Headers
    ${ZMQ_INCLUDE_DIRS}                     # ZeroMQ Headers
)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework Foundation -framework AppKit -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -framework Metal -framework QuartzCore -framework Accelerate -L/opt/homebrew/lib -lzmq")

set(MUJOCO_PLATFORM_LIBS
    ${MUJOCO_FRAMEWORK}
    /opt/homebrew/lib/libglfw.3.4.dylib
    -lobjc
)

# === 链接库 ===
target_link_libraries(RobotSim PRIVATE 
    ${MUJOCO_PLATFORM_LIBS} 
    CycloneDDS::ddsc
    ${OpenCV_LIBS} 
    planner            # Planner模块
    mujoco_simulate_lib # Simulate模块
    ${ZMQ_LIBRARY}     # ZeroMQ 库
)

if(BUILD_MUJOCO_SAMPLES)
    add_subdirectory(sample)
endif()

add_subdirectory(tools)
