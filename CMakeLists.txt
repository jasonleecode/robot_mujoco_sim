cmake_minimum_required(VERSION 3.22)
project(MujocoDDSExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OSX_ARCHITECTURES arm64)

option(BUILD_MUJOCO_SAMPLES "Build MuJoCo sample console apps in /sample" OFF)
option(BUILD_MUJOCO_SIMULATE "Build the MuJoCo simulate UI in /simulate" OFF)

# === 关键修复 1: 强制指定 Homebrew 路径 (Mac M1/M2/M3 必备) ===
# 这会让 find_package 优先去这里找 glfw3 和 CycloneDDS
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

# === MuJoCo 设置 (DMG 版本) ===
# 假设你的目录结构是 /Users/jason/mujoco-3.3.7/mujoco.framework
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
find_package(glfw3 REQUIRED)
find_package(CycloneDDS REQUIRED)

# === 添加可执行文件 ===
add_executable(MujocoDDSExample
    src/main.cpp
    src/RobotSim.cpp
    src/RobotSim.hpp
    src/dds_generated/ControlMsg.c
)

# === 包含头文件 ===
# 1. MuJoCo Framework 的 Headers (必须显式包含，否则找不到 <mujoco/mujoco.h>)
target_include_directories(MujocoDDSExample PRIVATE "${MUJOCO_FRAMEWORK}/Headers")
# 2. CycloneDDS 和 GLFW (通过 target link 自动带入，但也防止万一)
target_include_directories(MujocoDDSExample PRIVATE ${CycloneDDS_INCLUDE_DIRS})
# 3. DDS generated headers
target_include_directories(MujocoDDSExample PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/dds_generated")

set(MUJOCO_PLATFORM_LIBS
    "${MUJOCO_FRAMEWORK}"   # 直接链接 Framework 文件夹
    glfw                    # 链接 GLFW (Homebrew 版)
    "-framework Accelerate"
    "-framework Metal"
    "-framework QuartzCore"
    "-framework Cocoa"      # GLFW 通常也需要 Cocoa
    "-framework IOKit"
    "-framework CoreVideo"
)

# === 链接库 ===
target_link_libraries(MujocoDDSExample PRIVATE ${MUJOCO_PLATFORM_LIBS} CycloneDDS::ddsc)

if(BUILD_MUJOCO_SAMPLES)
    add_subdirectory(sample)
endif()

if(BUILD_MUJOCO_SIMULATE)
    add_subdirectory(simulate)
endif()

add_subdirectory(tools)