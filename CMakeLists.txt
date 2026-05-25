cmake_minimum_required(VERSION 3.22)
project(RobotSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ENABLE_ROS OFF CACHE BOOL "Disable ROS for standalone build" FORCE)

option(BUILD_MUJOCO_SAMPLES "Build MuJoCo sample console apps in /sample" OFF)
option(BUILD_MUJOCO_SIMULATE "Build the mujoco_simulate executable (library is always built)" OFF)

# DDS 选项：macOS 默认开启，Linux 默认关闭
if(APPLE)
  option(USE_DDS "Enable CycloneDDS control interface" ON)
else()
  option(USE_DDS "Enable CycloneDDS control interface" OFF)
endif()

# LibTorch 选项：启用基于模型的策略控制器推理
option(USE_TORCH "Enable LibTorch policy inference backend" OFF)

# === MuJoCo 路径设置 (平台自适应) ===
if(APPLE)
  list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
  set(CMAKE_OSX_ARCHITECTURES arm64)
  set(CMAKE_MACOSX_RPATH TRUE)

  set(MUJOCO_DIR "/Users/jason/mujoco-3.3.7")
  set(MUJOCO_FRAMEWORK "${MUJOCO_DIR}/mujoco.framework")
  if(NOT EXISTS "${MUJOCO_FRAMEWORK}")
    message(FATAL_ERROR "MuJoCo framework not found at: ${MUJOCO_FRAMEWORK}")
  endif()
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_FRAMEWORK}/Headers")

  set(CMAKE_BUILD_RPATH "${MUJOCO_DIR}")
  set(CMAKE_INSTALL_RPATH "${MUJOCO_DIR}")

  set(MUJOCO_PLATFORM_LIBS
      ${MUJOCO_FRAMEWORK}
      /opt/homebrew/lib/libglfw.3.4.dylib
      -lobjc
  )
else()
  # Linux: 标准共享库
  set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.6.0")
  if(NOT EXISTS "${MUJOCO_DIR}")
    message(FATAL_ERROR "MuJoCo not found at: ${MUJOCO_DIR}. Please install MuJoCo 3.6.0 to ~/.mujoco/")
  endif()
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
  set(MUJOCO_LIBRARY "${MUJOCO_DIR}/lib/libmujoco.so")

  set(CMAKE_BUILD_RPATH "${MUJOCO_DIR}/lib")
  set(CMAKE_INSTALL_RPATH "${MUJOCO_DIR}/lib")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

  set(MUJOCO_PLATFORM_LIBS
      ${MUJOCO_LIBRARY}
      glfw
      OpenGL::GL
      ${CMAKE_DL_LIBS}
  )
endif()

# === 查找依赖 ===
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# CycloneDDS (仅在 USE_DDS 时查找)
if(USE_DDS)
  find_package(CycloneDDS REQUIRED)
  message(STATUS "CycloneDDS enabled")
else()
  message(STATUS "CycloneDDS disabled (USE_DDS=OFF)")
endif()

# 查找 OpenCV
if(APPLE)
  find_package(OpenCV QUIET PATHS /opt/homebrew/opt/opencv NO_DEFAULT_PATH)
  if(NOT OpenCV_FOUND)
    find_package(OpenCV REQUIRED)
  endif()
  message(STATUS "OpenCV found: ${OpenCV_VERSION}")
else()
  # Ubuntu apt 安装 libopencv-{core,imgproc,imgcodecs}-dev 时不提供 OpenCVConfig.cmake，
  # 直接手动指定路径
  set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
  set(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs)
  message(STATUS "OpenCV: using manual paths at ${OpenCV_INCLUDE_DIRS}")
endif()

# 查找 ZeroMQ
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_library(ZMQ_LIBRARY zmq HINTS ${ZMQ_LIBRARY_DIRS})

# LibTorch (optional)
if(USE_TORCH)
  # 优先从 Python 环境自动定位 LibTorch cmake 前缀
  # 手动指定时（-DTorch_DIR 或 CMake 缓存中已有 Torch_DIR）跳过自动探测
  if(NOT Torch_DIR AND NOT DEFINED CACHE{Torch_DIR})
    execute_process(
      COMMAND python3 -c "import torch; print(torch.utils.cmake_prefix_path)"
      OUTPUT_VARIABLE TORCH_CMAKE_PREFIX
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(TORCH_CMAKE_PREFIX)
      list(APPEND CMAKE_PREFIX_PATH "${TORCH_CMAKE_PREFIX}")
      message(STATUS "LibTorch cmake prefix (from Python): ${TORCH_CMAKE_PREFIX}")
    endif()
  else()
    message(STATUS "LibTorch: using manually specified Torch_DIR=${Torch_DIR}")
  endif()
  find_package(Torch REQUIRED)
  message(STATUS "LibTorch found: ${TORCH_VERSION}")
  add_definitions(-DUSE_TORCH)
endif()

# === 添加子模块 ===
add_subdirectory(planner)
add_subdirectory(simulate)
# add_subdirectory(plugins)

# === 主可执行文件 ===
set(ROBOT_SIM_SOURCES
    src/main.cpp
    src/RobotSim.cpp
    src/RobotSim.hpp
    src/planner.cpp
    src/policy_controller.cpp
    src/robot_config.cpp
)
if(USE_DDS)
  list(APPEND ROBOT_SIM_SOURCES src/dds_generated/ControlMsg.c)
endif()
add_executable(RobotSim ${ROBOT_SIM_SOURCES})

# === 头文件路径 ===
target_include_directories(RobotSim PRIVATE
    "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dds_generated"
    ${OpenCV_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/simulate"
    ${ZMQ_INCLUDE_DIRS}
)

if(USE_DDS)
  target_include_directories(RobotSim PRIVATE ${CycloneDDS_INCLUDE_DIRS})
  target_compile_definitions(RobotSim PRIVATE USE_DDS)
endif()

# === 平台链接标志 ===
if(APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework Foundation -framework AppKit -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -framework Metal -framework QuartzCore -framework Accelerate -L/opt/homebrew/lib -lzmq")
endif()

# === 链接库 ===
find_package(Threads REQUIRED)
target_link_libraries(RobotSim PRIVATE
    ${MUJOCO_PLATFORM_LIBS}
    ${OpenCV_LIBS}
    planner
    mujoco_simulate_lib
    ${ZMQ_LIBRARY}
    Threads::Threads
    Eigen3::Eigen
)

if(USE_DDS)
  target_link_libraries(RobotSim PRIVATE CycloneDDS::ddsc)
endif()

if(USE_TORCH)
  target_link_libraries(RobotSim PRIVATE ${TORCH_LIBRARIES})
  # LibTorch requires C++14 at minimum; our project is already C++17
  set_property(TARGET RobotSim PROPERTY CXX_STANDARD 17)
endif()

if(BUILD_MUJOCO_SAMPLES)
  add_subdirectory(sample)
endif()

add_subdirectory(tools)

# === Headless planner test (no GLFW/MuJoCo required) ===
add_executable(PlannerTest
    src/planner_test.cpp
    src/planner.cpp
)
target_include_directories(PlannerTest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(PlannerTest PRIVATE
    planner
    Eigen3::Eigen
    Threads::Threads
)
target_compile_definitions(PlannerTest PRIVATE PLANNER_TEST_HEADLESS=1)

# === Headless MuJoCo physics simulation (no GLFW window) ===
add_executable(HeadlessSim
    src/headless_sim.cpp
    src/planner.cpp
    src/robot_config.cpp
)
target_include_directories(HeadlessSim PRIVATE
    "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${ZMQ_INCLUDE_DIRS}
)
target_link_libraries(HeadlessSim PRIVATE
    ${MUJOCO_LIBRARY}
    planner
    Eigen3::Eigen
    Threads::Threads
    ${ZMQ_LIBRARY}
)
