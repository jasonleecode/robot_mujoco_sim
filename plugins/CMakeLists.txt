cmake_minimum_required(VERSION 3.10)
project(mujoco_plugins)

# 设置插件输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)

# 使用与根目录相同的 MuJoCo 设置
set(MUJOCO_DIR "/Users/jason/mujoco-3.3.7")
set(MUJOCO_FRAMEWORK "${MUJOCO_DIR}/mujoco.framework")

# 检查路径是否存在
if(NOT EXISTS "${MUJOCO_FRAMEWORK}")
    message(FATAL_ERROR "MuJoCo framework not found at: ${MUJOCO_FRAMEWORK}. Please check the path.")
endif()

# 查找所有插件源文件
file(GLOB PLUGIN_SOURCES "*.cc" "*.cpp")

# 为每个插件源文件创建共享库
foreach(PLUGIN_SOURCE ${PLUGIN_SOURCES})
    get_filename_component(PLUGIN_NAME ${PLUGIN_SOURCE} NAME_WE)
    add_library(${PLUGIN_NAME} SHARED ${PLUGIN_SOURCE})
    target_include_directories(${PLUGIN_NAME} PRIVATE "${MUJOCO_FRAMEWORK}/Headers")
    target_link_libraries(${PLUGIN_NAME} PRIVATE ${MUJOCO_FRAMEWORK})
    set_target_properties(${PLUGIN_NAME} PROPERTIES
        PREFIX ""
        SUFFIX ".so"
    )
endforeach()