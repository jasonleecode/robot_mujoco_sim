cmake_minimum_required(VERSION 3.22)
project(planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(Eigen3 REQUIRED NO_MODULE)

# 创建静态库
add_library(${PROJECT_NAME} STATIC
    # Quadruped核心代码（不依赖ROS2）
    src/Quadruped/Link.cpp
    src/Quadruped/Joint.cpp
    src/Quadruped/Leg.cpp
    src/Quadruped/Robot.cpp
    src/Quadruped/Kinematics.cpp
    # 注意：Low_level代码严重依赖ROS2，暂时不包含
    # 后续可以重构移除ROS2依赖后再添加
)

# 包含头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# 链接Eigen3
target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
)

# 安装规则（可选）
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
)
