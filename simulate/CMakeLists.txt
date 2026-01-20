set(SIMULATE_STANDALONE OFF)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(MujocoMacOS OPTIONAL)
if(COMMAND enforce_mujoco_macosx_min_version)
  enforce_mujoco_macosx_min_version()
endif()

find_package(glfw3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

set(SIMULATE_SOURCES
  glfw_adapter.cc
  glfw_dispatch.cc
  platform_ui_adapter.cc
  simulate.cc
  lodepng.cpp
  glfw_corevideo.mm
  macos_gui.mm
)

# 总是构建库
add_library(mujoco_simulate_lib STATIC ${SIMULATE_SOURCES})
target_include_directories(mujoco_simulate_lib PUBLIC
  "${MUJOCO_FRAMEWORK}/Headers"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "/opt/homebrew/include"
  ${ZMQ_INCLUDE_DIRS}
)
target_link_libraries(mujoco_simulate_lib PUBLIC ${MUJOCO_PLATFORM_LIBS} ${ZMQ_LIBRARIES})
target_compile_definitions(mujoco_simulate_lib PRIVATE SIMULATE_AS_LIBRARY)

# 条件构建可执行文件（如果BUILD_MUJOCO_SIMULATE为ON）
if(BUILD_MUJOCO_SIMULATE)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -F/Users/jason/mujoco-3.3.7 -framework mujoco /opt/homebrew/lib/libglfw.3.4.dylib -L/opt/homebrew/lib -lzmq -lobjc -framework Foundation -framework AppKit -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -framework Metal -framework QuartzCore -framework Accelerate")
  add_executable(mujoco_simulate main.cc ${SIMULATE_SOURCES})
  target_include_directories(mujoco_simulate PRIVATE
    "${MUJOCO_FRAMEWORK}/Headers"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "/opt/homebrew/include"
    ${ZMQ_INCLUDE_DIRS}
  )
  target_link_libraries(mujoco_simulate PRIVATE ${MUJOCO_PLATFORM_LIBS} ${ZMQ_LIBRARIES})
endif()

