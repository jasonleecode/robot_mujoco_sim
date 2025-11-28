set(SIMULATE_STANDALONE OFF)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(MujocoMacOS OPTIONAL)
if(COMMAND enforce_mujoco_macosx_min_version)
  enforce_mujoco_macosx_min_version()
endif()

set(SIMULATE_SOURCES
  glfw_adapter.cc
  glfw_dispatch.cc
  platform_ui_adapter.cc
  simulate.cc
  main.cc
  lodepng.cpp
  glfw_corevideo.mm
  macos_gui.mm
)

add_executable(mujoco_simulate ${SIMULATE_SOURCES})
target_include_directories(mujoco_simulate PRIVATE
  "${MUJOCO_FRAMEWORK}/Headers"
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(mujoco_simulate PRIVATE ${MUJOCO_PLATFORM_LIBS})

