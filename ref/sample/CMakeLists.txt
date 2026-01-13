set(SAMPLE_STANDALONE OFF)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(MujocoMacOS OPTIONAL)
if(COMMAND enforce_mujoco_macosx_min_version)
  enforce_mujoco_macosx_min_version()
endif()

function(add_mujoco_sample source_file)
  get_filename_component(sample_name "${source_file}" NAME_WE)
  set(target_name "sample_${sample_name}")
  add_executable(${target_name} "${source_file}")
  target_include_directories(${target_name} PRIVATE
    "${MUJOCO_FRAMEWORK}/Headers"
    "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_link_libraries(${target_name} PRIVATE ${MUJOCO_PLATFORM_LIBS})
endfunction()

add_mujoco_sample(basic.cc)
add_mujoco_sample(compile.cc)
add_mujoco_sample(dependencies.cc)
add_mujoco_sample(record.cc)
add_mujoco_sample(testspeed.cc)

