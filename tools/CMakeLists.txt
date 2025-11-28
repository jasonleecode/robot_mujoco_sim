add_executable(dds_control_client dds_control_client.cpp)
target_include_directories(dds_control_client PRIVATE
    "${CMAKE_SOURCE_DIR}/src/dds_generated"
    "${CMAKE_SOURCE_DIR}/src"
)
target_sources(dds_control_client PRIVATE "${CMAKE_SOURCE_DIR}/src/dds_generated/ControlMsg.c")
target_link_libraries(dds_control_client PRIVATE CycloneDDS::ddsc)

