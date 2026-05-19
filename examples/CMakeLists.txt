set(ROZETA_EXAMPLES
    motor_test
    gps_reader
    lidar_scan_console
    camera_capture
    odometry_test
    simple_robot_loop
    robotour_demo
)
foreach(example ${ROZETA_EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} PRIVATE rozeta_static)
  target_compile_options(${example} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
