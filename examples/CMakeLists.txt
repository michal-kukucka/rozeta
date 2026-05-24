set(ROZETA_EXAMPLES
    motor_test
    gps_reader
    gps_serial_reader
    lidar_scan_console
    camera_capture
    odometry_test
    simple_robot_loop
    route_follower_demo
    imu_fusion_demo
    robotour_demo
)
if(ROZETA_WITH_SERIAL_MOTORS)
  list(APPEND ROZETA_EXAMPLES serial_motor_calibrate)
endif()
if(ROZETA_WITH_YDLIDAR)
  list(APPEND ROZETA_EXAMPLES ydlidar_scan_console)
endif()
foreach(example ${ROZETA_EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} PRIVATE rozeta_static)
  target_compile_options(${example} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
