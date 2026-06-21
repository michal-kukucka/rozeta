set(ROZETA_EXAMPLES
    motor_test
    gps_reader
    gps_serial_reader
    gps_network_reader
    lidar_scan_console
    camera_capture
    odometry_test
    simple_robot_loop
    route_follower_demo
    imu_fusion_demo
    robotour_demo
    replay_robotour_log
    replay_ui_snapshots
    mission_ui_dashboard
    mission_runtime_demo
    robotour_buchlovice_demo
    buchlovice_graph_route
    depth_obstacle_console
    hardware_smoke_matrix
    buchlovice_telemetry_converter
)
if(ROZETA_WITH_SERIAL_MOTORS)
  list(APPEND ROZETA_EXAMPLES serial_motor_calibrate)
endif()
if(ROZETA_WITH_YDLIDAR)
  list(APPEND ROZETA_EXAMPLES ydlidar_scan_console)
endif()
if(ROZETA_WITH_LDROBOT_LIDAR)
  list(APPEND ROZETA_EXAMPLES ldrobot_lidar_scan_console)
endif()
foreach(example ${ROZETA_EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} PRIVATE rozeta_static)
  target_compile_options(${example} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

add_executable(c_api_smoke c_api_smoke.c)
target_link_libraries(c_api_smoke PRIVATE rozeta_static)
target_compile_options(c_api_smoke PRIVATE -Wall -Wextra -Wpedantic)
