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
    field_operator_wizard
    buchlovice_telemetry_converter
)
set(ROZETA_HARDWARE_OPTIONAL_EXAMPLES)
if(ROZETA_WITH_SERIAL_MOTORS)
  list(APPEND ROZETA_HARDWARE_OPTIONAL_EXAMPLES serial_motor_calibrate)
endif()
if(ROZETA_WITH_YDLIDAR)
  list(APPEND ROZETA_HARDWARE_OPTIONAL_EXAMPLES ydlidar_scan_console)
endif()
if(ROZETA_WITH_LDROBOT_LIDAR)
  list(APPEND ROZETA_HARDWARE_OPTIONAL_EXAMPLES ldrobot_lidar_scan_console)
endif()
list(APPEND ROZETA_EXAMPLES ${ROZETA_HARDWARE_OPTIONAL_EXAMPLES})
foreach(example ${ROZETA_EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} PRIVATE rozeta_static)
  rozeta_apply_warnings(${example})
endforeach()

add_executable(c_api_smoke c_api_smoke.c)
target_link_libraries(c_api_smoke PRIVATE rozeta_static)
rozeta_apply_warnings(c_api_smoke)
