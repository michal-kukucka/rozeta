add_executable(rozeta_tests
    test_main.cpp
    test_geometry.cpp
    test_geodesy.cpp
    test_kinematics.cpp
    test_map_graph.cpp
    test_simulation.cpp
    test_geo_navigation.cpp
    test_simulation_scenarios.cpp
    test_ui_scene.cpp
    test_gps.cpp
    test_gps_receiver.cpp
    test_network_gps.cpp
    test_odometry.cpp
    test_coordinates.cpp
    test_logging.cpp
    test_obstacles.cpp
    test_motors.cpp
    test_config.cpp
    test_motor_calibration.cpp
    test_calibration.cpp
    test_hardware_smoke.cpp
    test_maps.cpp
    test_navigation.cpp
    test_runtime.cpp
    test_mission.cpp
    test_imu.cpp
    test_camera.cpp
    test_perception.cpp
    test_telemetry.cpp
    test_ui.cpp
    test_kinect_profile.cpp
    test_obstacle_behavior.cpp
    test_robotour_mission.cpp
    test_operator_io.cpp
    test_telemetry_mission.cpp
    test_m14_m15.cpp
    test_safety.cpp
    test_field_runner.cpp
    test_robotour_config.cpp
    test_health.cpp
    test_gps_gate.cpp
    test_safety_state.cpp
    test_faults.cpp
)
if(ROZETA_PLATFORM_POSIX)
  target_sources(rozeta_tests PRIVATE test_serial_port.cpp)
  target_compile_definitions(rozeta_tests PRIVATE ROZETA_PLATFORM_POSIX=1)
elseif(ROZETA_PLATFORM_WINDOWS)
  target_sources(rozeta_tests PRIVATE test_serial_port_win32.cpp)
  target_compile_definitions(rozeta_tests PRIVATE ROZETA_PLATFORM_WINDOWS=1)
endif()
if(ROZETA_WITH_SERIAL_MOTORS)
  target_sources(rozeta_tests PRIVATE test_serial_motors.cpp)
endif()
if(ROZETA_WITH_YDLIDAR)
  target_sources(rozeta_tests PRIVATE test_ydlidar_parser.cpp)
endif()
if(ROZETA_WITH_LDROBOT_LIDAR)
  target_sources(rozeta_tests PRIVATE test_ldrobot_lidar_parser.cpp)
endif()
function(rozeta_label_test test_name)
  set_tests_properties(${test_name} PROPERTIES LABELS "${ARGN}")
endfunction()

target_link_libraries(rozeta_tests PRIVATE rozeta_static)
target_include_directories(rozeta_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
rozeta_apply_warnings(rozeta_tests)
add_test(NAME rozeta_tests COMMAND rozeta_tests)
add_test(NAME rozeta_serial_tests COMMAND rozeta_tests)
rozeta_label_test(rozeta_tests unit portable)
if(ROZETA_PLATFORM_MACOS)
  rozeta_label_test(rozeta_serial_tests unit portable posix macos)
elseif(ROZETA_PLATFORM_POSIX)
  rozeta_label_test(rozeta_serial_tests unit portable posix)
elseif(ROZETA_PLATFORM_WINDOWS)
  rozeta_label_test(rozeta_serial_tests unit portable windows)
endif()

add_executable(rozeta_c_api_smoke c_api_smoke.c)
target_link_libraries(rozeta_c_api_smoke PRIVATE rozeta_static)
rozeta_apply_warnings(rozeta_c_api_smoke)
add_test(NAME rozeta_c_api_smoke COMMAND rozeta_c_api_smoke)
rozeta_label_test(rozeta_c_api_smoke unit portable)

add_executable(rozeta_optional_backend_header_smoke optional_backend_header_smoke.cpp)
target_link_libraries(rozeta_optional_backend_header_smoke PRIVATE rozeta_static)
target_compile_definitions(rozeta_optional_backend_header_smoke
    PRIVATE ROZETA_WITH_OPENCV ROZETA_WITH_KINECT ROZETA_WITH_LDROBOT_LIDAR)
rozeta_apply_warnings(rozeta_optional_backend_header_smoke)
add_test(NAME rozeta_optional_backend_header_smoke COMMAND rozeta_optional_backend_header_smoke)
rozeta_label_test(rozeta_optional_backend_header_smoke unit portable hardware-optional)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(NAME rozeta_docs_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/verify_docs.py)
  rozeta_label_test(rozeta_docs_contract portable)
  add_test(
      NAME rozeta_cmake_portability_contract
      COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_SOURCE_DIR}/tests/test_cmake_portability_contract.py)
  rozeta_label_test(rozeta_cmake_portability_contract portable)
  add_test(
      NAME rozeta_serial_transport_split_contract
      COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_SOURCE_DIR}/tests/test_serial_transport_split_contract.py)
  rozeta_label_test(rozeta_serial_transport_split_contract portable posix windows)
  add_test(
      NAME rozeta_socket_transport_split_contract
      COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_SOURCE_DIR}/tests/test_socket_transport_split_contract.py)
  rozeta_label_test(rozeta_socket_transport_split_contract portable posix windows)
  add_test(
      NAME rozeta_platform_aware_ctest_contract
      COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_SOURCE_DIR}/tests/test_platform_aware_ctest_contract.py)
  rozeta_label_test(rozeta_platform_aware_ctest_contract portable posix windows)
  add_test(
      NAME rozeta_dll_export_contract
      COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_SOURCE_DIR}/tests/test_dll_export_contract.py)
  rozeta_label_test(rozeta_dll_export_contract portable windows)
  add_test(
      NAME rozeta_windows_ci_matrix_contract
      COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_SOURCE_DIR}/tests/test_windows_ci_matrix_contract.py)
  rozeta_label_test(rozeta_windows_ci_matrix_contract portable windows)
  add_test(
      NAME rozeta_release_readiness_contract
      COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_SOURCE_DIR}/tests/test_release_readiness_contract.py)
  rozeta_label_test(rozeta_release_readiness_contract portable posix windows)
  add_test(
      NAME rozeta_arduino_bridge_contract
      COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_SOURCE_DIR}/tests/test_arduino_bridge_contract.py)
  rozeta_label_test(rozeta_arduino_bridge_contract portable)
  add_test(
      NAME rozeta_osm_import_tool
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/test_osm_import_tool.py)
  rozeta_label_test(rozeta_osm_import_tool portable)
endif()

if(Python3_Interpreter_FOUND)
  add_test(
      NAME rozeta_opencv_qr_stub_smoke
      COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_SOURCE_DIR}/scripts/smoke_opencv_qr_stub.py
          ${CMAKE_CXX_COMPILER})
  rozeta_label_test(rozeta_opencv_qr_stub_smoke portable hardware-optional)
endif()

if(TARGET robot_simulator)
  # Headless end-to-end simulation. Each case fails the build if the robot does
  # not finish the run, so a regression in planning, kinematics, the sensor
  # models or the follower shows up as a red test rather than a slower demo.
  set(ROZETA_SIM_CATALOG ${CMAKE_SOURCE_DIR}/data/maps/maps.json)
  add_test(
      NAME rozeta_simulator_manual
      COMMAND robot_simulator --mode manual --catalog ${ROZETA_SIM_CATALOG} --quiet)
  rozeta_label_test(rozeta_simulator_manual simulation portable)
  add_test(
      NAME rozeta_simulator_plan
      COMMAND robot_simulator --mode plan --catalog ${ROZETA_SIM_CATALOG} --map castle_park)
  rozeta_label_test(rozeta_simulator_plan simulation portable)
  add_test(
      NAME rozeta_simulator_follow
      COMMAND robot_simulator --mode follow --catalog ${ROZETA_SIM_CATALOG} --map castle_park
              --seed 20260815 --quiet --log-every 0
              --svg ${CMAKE_CURRENT_BINARY_DIR}/simulator_run.svg)
  rozeta_label_test(rozeta_simulator_follow simulation portable)
  add_test(
      NAME rozeta_simulator_follow_with_lidar
      COMMAND robot_simulator --mode follow --catalog ${ROZETA_SIM_CATALOG} --map castle_park
              --corridor 3.0 --seed 4242 --quiet --log-every 0)
  rozeta_label_test(rozeta_simulator_follow_with_lidar simulation portable)
  add_test(
      NAME rozeta_simulator_follow_village
      COMMAND robot_simulator --mode follow --catalog ${ROZETA_SIM_CATALOG} --map village
              --seed 7 --quiet --log-every 0)
  rozeta_label_test(rozeta_simulator_follow_village simulation portable)
  # Failure paths must be loud: an unknown map, an unroutable destination and a
  # tick budget that cannot be met all have to exit non-zero.
  add_test(
      NAME rozeta_simulator_unknown_map_fails
      COMMAND robot_simulator --mode plan --catalog ${ROZETA_SIM_CATALOG} --map no_such_place)
  set_tests_properties(rozeta_simulator_unknown_map_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(rozeta_simulator_unknown_map_fails simulation portable)
  add_test(
      NAME rozeta_simulator_unreachable_goal_fails
      COMMAND robot_simulator --mode plan --catalog ${ROZETA_SIM_CATALOG} --map castle_park
              --goal 0.5,0.5)
  set_tests_properties(rozeta_simulator_unreachable_goal_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(rozeta_simulator_unreachable_goal_fails simulation portable)
  add_test(
      NAME rozeta_simulator_tick_budget_fails
      COMMAND robot_simulator --mode follow --catalog ${ROZETA_SIM_CATALOG} --map castle_park
              --max-ticks 5 --quiet --log-every 0)
  set_tests_properties(rozeta_simulator_tick_budget_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(rozeta_simulator_tick_budget_fails simulation portable)
  add_test(
      NAME rozeta_simulator_bad_arguments_fail
      COMMAND robot_simulator --mode nonsense)
  set_tests_properties(rozeta_simulator_bad_arguments_fail PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(rozeta_simulator_bad_arguments_fail simulation portable)
endif()

if(TARGET field_operator_wizard)
  add_test(
      NAME field_operator_wizard_script_complete
      COMMAND field_operator_wizard --script continue,continue,continue,continue)
  rozeta_label_test(field_operator_wizard_script_complete portable)
  add_test(
      NAME field_operator_wizard_script_incomplete_fails
      COMMAND field_operator_wizard --script continue)
  set_tests_properties(field_operator_wizard_script_incomplete_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(field_operator_wizard_script_incomplete_fails portable)
  add_test(
      NAME field_operator_wizard_script_abort_fails
      COMMAND field_operator_wizard --script quit)
  set_tests_properties(field_operator_wizard_script_abort_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(field_operator_wizard_script_abort_fails portable)
  add_test(
      NAME field_operator_wizard_script_unknown_fails
      COMMAND field_operator_wizard --script nope)
  set_tests_properties(field_operator_wizard_script_unknown_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(field_operator_wizard_script_unknown_fails portable)
  if(Python3_Interpreter_FOUND)
    add_test(
        NAME field_operator_wizard_cli_contract
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/test_field_operator_wizard_cli.py
            $<TARGET_FILE:field_operator_wizard>)
    rozeta_label_test(field_operator_wizard_cli_contract portable)
  endif()
endif()

# The live window's key map and click unprojection cannot be reached by running
# the application, so they get their own binary that pushes SDL events.
if(ROZETA_WITH_SDL2)
  add_executable(rozeta_view_input_tests test_simulator_view_input.cpp)
  target_link_libraries(rozeta_view_input_tests PRIVATE rozeta_static)
  target_include_directories(rozeta_view_input_tests PRIVATE ${CMAKE_SOURCE_DIR}/examples)
  target_compile_definitions(rozeta_view_input_tests PRIVATE ROZETA_WITH_SDL2)
  if(TARGET SDL2::SDL2)
    target_link_libraries(rozeta_view_input_tests PRIVATE SDL2::SDL2)
  else()
    target_include_directories(rozeta_view_input_tests PRIVATE ${SDL2_INCLUDE_DIR})
    target_link_libraries(rozeta_view_input_tests PRIVATE ${SDL2_LIBRARY})
  endif()
  rozeta_apply_warnings(rozeta_view_input_tests)
  add_test(NAME rozeta_view_input COMMAND rozeta_view_input_tests)
  rozeta_label_test(rozeta_view_input portable hardware-optional)
endif()

if(TARGET robotour_app)
  # End-to-end runs of the configurable application. Each case exercises a
  # different way of configuring it — command line, preset file, mission mode —
  # so a regression in the configuration layer or in the assembled stack fails
  # the build rather than surfacing on a robot.
  set(ROZETA_APP_CATALOG ${CMAKE_SOURCE_DIR}/data/maps/maps.json)
  set(ROZETA_APP_PRESETS ${CMAKE_SOURCE_DIR}/examples/presets)

  add_test(
      NAME robotour_app_list_keys
      COMMAND robotour_app --list-keys)
  rozeta_label_test(robotour_app_list_keys portable)

  add_test(
      NAME robotour_app_dry_run
      COMMAND robotour_app --set map.catalog=${ROZETA_APP_CATALOG}
              --set map.id=castle_park --dry-run --set app.quiet=true)
  rozeta_label_test(robotour_app_dry_run portable simulation)

  # A dry run on a hardware preset must open no device and must report the
  # missing E-STOP, so running it on an unpowered robot is safe and honest.
  add_test(
      NAME robotour_app_hardware_dry_run_requires_estop
      COMMAND robotour_app --base buchlovice --set map.catalog=${ROZETA_APP_CATALOG}
              --dry-run --set app.quiet=true)
  set_tests_properties(robotour_app_hardware_dry_run_requires_estop
      PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(robotour_app_hardware_dry_run_requires_estop portable)

  add_test(
      NAME robotour_app_follow_castle_park
      COMMAND robotour_app --set map.catalog=${ROZETA_APP_CATALOG}
              --set map.id=castle_park --set app.log_every=0
              --set app.svg=${CMAKE_CURRENT_BINARY_DIR}/robotour_app_run.svg
              --set app.telemetry_csv=${CMAKE_CURRENT_BINARY_DIR}/robotour_app_ticks.csv)
  rozeta_label_test(robotour_app_follow_castle_park portable simulation)

  add_test(
      NAME robotour_app_follow_with_lidar
      COMMAND robotour_app --set map.catalog=${ROZETA_APP_CATALOG}
              --set map.id=castle_park --set sim.corridor_half_width_m=2.5
              --set app.log_every=0 --set app.quiet=true)
  rozeta_label_test(robotour_app_follow_with_lidar portable simulation)

  add_test(
      NAME robotour_app_preset_file
      COMMAND robotour_app --preset ${ROZETA_APP_PRESETS}/prague_stromovka.preset
              --set map.catalog=${ROZETA_APP_CATALOG} --set app.log_every=0
              --set app.svg=${CMAKE_CURRENT_BINARY_DIR}/robotour_app_prague.svg)
  rozeta_label_test(robotour_app_preset_file portable simulation)

  # Three planned legs with an acknowledgement between them: the transport task
  # Robotour actually sets, not a point-to-point drive.
  add_test(
      NAME robotour_app_mission_legs
      COMMAND robotour_app --preset ${ROZETA_APP_PRESETS}/prague_mission.preset
              --set map.catalog=${ROZETA_APP_CATALOG} --set app.log_every=0
              --set app.svg=${CMAKE_CURRENT_BINARY_DIR}/robotour_app_mission.svg
              --set app.event_log=${CMAKE_CURRENT_BINARY_DIR}/robotour_app_events.csv)
  rozeta_label_test(robotour_app_mission_legs portable simulation)

  # An operator gate with no window to release it must not hang a headless run.
  add_test(
      NAME robotour_app_headless_ignores_operator_gate
      COMMAND robotour_app --set map.catalog=${ROZETA_APP_CATALOG}
              --set map.id=castle_park --set app.auto_start=false
              --set app.log_every=0 --set app.quiet=true)
  rozeta_label_test(robotour_app_headless_ignores_operator_gate portable simulation)

  add_test(
      NAME robotour_app_unknown_key_fails
      COMMAND robotour_app --set no.such.key=1)
  set_tests_properties(robotour_app_unknown_key_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(robotour_app_unknown_key_fails portable)

  add_test(
      NAME robotour_app_bad_value_fails
      COMMAND robotour_app --set follower.cruise_speed=hurry)
  set_tests_properties(robotour_app_bad_value_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(robotour_app_bad_value_fails portable)

  add_test(
      NAME robotour_app_out_of_range_value_fails
      COMMAND robotour_app --set follower.cruise_speed=3)
  set_tests_properties(robotour_app_out_of_range_value_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(robotour_app_out_of_range_value_fails portable)

  add_test(
      NAME robotour_app_unknown_map_fails
      COMMAND robotour_app --set map.catalog=${ROZETA_APP_CATALOG}
              --set map.id=no_such_place --dry-run)
  set_tests_properties(robotour_app_unknown_map_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(robotour_app_unknown_map_fails portable)

  add_test(
      NAME robotour_app_tick_budget_fails
      COMMAND robotour_app --set map.catalog=${ROZETA_APP_CATALOG}
              --set map.id=castle_park --set sim.max_ticks=50 --set app.quiet=true
              --set app.log_every=0)
  set_tests_properties(robotour_app_tick_budget_fails PROPERTIES WILL_FAIL TRUE)
  rozeta_label_test(robotour_app_tick_budget_fails portable simulation)

  if(Python3_Interpreter_FOUND)
    add_test(
        NAME robotour_app_config_round_trip
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/test_robotour_app_cli.py
            $<TARGET_FILE:robotour_app>
            ${ROZETA_APP_CATALOG}
            ${ROZETA_APP_PRESETS})
    rozeta_label_test(robotour_app_config_round_trip portable)
  endif()
endif()
