add_executable(rozeta_tests
    test_main.cpp
    test_gps.cpp
    test_gps_receiver.cpp
    test_network_gps.cpp
    test_odometry.cpp
    test_coordinates.cpp
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
if(ROZETA_PLATFORM_POSIX)
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
