add_executable(rozeta_tests
    test_main.cpp
    test_gps.cpp
    test_gps_receiver.cpp
    test_odometry.cpp
    test_coordinates.cpp
    test_obstacles.cpp
    test_motors.cpp
    test_config.cpp
    test_serial_port.cpp
    test_motor_calibration.cpp
    test_maps.cpp
    test_navigation.cpp
    test_imu.cpp
    test_camera.cpp
    test_telemetry.cpp
    test_ui.cpp
)
if(ROZETA_WITH_SERIAL_MOTORS)
  target_sources(rozeta_tests PRIVATE test_serial_motors.cpp)
endif()
if(ROZETA_WITH_YDLIDAR)
  target_sources(rozeta_tests PRIVATE test_ydlidar_parser.cpp)
endif()
target_link_libraries(rozeta_tests PRIVATE rozeta_static)
target_include_directories(rozeta_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_compile_options(rozeta_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME rozeta_tests COMMAND rozeta_tests)
add_test(NAME rozeta_serial_tests COMMAND rozeta_tests)

add_executable(rozeta_c_api_smoke c_api_smoke.c)
target_link_libraries(rozeta_c_api_smoke PRIVATE rozeta_static)
target_compile_options(rozeta_c_api_smoke PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME rozeta_c_api_smoke COMMAND rozeta_c_api_smoke)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(NAME rozeta_docs_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/verify_docs.py)
endif()
