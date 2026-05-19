add_executable(rozeta_tests
    test_main.cpp
    test_gps.cpp
    test_odometry.cpp
    test_coordinates.cpp
    test_obstacles.cpp
    test_motors.cpp
    test_config.cpp
)
target_link_libraries(rozeta_tests PRIVATE rozeta_static)
target_compile_options(rozeta_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME rozeta_tests COMMAND rozeta_tests)
