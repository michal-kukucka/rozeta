#include <iostream>
#include <vector>
#include <functional>

void test_gps_parses_gga_fix();
void test_gps_parses_rmc_course_and_speed();
void test_odometry_differential_drive_forward_and_turn();
void test_coordinate_local_conversion();
void test_obstacle_sector_calculation();
void test_motor_command_validation_and_estop();
void test_config_loader_reads_key_values();
void test_serial_port_open_timeout_write_read_and_close();
void test_serial_port_rejects_invalid_configuration();

int main(){
    std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"gps_gga", test_gps_parses_gga_fix},
        {"gps_rmc", test_gps_parses_rmc_course_and_speed},
        {"odometry", test_odometry_differential_drive_forward_and_turn},
        {"coordinates", test_coordinate_local_conversion},
        {"obstacles", test_obstacle_sector_calculation},
        {"motors", test_motor_command_validation_and_estop},
        {"config", test_config_loader_reads_key_values},
        {"serial_port", test_serial_port_open_timeout_write_read_and_close},
        {"serial_invalid_config", test_serial_port_rejects_invalid_configuration},
    };
    int failed = 0;
    for (auto &test : tests) {
        try { test.second(); std::cout << "[PASS] " << test.first << "\n"; }
        catch (const std::exception &e) { ++failed; std::cerr << "[FAIL] " << test.first << ": " << e.what() << "\n"; }
    }
    return failed == 0 ? 0 : 1;
}
