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
void test_motor_calibration_save_load_round_trip();
void test_motor_calibration_load_rejects_invalid_values();
void test_motor_calibration_load_missing_file_returns_error();
#ifdef ROZETA_WITH_SERIAL_MOTORS
void test_serial_motor_formats_normalized_speed_commands();
void test_serial_motor_rejects_invalid_speed_without_writing();
void test_serial_motor_stop_writes_configured_stop_command();
void test_serial_motor_emergency_stop_writes_stop_then_refuses_motion();
void test_serial_motor_clear_emergency_stop_allows_motion();
void test_serial_motor_emergency_stop_writes_stop_even_when_motion_config_is_invalid();
void test_serial_motor_rejects_command_prefix_with_control_characters();
void test_serial_motor_propagates_transport_write_errors();
#endif

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
        {"motor_calibration_round_trip", test_motor_calibration_save_load_round_trip},
        {"motor_calibration_invalid", test_motor_calibration_load_rejects_invalid_values},
        {"motor_calibration_missing", test_motor_calibration_load_missing_file_returns_error},
#ifdef ROZETA_WITH_SERIAL_MOTORS
        {"serial_motor_format", test_serial_motor_formats_normalized_speed_commands},
        {"serial_motor_invalid_speed", test_serial_motor_rejects_invalid_speed_without_writing},
        {"serial_motor_stop", test_serial_motor_stop_writes_configured_stop_command},
        {"serial_motor_estop", test_serial_motor_emergency_stop_writes_stop_then_refuses_motion},
        {"serial_motor_clear_estop", test_serial_motor_clear_emergency_stop_allows_motion},
        {"serial_motor_estop_ignores_motion_config", test_serial_motor_emergency_stop_writes_stop_even_when_motion_config_is_invalid},
        {"serial_motor_prefix_validation", test_serial_motor_rejects_command_prefix_with_control_characters},
        {"serial_motor_write_error", test_serial_motor_propagates_transport_write_errors},
#endif
    };
    int failed = 0;
    for (auto &test : tests) {
        try { test.second(); std::cout << "[PASS] " << test.first << "\n"; }
        catch (const std::exception &e) { ++failed; std::cerr << "[FAIL] " << test.first << ": " << e.what() << "\n"; }
    }
    return failed == 0 ? 0 : 1;
}
