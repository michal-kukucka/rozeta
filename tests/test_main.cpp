#include <iostream>
#include <vector>
#include <functional>

void test_gps_parses_gga_fix();
void test_gps_parses_rmc_course_and_speed();
void test_gps_validates_good_and_bad_checksums();
void test_gps_rejects_missing_and_malformed_checksums();
void test_gps_parser_detailed_rejects_invalid_checksum();
void test_gps_parser_accepts_lowercase_checksum_and_crlf();
void test_gps_stream_buffers_fragmented_and_multiple_lines();
void test_gps_stream_discards_garbage_before_sentence();
void test_gps_serial_receiver_reads_fragmented_fix_from_pty();
void test_gps_serial_receiver_skips_bad_checksum_then_returns_good_fix();
void test_gps_serial_receiver_timeout_reports_status();
void test_gps_serial_receiver_rejects_invalid_config();
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
#ifdef ROZETA_WITH_YDLIDAR
void test_ydlidar_parser_parses_sample_frame();
void test_ydlidar_parser_accepts_fragmented_frame();
void test_ydlidar_parser_discards_garbage_before_frame();
void test_ydlidar_parser_rejects_invalid_packets_without_throwing();
void test_ydlidar_parser_normalizes_wraparound_angles();
void test_ydlidar_backend_invalid_device_reports_hardware_unavailable();
#endif

int main(){
    std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"gps_gga", test_gps_parses_gga_fix},
        {"gps_rmc", test_gps_parses_rmc_course_and_speed},
        {"gps_checksum", test_gps_validates_good_and_bad_checksums},
        {"gps_checksum_malformed", test_gps_rejects_missing_and_malformed_checksums},
        {"gps_detailed_bad_checksum", test_gps_parser_detailed_rejects_invalid_checksum},
        {"gps_detailed_crlf", test_gps_parser_accepts_lowercase_checksum_and_crlf},
        {"gps_stream_fragmented", test_gps_stream_buffers_fragmented_and_multiple_lines},
        {"gps_stream_garbage", test_gps_stream_discards_garbage_before_sentence},
        {"gps_serial_fragmented", test_gps_serial_receiver_reads_fragmented_fix_from_pty},
        {"gps_serial_skip_bad", test_gps_serial_receiver_skips_bad_checksum_then_returns_good_fix},
        {"gps_serial_timeout", test_gps_serial_receiver_timeout_reports_status},
        {"gps_serial_invalid_config", test_gps_serial_receiver_rejects_invalid_config},
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
#ifdef ROZETA_WITH_YDLIDAR
        {"ydlidar_parse_fixture", test_ydlidar_parser_parses_sample_frame},
        {"ydlidar_fragmented", test_ydlidar_parser_accepts_fragmented_frame},
        {"ydlidar_garbage", test_ydlidar_parser_discards_garbage_before_frame},
        {"ydlidar_invalid_safe", test_ydlidar_parser_rejects_invalid_packets_without_throwing},
        {"ydlidar_wraparound", test_ydlidar_parser_normalizes_wraparound_angles},
        {"ydlidar_invalid_device", test_ydlidar_backend_invalid_device_reports_hardware_unavailable},
#endif
    };
    int failed = 0;
    for (auto &test : tests) {
        try { test.second(); std::cout << "[PASS] " << test.first << "\n"; }
        catch (const std::exception &e) { ++failed; std::cerr << "[FAIL] " << test.first << ": " << e.what() << "\n"; }
    }
    return failed == 0 ? 0 : 1;
}
