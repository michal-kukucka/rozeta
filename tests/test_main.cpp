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
void test_depth_frame_extracts_nearest_obstacle_sectors();
void test_depth_frame_to_point_cloud_projects_valid_pixels();
void test_depth_frame_ignores_invalid_pixels_and_bad_fixtures();
void test_motor_command_validation_and_estop();
void test_config_loader_reads_key_values();
void test_serial_port_open_timeout_write_read_and_close();
void test_serial_port_rejects_invalid_configuration();
void test_motor_calibration_save_load_round_trip();
void test_motor_calibration_load_rejects_invalid_values();
void test_motor_calibration_load_missing_file_returns_error();
void test_maps_nearest_path_index_selects_closest_path();
void test_maps_nearest_path_index_empty_map_returns_invalid_index();
void test_maps_csv_loader_loads_fixture_route_and_sorts_by_sequence();
void test_maps_csv_loader_loads_multiple_paths();
void test_maps_csv_loader_reports_missing_file();
void test_maps_csv_loader_reports_invalid_row();
void test_maps_csv_loader_reports_empty_route();
void test_navigation_go_to_waypoint_stops_inside_tolerance();
void test_navigation_route_follower_advances_waypoints();
void test_navigation_route_follower_finishes_route();
void test_navigation_route_follower_empty_route_reports_finished();
void test_navigation_route_follower_obstacle_does_not_advance_unless_reached();
void test_imu_tilt_detects_lateral_acceleration_threshold();
void test_imu_collision_detects_total_acceleration_spike();
void test_imu_pose_fusion_normalizes_heading_and_blends_gps_correction();
void test_imu_pose_fusion_rejects_invalid_weights();
void test_imu_pose_fusion_ignores_gps_without_origin_or_fix();
void test_imu_pose_fusion_weight_boundaries_select_sources();
void test_imu_pose_fusion_replays_fixture_samples();
void test_camera_expected_byte_size_and_shape_for_rgb_frame();
void test_camera_validates_fake_camera_frame_metadata_and_payload();
void test_camera_rejects_invalid_metadata_and_payload_size();
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
        {"depth_obstacles", test_depth_frame_extracts_nearest_obstacle_sectors},
        {"depth_point_cloud", test_depth_frame_to_point_cloud_projects_valid_pixels},
        {"depth_edge_cases", test_depth_frame_ignores_invalid_pixels_and_bad_fixtures},
        {"motors", test_motor_command_validation_and_estop},
        {"config", test_config_loader_reads_key_values},
        {"serial_port", test_serial_port_open_timeout_write_read_and_close},
        {"serial_invalid_config", test_serial_port_rejects_invalid_configuration},
        {"motor_calibration_round_trip", test_motor_calibration_save_load_round_trip},
        {"motor_calibration_invalid", test_motor_calibration_load_rejects_invalid_values},
        {"motor_calibration_missing", test_motor_calibration_load_missing_file_returns_error},
        {"maps_nearest_path", test_maps_nearest_path_index_selects_closest_path},
        {"maps_empty_index", test_maps_nearest_path_index_empty_map_returns_invalid_index},
        {"maps_csv_fixture", test_maps_csv_loader_loads_fixture_route_and_sorts_by_sequence},
        {"maps_csv_multiple_paths", test_maps_csv_loader_loads_multiple_paths},
        {"maps_missing_file", test_maps_csv_loader_reports_missing_file},
        {"maps_invalid_row", test_maps_csv_loader_reports_invalid_row},
        {"maps_empty_route", test_maps_csv_loader_reports_empty_route},
        {"navigation_waypoint_tolerance", test_navigation_go_to_waypoint_stops_inside_tolerance},
        {"navigation_route_advance", test_navigation_route_follower_advances_waypoints},
        {"navigation_route_complete", test_navigation_route_follower_finishes_route},
        {"navigation_route_empty", test_navigation_route_follower_empty_route_reports_finished},
        {"navigation_route_obstacle", test_navigation_route_follower_obstacle_does_not_advance_unless_reached},
        {"imu_tilt", test_imu_tilt_detects_lateral_acceleration_threshold},
        {"imu_collision", test_imu_collision_detects_total_acceleration_spike},
        {"imu_pose_fusion", test_imu_pose_fusion_normalizes_heading_and_blends_gps_correction},
        {"imu_invalid_weights", test_imu_pose_fusion_rejects_invalid_weights},
        {"imu_gps_edge_cases", test_imu_pose_fusion_ignores_gps_without_origin_or_fix},
        {"imu_weight_boundaries", test_imu_pose_fusion_weight_boundaries_select_sources},
        {"imu_fixture_replay", test_imu_pose_fusion_replays_fixture_samples},
        {"camera_shape", test_camera_expected_byte_size_and_shape_for_rgb_frame},
        {"camera_fake_frame", test_camera_validates_fake_camera_frame_metadata_and_payload},
        {"camera_invalid_frame", test_camera_rejects_invalid_metadata_and_payload_size},
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
