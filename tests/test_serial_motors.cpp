#include "test_helpers.hpp"

#include <rozeta/motors.hpp>
#include "internal/serial_motor_backend.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace {

class FakeMotorTransport final : public rozeta::internal::MotorSerialTransport {
public:
    rozeta::Status writeAll(const std::uint8_t* data, std::size_t size) override {
        writes.emplace_back(reinterpret_cast<const char*>(data), size);
        if (!next_status.ok()) {
            return next_status;
        }
        return rozeta::Status::okStatus();
    }

    std::vector<std::string> writes{};
    rozeta::Status next_status{rozeta::Status::okStatus()};
};

rozeta::motors::SerialMotorConfig testConfig() {
    rozeta::motors::SerialMotorConfig config;
    config.max_command = 255;
    config.command_prefix = "M";
    config.stop_command = "STOP!\n";
    config.calibration.max_speed = 1.0;
    config.calibration.left_scale = 1.0;
    config.calibration.right_scale = 1.0;
    return config;
}

} // namespace

void test_serial_motor_formats_normalized_speed_commands() {
    FakeMotorTransport transport;
    auto config = testConfig();
    rozeta::internal::SerialMotorBackend backend(config, transport);

    rozeta::Status status = backend.setSpeed(1.0, -0.5);

    REQUIRE_TRUE(status.ok());
    REQUIRE_EQ(transport.writes.size(), static_cast<std::size_t>(1));
    REQUIRE_EQ(transport.writes[0], std::string("M 255 -128\n"));

    config.calibration.left_scale = 0.5;
    FakeMotorTransport scaled_transport;
    rozeta::internal::SerialMotorBackend scaled_backend(config, scaled_transport);
    status = scaled_backend.setSpeed(1.0, 1.0);

    REQUIRE_TRUE(status.ok());
    REQUIRE_EQ(scaled_transport.writes[0], std::string("M 128 255\n"));
}

void test_serial_motor_rejects_invalid_speed_without_writing() {
    FakeMotorTransport transport;
    rozeta::internal::SerialMotorBackend backend(testConfig(), transport);

    rozeta::Status status = backend.setSpeed(1.01, 0.0);

    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
    REQUIRE_TRUE(transport.writes.empty());
}

void test_serial_motor_stop_writes_configured_stop_command() {
    FakeMotorTransport transport;
    rozeta::internal::SerialMotorBackend backend(testConfig(), transport);

    rozeta::Status status = backend.stop();

    REQUIRE_TRUE(status.ok());
    REQUIRE_EQ(transport.writes.size(), static_cast<std::size_t>(1));
    REQUIRE_EQ(transport.writes[0], std::string("STOP!\n"));
}

void test_serial_motor_emergency_stop_writes_stop_then_refuses_motion() {
    FakeMotorTransport transport;
    rozeta::internal::SerialMotorBackend backend(testConfig(), transport);

    backend.emergencyStop();

    REQUIRE_TRUE(backend.isEmergencyStopped());
    REQUIRE_EQ(transport.writes.size(), static_cast<std::size_t>(1));
    REQUIRE_EQ(transport.writes[0], std::string("STOP!\n"));

    rozeta::Status status = backend.setSpeed(0.1, 0.1);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::EmergencyStopped));
    REQUIRE_EQ(transport.writes.size(), static_cast<std::size_t>(1));
}

void test_serial_motor_clear_emergency_stop_allows_motion() {
    FakeMotorTransport transport;
    rozeta::internal::SerialMotorBackend backend(testConfig(), transport);

    backend.emergencyStop();
    backend.clearEmergencyStop();
    rozeta::Status status = backend.setSpeed(0.1, 0.1);

    REQUIRE_TRUE(status.ok());
    REQUIRE_TRUE(!backend.isEmergencyStopped());
    REQUIRE_EQ(transport.writes.size(), static_cast<std::size_t>(2));
    REQUIRE_EQ(transport.writes[1], std::string("M 26 26\n"));
}

void test_serial_motor_emergency_stop_writes_stop_even_when_motion_config_is_invalid() {
    FakeMotorTransport transport;
    auto config = testConfig();
    config.calibration.max_speed = -1.0;
    config.max_command = 0;
    rozeta::internal::SerialMotorBackend backend(config, transport);

    backend.emergencyStop();

    REQUIRE_TRUE(backend.isEmergencyStopped());
    REQUIRE_EQ(transport.writes.size(), static_cast<std::size_t>(1));
    REQUIRE_EQ(transport.writes[0], std::string("STOP!\n"));
}

void test_serial_motor_rejects_command_prefix_with_control_characters() {
    FakeMotorTransport transport;
    auto config = testConfig();
    config.command_prefix = "M\nBAD";
    rozeta::internal::SerialMotorBackend backend(config, transport);

    rozeta::Status status = backend.setSpeed(0.1, 0.1);

    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
    REQUIRE_TRUE(transport.writes.empty());
}

void test_serial_motor_propagates_transport_write_errors() {
    FakeMotorTransport transport;
    transport.next_status = rozeta::Status::error(rozeta::ErrorCode::IoError, "fake write failure");
    rozeta::internal::SerialMotorBackend backend(testConfig(), transport);

    rozeta::Status status = backend.setSpeed(0.1, 0.1);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::IoError));

    status = backend.stop();
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::IoError));

    backend.emergencyStop();
    REQUIRE_TRUE(backend.isEmergencyStopped());
}
