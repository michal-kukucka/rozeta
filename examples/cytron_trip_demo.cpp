// Trip demo for the default drive path: Cytron MDDS30 behind the Arduino UNO
// bridge sketch in arduino/mdds30_bridge/, driven through motors::SmoothDrive
// so the robot accelerates and brakes smoothly instead of stepping speeds.
//
//   ./cytron_trip_demo                                   # mock controller
//   ./cytron_trip_demo --device /dev/cu.usbmodem14201    # real bridge
//
// The serial path needs -DROZETA_WITH_SERIAL_MOTORS=ON and an Arduino that has
// the bridge sketch flashed (see docs/arduino_mdds30_bridge.md).

#include <rozeta/motors.hpp>

#include <chrono>
#include <iostream>
#include <string>

namespace {

constexpr std::chrono::milliseconds kTick{50};
constexpr std::chrono::milliseconds kCruisePhase{4000};
constexpr std::chrono::milliseconds kBrakePhase{4000};

int runTrip(rozeta::motors::MotorController& controller) {
    rozeta::motors::SmoothDrive drive(controller, rozeta::motors::cytronMdds30DriveProfile());

    if (!drive.setTarget(0.8, 0.8).ok()) {
        std::cerr << "invalid drive target\n";
        return 1;
    }

    // Phase 1: accelerate to cruise speed and hold it.
    std::chrono::milliseconds now{0};
    for (; now < kCruisePhase; now += kTick) {
        const rozeta::Status status = drive.tick(now);
        if (!status.ok()) {
            std::cerr << "drive tick failed: " << status.message << "\n";
            return 1;
        }
        if (now.count() % 500 == 0) {
            std::cout << "t=" << now.count() << "ms left=" << drive.currentSpeeds().left
                      << " right=" << drive.currentSpeeds().right << "\n";
        }
    }

    // Phase 2: fluent brake down to standstill; the last tick issues stop().
    if (!drive.brake().ok()) {
        std::cerr << "brake request failed\n";
        return 1;
    }
    const std::chrono::milliseconds brake_end = now + kBrakePhase;
    for (; now < brake_end; now += kTick) {
        const rozeta::Status status = drive.tick(now);
        if (!status.ok()) {
            std::cerr << "drive tick failed: " << status.message << "\n";
            return 1;
        }
        if (now.count() % 500 == 0) {
            std::cout << "t=" << now.count() << "ms left=" << drive.currentSpeeds().left
                      << " right=" << drive.currentSpeeds().right << "\n";
        }
        if (drive.stopped()) {
            break;
        }
    }

    std::cout << "stopped=" << (drive.stopped() ? "yes" : "no") << "\n";
    return drive.stopped() ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    std::string device;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--device" && i + 1 < argc) {
            device = argv[++i];
        } else if (arg == "--help") {
            std::cout << "cytron_trip_demo [--device <serial-port>]\n";
            return 0;
        }
    }

    if (device.empty()) {
        std::cout << "backend=mock\n";
        rozeta::motors::MockMotorController motors;
        return runTrip(motors);
    }

#ifdef ROZETA_WITH_SERIAL_MOTORS
    std::cout << "backend=cytron_mdds30 device=" << device << "\n";
    rozeta::motors::SerialMotorController motors(rozeta::motors::cytronMdds30Config(device));
    const rozeta::Status opened = motors.open();
    if (!opened.ok()) {
        std::cerr << "failed to open " << device << ": " << opened.message << "\n";
        return 1;
    }
    const int result = runTrip(motors);
    motors.close();
    return result;
#else
    std::cerr << "serial motor backend not built; configure with -DROZETA_WITH_SERIAL_MOTORS=ON\n";
    return 1;
#endif
}
