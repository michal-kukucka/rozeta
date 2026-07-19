#include <rozeta/motors.hpp>

#include <chrono>
#include <iostream>

int main() {
    rozeta::motors::MockMotorController motors;
    motors.setSpeed(0.2, 0.2);

    auto command = motors.lastCommand();
    std::cout << "left=" << command.left_speed
              << " right=" << command.right_speed << "\n";

    motors.stop();

    // Linear acceleration to (0.8, 0.8) over 1 s, ticked deterministically.
    const auto ramp = rozeta::motors::SpeedRamp::accelerate({0.8, 0.8}, std::chrono::milliseconds(1000));
    for (int ms = 0; ms <= 1000; ms += 250) {
        if (!ramp.applyAt(motors, std::chrono::milliseconds(ms)).ok()) {
            std::cerr << "ramp tick failed\n";
            return 1;
        }
        command = motors.lastCommand();
        std::cout << "t=" << ms << "ms left=" << command.left_speed
                  << " right=" << command.right_speed << "\n";
    }

    // Linear deceleration back to zero; the final tick issues stop().
    const auto decel = rozeta::motors::SpeedRamp::decelerate({0.8, 0.8}, std::chrono::milliseconds(500));
    for (int ms = 0; ms <= 500; ms += 250) {
        if (!decel.applyAt(motors, std::chrono::milliseconds(ms)).ok()) {
            std::cerr << "decel tick failed\n";
            return 1;
        }
    }
    command = motors.lastCommand();
    std::cout << "after decel left=" << command.left_speed
              << " right=" << command.right_speed << "\n";
}
