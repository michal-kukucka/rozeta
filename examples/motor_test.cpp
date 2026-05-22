#include <rozeta/motors.hpp>

#include <iostream>

int main() {
    rozeta::motors::MockMotorController motors;
    motors.setSpeed(0.2, 0.2);

    auto command = motors.lastCommand();
    std::cout << "left=" << command.left_speed
              << " right=" << command.right_speed << "\n";

    motors.stop();
}
