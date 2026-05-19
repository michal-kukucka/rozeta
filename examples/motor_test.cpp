#include <iostream>
#include <rozeta/motors.hpp>
int main(){ rozeta::motors::MockMotorController motors; motors.setSpeed(0.2,0.2); auto c=motors.lastCommand(); std::cout << "left=" << c.left_speed << " right=" << c.right_speed << "\n"; motors.stop(); }
