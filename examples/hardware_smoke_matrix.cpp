#include <rozeta/hardware_smoke.hpp>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    rozeta::hardware_smoke::HardwareSmokeConfig config;
    config.allow_sensor_only = true;
    config.allow_motor_motion = false;

    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--with-motors") {
            config.allow_motor_motion = true;
        } else if (arg == "--no-estop") {
            config.require_estop_latch = false;
        } else if (arg == "--wheels-not-lifted") {
            config.require_wheels_lifted = false;
        } else if (arg == "--help") {
            std::cout << "Usage: hardware_smoke_matrix [--with-motors] [--no-estop] [--wheels-not-lifted]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << '\n';
            return 2;
        }
    }

    const auto matrix = rozeta::hardware_smoke::buildHardwareSmokeMatrix(config);
    std::cout << rozeta::hardware_smoke::renderHardwareSmokeMatrix(matrix);
    return matrix.ok() ? 0 : 1;
}
