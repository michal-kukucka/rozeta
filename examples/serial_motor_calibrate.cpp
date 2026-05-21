#include <rozeta/motors.hpp>

#include <iostream>
#include <string>

namespace {

void printUsage() {
    std::cout << "serial_motor_calibrate --dry-run [--output motor_calibration.ini]\n"
              << "  --dry-run       Print/save calibration without opening hardware.\n"
              << "  --output PATH   Save the generated calibration file.\n";
}

} // namespace

int main(int argc, char** argv) {
    bool dry_run = false;
    std::string output;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--dry-run") {
            dry_run = true;
        } else if (arg == "--output" && i + 1 < argc) {
            output = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printUsage();
            return 2;
        }
    }

    if (!dry_run) {
        std::cerr << "This calibration helper currently requires --dry-run to avoid accidental motor motion.\n";
        printUsage();
        return 2;
    }

    rozeta::motors::MotorCalibration calibration;
    std::cout << "Rozeta serial motor calibration dry run\n"
              << "max_speed=" << calibration.max_speed << "\n"
              << "left_scale=" << calibration.left_scale << "\n"
              << "right_scale=" << calibration.right_scale << "\n"
              << "pwm_frequency_hz=" << calibration.pwm_frequency_hz << "\n"
              << "No serial device was opened and no motor command was sent.\n";

    if (!output.empty()) {
        rozeta::Status status = rozeta::motors::saveMotorCalibration(calibration, output);
        if (!status.ok()) {
            std::cerr << "Failed to save calibration: " << status.message << "\n";
            return 1;
        }
        std::cout << "Saved calibration to " << output << "\n";
    }

    return 0;
}
