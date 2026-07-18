#include <rozeta/motors.hpp>

#include <iostream>
#include <string>

namespace {

void printUsage() {
    std::cout << "serial_motor_calibrate --dry-run [--buchlovice-binary] [--cytron-mdds30] [--output motor_calibration.ini]\n"
              << "  --dry-run             Print/save calibration without opening hardware.\n"
              << "  --buchlovice-binary   Print BuchloviceBinary packet defaults for smoke planning.\n"
              << "  --cytron-mdds30       Print CytronMdds30 bridge protocol defaults for smoke planning.\n"
              << "  --output PATH         Save the generated calibration file.\n";
}

} // namespace

int main(int argc, char** argv) {
    bool dry_run = false;
    std::string output;
    bool buchlovice_binary = false;
    bool cytron_mdds30 = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--dry-run") {
            dry_run = true;
        } else if (arg == "--buchlovice-binary") {
            buchlovice_binary = true;
        } else if (arg == "--cytron-mdds30") {
            cytron_mdds30 = true;
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

    if (buchlovice_binary) {
        std::cout << "protocol=BuchloviceBinary\n"
                  << "packet=[255, pwm_right, pwm_left, reg, lrc, 13, 10]\n"
                  << "repeat_interval_ms=200\n";
    }

    if (cytron_mdds30) {
        std::cout << "protocol=CytronMdds30\n"
                  << "command=M L=<-100..100> R=<-100..100>\n"
                  << "stop_command=STOP\n"
                  << "repeat_interval_ms=100\n"
                  << "bridge_watchdog_ms=300\n";
    }

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
