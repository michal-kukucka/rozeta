#include <rozeta/gps.hpp>

#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

struct Options {
    std::string device{};
    int baud{9600};
    std::string sample{"tests/fixtures/gps/robotour_sample.nmea"};
    bool help{false};
};

void printUsage() {
    std::cout << "gps_serial_reader [--device /dev/ttyUSB0] [--baud 9600] [--sample tests/fixtures/gps/robotour_sample.nmea]\n"
              << "\n"
              << "Without --device, the example reads the sample file so it works without GPS hardware.\n";
}

bool parseArgs(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
            printUsage();
            return false;
        }
        if (arg == "--device" && i + 1 < argc) {
            options.device = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            try {
                options.baud = std::stoi(argv[++i]);
            } catch (const std::exception&) {
                std::cerr << "Invalid --baud value\n";
                return false;
            }
        } else if (arg == "--sample" && i + 1 < argc) {
            options.sample = argv[++i];
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << "\n";
            printUsage();
            return false;
        }
    }
    return true;
}

void printFix(const rozeta::gps::GpsFix& fix) {
    std::cout << std::fixed << std::setprecision(6)
              << "fix lat=" << fix.latitude
              << " lon=" << fix.longitude
              << std::setprecision(1)
              << " alt=" << fix.altitude_m
              << " sats=" << fix.satellite_count
              << std::setprecision(2)
              << " speed=" << fix.speed_mps
              << " course=" << fix.course_deg << "\n";
}

int readSample(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        std::cerr << "Could not open sample file: " << path << "\n";
        return 1;
    }

    rozeta::gps::NmeaParser parser;
    std::string line;
    int valid_count = 0;
    int checksum_failures = 0;
    while (std::getline(input, line)) {
        auto result = parser.parseLineDetailed(line);
        if (result.ok() && result.fix.valid) {
            printFix(result.fix);
            ++valid_count;
        } else if (result.code == rozeta::gps::NmeaParseCode::InvalidChecksum ||
                   result.code == rozeta::gps::NmeaParseCode::MissingChecksum) {
            ++checksum_failures;
        }
    }

    std::cout << "sample summary valid_fixes=" << valid_count
              << " checksum_failures=" << checksum_failures << "\n";
    return valid_count > 0 ? 0 : 2;
}

int readDevice(const Options& options) {
    rozeta::gps::GpsReceiverConfig config;
    config.device = options.device;
    config.baud_rate = options.baud;
    rozeta::gps::SerialGpsReceiver receiver(config);
    auto status = receiver.open();
    if (!status.ok()) {
        std::cerr << "Failed to open GPS device: " << status.message << "\n";
        return 1;
    }

    auto fix = receiver.readFix();
    if (!fix) {
        std::cerr << "No GPS fix available yet: " << receiver.lastStatus().message << "\n";
        return 2;
    }
    printFix(*fix);
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArgs(argc, argv, options)) {
        return options.help ? 0 : 1;
    }

    if (!options.device.empty()) {
        return readDevice(options);
    }
    return readSample(options.sample);
}
