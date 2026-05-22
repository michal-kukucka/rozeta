#include <rozeta/lidar.hpp>

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string device{};
    int baud{128000};
    std::string sample{"tests/fixtures/lidar/ydlidar_frame.bin"};
    bool help{false};
};

void printUsage() {
    std::cout << "ydlidar_scan_console [--sample tests/fixtures/lidar/ydlidar_frame.bin] [--device /dev/ttyUSB0] [--baud 128000]\n"
              << "\n"
              << "Without --device, the example replays a binary YDLIDAR-style fixture so it works without hardware.\n";
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

std::vector<std::uint8_t> readBinary(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open sample file: " + path);
    }
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

int runSample(const std::string& path) {
#ifndef ROZETA_WITH_YDLIDAR
    std::cerr << "YDLIDAR support is not enabled. Reconfigure with -DROZETA_WITH_YDLIDAR=ON.\n";
    return 3;
#else
    auto bytes = readBinary(path);
    auto points = rozeta::lidar::parseYdLidarPacketStream(bytes.data(), bytes.size());
    auto valid = rozeta::lidar::filterInvalid(points);
    std::cout << "ydlidar sample bytes=" << bytes.size()
              << " points=" << points.size()
              << " valid=" << valid.size() << "\n";
    std::cout << rozeta::lidar::renderConsoleScan(points, 61, 5.0) << "\n";
    return valid.empty() ? 2 : 0;
#endif
}

int runDevice(const Options& options) {
#ifndef ROZETA_WITH_YDLIDAR
    std::cerr << "YDLIDAR support is not enabled. Reconfigure with -DROZETA_WITH_YDLIDAR=ON.\n";
    return 3;
#else
    rozeta::lidar::YdLidarConfig config;
    config.device = options.device;
    config.baud_rate = options.baud;
    rozeta::lidar::YdLidarScanner scanner(config);
    auto status = scanner.initialize(options.device);
    if (!status.ok()) {
        std::cerr << "Failed to open YDLIDAR device: " << status.message << "\n";
        return 1;
    }
    status = scanner.start();
    if (!status.ok()) {
        std::cerr << "Failed to start YDLIDAR scan: " << status.message << "\n";
        return 1;
    }
    auto scan = scanner.readScan();
    scanner.stop();
    std::cout << "ydlidar device points=" << scan.points.size() << "\n";
    if (!scan.points.empty()) {
        std::cout << rozeta::lidar::renderConsoleScan(scan.points, 61, 5.0) << "\n";
        return 0;
    }
    return 2;
#endif
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArgs(argc, argv, options)) {
        return options.help ? 0 : 1;
    }
    try {
        if (!options.device.empty()) {
            return runDevice(options);
        }
        return runSample(options.sample);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
}
