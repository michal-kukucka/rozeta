#include <rozeta/lidar.hpp>

#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string device{};
    int baud{230400};
    std::string sample{"tests/fixtures/lidar/ldrobot_ld06_frame.bin"};
    std::size_t required_frames{1};
    std::size_t max_probe_bytes{512};
    double min_distance_m{0.05};
    double max_distance_m{12.0};
    std::uint8_t min_intensity{0};
    bool help{false};
};

void printUsage() {
    std::cout << "ldrobot_lidar_scan_console [--sample tests/fixtures/lidar/ldrobot_ld06_frame.bin]\n"
              << "  [--device /dev/ttyUSB0] [--baud 230400]\n"
              << "  [--required-frames 2] [--max-probe-bytes 512]\n"
              << "  [--min-distance-m 0.05] [--max-distance-m 12.0] [--min-intensity 0]\n\n"
              << "Without --device, the example replays a binary LDROBOT LD06/LD19-style fixture.\n";
}

bool parseSize(const std::string& text, std::size_t& value) {
    try {
        value = static_cast<std::size_t>(std::stoull(text));
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseDouble(const std::string& text, double& value) {
    try {
        value = std::stod(text);
        return true;
    } catch (const std::exception&) {
        return false;
    }
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
        } else if (arg == "--required-frames" && i + 1 < argc) {
            if (!parseSize(argv[++i], options.required_frames)) {
                std::cerr << "Invalid --required-frames value\n";
                return false;
            }
        } else if (arg == "--max-probe-bytes" && i + 1 < argc) {
            if (!parseSize(argv[++i], options.max_probe_bytes)) {
                std::cerr << "Invalid --max-probe-bytes value\n";
                return false;
            }
        } else if (arg == "--min-distance-m" && i + 1 < argc) {
            if (!parseDouble(argv[++i], options.min_distance_m)) {
                std::cerr << "Invalid --min-distance-m value\n";
                return false;
            }
        } else if (arg == "--max-distance-m" && i + 1 < argc) {
            if (!parseDouble(argv[++i], options.max_distance_m)) {
                std::cerr << "Invalid --max-distance-m value\n";
                return false;
            }
        } else if (arg == "--min-intensity" && i + 1 < argc) {
            std::size_t value = 0;
            if (!parseSize(argv[++i], value) || value > 255) {
                std::cerr << "Invalid --min-intensity value\n";
                return false;
            }
            options.min_intensity = static_cast<std::uint8_t>(value);
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

rozeta::lidar::LdRobotLidarDetectionConfig detectionConfig(const Options& options) {
    rozeta::lidar::LdRobotLidarDetectionConfig config;
    config.required_valid_frames = options.required_frames;
    config.max_probe_bytes = options.max_probe_bytes;
    config.min_distance_m = options.min_distance_m;
    config.max_distance_m = options.max_distance_m;
    config.min_intensity = options.min_intensity;
    return config;
}

int runSample(const Options& options) {
#ifndef ROZETA_WITH_LDROBOT_LIDAR
    std::cerr << "LDROBOT LiDAR support is not enabled. Reconfigure with -DROZETA_WITH_LDROBOT_LIDAR=ON.\n";
    return 3;
#else
    auto bytes = readBinary(options.sample);
    auto detect = rozeta::lidar::detectLdRobotLidarPacketStream(
        bytes.data(),
        bytes.size(),
        detectionConfig(options));
    auto points = rozeta::lidar::parseLdRobotLidarPacketStream(
        bytes.data(),
        bytes.size(),
        detectionConfig(options));
    auto valid = rozeta::lidar::filterInvalid(points, options.min_distance_m, options.max_distance_m);
    std::cout << "ldrobot sample bytes=" << bytes.size()
              << " points=" << points.size()
              << " valid=" << valid.size()
              << " detected=" << (detect.compatible ? "yes" : "no")
              << " frames=" << detect.valid_frames << "\n";
    std::cout << rozeta::lidar::renderConsoleScan(points, 61, 5.0) << "\n";
    return valid.empty() ? 2 : 0;
#endif
}

int runDevice(const Options& options) {
#ifndef ROZETA_WITH_LDROBOT_LIDAR
    std::cerr << "LDROBOT LiDAR support is not enabled. Reconfigure with -DROZETA_WITH_LDROBOT_LIDAR=ON.\n";
    return 3;
#else
    rozeta::lidar::LdRobotLidarConfig config;
    config.device = options.device;
    config.baud_rate = options.baud;
    config.detection = detectionConfig(options);
    rozeta::lidar::LdRobotLidarScanner scanner(config);
    auto status = scanner.initialize(options.device);
    if (!status.ok()) {
        std::cerr << "Failed to open LDROBOT LiDAR device: " << status.message << "\n";
        return 1;
    }
    status = scanner.start();
    if (!status.ok()) {
        std::cerr << "Failed to start LDROBOT LiDAR scan: " << status.message << "\n";
        return 1;
    }
    auto scan = scanner.readScan();
    scanner.stop();
    std::cout << "ldrobot device points=" << scan.points.size() << "\n";
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
        return runSample(options);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
}
