#include <rozeta/imu.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> values;
    std::stringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ',')) {
        values.push_back(value);
    }
    return values;
}

void usage(const char* program) {
    std::cerr << "Usage: " << program << " --sample tests/fixtures/imu/basic.csv\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string sample_path;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--sample" && i + 1 < argc) {
            sample_path = argv[++i];
        }
    }

    if (sample_path.empty()) {
        usage(argv[0]);
        return 1;
    }

    std::ifstream input(sample_path);
    if (!input) {
        std::cerr << "Unable to open IMU fusion sample: " << sample_path << "\n";
        return 1;
    }

    rozeta::imu::PoseFusion fusion({0.25, 0.60});
    fusion.setGpsOrigin({48.0000000, 17.0000000, 200.0});

    std::string line;
    std::getline(input, line);
    int samples = 0;
    rozeta::imu::PoseFusionResult last;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        const auto fields = splitCsvLine(line);
        if (fields.size() != 11) {
            std::cerr << "Malformed sample row: " << line << "\n";
            return 1;
        }

        rozeta::imu::PoseFusionInput sample;
        sample.odometry_pose = {
            std::stod(fields[1]),
            std::stod(fields[2]),
            std::stod(fields[3]),
        };
        sample.gps_fix = {
            std::stod(fields[4]),
            std::stod(fields[5]),
            std::stod(fields[6]),
        };
        sample.imu.heading_rad = std::stod(fields[7]);
        sample.imu.accelerometer_mps2 = {
            std::stod(fields[8]),
            std::stod(fields[9]),
            std::stod(fields[10]),
        };

        last = fusion.update(sample);
        if (!last.status.ok()) {
            std::cerr << "Fusion failed: " << last.status.message << "\n";
            return 1;
        }
        ++samples;
    }

    std::cout << "IMU fusion demo processed " << samples << " samples; final pose "
              << "x=" << last.pose.x << " y=" << last.pose.y
              << " heading=" << last.pose.heading << "\n";
    return samples > 0 ? 0 : 1;
}
