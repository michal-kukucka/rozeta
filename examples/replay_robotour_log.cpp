#include <rozeta/telemetry.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

bool near(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: replay_robotour_log <telemetry.csv>\n";
        std::cerr << "Schema: " << rozeta::telemetry::kReplaySchemaVersion << "\n";
        return 2;
    }

    const auto log = rozeta::telemetry::loadReplayLog(argv[1]);
    if (!log.status.ok()) {
        std::cerr << "Failed to load replay log: " << log.status.message << "\n";
        return 1;
    }

    const auto replay = rozeta::telemetry::replayNavigation(log.samples);
    if (!replay.status.ok()) {
        std::cerr << "Replay failed: " << replay.status.message << "\n";
        return 1;
    }

    std::cout << "Rozeta Robotour telemetry replay\n";
    std::cout << "schema=" << rozeta::telemetry::kReplaySchemaVersion
              << " samples=" << replay.decisions.size() << "\n";

    bool deterministic = true;
    for (std::size_t i = 0; i < replay.decisions.size(); ++i) {
        const auto& sample = log.samples[i];
        const auto& actual = replay.decisions[i];
        const auto& expected = sample.expected_decision;
        const bool match = actual.reason == expected.reason &&
            actual.emergency_stop == expected.emergency_stop &&
            near(actual.motor.left_speed, expected.motor.left_speed) &&
            near(actual.motor.right_speed, expected.motor.right_speed) &&
            near(actual.motor.left_speed, sample.recorded_motor.left_speed) &&
            near(actual.motor.right_speed, sample.recorded_motor.right_speed);
        deterministic = deterministic && match;

        std::cout << std::fixed << std::setprecision(3)
                  << "t=" << sample.timestamp_ms << "ms"
                  << " pose=(" << sample.pose.x << "," << sample.pose.y << ")"
                  << " target=(" << sample.target.x << "," << sample.target.y << ")"
                  << " decision=\"" << actual.reason << "\""
                  << " left=" << actual.motor.left_speed
                  << " right=" << actual.motor.right_speed
                  << " estop=" << (actual.emergency_stop ? "true" : "false")
                  << " match=" << (match ? "yes" : "no") << "\n";
    }

    if (!deterministic) {
        std::cerr << "Replay decisions differ from fixture expectations.\n";
        return 1;
    }

    std::cout << "Replay deterministic: all decisions match fixture expectations.\n";
    return 0;
}
