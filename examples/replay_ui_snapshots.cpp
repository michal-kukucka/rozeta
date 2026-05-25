#include <rozeta/telemetry.hpp>
#include <rozeta/ui.hpp>

#include <iostream>
#include <string>

namespace {

rozeta::maps::OfflineMap mapFromReplaySamples(
    const std::vector<rozeta::telemetry::ReplaySample>& samples) {
    rozeta::maps::OfflineMap map;
    rozeta::maps::MapPath path;
    path.id = "telemetry";
    path.points.reserve(samples.size());
    for (const auto& sample : samples) {
        if (!sample.gps.valid) {
            continue;
        }
        path.points.push_back({
            sample.gps.latitude,
            sample.gps.longitude,
            sample.gps.altitude_m,
        });
    }
    if (!path.points.empty()) {
        map.paths.push_back(std::move(path));
    }
    return map;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: replay_ui_snapshots <telemetry.csv>\n";
        std::cerr << "Schema: " << rozeta::telemetry::kReplaySchemaVersion << "\n";
        return 2;
    }

    const auto log = rozeta::telemetry::loadReplayLog(argv[1]);
    if (!log.status.ok()) {
        std::cerr << "Failed to load replay log: " << log.status.message << "\n";
        return 1;
    }

    const auto map = mapFromReplaySamples(log.samples);
    const auto replay = rozeta::telemetry::replayUiSnapshots(
        log.samples,
        map,
        {960, 720, 32});
    if (!replay.status.ok()) {
        std::cerr << "Failed to build UI snapshots: " << replay.status.message << "\n";
        return 1;
    }

    std::cout << "Rozeta telemetry UI replay\n";
    std::cout << "schema=" << rozeta::telemetry::kReplaySchemaVersion
              << " snapshots=" << replay.snapshots.size() << "\n";

    for (std::size_t i = 0; i < replay.snapshots.size(); ++i) {
        std::cout << "--- frame " << i << " ---\n";
        std::cout << rozeta::ui::renderTextDashboard(replay.snapshots[i]);
    }
    return 0;
}
