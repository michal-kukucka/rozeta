#include <rozeta/telemetry.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string readAll(std::istream& input) {
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

} // namespace

int main(int argc, char** argv) {
    std::string text;
    if (argc > 2) {
        std::cerr << "usage: buchlovice_telemetry_converter [legacy-log.txt]\n";
        return 2;
    }

    if (argc == 2) {
        std::ifstream file(argv[1]);
        if (!file) {
            std::cerr << "failed to open " << argv[1] << "\n";
            return 2;
        }
        text = readAll(file);
    } else {
        text = readAll(std::cin);
    }

    const auto result = rozeta::telemetry::convertBuchloviceTelemetry(text);
    if (!result.status.ok()) {
        std::cerr << "conversion failed: " << result.status.message << "\n";
        return 1;
    }

    const auto& header = rozeta::telemetry::missionTickCsvHeader();
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (i > 0) {
            std::cout << ',';
        }
        std::cout << header[i];
    }
    std::cout << '\n';

    for (const auto& tick : result.ticks) {
        std::cout << rozeta::telemetry::formatMissionTickCsv(tick) << '\n';
    }

    if (!result.events.empty()) {
        std::cerr << "events converted: " << result.events.size() << "\n";
    }
    return 0;
}
