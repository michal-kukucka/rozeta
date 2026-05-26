#include <rozeta/gps.hpp>

#include <iomanip>
#include <iostream>
#include <string>

namespace {

struct Options {
    std::string protocol{"payload"};
    std::string host{"127.0.0.1"};
    int port{5005};
    std::string payload{"{\"lat\": 48.1486, \"lon\": 17.1077}"};
    bool help{false};
};

void printUsage() {
    std::cout << "gps_network_reader [--payload '48.1,17.1']\n"
              << "                   [--tcp 127.0.0.1:5005 | --udp 127.0.0.1:5005]\n"
              << "\n"
              << "Default mode parses a JSON/plain/NMEA payload without opening sockets.\n";
}

bool splitEndpoint(const std::string& endpoint, std::string& host, int& port) {
    const auto colon = endpoint.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= endpoint.size()) {
        return false;
    }
    host = endpoint.substr(0, colon);
    try {
        port = std::stoi(endpoint.substr(colon + 1));
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

bool parseArgs(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
            printUsage();
            return false;
        }
        if (arg == "--payload" && i + 1 < argc) {
            options.protocol = "payload";
            options.payload = argv[++i];
        } else if ((arg == "--tcp" || arg == "--udp") && i + 1 < argc) {
            options.protocol = arg == "--tcp" ? "tcp" : "udp";
            if (!splitEndpoint(argv[++i], options.host, options.port)) {
                std::cerr << "Endpoint must be host:port\n";
                return false;
            }
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
              << std::setprecision(2)
              << " speed=" << fix.speed_mps
              << " course=" << fix.course_deg << "\n";
}

int parsePayload(const std::string& payload) {
    auto parsed = rozeta::gps::parseGpsPayload(payload);
    if (!parsed.ok() || !parsed.fix.valid) {
        std::cerr << "Could not parse GPS payload: " << parsed.message << "\n";
        return 2;
    }
    printFix(parsed.fix);
    return 0;
}

int readNetwork(const Options& options) {
    rozeta::gps::NetworkGpsReceiverConfig config;
    config.protocol = options.protocol == "tcp" ? rozeta::gps::NetworkGpsProtocol::Tcp : rozeta::gps::NetworkGpsProtocol::Udp;
    config.host = options.host;
    config.port = options.port;

    rozeta::gps::NetworkGpsReceiver receiver(config);
    auto status = receiver.open();
    if (!status.ok()) {
        std::cerr << "Failed to open GPS network receiver: " << status.message << "\n";
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
    if (options.protocol == "payload") {
        return parsePayload(options.payload);
    }
    return readNetwork(options);
}
