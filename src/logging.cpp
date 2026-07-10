#include <rozeta/logging.hpp>

#include <chrono>
#include <iostream>
#include <mutex>

namespace rozeta::logging {
namespace {

std::mutex& activeLoggerMutex() {
    static std::mutex mutex;
    return mutex;
}

std::shared_ptr<Logger>& activeLoggerRef() {
    static auto logger = std::shared_ptr<Logger>(new ConsoleLogger());
    return logger;
}

// RFC 4180 quoting so channels/messages containing '"', ',' or newlines
// cannot corrupt the CSV row structure of telemetry logs.
std::string csvQuote(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (char c : value) {
        if (c == '"') {
            out += "\"\"";
        } else {
            out.push_back(c);
        }
    }
    out.push_back('"');
    return out;
}

} // namespace

const char* levelName(Level level) {
    switch (level) {
        case Level::Debug:
            return "debug";
        case Level::Info:
            return "info";
        case Level::Warning:
            return "warning";
        case Level::Error:
            return "error";
    }
    return "unknown";
}

void ConsoleLogger::log(Level level, const std::string& channel, const std::string& message) {
    std::cerr << levelName(level) << "," << channel << "," << message << "\n";
}

CsvFileLogger::CsvFileLogger(const std::string& path) : file_(path, std::ios::app) {
    if (file_.tellp() == 0) {
        file_ << "level,channel,message\n";
    }
}

void CsvFileLogger::log(Level level, const std::string& channel, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    file_ << levelName(level) << ',' << csvQuote(channel) << ',' << csvQuote(message) << "\n";
    file_.flush();
}

bool CsvFileLogger::isOpen() const {
    return file_.is_open();
}

void setLogger(std::shared_ptr<Logger> logger) {
    std::lock_guard<std::mutex> lock(activeLoggerMutex());
    activeLoggerRef() = std::move(logger);
}

std::shared_ptr<Logger> getLogger() {
    std::lock_guard<std::mutex> lock(activeLoggerMutex());
    return activeLoggerRef();
}

void log(Level level, const std::string& channel, const std::string& message) {
    auto logger = getLogger();
    if (logger) {
        logger->log(level, channel, message);
    }
}

} // namespace rozeta::logging
