#pragma once
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <rozeta/core.hpp>

namespace rozeta::logging {

enum class Level { Debug, Info, Warning, Error };

class Logger {
public:
    virtual ~Logger() = default;
    virtual void log(Level level, const std::string& channel, const std::string& message) = 0;
};

class ConsoleLogger final : public Logger {
public:
    void log(Level level, const std::string& channel, const std::string& message) override;
};

class CsvFileLogger final : public Logger {
public:
    explicit CsvFileLogger(const std::string& path);
    void log(Level level, const std::string& channel, const std::string& message) override;
private:
    std::mutex mutex_;
    std::ofstream file_;
};

void setLogger(std::shared_ptr<Logger> logger);
std::shared_ptr<Logger> getLogger();
void log(Level level, const std::string& channel, const std::string& message);
const char* levelName(Level level);

} // namespace rozeta::logging
