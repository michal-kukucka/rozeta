#include "test_helpers.hpp"

#include <rozeta/logging.hpp>

#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace rozeta;

namespace {

class RecordingLogger final : public logging::Logger {
public:
    void log(logging::Level level,
             const std::string& channel,
             const std::string& message) override {
        entries.push_back(logging::levelName(level) + std::string("/") + channel + "/" + message);
    }

    std::vector<std::string> entries;
};

std::string readAll(const char* path) {
    std::ifstream in(path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

} // namespace

void test_logging_global_logger_swap_and_null_safe(){
    auto original = logging::getLogger();
    auto recorder = std::make_shared<RecordingLogger>();
    logging::setLogger(recorder);
    logging::log(logging::Level::Warning, "nav", "off route");
    REQUIRE_EQ(recorder->entries.size(), static_cast<std::size_t>(1));
    REQUIRE_EQ(recorder->entries[0], std::string("warning/nav/off route"));
    // Null logger must drop messages instead of crashing.
    logging::setLogger(nullptr);
    logging::log(logging::Level::Error, "nav", "dropped");
    logging::setLogger(original);
}

void test_logging_csv_file_logger_escapes_quotes_commas_newlines(){
    const char* path = "rozeta_test_log.csv";
    std::remove(path);
    {
        logging::CsvFileLogger logger(path);
        REQUIRE_TRUE(logger.isOpen());
        logger.log(logging::Level::Info, "gps,serial", "fix \"stale\"\nretrying");
        logger.log(logging::Level::Debug, "nav", "plain");
    }
    const std::string content = readAll(path);
    REQUIRE_TRUE(content.find("level,channel,message\n") == 0);
    // RFC 4180: fields quoted, embedded quotes doubled, newline kept inside quotes.
    REQUIRE_TRUE(content.find("info,\"gps,serial\",\"fix \"\"stale\"\"\nretrying\"\n") != std::string::npos);
    REQUIRE_TRUE(content.find("debug,\"nav\",\"plain\"\n") != std::string::npos);
    std::remove(path);
}

void test_logging_csv_file_logger_reports_unopenable_path(){
    logging::CsvFileLogger logger("rozeta_missing_dir/does/not/exist.csv");
    REQUIRE_TRUE(!logger.isOpen());
    // Logging into a failed file must not crash.
    logger.log(logging::Level::Info, "nav", "ignored");
}
