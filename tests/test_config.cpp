#include "test_helpers.hpp"
#include <cstdio>
#include <fstream>
#include <rozeta/core.hpp>
using namespace rozeta;
void test_config_loader_reads_key_values(){
    const char* path = "rozeta_test_config.ini";
    std::ofstream out(path);
    out << "wheel_base=0.52\nname=robotour\n# ignored\n";
    out.close();
    Config cfg = Config::load(path);
    REQUIRE_NEAR(cfg.getDouble("wheel_base", 0.0), 0.52, 1e-9);
    REQUIRE_EQ(cfg.getString("name", ""), std::string("robotour"));
    REQUIRE_EQ(cfg.getString("missing", "fallback"), std::string("fallback"));
    std::remove(path);
}

void test_config_load_file_reports_missing_and_loads_existing(){
    Config cfg;
    Status missing = Config::loadFile("rozeta_no_such_config.ini", cfg);
    REQUIRE_TRUE(!missing.ok());
    REQUIRE_TRUE(missing.code == ErrorCode::IoError);
    Status empty_path = Config::loadFile("", cfg);
    REQUIRE_TRUE(empty_path.code == ErrorCode::InvalidArgument);

    const char* path = "rozeta_test_config_status.ini";
    std::ofstream out(path);
    out << "speed=1.5\n";
    out.close();
    Status ok = Config::loadFile(path, cfg);
    REQUIRE_TRUE(ok.ok());
    REQUIRE_NEAR(cfg.getDouble("speed", 0.0), 1.5, 1e-9);
    std::remove(path);
}
