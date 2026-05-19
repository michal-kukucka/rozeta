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
