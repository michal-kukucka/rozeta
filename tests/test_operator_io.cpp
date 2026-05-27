#include "test_helpers.hpp"
#include <rozeta/operator_io.hpp>

#include <chrono>
#include <stdexcept>
#include <string>

using namespace rozeta;
using namespace std::chrono;

void test_operator_input_mock_routes_events() {
    operator_io::MockOperatorInput input;
    std::vector<operator_io::OperatorKey> received;

    input.onKey([&](operator_io::OperatorKey key) {
        received.push_back(key);
    });

    // Nothing received yet
    REQUIRE_TRUE(received.empty());

    // Inject events
    input.injectKey(operator_io::OperatorKey::Quit);
    input.injectKey(operator_io::OperatorKey::SwitchCamera);
    input.injectKey(operator_io::OperatorKey::ToggleKinect);

    REQUIRE_TRUE(received.size() == 3);
    REQUIRE_TRUE(received[0] == operator_io::OperatorKey::Quit);
    REQUIRE_TRUE(received[1] == operator_io::OperatorKey::SwitchCamera);
    REQUIRE_TRUE(received[2] == operator_io::OperatorKey::ToggleKinect);
}

void test_operator_input_multiple_listeners() {
    operator_io::MockOperatorInput input;

    int count_a = 0;
    int count_b = 0;

    input.onKey([&](operator_io::OperatorKey) { ++count_a; });
    input.onKey([&](operator_io::OperatorKey) { ++count_b; });

    input.injectKey(operator_io::OperatorKey::Continue);
    input.injectKey(operator_io::OperatorKey::Continue);

    REQUIRE_TRUE(count_a == 2);
    REQUIRE_TRUE(count_b == 2);
}

void test_operator_input_inject_all_keys() {
    operator_io::MockOperatorInput input;
    std::vector<operator_io::OperatorKey> received;

    input.onKey([&](operator_io::OperatorKey key) {
        received.push_back(key);
    });

    input.injectKey(operator_io::OperatorKey::Quit);
    input.injectKey(operator_io::OperatorKey::ToggleKinect);
    input.injectKey(operator_io::OperatorKey::SwitchCamera);
    input.injectKey(operator_io::OperatorKey::Continue);
    input.injectKey(operator_io::OperatorKey::Spacebar);

    REQUIRE_TRUE(received.size() == 5);
}

void test_beeper_mock_records_beeps() {
    operator_io::MockBeeper beeper;
    std::vector<std::string> beeps;

    beeper.onBeep([&](const std::string& pattern) {
        beeps.push_back(pattern);
    });

    beeper.beep("short");
    beeper.beep("long");
    beeper.beep("double");

    REQUIRE_TRUE(beeps.size() == 3);
    REQUIRE_TRUE(beeps[0] == "short");
    REQUIRE_TRUE(beeps[1] == "long");
    REQUIRE_TRUE(beeps[2] == "double");
}

void test_beeper_mock_silence_until_beeped() {
    operator_io::MockBeeper beeper;
    int count = 0;

    beeper.onBeep([&](const std::string&) { ++count; });
    REQUIRE_TRUE(count == 0);

    beeper.beep("arrival");
    REQUIRE_TRUE(count == 1);
}

void test_operator_io_headless_text_dashboard_renders_phase() {
    operator_io::HeadlessDashboard dashboard;

    std::string output = dashboard.renderPhase("Driving", 1, 48.123, 17.456);
    REQUIRE_TRUE(!output.empty());
    REQUIRE_TRUE(output.find("Driving") != std::string::npos);
    REQUIRE_TRUE(output.find("leg 1") != std::string::npos);
}
