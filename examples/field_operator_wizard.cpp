#include <rozeta/operator_io.hpp>

#include <cctype>
#include <iostream>
#include <sstream>
#include <string>

namespace {

struct ParsedKey {
    rozeta::operator_io::OperatorKey key{rozeta::operator_io::OperatorKey::Quit};
    bool valid{false};
};

std::string trim(std::string token) {
    while (!token.empty() && std::isspace(static_cast<unsigned char>(token.front()))) {
        token.erase(token.begin());
    }
    while (!token.empty() && std::isspace(static_cast<unsigned char>(token.back()))) {
        token.pop_back();
    }
    return token;
}

std::string sanitizeDiagnostic(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (const unsigned char ch : text) {
        if ((ch < 0x20) || (ch >= 0x7f)) {
            out.push_back('?');
        } else {
            out.push_back(static_cast<char>(ch));
        }
    }
    return out;
}

ParsedKey parseKey(const std::string& raw_token) {
    using rozeta::operator_io::OperatorKey;

    const auto token = trim(raw_token);
    if (token == "continue" || token == "c") {
        return {OperatorKey::Continue, true};
    }
    if (token == "quit" || token == "q") {
        return {OperatorKey::Quit, true};
    }
    if (token == "space") {
        return {OperatorKey::Spacebar, true};
    }
    if (token == "kinect") {
        return {OperatorKey::ToggleKinect, true};
    }
    if (token == "camera") {
        return {OperatorKey::SwitchCamera, true};
    }

    return {OperatorKey::Quit, false};
}

bool runScripted(const std::string& script) {
    rozeta::operator_io::FieldOperatorWizard wizard;
    auto state = wizard.state();
    std::cout << rozeta::operator_io::renderOperatorWizard(state);

    std::istringstream input(script);
    std::string token;
    while (std::getline(input, token, ',')) {
        const auto parsed = parseKey(token);
        if (!parsed.valid) {
            std::cerr << "unknown wizard script token: " << sanitizeDiagnostic(trim(token)) << "\n";
            state = wizard.handleKey(rozeta::operator_io::OperatorKey::Quit);
            std::cout << rozeta::operator_io::renderOperatorWizard(state);
            return false;
        }

        state = wizard.handleKey(parsed.key);
        std::cout << rozeta::operator_io::renderOperatorWizard(state);
        if (state.ready_to_start || state.aborted) {
            break;
        }
    }

    return state.ready_to_start;
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--script") {
        return runScripted(argv[2]) ? 0 : 1;
    }

    if (argc != 1) {
        std::cerr << "usage: field_operator_wizard [--script continue,continue,continue,continue]\n";
        return 1;
    }

    rozeta::operator_io::FieldOperatorWizard wizard;
    auto state = wizard.state();
    std::cout << rozeta::operator_io::renderOperatorWizard(state);
    std::cout << "type c=continue, q=quit, space=abort, camera/kinect=invalid demo key\n";

    std::string token;
    while (!state.ready_to_start && !state.aborted && std::cin >> token) {
        const auto parsed = parseKey(token);
        if (!parsed.valid) {
            std::cerr << "unknown wizard input token: " << sanitizeDiagnostic(trim(token)) << "\n";
            state = wizard.handleKey(rozeta::operator_io::OperatorKey::Quit);
        } else {
            state = wizard.handleKey(parsed.key);
        }
        std::cout << rozeta::operator_io::renderOperatorWizard(state);
    }

    return state.ready_to_start ? 0 : 1;
}
