#include <rozeta/operator_io.hpp>

#include <sstream>

namespace rozeta::operator_io {

// ── MockOperatorInput ────────────────────────────────────────────

void MockOperatorInput::onKey(std::function<void(OperatorKey)> handler) {
    handlers_.push_back(std::move(handler));
}

void MockOperatorInput::injectKey(OperatorKey key) {
    for (auto& handler : handlers_) {
        handler(key);
    }
}

// ── MockBeeper ───────────────────────────────────────────────────

void MockBeeper::beep(const std::string& pattern) {
    for (auto& listener : listeners_) {
        listener(pattern);
    }
}

void MockBeeper::onBeep(std::function<void(const std::string&)> listener) {
    listeners_.push_back(std::move(listener));
}

// ── HeadlessDashboard ────────────────────────────────────────────

std::string HeadlessDashboard::renderPhase(
    const std::string& phase,
    int leg,
    double lat,
    double lon) const {
    std::ostringstream os;
    os << "Phase: " << phase
       << " | leg " << leg
       << " | pos " << lat << "," << lon;
    return os.str();
}

} // namespace rozeta::operator_io
