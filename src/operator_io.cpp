#include <rozeta/operator_io.hpp>

#include <sstream>

namespace rozeta::operator_io {

namespace {

std::string sanitizeOperatorText(const std::string& text) {
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

const char* stepLabel(OperatorWizardStep step) {
    switch (step) {
    case OperatorWizardStep::VerifyEstop:
        return "verify-estop";
    case OperatorWizardStep::ConfirmLiftedWheels:
        return "confirm-lifted-wheels";
    case OperatorWizardStep::LoadFieldPreset:
        return "load-field-preset";
    case OperatorWizardStep::StartMission:
        return "start-mission";
    case OperatorWizardStep::Complete:
        return "complete";
    case OperatorWizardStep::Aborted:
        return "aborted";
    }
    return "unknown";
}

std::string promptForStep(OperatorWizardStep step) {
    switch (step) {
    case OperatorWizardStep::VerifyEstop:
        return "Verify physical E-STOP is released, then press Continue.";
    case OperatorWizardStep::ConfirmLiftedWheels:
        return "Confirm lifted wheels for motor checks, then press Continue.";
    case OperatorWizardStep::LoadFieldPreset:
        return "Load and review the field preset, then press Continue.";
    case OperatorWizardStep::StartMission:
        return "Final operator check complete; press Continue to arm mission start.";
    case OperatorWizardStep::Complete:
        return "Operator wizard complete; mission may start.";
    case OperatorWizardStep::Aborted:
        return "Operator wizard aborted; mission start is blocked.";
    }
    return "Unknown operator wizard step.";
}

OperatorWizardState makeState(OperatorWizardStep step, const std::string& beep_pattern) {
    OperatorWizardState state;
    state.step = step;
    state.prompt = promptForStep(step);
    state.ready_to_start = step == OperatorWizardStep::Complete;
    state.aborted = step == OperatorWizardStep::Aborted;
    state.beep_pattern = beep_pattern;
    return state;
}

OperatorWizardStep nextStep(OperatorWizardStep step) {
    switch (step) {
    case OperatorWizardStep::VerifyEstop:
        return OperatorWizardStep::ConfirmLiftedWheels;
    case OperatorWizardStep::ConfirmLiftedWheels:
        return OperatorWizardStep::LoadFieldPreset;
    case OperatorWizardStep::LoadFieldPreset:
        return OperatorWizardStep::StartMission;
    case OperatorWizardStep::StartMission:
        return OperatorWizardStep::Complete;
    case OperatorWizardStep::Complete:
    case OperatorWizardStep::Aborted:
        return step;
    }
    return OperatorWizardStep::Aborted;
}

} // namespace

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

// ── FieldOperatorWizard ──────────────────────────────────────────

FieldOperatorWizard::FieldOperatorWizard()
    : state_(makeState(OperatorWizardStep::VerifyEstop, "")) {}

OperatorWizardState FieldOperatorWizard::state() const {
    return state_;
}

OperatorWizardState FieldOperatorWizard::handleKey(OperatorKey key) {
    if (state_.step == OperatorWizardStep::Complete || state_.step == OperatorWizardStep::Aborted) {
        return state_;
    }

    if (key == OperatorKey::Quit || key == OperatorKey::Spacebar) {
        state_ = makeState(OperatorWizardStep::Aborted, "long");
        return state_;
    }

    if (key != OperatorKey::Continue) {
        state_.beep_pattern = "invalid";
        return state_;
    }

    const auto next = nextStep(state_.step);
    state_ = makeState(next, next == OperatorWizardStep::Complete ? "double" : "short");
    return state_;
}

std::string renderOperatorWizard(const OperatorWizardState& state) {
    std::ostringstream os;
    os << "ROZETA FIELD OPERATOR WIZARD\n"
       << "step: " << stepLabel(state.step) << "\n"
       << "prompt: " << sanitizeOperatorText(state.prompt) << "\n"
       << "ready_to_start: " << (state.ready_to_start ? "yes" : "no") << "\n"
       << "aborted: " << (state.aborted ? "yes" : "no") << "\n"
       << "beep: " << sanitizeOperatorText(state.beep_pattern) << "\n";
    return os.str();
}

} // namespace rozeta::operator_io
