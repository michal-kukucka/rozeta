#pragma once

#include <rozeta/core.hpp>

#include <functional>
#include <string>
#include <vector>

namespace rozeta::operator_io {

enum class OperatorKey {
    Quit,
    ToggleKinect,
    SwitchCamera,
    Continue,
    Spacebar,
};

class OperatorInput {
public:
    virtual ~OperatorInput() = default;
    virtual void onKey(std::function<void(OperatorKey)> handler) = 0;
};

class MockOperatorInput final : public OperatorInput {
public:
    void onKey(std::function<void(OperatorKey)> handler) override;
    void injectKey(OperatorKey key);

private:
    std::vector<std::function<void(OperatorKey)>> handlers_;
};

class Beeper {
public:
    virtual ~Beeper() = default;
    virtual void beep(const std::string& pattern) = 0;
    virtual void onBeep(std::function<void(const std::string&)> listener) = 0;
};

class MockBeeper final : public Beeper {
public:
    void beep(const std::string& pattern) override;
    void onBeep(std::function<void(const std::string&)> listener) override;

private:
    std::vector<std::function<void(const std::string&)>> listeners_;
};

class HeadlessDashboard {
public:
    std::string renderPhase(
        const std::string& phase,
        int leg,
        double lat,
        double lon) const;
};

} // namespace rozeta::operator_io
