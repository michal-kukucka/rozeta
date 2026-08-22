#include <rozeta/clock.hpp>

#include <algorithm>

namespace rozeta {

SystemClock::SystemClock()
    : start_(std::chrono::steady_clock::now())
{
}

std::chrono::milliseconds SystemClock::nowMs() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_);
}

SimulatedClock::SimulatedClock(std::chrono::milliseconds start)
    : now_(std::max(std::chrono::milliseconds{0}, start))
{
}

std::chrono::milliseconds SimulatedClock::nowMs() const
{
    return now_;
}

void SimulatedClock::advance(std::chrono::milliseconds delta)
{
    if (delta.count() <= 0) {
        return;
    }
    now_ += delta;
}

void SimulatedClock::advanceSeconds(double seconds)
{
    if (!(seconds > 0.0)) {
        return;
    }
    advance(std::chrono::milliseconds{static_cast<long long>(seconds * 1000.0 + 0.5)});
}

void SimulatedClock::setNow(std::chrono::milliseconds value)
{
    now_ = std::max(now_, value);
}

void SimulatedClock::reset(std::chrono::milliseconds start)
{
    now_ = std::max(std::chrono::milliseconds{0}, start);
}

ClockPtr makeSystemClock()
{
    return std::make_shared<SystemClock>();
}

std::shared_ptr<SimulatedClock> makeSimulatedClock(std::chrono::milliseconds start)
{
    return std::make_shared<SimulatedClock>(start);
}

} // namespace rozeta
