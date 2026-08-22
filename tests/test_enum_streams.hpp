#pragma once

/// Stream operators so REQUIRE_EQ can print the library's enums by name
/// instead of failing to compile. Test-only: the library itself has no reason
/// to pull <ostream> into its public headers.

#include <rozeta/core.hpp>
#include <rozeta/faults.hpp>
#include <rozeta/gps_gate.hpp>
#include <rozeta/health.hpp>
#include <rozeta/safety_state.hpp>

#include <ostream>

namespace rozeta {

inline std::ostream& operator<<(std::ostream& out, ErrorCode code)
{
    return out << static_cast<int>(code);
}

namespace health {
inline std::ostream& operator<<(std::ostream& out, HealthState state)
{
    return out << toString(state);
}
} // namespace health

namespace safety {
inline std::ostream& operator<<(std::ostream& out, SafetyState state)
{
    return out << toString(state);
}
} // namespace safety

namespace gps {
inline std::ostream& operator<<(std::ostream& out, FixRejectReason reason)
{
    return out << toString(reason);
}
} // namespace gps

namespace faults {
inline std::ostream& operator<<(std::ostream& out, FaultType type)
{
    return out << toString(type);
}
} // namespace faults

namespace motors {
inline std::ostream& operator<<(std::ostream& out, Direction direction)
{
    return out << static_cast<int>(direction);
}
} // namespace motors

} // namespace rozeta
