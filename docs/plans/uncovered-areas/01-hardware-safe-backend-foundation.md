# M1 — Hardware-safe Backend Foundation Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Add shared Linux/POSIX backend primitives so real hardware modules can be implemented safely without duplicating serial, timeout, error, and lifecycle handling.

**Architecture:** Introduce small internal utilities under `src/internal/` and optionally `include/rozeta/backends/` only if public configuration structs are needed. Keep public module APIs stable unless a missing configuration contract is discovered through tests.

**Tech Stack:** C++17, POSIX file descriptors, termios, chrono, CTest.

---

## Gap evidence

- `docs/architecture.md` says real hardware backends should be added behind interfaces.
- Motor/GPS/LiDAR docs all mention future serial/device backends.
- No shared serial helper exists today, so M2/M3/M4 would otherwise duplicate risky code.

## Tasks

### Task 1: Add internal serial-port contract tests

**Objective:** Define open/read/write/timeout behavior without requiring hardware.

**Files:**
- Create: `tests/test_serial_port.cpp`
- Modify: `tests/CMakeLists.txt`

**Steps:**
1. Write tests around a pseudo-terminal pair or an injectable fake descriptor.
2. Assert configured baud, timeout return, write round-trip, and close idempotency.
3. Run `cmake --build build --parallel 2 && ctest --test-dir build -R serial --output-on-failure`; expect RED.

### Task 2: Implement internal serial utility

**Objective:** Provide reusable POSIX serial operations for hardware backends.

**Files:**
- Create: `src/internal/serial_port.hpp`
- Create: `src/internal/serial_port.cpp`
- Modify: `CMakeLists.txt`

**Steps:**
1. Implement RAII close behavior.
2. Map invalid devices/timeouts to `Status` with clear `ErrorCode`.
3. Support baud rates needed by motor/GPS/YDLIDAR.
4. Run serial tests; expect GREEN.

### Task 3: Add backend lifecycle and safety docs

**Objective:** Document how optional backends must fail safely.

**Files:**
- Modify: `docs/architecture.md`
- Modify: `docs/maintenance.md`
- Modify: `docs/diagrams/module-map.html`

**Verification:**
- `python3 scripts/verify_docs.py`
- `doxygen Doxyfile`

## Acceptance criteria

- Internal serial helper has deterministic tests without real devices.
- Hardware-unavailable conditions return `Status` errors, never crashes.
- No new mandatory runtime dependencies.
- Docs explain backend lifecycle, permissions, timeouts and safe fallback.

## Implementation progress

Status: completed.

Research summary:

- ROS 2/YARP-style backend lifecycle inspired the open/configure/read-write/close separation and `Status`-based failure reporting.
- WPILib/libserial-style serial wrappers inspired the small RAII class shape and explicit serial settings.
- Robotics testing practice favored pseudo-terminal tests for transport behavior and fake transports for future protocol layers.

Implemented files:

- `src/internal/serial_port.hpp`
- `src/internal/serial_port.cpp`
- `tests/test_serial_port.cpp`

Modified files:

- `include/rozeta/core.hpp` — appended `ErrorCode::Timeout`.
- `CMakeLists.txt` — introduced shared `ROZETA_SOURCES` and built serial utility into static/shared libraries.
- `tests/CMakeLists.txt` and `tests/test_main.cpp` — added serial test coverage and `rozeta_serial_tests` CTest selector.
- `docs/architecture.md`, `docs/maintenance.md`, `docs/api-reference.md`, `docs/diagrams/module-map.html` — documented backend lifecycle, timeouts and safety constraints.

Acceptance criteria status:

- [x] Internal serial helper has deterministic tests without real devices.
- [x] Hardware-unavailable conditions return `Status` errors, never crashes.
- [x] No new mandatory runtime dependencies.
- [x] Docs explain backend lifecycle, permissions, timeouts and safe fallback.

Verification:

```bash
ctest --test-dir build-m1 -R serial --output-on-failure
```
