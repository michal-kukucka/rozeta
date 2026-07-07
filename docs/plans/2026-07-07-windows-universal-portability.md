# Rozeta Windows 10/11 Universal Portability Milestones Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Make Rozeta usable from the same repository on Linux and Windows 10/11 without creating a separate Windows fork.

**Architecture:** Keep the current public C++17 module APIs and C ABI stable, then move OS-specific code behind small internal platform adapters. The default library should remain dependency-light and buildable on both Linux and Windows; hardware backends stay optional and are compiled only when their platform/dependency contract is satisfied.

**Tech Stack:** CMake 3.16+, C++17, CTest, GitHub Actions matrix, Windows MSVC/MinGW, POSIX serial/socket APIs on Linux, Win32 serial API and Winsock on Windows.

---

## Observation Summary

### Current repository state checked

- Repository: `/home/michal/projects/rozeta`
- Remote: `https://github.com/michal-kukucka/rozeta.git`
- HEAD checked during observation: `1142ed8 feat: add optional libtorch perception backend`
- Working tree before this plan: clean.
- Baseline Linux verification command run:
  - `cmake -S . -B build-windows-portability-observation -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON`
  - `cmake --build build-windows-portability-observation --parallel 2`
  - `ctest --test-dir build-windows-portability-observation --output-on-failure`
- Baseline result: `100% tests passed, 0 tests failed out of 12` on Linux/GCC 13.3.

### Is same-repository multiplatform possible?

Yes, bro — this is absolutely feasible in the same repository. Rozeta already has the right shape:

- Public headers mostly expose value types, interfaces, `Status`, and dependency-free algorithms.
- Hardware integrations are already optional through CMake flags such as `ROZETA_WITH_OPENCV`, `ROZETA_WITH_SERIAL_MOTORS`, `ROZETA_WITH_YDLIDAR`, `ROZETA_WITH_LDROBOT_LIDAR`, and `ROZETA_WITH_KINECT`.
- CMake already exports installable package targets with `find_package(rozeta CONFIG REQUIRED)` and `rozeta::rozeta`.
- Tests are mostly dependency-free and use mocks/fixtures.
- The UI snapshot layer is render-backend neutral.

A new repository is **not needed**. The correct path is one repository with platform-specific internals, CI matrix, and explicit capability flags.

### Main portability blockers found

#### P0 blockers: code that will not compile on native Windows today

- `src/internal/serial_port.cpp`
  - Includes POSIX-only headers: `<poll.h>`, `<termios.h>`, `<unistd.h>`, `<fcntl.h>`.
  - Uses POSIX types/functions: `speed_t`, `B9600`, `pollfd`, `poll`, `open`, `read`, `write`, `close`, `tcgetattr`, `cfmakeraw`, `cfsetispeed`, `tcsetattr`, `tcflush`.
  - Public internal header stores `int fd_`, which is POSIX-specific and leaks the handle model into the class layout.

- `src/gps.cpp`
  - Includes POSIX socket and file-control headers: `<fcntl.h>`, `<poll.h>`, `<sys/socket.h>`, `<sys/time.h>`, `<arpa/inet.h>`, `<unistd.h>`.
  - Uses `sockaddr_in`, `AF_INET`, `SOCK_STREAM`, `SOCK_DGRAM`, `setsockopt`, `inet_pton`, `socket`, `bind`, `connect`, `recv`, `getsockopt`, `close`, `fcntl`, `poll`, `errno`, `EINPROGRESS`, `EAGAIN`, `EWOULDBLOCK`.
  - Windows needs Winsock startup/cleanup, `SOCKET`, `closesocket`, `ioctlsocket`, `WSAGetLastError`, `WSAEWOULDBLOCK`, and different timeout option typing.

- Tests with POSIX-only setup:
  - `tests/test_serial_port.cpp`: `<termios.h>`, `<unistd.h>`, `<fcntl.h>`.
  - `tests/test_network_gps.cpp`: `<arpa/inet.h>`, `<sys/socket.h>`, `<unistd.h>`.
  - `tests/test_gps_receiver.cpp`: `<fcntl.h>`, `<unistd.h>`.
  - `tests/test_motor_calibration.cpp`: `<unistd.h>`.
  - `tests/test_calibration.cpp`: `<unistd.h>`.

#### P1 blockers: CMake/toolchain assumptions

- Top-level `CMakeLists.txt` applies `-Wall -Wextra -Wpedantic` unconditionally to all targets. MSVC requires `/W4` or similar instead.
- `target_link_libraries(rozeta_static PUBLIC m stdc++)` is Linux-specific and currently guarded by `UNIX AND NOT APPLE`, so Windows is safe there, but it should be centralized with the rest of platform logic.
- Shared library exports are not Windows-ready yet. A Windows DLL needs exported symbols, either via `WINDOWS_EXPORT_ALL_SYMBOLS` for a quick first pass or explicit `ROZETA_API` annotations for stable ABI quality.
- CI only runs Ubuntu Debug/Release. There is no Windows matrix, no MSVC compile, and no MinGW cross/smoke job.

#### P1 blockers: default device naming and docs

- Public defaults and docs are Linux-first:
  - `GpsReceiverConfig::device` defaults to `/dev/ttyUSB0`.
  - README build examples use Unix `mkdir`, `cd`, `make`, and Unix-style executable paths.
  - Architecture docs explicitly say Linux-first.
- Windows docs need COM-port examples such as `COM3` or `\\.\COM10`, PowerShell commands, Visual Studio generator examples, and vcpkg dependency notes for optional backends.

#### P2 blockers: optional third-party backend policy

- OpenCV is cross-platform but dependency discovery/install instructions need vcpkg/Chocolatey/official CMake package guidance.
- LibTorch is cross-platform but heavier; Windows instructions need `CMAKE_PREFIX_PATH` and runtime DLL path notes.
- libfreenect/Kinect may be possible but should be marked Windows experimental until verified.
- Serial motor/YDLIDAR/LDROBOT backends become portable once the shared serial transport is portable.

---

## Target End State

By the end of this reimplementation, Rozeta should support these build profiles from one repository:

- Linux default: current behavior preserved.
- Windows default static library: builds with MSVC on Windows 10/11 with tests and examples that do not require real hardware.
- Windows default shared library: builds as DLL with documented exports and import library.
- Windows serial-enabled profile: builds serial motor/GPS/LiDAR serial backends using Win32 serial transport.
- Windows network GPS profile: builds TCP/UDP GPS receiver using Winsock.
- Optional dependency profiles: OpenCV/LibTorch/Kinect documented and gated per platform.

---

## Milestone 0: Protect the Current Baseline

**Objective:** Freeze current Linux behavior before portability refactors.

**Files:**
- Modify: `.gitignore`
- Create: `docs/plans/2026-07-07-windows-universal-portability.md` already created by this observation
- Modify: no source files yet

**Steps:**

1. Add `build-windows-portability-observation/` to `.gitignore` or delete the directory before commit.
2. Run current baseline:
   ```bash
   cmake -S . -B build-linux-baseline -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
   cmake --build build-linux-baseline --parallel 2
   ctest --test-dir build-linux-baseline --output-on-failure
   python3 scripts/verify_docs.py
   git diff --check
   ```
3. Expected: all tests pass; docs contract passes; no whitespace errors.
4. Commit:
   ```bash
   git add docs/plans/2026-07-07-windows-universal-portability.md .gitignore
   git commit -m "docs: plan Windows portability milestones"
   ```

**Verification:** Linux baseline remains green before any platform split.

---

## Milestone 1: Add Platform Detection and Compiler-Portable Warnings

**Objective:** Make CMake express supported platforms cleanly and stop passing GCC warning flags to MSVC.

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `examples/CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Create: `cmake/RozetaCompilerOptions.cmake`
- Create: `cmake/RozetaPlatform.cmake`

**Tasks:**

1. Create `cmake/RozetaPlatform.cmake` with normalized booleans:
   - `ROZETA_PLATFORM_WINDOWS`
   - `ROZETA_PLATFORM_POSIX`
   - `ROZETA_PLATFORM_LINUX`
   - `ROZETA_PLATFORM_MACOS`
2. Create `cmake/RozetaCompilerOptions.cmake` with function `rozeta_apply_warnings(target)`:
   - MSVC: `/W4 /permissive-`
   - GNU/Clang: `-Wall -Wextra -Wpedantic`
3. Replace all direct `target_compile_options(... -Wall -Wextra -Wpedantic)` calls with `rozeta_apply_warnings(target)`.
4. Keep Linux `m` linking guarded behind `ROZETA_PLATFORM_LINUX`.
5. Configure/build on Linux.
6. Configure on Windows later with:
   ```powershell
   cmake -S . -B build-win -G "Visual Studio 17 2022" -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
   cmake --build build-win --config Debug --parallel
   ```

**Verification:** Linux build output remains warning-clean; Windows configure no longer fails on `-Wall` flags.

---

## Milestone 2: Split Internal Serial Transport by Platform

**Objective:** Keep `rozeta::internal::SerialPort` API stable while moving OS-specific implementation into POSIX and Win32 files.

**Files:**
- Modify: `src/internal/serial_port.hpp`
- Rename/split: `src/internal/serial_port.cpp`
- Create: `src/internal/serial_port_posix.cpp`
- Create: `src/internal/serial_port_win32.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/test_serial_port.cpp`
- Modify: `tests/test_gps_receiver.cpp`

**Tasks:**

1. Change `SerialPort` private native handle storage from `int fd_` to an opaque pointer/private `Impl` or platform-neutral integer type guarded internally.
2. Keep public internal methods stable:
   - `Status open(const SerialPortConfig&)`
   - `void close() noexcept`
   - `bool isOpen() const noexcept`
   - `Status readSome(...)`
   - `Status writeAll(...)`
3. Move current POSIX code unchanged in behavior to `serial_port_posix.cpp`.
4. Add Windows implementation using Win32 APIs:
   - `CreateFileA` for `COMx` / `\\.\COM10` paths
   - `GetCommState` / `SetCommState`
   - `COMMTIMEOUTS`
   - `ReadFile` / `WriteFile`
   - `CloseHandle`
   - Map Windows errors to existing `ErrorCode` values.
5. Update CMake source selection:
   - POSIX: `src/internal/serial_port_posix.cpp`
   - Windows: `src/internal/serial_port_win32.cpp`
6. Split tests into portable validation tests and platform-only pseudo-terminal tests:
   - POSIX pseudo-terminal tests compile only on POSIX.
   - Windows tests cover invalid COM device, unsupported baud, timeout config, and no-hardware status mapping.

**Verification:**

- Linux serial tests still pass.
- Windows default build compiles serial transport.
- Invalid device tests return `HardwareUnavailable`, not a crash or exception.

---

## Milestone 3: Split GPS Network Transport by Platform

**Objective:** Make `NetworkGpsReceiver` work on Windows via Winsock and on Linux via current POSIX sockets.

**Files:**
- Modify: `src/gps.cpp`
- Create: `src/internal/socket_transport.hpp`
- Create: `src/internal/socket_transport_posix.cpp`
- Create: `src/internal/socket_transport_win32.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/test_network_gps.cpp`

**Tasks:**

1. Extract socket operations from `src/gps.cpp` into a minimal internal transport layer:
   - create socket
   - set receive timeout
   - set nonblocking
   - connect with timeout
   - bind UDP
   - recv bytes
   - close socket
   - last-error-to-`Status`
2. Implement POSIX transport with current behavior preserved.
3. Implement Windows transport with Winsock:
   - RAII `WSAStartup` / `WSACleanup` guard
   - `SOCKET` handle
   - `closesocket`
   - `ioctlsocket(FIONBIO)`
   - `select` or `WSAPoll` for connect timeout
   - `WSAGetLastError` mappings
4. Link `Ws2_32` on Windows only.
5. Keep public `NetworkGpsReceiver` API unchanged.
6. Update tests:
   - Use portable C++ test helper for UDP/TCP local loopback where possible.
   - Guard any POSIX-only helper code.

**Verification:**

- Linux network GPS tests still pass.
- Windows network GPS tests can bind/read localhost UDP/TCP payloads.
- No Winsock symbols leak into public headers.

---

## Milestone 4: Make Tests and Examples Platform-Aware

**Objective:** Ensure `ROZETA_BUILD_TESTS=ON` and `ROZETA_BUILD_EXAMPLES=ON` are useful on Windows without manual pruning.

**Files:**
- Modify: `tests/CMakeLists.txt`
- Modify: `examples/CMakeLists.txt`
- Modify: POSIX-specific tests listed above
- Modify: examples that assume Unix paths or executable behavior if any are found during Windows compile

**Tasks:**

1. Add CMake labels for tests:
   - `unit`
   - `portable`
   - `posix`
   - `windows`
   - `hardware-optional`
2. Ensure default `ctest` on Windows excludes tests requiring POSIX pseudo-terminals or Unix shell scripts.
3. Replace shell-script tests with Python equivalents where practical, especially `scripts/smoke_opencv_qr_stub.sh` if it should run on Windows.
4. Keep examples compiling by default; only hardware-specific examples are gated by backend options.
5. Add Windows-safe fixture paths using `std::filesystem::path` where examples/tests concatenate paths.

**Verification:**

- Linux: all current default tests still pass.
- Windows: `ctest -C Debug --output-on-failure` passes default portable tests.
- CI reports clear labels for skipped platform-specific tests.

---

## Milestone 5: Add Windows DLL/API Export Strategy

**Objective:** Support both static library consumers and Windows DLL consumers safely.

**Files:**
- Create: `include/rozeta/export.h`
- Modify: `include/rozeta/c_api.h`
- Modify: public C++ headers that should export classes/functions from DLL
- Modify: `CMakeLists.txt`
- Modify: `examples/consumer/CMakeLists.txt`

**Tasks:**

1. Add `ROZETA_API` macro:
   - static build: empty
   - Windows shared build while building library: `__declspec(dllexport)`
   - Windows shared consumer: `__declspec(dllimport)`
   - GCC/Clang optional visibility attributes if desired later
2. Apply `ROZETA_API` to C ABI functions first.
3. Decide whether full C++ API DLL export is required now; if yes, annotate public classes/functions module by module.
4. Add compile definitions:
   - `ROZETA_BUILDING_LIBRARY`
   - `ROZETA_STATIC_DEFINE` for static builds
5. Validate `examples/consumer` against installed static and shared builds.

**Verification:**

- Windows static consumer links.
- Windows shared consumer links to import library and runs with DLL in PATH.
- Linux shared/static behavior unchanged.

---

## Milestone 6: Add Windows CI Matrix

**Objective:** Make Windows support enforced by automation, not memory.

**Files:**
- Modify: `.github/workflows/ci.yml`
- Optional create: `.github/workflows/windows-portability.yml` if separate workflow is preferred

**Tasks:**

1. Expand CI matrix:
   - Ubuntu Debug/Release current jobs
   - Windows latest + MSVC Debug
   - Windows latest + MSVC Release
2. Use platform-specific configure/build/test commands:
   - Linux: existing commands
   - Windows:
     ```powershell
     cmake -S . -B build -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
     cmake --build build --config ${{ matrix.build_type }} --parallel 2
     ctest --test-dir build -C ${{ matrix.build_type }} --output-on-failure
     ```
3. Keep docs verifier running on both platforms if Python script is portable; otherwise run docs verifier on Ubuntu and add Windows path tests separately.
4. Add artifact upload for CMake configure logs on failure.

**Verification:** Pull requests cannot regress Windows compile/test support.

---

## Milestone 7: Update User Documentation for Universal Use

**Objective:** Document Rozeta as universal/multiplatform while being honest about hardware backend maturity.

**Files:**
- Modify: `README.md`
- Modify: `docs/architecture.md`
- Modify: `docs/module_overview.md`
- Modify: `docs/gps_module.md`
- Modify: `docs/motor_module.md`
- Modify: `docs/lidar_module.md`
- Modify: `docs/api-reference.md`
- Modify: `docs/index.html`

**Tasks:**

1. Replace “Linux-first” framing with “cross-platform core, Linux-proven hardware stack, Windows-supported core/transport stack”.
2. Add build sections:
   - Linux Make/Ninja
   - Windows Visual Studio/MSVC
   - Windows PowerShell commands
   - optional vcpkg dependency notes
3. Add serial device naming guide:
   - Linux: `/dev/ttyUSB0`, `/dev/serial/by-id/...`
   - Windows: `COM3`, `\\.\COM10`
4. Add capability matrix in Markdown bullets, not a hard-to-read table:
   - Core algorithms: Linux + Windows
   - C ABI: Linux + Windows
   - Serial GPS/motors/LiDAR: Linux + Windows after milestones 2/3
   - OpenCV: Linux + Windows with dependency installed
   - Kinect/libfreenect: Linux verified, Windows experimental until proven
5. Update release checklist to include Windows CI.

**Verification:** Docs contract passes and README commands are copy-pasteable for both OS families.

---

## Milestone 8: Optional Backend Validation on Windows

**Objective:** Verify optional dependencies individually instead of blocking the whole library on them.

**Files:**
- Modify: `CMakeLists.txt`
- Modify: backend docs
- Optional create: `docs/windows_optional_backends.md`

**Tasks:**

1. Validate OpenCV via vcpkg or official OpenCV package:
   ```powershell
   cmake -S . -B build-opencv-win -DROZETA_WITH_OPENCV=ON -DROZETA_BUILD_TESTS=ON
   cmake --build build-opencv-win --config Release --parallel
   ctest --test-dir build-opencv-win -C Release --output-on-failure
   ```
2. Validate LibTorch separately with `CMAKE_PREFIX_PATH`.
3. Validate serial LiDAR backends using sample replay first, real hardware later.
4. Mark Kinect/libfreenect support as experimental until a Windows device smoke test exists.
5. Do not make optional dependencies part of required default CI unless they are cached/reliable.

**Verification:** Optional backend failures produce clear CMake errors and do not break default builds.

---

## Milestone 9: Release Universal Portability Version

**Objective:** Package and release a version that consumers can confidently use from Linux or Windows.

**Files:**
- Modify: `docs/release.md`
- Modify: `README.md`
- Modify: version metadata if release process requires it

**Tasks:**

1. Run full local Linux verification:
   ```bash
   cmake -S . -B build-release-linux -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
   cmake --build build-release-linux --parallel 2
   ctest --test-dir build-release-linux --output-on-failure
   python3 scripts/verify_docs.py
   git diff --check
   ```
2. Run full Windows CI and inspect logs.
3. Install and consume from `examples/consumer` on Linux and Windows.
4. Document exact supported profiles in release notes.
5. Tag only after maintainer approval.

**Verification:** One repository, one CI matrix, platform-specific internals, stable public APIs, and green default Linux/Windows builds.

---

## Recommended Implementation Order

1. CMake platform/warnings cleanup.
2. Serial transport split.
3. Network socket transport split.
4. Test gating and portable test helpers.
5. DLL export/import support.
6. Windows CI.
7. Documentation update.
8. Optional backend validation.
9. Release.

This order keeps every commit small and keeps Linux green while Windows support is progressively unlocked.

---

## Risk Notes

- The largest real risk is not algorithms; it is OS I/O behavior: serial timeouts, nonblocking connect, socket errors, and Windows handle lifetime.
- Avoid scattering `#ifdef _WIN32` through robot modules. Keep conditionals inside `src/internal/*_posix.cpp`, `src/internal/*_win32.cpp`, and CMake source selection.
- Do not require OpenCV, LibTorch, Kinect, or real serial hardware in the default Windows build.
- Prefer static library support first; DLL exports should be explicit and tested before advertising shared-library ABI stability.
