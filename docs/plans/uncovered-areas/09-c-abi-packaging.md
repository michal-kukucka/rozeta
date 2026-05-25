# M9 — Stable C ABI, Install and Export Packaging Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Make Rozeta consumable by C and downstream CMake projects with install targets, exported package config, versioning, and a broader stable C ABI.

Status: completed locally. Delivered expanded value-type C ABI helpers, C smoke tests/examples, install/export CMake package config and downstream consumer verification.

**Architecture:** Keep C++ as primary implementation. C ABI functions wrap stable, value-type operations only; avoid exposing C++ ownership or templates through C.

**Tech Stack:** CMake install/export, CPack optional, C ABI smoke tests, Doxygen.

---

## Gap evidence

- `include/rozeta/c_api.h` currently exposes only pose/obstacle structs and `rozeta_version`.
- No install/export package config exists.

## Tasks

1. Add C smoke test compiling with `cc` against `include/rozeta/c_api.h`.
2. Implement missing `rozeta_version()` if not already exported through the library.
3. Add C wrappers for pure functions: angle normalization, distance, obstacle sector calculation where safe.
4. Add CMake `install(TARGETS ...)`, `install(DIRECTORY include/)`, and package config files.
5. Add `examples/c_api_smoke.c`.
6. Add docs: install from source, consume with `find_package(rozeta CONFIG REQUIRED)`.
7. Update Doxygen groups for C and C++ APIs.

## Verification

```bash
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/tmp/rozeta-install -DROZETA_BUILD_TESTS=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
cmake --install build
cmake -S examples/consumer -B /tmp/rozeta-consumer -DCMAKE_PREFIX_PATH=/tmp/rozeta-install
```

## Acceptance criteria

- Downstream CMake consumer builds from installed package.
- C ABI smoke example links and runs.
- Public install docs are accurate.
