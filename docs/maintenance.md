# Documentation maintenance contract

Rozeta documentation is treated as part of the product. Every implementation change that affects public APIs, modules, examples, payloads, build options, safety behavior, or user workflows must update documentation in the **same commit**.

## Required workflow for code changes

1. Change the code and tests.
2. Update the matching documentation page:
   - public header/API behavior → header comments + `docs/api-reference.md`
   - module responsibility/status → `docs/module_overview.md` and module page
   - Robotour/user flow → `docs/robotour_use_case.md`
   - architecture/data flow → `docs/architecture.md` and `docs/diagrams/module-map.html`
   - build/test/docs process → `README.md`, `CONTRIBUTING.md`, or this file
3. Run:

```bash
python3 scripts/verify_docs.py
cmake -S . -B build -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

4. If Doxygen is installed, also run:

```bash
doxygen Doxyfile
```

## Drift prevention rules

- Public headers in `include/rozeta/` must be listed in `scripts/verify_docs.py` with their canonical user-facing documentation page.
- Examples in `examples/` must be referenced from the README, Robotour use case, or API reference.
- Interactive diagrams must keep their data in `ROZETA_MODULE_MODEL` so layout and content can be reviewed in one place.
- Do not commit generated Doxygen output unless the release/page workflow explicitly asks for it. The source config is `Doxyfile`.
- Do not replace vector/HTML diagrams with raster-only screenshots.

## Review checklist

Before opening a PR or pushing to `main`:

- [ ] Public API comments and user-facing docs agree.
- [ ] New modules appear in the module overview and diagram model.
- [ ] New examples are mentioned and smoke-tested.
- [ ] `scripts/verify_docs.py` passes locally and in CI.
- [ ] CMake/CTest still pass.

## Backend implementation checklist

When adding a real hardware backend, keep the M1 transport contract intact:

- Start with a failing CTest that uses a mock, fixture file, or pseudo-terminal instead of requiring physical hardware.
- Keep device dependencies optional behind CMake flags.
- Map failures to `Status` and `ErrorCode`; do not throw across public module APIs.
- Use finite timeouts for serial reads/writes and document default values.
- Document Linux permissions, udev recommendations and hardware-unavailable behavior in the matching module doc.
- Add or update an opt-in smoke hook such as `scripts/smoke_ui_backends.sh`; default CI must still pass without hardware.
- Compile optional public headers with feature macros when practical, even if local libraries/devices are absent.
- Update `docs/diagrams/module-map.html` whenever a backend changes module relationships or data flow.
