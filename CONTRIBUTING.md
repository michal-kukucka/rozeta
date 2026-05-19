# Contributing to Rozeta

Rozeta is intentionally small, modular and Linux-first. Contributions should keep modules easy to test without physical robot hardware.

## Development workflow

1. Create a branch from `main`.
2. Add or update tests before changing behavior.
3. Keep public APIs in `include/rozeta/` and implementations in `src/`.
4. Keep hardware-specific code behind interfaces so modules remain mockable.
5. Update documentation in `docs/` when APIs, modules or examples change.

## Local verification

```bash
cmake -S . -B build -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/examples/robotour_demo
```

## Coding style

- Prefer clear C++17 over clever templates.
- Use dependency injection for hardware devices.
- Keep structs simple and explicit.
- Avoid mandatory heavyweight dependencies in core modules.
- Real hardware backends should be optional and isolated from the tested core.
