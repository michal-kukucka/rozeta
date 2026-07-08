# Release checklist

M9 — Release Universal Portability Version packages Rozeta as one same-repository C/C++ library that downstream consumers can use from Linux or Windows. Releases are dry-run-first: Do not create or push a git tag until maintainer approval.

## Release candidate preflight

Use the scripted preflight first. The default mode prints commands only:

```bash
python3 scripts/verify_release_readiness.py --dry-run
```

Execute the full Linux release/install/consumer preflight on the release machine:

```bash
python3 scripts/verify_release_readiness.py --run
```

The script verifies the default dependency-free profile with optional dependencies off:

- `ROZETA_WITH_OPENCV=OFF`
- `ROZETA_WITH_KINECT=OFF`
- `ROZETA_WITH_LIBTORCH=OFF`

## Required local verification

From the repository root, the release preflight expands to these gates:

```bash
cmake -S . -B build-release-linux \
  -DROZETA_BUILD_TESTS=ON \
  -DROZETA_BUILD_EXAMPLES=ON \
  -DROZETA_WITH_OPENCV=OFF \
  -DROZETA_WITH_KINECT=OFF \
  -DROZETA_WITH_LIBTORCH=OFF
cmake --build build-release-linux --parallel 2
ctest --test-dir build-release-linux --output-on-failure
python3 scripts/verify_docs.py
git diff --check
```

## Install and consumer verification

The release must install package exports and prove that an external project can consume them. The Linux preflight uses:

```bash
cmake -S . -B build-release-install \
  -DCMAKE_INSTALL_PREFIX="$PWD/build-release-prefix" \
  -DROZETA_BUILD_TESTS=ON \
  -DROZETA_BUILD_EXAMPLES=ON \
  -DROZETA_WITH_OPENCV=OFF \
  -DROZETA_WITH_KINECT=OFF
cmake --build build-release-install --parallel 2
ctest --test-dir build-release-install --output-on-failure
cmake --install build-release-install --prefix "$PWD/build-release-prefix"
cmake -S examples/consumer -B build-release-consumer \
  -DCMAKE_PREFIX_PATH="$PWD/build-release-prefix"
cmake --build build-release-consumer --parallel 2
```

The consumer fixture must continue to cover both public package entry points:

- `find_package(rozeta CONFIG REQUIRED)`
- `consumer_c`
- `consumer_cpp`

On Windows/MSVC, run the same install/consumer pattern from Developer PowerShell/cmd and keep multi-config flags explicit:

```powershell
cmake -S . -B build-release-install `
  -DCMAKE_INSTALL_PREFIX=$PWD/build-release-prefix `
  -DROZETA_BUILD_TESTS=ON `
  -DROZETA_BUILD_EXAMPLES=ON `
  -DROZETA_WITH_OPENCV=OFF `
  -DROZETA_WITH_KINECT=OFF
cmake --build build-release-install --config Release --parallel 2
ctest --test-dir build-release-install -C Release --output-on-failure
cmake --install build-release-install --config Release --prefix $PWD/build-release-prefix
cmake -S examples/consumer -B build-release-consumer `
  -DCMAKE_PREFIX_PATH=$PWD/build-release-prefix
cmake --build build-release-consumer --config Release --parallel 2
```

## Remote CI acceptance

Before tagging, GitHub Actions must be green on the release commit:

- Ubuntu Debug/Release CI must be green.
- Windows/MSVC Debug/Release CI must be green.
- Documentation verifier and CTest labels must pass in the matrix.
- Default CI must remain hardware-free and must not enable optional OpenCV, LibTorch, Kinect, YDLIDAR or LDROBOT dependencies.

## Supported profiles for release notes

Candidate tag name: `v0.1.0-universal`.

Supported default profile:

- Cross-platform core algorithms, package exports and public headers.
- Stable value-type C ABI for core math/runtime/safety helpers already exported through `ROZETA_C_API`.
- Windows-supported core/transport stack: Win32 serial, Winsock GPS network transport, package consumers and Debug/Release CI.
- Linux-proven hardware stack: POSIX serial, existing field hardware runbooks and Linux sample/replay tools.
- Optional backend profiles stay opt-in and are validated separately with `scripts/smoke_optional_backends.py`.

Explicit maturity limits:

- OpenCV and LibTorch are optional package profiles, not default release requirements.
- Kinect/libfreenect remains Linux verified and Windows experimental until a physical Windows device smoke is captured.
- Real serial LiDAR/motor hardware remains an operator smoke, while parser/replay and public APIs stay CI-gated.

## Optional generated API docs

If Doxygen is installed, also run one of:

```bash
doxygen Doxyfile
# or, when configured with Doxygen found:
cmake --build build-release-linux --target rozeta_docs
```

## Pre-tag checks

1. Confirm the working tree is clean except for intentional release edits before committing.
2. Run `python3 scripts/verify_release_readiness.py --run` on Linux.
3. Verify the latest GitHub Actions matrix for Ubuntu Debug/Release and Windows/MSVC Debug/Release.
4. Run `git status --short`, `git log --oneline -1` and `git rev-parse HEAD`.
5. Confirm `v0.1.0-universal` does not already exist:
   ```bash
   git rev-parse --verify v0.1.0-universal 2>/dev/null && echo "tag exists"
   ```
6. Do not create or push a git tag until maintainer approval.

## Safe tag publication after approval

There is no `git tag --dry-run` mode. After maintainer approval, create the tag locally and dry-run the push before publishing:

```bash
git tag -a v0.1.0-universal -m "Rozeta universal portability release"
git push --dry-run origin v0.1.0-universal
git push origin v0.1.0-universal
```
