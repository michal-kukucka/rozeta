# Release checklist

Rozeta releases are intentionally dry-run-first. Do not create or push a git tag until every local and CI verification step below passes on the exact commit to be released.

## Required local verification

From the repository root:

```bash
cmake -S . -B build-m10 -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build-m10 --parallel 2
ctest --test-dir build-m10 --output-on-failure
./build-m10/examples/replay_robotour_log tests/fixtures/replay/basic_robotour.csv
python3 scripts/verify_docs.py
```

If Doxygen is installed, also run one of:

```bash
doxygen Doxyfile
# or, when configured with Doxygen found:
cmake --build build-m10 --target rozeta_docs
```

## Pre-tag checks

1. Confirm the working tree is clean except for intentional release edits before committing.
2. Run `git diff --check` before the release commit.
3. Review CI for the Debug/Release matrix and documentation verifier.
4. Verify the replay fixture uses schema `rozeta.telemetry.v1` and covers GPS, LiDAR/depth, pose, navigation decision and motor command columns.
5. Confirm `docs/index.html`, `README.md`, `docs/api-reference.md` and `docs/robotour_use_case.md` reference new public examples.
6. Confirm no unrelated interactive diagram work is included in the release-hardening change.

## Safe tag preflight

There is no `git tag --dry-run` mode. Use these checks first, then create and push
the tag only after maintainer approval.

```bash
git status --short
git rev-parse --verify v0.1.0-m10 2>/dev/null && echo "tag exists"
git log --oneline -1
```

After the maintainer approves a real release, create the tag locally and dry-run
the push before publishing:

```bash
git tag -a v0.1.0-m10 -m "Rozeta M10 telemetry replay release hardening"
git push --dry-run origin v0.1.0-m10
git push origin v0.1.0-m10
```
