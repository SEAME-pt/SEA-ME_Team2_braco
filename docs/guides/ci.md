# Guide: CI, formatting and coverage

[![CI](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml)

Every pull request and every push to `main` runs [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml). No robot,
serial port or display is needed. Every job only calls `make`, so a red check can be reproduced on a workstation with
the same commands, from the repo folder.

## Jobs

| Job | What it runs | Reproduce locally |
|---|---|---|
| `host` | `make ci`: `make format-check` (C++ and Python formatting, see [Code formatting](#code-formatting)), `make lint` (compose files for both machines, ruff check of the Python outside `ros2_ws/src`, shellcheck, hadolint) and `make web-test` (pytest `web/tests`, in a venv from `web/requirements*.txt`, with the coverage of `web/server`, failing below `COV_MIN_WEB`), one step each, all run even when one fails; creates `docker/.env` if missing | `make ci`, or one of its three targets |
| `ros` | Builds the Dockerfile's `base` stage (Ubuntu 26.04 + ROS 2 Lyrical), then `rosdep install`, `colcon build` and `colcon test` of `ros2_ws/src` (with `mycobot_cpp`) in a throw-away container, the test results uploaded as the `test-results` artifact; then cppcheck on every C++ package, and clang-tidy (`ros2_ws/.clang-tidy`, warnings as errors) from that build's `compile_commands.json`. Both also run when a test failed | `make ci-ros` (builds the image the first time), then `make cppcheck` and `make tidy` |
| `coverage` | Same image as `ros`, in parallel: the GoogleTests built with `--coverage` in `ros2_ws/coverage/` (gcovr) and the pytests (pytest-cov), uploaded as the `coverage` artifact (HTML per line; Python in `python/`); fails below `COV_MIN_CXX`, or below `COV_MIN_PY` once there is ROS Python to measure (none yet, see [Coverage](#coverage)) | `make coverage` (report in `ros2_ws/coverage/html`) |

- `make ci` creates `.venv` from `web/requirements*.txt` by itself; `make setup` (on the PC) adds
  `requirements-dev.txt` to the same venv.
- shellcheck and hadolint need no install: without a local binary, `make lint` runs the images pinned in
  `SHELLCHECK_IMAGE` and `HADOLINT_IMAGE` through docker, so it never skips a check that CI runs.
- A PR that changes only files under `docs/` or `.md` files skips `ros` and `coverage` (a small `changes` job
  decides from `git diff`); a skipped job counts as passed for the required checks. `host` always runs, and pushes
  to `main` always run everything.
- Caches: pip (`actions/setup-python`), the image layers (BuildKit `type=gha`), and ccache + colcon `build/`
  (`actions/cache`, reset whenever `docker/` changes). A warm run only recompiles what changed.
- The arm64 Jetson image is not built in CI (QEMU is too slow); build it by hand with `make jetson-image`.
- `ci-ros` writes to `ros2_ws/build`, `install` and `log`, like `make build`, plus its ccache in `ros2_ws/.ccache`.

## Coverage

- Floors: `COV_MIN_CXX`, `COV_MIN_PY` and `COV_MIN_WEB` in [`mk/ci.mk`](../../mk/ci.mk), 80% for every component. A PR
  below a floor adds tests; the floors are not lowered to make it pass.
- `scripts/` directories are not measured, so the ROS Python (today only `braco_camera/scripts/camera.py`) has
  nothing to report yet, and `COV_MIN_PY` gates nothing until a ROS Python module outside `scripts/` has tests.
  Only coverage.py's "No data to report." skips it; any other coverage.py error fails the job.
- Each run's summary page shows the coverage totals against their floors, with the C++ per package
  (`scripts/coverage_summary.py`, from the JSON reports; written on failure too). Locally the table is dropped; to see
  it, run e.g. `GITHUB_STEP_SUMMARY=/dev/stdout make ci`.

## Code formatting

C++ is formatted by clang-format (`.clang-format`) and Python by ruff (`.ruff.toml`). Both are pinned in
`requirements-dev.txt`, so everyone formats the same way.

| Command | What it does |
|---|---|
| `make format` | Fixes the formatting of every C++ and Python file |
| `make format-check` | Only checks, changes nothing; prints `OK` or the files to fix |

- The pre-push hook (`.githooks/pre-push`, bound by `make setup`) runs `make format-check` and blocks the push if it
  fails. Fix with `make format`, then commit and push again.
- CI runs the same check (`make ci`, the `host` job), so a push that skipped the hook still fails on the PR.
- `ros2_ws/src` is skipped, except `ros2_ws/src/mycobot_cpp` (plain C++, no ament linters): the ROS 2 packages are
  checked by their own linters (`ament_uncrustify`, `ament_flake8`) in `make test`, because their style rules differ
  from clang-format and ruff.
