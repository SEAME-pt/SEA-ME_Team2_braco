# Guide: CI, formatting and coverage

[![CI](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml)

Every pull request and every push to `main` runs [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml). No robot,
serial port or display is needed. Every job only calls `make`, so a red check can be reproduced on a workstation with
the same commands, from the repo folder.

## Jobs

| Job | What it runs | Reproduce locally |
|---|---|---|
| `host` | `make ci`: `make format-check` (C++ and Python formatting, see [Code formatting](#code-formatting)), `make lint` (compose files for both machines, ruff check of the Python outside `ros2_ws/src`, shellcheck, hadolint) and `make web-test` (pytest `web/tests`, in a venv from `web/requirements*.txt`, with the coverage of `web/server`, failing below `COV_MIN_WEB`) and `make timing-test` (pytest `scripts/timing/test_timing.py`: the reply framing and statistics of the bench scripts, no serial port, no ROS), one step each, all run even when one fails; creates `docker/.env` if missing | `make ci`, or one of its four targets |
| `ros` | Builds the Dockerfile's `ci` stage (`base`: Ubuntu 26.04 + ROS 2 Lyrical, plus clang-tidy, gcovr, pytest-cov and MoveIt 2), then `rosdep install`, `colcon build` and `colcon test` of `ros2_ws/src` (with `mycobot_cpp` and the [MoveIt test](#moveit-test)) in a throw-away container, the test results uploaded as the `test-results` artifact; then cppcheck on every C++ package, and clang-tidy (`ros2_ws/.clang-tidy`, warnings as errors) from that build's `compile_commands.json`. Both also run when a test failed | `make ci-ros` (builds the image the first time), then `make cppcheck` and `make tidy` |
| `coverage` | Same image as `ros`, in parallel: the GoogleTests built with `--coverage` in `ros2_ws/coverage/` (gcovr) and the pytests (pytest-cov), uploaded as the `coverage` artifact (HTML per line; Python in `python/`); fails below `COV_MIN_CXX` or `COV_MIN_PY` (the ROS Python includes the launch files, see [Coverage](#coverage)) | `make coverage` (report in `ros2_ws/coverage/html`) |

- `make ci` creates `.venv` from `web/requirements*.txt` by itself; `make setup` (on the PC) adds
  `requirements-dev.txt` to the same venv.
- shellcheck and hadolint need no install: without a local binary, `make lint` runs the images pinned in
  `SHELLCHECK_IMAGE` and `HADOLINT_IMAGE` through docker, so it never skips a check that CI runs.
- `make lint`, `make format` and `make format-check` take their files from `git ls-files`: a new file is skipped
  until it is `git add`ed. Add it first, or a check that passes locally can fail once the file is on the PR.
- A PR that changes only files under `docs/` or `.md` files skips `ros` and `coverage` (a small `changes` job
  decides from `git diff`); a skipped job counts as passed for the required checks. `host` always runs, and pushes
  to `main` always run everything.
- Caches: pip (`actions/setup-python`), the image layers (BuildKit `type=gha`), and ccache + colcon `build/`
  (`actions/cache`, reset when the Dockerfile, `docker/config/` or `docker/scripts/` change). A warm run only
  recompiles what changed. Only pushes to `main` save the image layers and the build caches: a PR reads `main`'s but
  can't share its own with other PRs, so saving them would only fill the 10 GB repo cache.
- The arm64 Jetson image is not built in CI (QEMU is too slow); build it by hand with `make jetson-image`.
- `ci-ros` writes to `ros2_ws/build`, `install` and `log`, like `make build`, plus its ccache in `ros2_ws/.ccache`.

## MoveIt test

`braco_moveit_config/test/test_plan_execute.launch.py` (#88) runs `demo.launch.py` headless (mock hardware, no RViz
or Gazebo), plans and executes to `park`, and checks the result and the final `/joint_states`.

- A hang fails after 120 s (`add_launch_test ... TIMEOUT 120`); a run takes about 6 s.
- It runs on `ROS_DOMAIN_ID` 29, kept free for it (`make lint` refuses it in `team-domains.tsv`), and discovers only
  on localhost, so it never meets a teammate's `sim`, `dev` or the real arm.
- `move_group`'s exit code is not checked: it segfaults on Ctrl+C/SIGINT (MoveIt 2.15), after the test.
- Alone, in `make shell`: `colcon test --packages-select braco_moveit_config --ctest-args -R plan_execute`, then
  `colcon test-result --verbose`.

## braco_hardware test

`braco_hardware/test/test_braco_system.cpp` (#125) loads the `BracoSystem` plugin from the real-arm URDF and reads a
spawned `arm_driver` that talks to a fake arm on a pseudo-terminal (`PtyArm`), never a real serial port.

- It runs on `ROS_DOMAIN_ID` 28, kept free for it (`make lint` refuses it in `team-domains.tsv`), and discovers only
  on localhost; the spawned `arm_driver` inherits both. It never shares a domain with the real arm (22), a teammate's
  `sim` or `dev`, or the MoveIt test (29).
- A hang fails after 120 s (`ament_add_ros_isolated_gtest ... TIMEOUT 120`). If CTest kills the test at that point,
  an `arm_driver` can be left running: check with `ps aux | grep arm_driver`.
- Alone, in `make shell`: `colcon test --packages-select braco_hardware`, then `colcon test-result --verbose`.

## Coverage

- Floors: `COV_MIN_CXX`, `COV_MIN_PY` and `COV_MIN_WEB` in [`mk/ci.mk`](../../mk/ci.mk), 80% for every component. A PR
  below a floor adds tests; the floors are not lowered to make it pass.
- The ROS Python in scope is every `.py` of `ros2_ws/src` that git tracks outside `script/`, `scripts/` and `test/`,
  launch files included (today the 7 launch files, all at 100%). A file no test loads still counts, at 0%
  (`scripts/coverage_touch.py`), so a new launch file without a test lowers the total. A new file counts once git
  tracks it. The helper's own test (`scripts/test_coverage_touch.py`) runs in the same job, on the coverage.py of
  the `ci` image; if it fails, the job fails.
- A launch file is tested by loading it from `src/` (not the installed copy, which is not measured) and calling
  `generate_launch_description()`: nothing starts. See `braco_bringup/test/test_launch.py`, or
  `braco_moveit_config/test/test_launch.py` for exit handlers driven with a `ProcessExited` event.
- With files in scope, missing or unreadable coverage data fails the job. Only an empty list skips `COV_MIN_PY`
  (`make coverage COV_PY_FILES=`).
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
