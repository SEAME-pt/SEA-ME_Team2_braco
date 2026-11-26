# Guide: CI, formatting and coverage

[![CI](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml)

Every pull request and every push to `main` runs [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml). No robot,
serial port or display is needed, except for the manual `hil` job on the Jetson ([HIL runner](#hil-runner)). Every
other job only calls `make`, so a red check can be reproduced on a workstation with the same commands, from the repo
folder.

## Jobs

| Job | What it runs | Reproduce locally |
|---|---|---|
| `host` | `make ci`: `make format-check` (C++ and Python formatting, see [Code formatting](#code-formatting)), `make lint` (compose files for both machines, ruff check of the Python outside `ros2_ws/src`, shellcheck, hadolint) and `make web-test` (pytest `web/tests`, in a venv from `web/requirements*.txt`, with the coverage of `web/server`, failing below `COV_MIN_WEB`) and `make timing-test` (pytest `scripts/timing/test_timing.py`: the reply framing and statistics of the bench scripts and the stop-time analysis of `make fault`, no serial port, no ROS), one step each, all run even when one fails; creates `docker/.env` if missing | `make ci`, or one of its four targets |
| `ros` | Builds the Dockerfile's `ci` stage (`base`: Ubuntu 26.04 + ROS 2 Lyrical, plus clang-tidy, gcovr, pytest-cov and MoveIt 2), then `rosdep install`, `colcon build` and `colcon test` of `ros2_ws/src` (with `mycobot_cpp` and the [MoveIt test](#moveit-test)) in a throw-away container, the test results uploaded as the `test-results` artifact; then cppcheck on every C++ package, and clang-tidy (`ros2_ws/.clang-tidy`, warnings as errors) from that build's `compile_commands.json`. Both also run when a test failed | `make ci-ros` (builds the image the first time), then `make cppcheck` and `make tidy` |
| `coverage` | Same image as `ros`, in parallel: the GoogleTests built with `--coverage` in `ros2_ws/coverage/` (gcovr) and the pytests (pytest-cov), uploaded as the `coverage` artifact (HTML per line; Python in `python/`); fails below `COV_MIN_CXX` or `COV_MIN_PY` (the ROS Python includes the launch files, see [Coverage](#coverage)) | `make coverage` (report in `ros2_ws/coverage/html`) |
| `hil` | Manual only, on the Jetson runner next to the real arm ([HIL runner](#hil-runner)): `uname -m` (fails unless `aarch64`), `docker ps` and `ls -l /dev/ttyTHS1`. It only stats the serial port, never opens it, and never runs `ros2`, so the arm never moves. Skipped on every PR and push | Not locally: Actions → CI → Run workflow with `hil` ticked, or `gh workflow run ci.yml --ref <branch> -f hil=true`; then a reviewer approves the `hil` environment |

- `make ci` creates `.venv` from `web/requirements*.txt` by itself; `make setup` (on the PC) adds
  `requirements-dev.txt` to the same venv.
- shellcheck and hadolint need no install: without a local binary, `make lint` runs the images pinned in
  `SHELLCHECK_IMAGE` and `HADOLINT_IMAGE` through docker, so it never skips a check that CI runs.
- `make lint`, `make format` and `make format-check` take their files from `git ls-files`: a new file is skipped
  until it is `git add`ed. Add it first, or a check that passes locally can fail once the file is on the PR.
- A small `changes` job decides from `git diff` what a change needs (a PR: its base to its head; a push: from the old
  `main` to the new one, so a docs-only merge does not rebuild). A skipped job or step counts as passed for the
  required checks. `host` always runs (under a minute).
  - `ros` and `coverage` run only when a file they read changed: `ros2_ws/` (except `.md`), `docker/Dockerfile`,
    `docker/config/`, `docker/scripts/`, `scripts/`, `Makefile`, `mk/` or `.github/workflows/`. Docs, compose files,
    `docker/tests/`, `web/`, templates and the like are checked by `host` alone. A folder that `ros` or `coverage`
    starts to read (a new mount in `mk/ci.mk`, a new `COPY` in the Dockerfile) must be added to `ros_input` in the
    `changes` job, or a change to it skips the jobs that test it.
  - `make tidy` runs only when a C++ input changed: a `.cpp`, `.cc`, `.hpp` or `.h`, a `CMakeLists.txt`,
    `package.xml`, `.msg`/`.srv`/`.action` or `.clang-tidy` in `ros2_ws/`, the image (`docker/Dockerfile`,
    `docker/config/`, `docker/scripts/`), `Makefile`, `mk/ci.mk` or the workflow (`tidy_input`).
  - A rename lists both paths, so moving code into `docs/` still builds. A manual run (`workflow_dispatch`), the first
    push of a branch, or a failed diff runs everything.
- `make tidy` goes through [ctcache](https://github.com/matus-chochlik/ctcache) (in the `ci` image): a file that
  passed before with the same preprocessed source (headers and comments included, so a header change or a removed
  `NOLINT` re-checks it), the same config and arguments is not analysed again. Its cache is `ros2_ws/.ctcache`, also
  kept locally, so a second `make tidy` on a workstation only re-checks what changed. The log ends with the cache's
  hits and misses. A finding is never cached. The databases of every package are merged into `build/.tidy`, so one
  `run-clang-tidy` keeps every core busy.
- Caches: pip (`actions/setup-python`), the image layers (BuildKit `type=gha`), ccache + colcon `build/` and the
  ctcache (`actions/cache`, reset when the Dockerfile, `docker/config/` or `docker/scripts/` change). A warm run only
  recompiles and re-checks what changed. Only pushes to `main` save the image layers and the build caches (the
  ctcache only after a passing `make tidy`): a PR reads `main`'s but can't share its own with other PRs, so saving them
  would only fill the 10 GB repo cache. A `main` push that skipped `ros` or `make tidy` saves nothing: the next run
  restores the last saved cache by key prefix, which is still current because the skipped files never reach it.
- The arm64 Jetson image is not built in CI (QEMU is too slow); build it by hand with `make jetson-image`. `hil` runs
  on the Jetson but builds nothing either: it only checks the machine.
- `ci-ros` writes to `ros2_ws/build`, `install` and `log`, like `make build`, plus its ccache in `ros2_ws/.ccache`.

## HIL runner

A self-hosted GitHub Actions runner on the Jetson, next to the real arm (#94), for the manual `hil` job. The real HIL
checks (serial, safe state, logs) come in #143.

- **Runner:** `jetson-braco`, labels `self-hosted`, `Linux`, `ARM64` and `braco-arm` (the job's
  `runs-on: [self-hosted, braco-arm]`). It runs as its own user `gh-runner`, in `docker` but not in `dialout`, as a
  service (`svc.sh` in `~gh-runner/actions-runner`, started at boot). Job checkouts go in its `_work/`.
- **Gates:** the repo is public and a self-hosted runner runs whatever code a workflow gives it, so:
  - manual only: `workflow_dispatch` with the boolean input `hil` (default false); every PR and push skips the job;
  - the `hil` environment: a required reviewer approves each run, and only `main` (plus a branch under development)
    may deploy to it; another branch fails before reaching the Jetson;
  - workflows from fork PRs need approval.
- **Limits:** one `hil` run at a time (`concurrency: hil`, never cancelled), 5 min timeout. A run queued while the
  Jetson is off waits (up to 24 h): `hil` is never a required check.
- **Rules for any job on it:**
  - never open `/dev/ttyTHS1`: `arm_driver` is its only owner
    ([ADR-0002](../decisions/ADR-0002-unique-control.md));
  - never `docker compose up`/`down` from the job's checkout: `docker/compose.yml` sets `name: braco`, so it would act
    on the live `real` container. Use `docker exec` or another project name (`-p`);
  - the Jetson is shared: agree a slot with the team first. `concurrency` only keeps CI runs apart, not people.
- Recovery (runner offline, re-register, update): #144.

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
  launch files included (today the 8 launch files). A file no test loads still counts, at 0%
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
