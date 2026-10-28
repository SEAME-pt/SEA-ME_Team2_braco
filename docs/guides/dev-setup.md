# Developer Setup Guide (Sprint 0)

[![CI](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml)

This guide documents how to set up the development environment on developer hardware to interface with our robot platform (**Elephant Robotics myCobot 280 Jetson Nano**). It details system toolchains, the host Python and C++ tools, and how to verify the link to the arm.

> **Safety Notice:** Before powering on or executing any motion on the physical arm, review [Lab Safety Induction & Workstation Rules](safety.md). Ensure the robot is clamped to the workstation, the 350 mm perimeter is clear, and the arm is manually placed in the upright configuration ($0^\circ$) before engaging motor power.

---

## 1. Environment Specifications

The versions below represent the baseline software environment of the physical robot (NVIDIA Jetson Nano) alongside development workstation specifications:

| Component | Robot reports | Container provides | How obtained |
|---|---|---|---|
| OS | Ubuntu 20.04.6 LTS (`focal`) | Ubuntu 26.04 (`resolute`) | `lsb_release -a` |
| Architecture | `aarch64` | `x86_64` or `aarch64`, host-dependent | `uname -m` |
| Compiler | GCC 8.4.0 | Ubuntu 26.04's default GCC | `gcc --version` |
| Python | Python 3.8.10 | 3.14 (Ubuntu 26.04 system Python) | `python3 --version` |
| ROS 2 | galactic (native, not used by braco) | `lyrical` (`ros-lyrical-ros-base`) | `printenv ROS_DISTRO` |
| CMake | 3.16.3 | Ubuntu 26.04's default CMake | `cmake --version` |
| Colcon | 0.17.0 | from the ROS 2 apt repository | `pip3 show colcon-core` |

The robot column is the Jetson host; braco itself runs in the containers (see
[ADR-0001](../decisions/ADR-0001-ros2-in-docker-on-jetson-nano.md) and [docker/README.md](../../docker/README.md)).

---

## 2. Developer Hardware Software Installation

Developers work either directly on the Jetson Nano controller or on external host machines (Linux / Ubuntu 20.04/22.04/24.04, or Windows WSL2) connected via serial or network.

### 2.1 System Prerequisites & Packages

Install essential compilation and build tools:

```bash
sudo apt-get update && sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    python3 \
    python3-pip \
    python3-venv
```

### 2.2 Serial Port Access & Group Permissions

Communication with the robot uses high-speed UART / serial at `1,000,000` baud. Ensure your user account belongs to the `dialout` group to access serial devices without requiring `sudo`:

```bash
sudo usermod -aG dialout $USER
```

> **Note:** Log out and log back in (or run `newgrp dialout`) for this change to take effect.

#### Hardware Serial Port Mapping:
* **Onboard Jetson Nano carrier:** `/dev/ttyTHS1`
* **External Workstation via USB-to-UART Adapter:** `/dev/ttyUSB0` or `/dev/ttyACM0` (verify using `dmesg | grep tty` after plugging in).

### 2.3 Repository Initialization

After cloning the repository, prepare the machine once:

```bash
make setup
```

It creates `docker/.env` for this machine (PC or Jetson, detected), on the PC the development venv (see
[2.4](#24-host-python-web-page-tests-and-dev-tools)), binds the git hooks in `.githooks/` (commit message format, and a code
format check before every push, see [4.1](#41-code-formatting)), and runs `make doctor`. Then `make up` turns the
braco system on (see [docker/README.md](../../docker/README.md)); `make` lists every command.

### 2.4 Host Python (web page tests and dev tools)

Everything that talks to the arm runs in the containers, the web page too (`make web`). On the host, Python is only
needed for the page's tests and the development tools; `make ci` creates `.venv` from `web/requirements*.txt` by itself:

```bash
make ci     # format check, lint and the web page tests, as the CI `host` job
```

On the PC, `make setup` also installs `requirements-dev.txt` in the same venv, for development only:

* `ruff`, `clang-format`: the code formatters used by `make format` and the pre-push hook ([4.1](#41-code-formatting)).
* `pymycobot`, `pyserial`, `numpy`: the same versions as the container, so the editor resolves imports and
  autocompletes.

The hook and `make format` use the venv without activating it. Activate it only to run `python` or `ruff` by hand:

```bash
source .venv/bin/activate
```

### 2.5 C++ Toolchain (stand-alone tools)

The myCobot 280 protocol lives in `ros2_ws/src/mycobot_cpp`, plain C++17 with POSIX serial (`<termios.h>`) and no
ROS dependency. colcon builds it with the rest of the workspace (`make build`). Its stand-alone tools (`zero`, `park`,
`pid_tool`, `temps_lcd`, `joystick_all_joints`, `joystick_display`) build with its own Makefile:

```bash
make -C ros2_ws/src/mycobot_cpp          # g++ and GNU Make; binaries land next to the Makefile
```

They open the serial port directly, so run them only with the system down (`make down`): with the system up the port
belongs to `arm_driver` and they fail with "Device or resource busy".

---

## 3. Verifying the Link to the Arm

With the system up on the Jetson (`make up`):

```bash
make status   # containers, the arm as the page sees it (connected, angles, temperatures)
make temps    # servo temperatures, read through arm_driver
```

* **Pass Criteria:**
  1. `make status` shows the arm connected and six joint angles.
  2. `make temps` prints six temperatures.
  3. The page at `http://<jetson>:8000` shows live angles.

> **History:** Sprint 0 verified the serial link with Python and C++ hello-world demos in `tests/` (run on the
> physical arm on 2026-10-09, `/dev/ttyTHS1` at 1,000,000 baud). They were removed on 2026-10-22, once `arm_driver`
> became the only owner of the port; they remain in the git history (`git show bf96fc1:tests/hello_world.py`).

---

## 4. Continuous Integration (CI)

Every pull request and every push to `main` runs [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml). No robot,
serial port or display is needed. Every job only calls `make`, so a red check can be reproduced on a workstation with
the same commands:

| Job | What it runs | Reproduce locally |
|---|---|---|
| `host` | `make ci`: `make format-check` (C++ and Python formatting, see [4.1](#41-code-formatting)), `make lint` (compose files for both machines, ruff check of the Python outside `ros2_ws/src`, shellcheck, hadolint) and the web page tests (pytest `web/tests`, in a venv from `web/requirements*.txt`, with the coverage of `web/server`, failing below `COV_MIN_WEB`); creates `docker/.env` if missing | `make ci` |
| `ros` | Builds the Dockerfile's `base` stage (Ubuntu 26.04 + ROS 2 Lyrical), then `rosdep install`, `colcon build` and `colcon test` of `ros2_ws/src` (with `mycobot_cpp`) in a throw-away container | `make ci-ros` (builds the image the first time) |
| `coverage` | Same image as `ros`, in parallel: the GoogleTests built with `--coverage` in `ros2_ws/coverage/` (gcovr) and the pytests (pytest-cov), uploaded as the `coverage` artifact (HTML per line; Python in `python/`); fails below `COV_MIN_CXX` or `COV_MIN_PY` | `make coverage` (report in `ros2_ws/coverage/html`) |

* shellcheck and hadolint need no install: without a local binary, `make lint` runs the images pinned in
  `SHELLCHECK_IMAGE` and `HADOLINT_IMAGE` through docker, so it never skips a check that CI runs.
* Caches: pip (`actions/setup-python`), the image layers (BuildKit `type=gha`), and ccache + colcon `build/`
  (`actions/cache`, reset whenever `docker/` changes). A warm run only recompiles what changed.
* Coverage floors: `COV_MIN_CXX`, `COV_MIN_PY` and `COV_MIN_WEB` in [`mk/ci.mk`](../../mk/ci.mk), 80% for every
  component. A PR below a floor adds tests; the floors are not lowered to make it pass. `scripts/` directories are
  not measured, so the ROS Python (today only `braco_camera/scripts/camera.py`) has nothing to report yet.
* Each run's summary page shows the coverage totals against their floors, with the C++ per package
  (`scripts/coverage_summary.py`, from the JSON reports; written on failure too). Locally the table is dropped; to see
  it, run e.g. `GITHUB_STEP_SUMMARY=/dev/stdout make ci`.
* The arm64 Jetson image is not built in CI (QEMU is too slow); build it by hand with `make jetson-image`.
* `ci-ros` writes to `ros2_ws/build`, `install` and `log`, like `make build`, plus its ccache in `ros2_ws/.ccache`.

### 4.1 Code Formatting

C++ is formatted by clang-format (`.clang-format`) and Python by ruff (`.ruff.toml`). Both are pinned in
`requirements-dev.txt`, so everyone formats the same way.

| Command | What it does |
|---|---|
| `make format` | Fixes the formatting of every C++ and Python file |
| `make format-check` | Only checks, changes nothing; prints `OK` or the files to fix |

* The pre-push hook (`.githooks/pre-push`) runs `make format-check` and blocks the push if it fails. Fix with
  `make format`, then commit and push again.
* CI runs the same check (`make ci`, the `host` job), so a push that skipped the hook still fails on the PR.
* `ros2_ws/src` is skipped, except `ros2_ws/src/mycobot_cpp` (plain C++, no ament linters): the ROS 2 packages are
  checked by their own linters (`ament_uncrustify`, `ament_flake8`) in `make test`, because their style rules differ
  from clang-format and ruff.

---

## 5. Troubleshooting & FAQ

| Problem | Cause | Solution |
|---|---|---|
| `Permission denied: '/dev/ttyTHS1'` | User not in `dialout` group | Run `sudo usermod -aG dialout $USER`, then log out and back in. |
| `Device or resource busy` | Another process is accessing UART (with the system up it is the `arm_driver` node in the `real` container, which holds the port exclusively) | Check active locks using `lsof /dev/ttyTHS1` or `fuser /dev/ttyTHS1`. Don't stop the driver just to run a script: use `make park`, `make zero` or `make temps` from the repo root (see [arm-rest-mode.md](arm-rest-mode.md)). |
| `[WARN] No response to get_angles query` | Base power switch off or wrong baud rate | Ensure the circular 60W power switch on the base is turned ON and baud rate is set to `1000000`. |
| Motors locked or resisting movement | Active closed-loop holding torque | Run torque release script (see [Safety Guide](safety.md#52-software-operational-stop--holding-torque-behavior)): `mc.release_all_servos()`. With the system up, `make park` releases the joints and keeps the driver monitoring temperatures. |

---

## 6. Changelog

| Date | Change | Author |
|---|---|---|
| *2026-10-09* | Initial setup draft | d-branco |
| *2026-10-09* | Added hardware installation guides, C++ & Python test suites, and CMake build support | d-branco |
| *2026-10-09* | Fixed CMake output path to tests/ directory and recorded successful physical robot test verification | d-branco |
| *2026-10-20* | Added the CI section (GitHub Action, issue #16) and its status badge | Petrichor |
| *2026-10-21* | New make commands (issue #54): `make setup`, `make ci`, `make ci-ros`, `make park` ... | Denionline |
| *2026-10-21* | Code formatting: `make format`, `make format-check` and the pre-push hook (issue #58) | carlaugu |
| *2026-10-21* | Host venv: `make setup` creates it from `requirements-dev.txt` (container versions) | carlaugu |
| *2026-10-22* | Root cleanup: `libs/cpp` moved to `ros2_ws/src/mycobot_cpp`; `tests/` (hello-world demos), `playground/`, `requirements.txt` and `.python-version` removed | Denionline |
