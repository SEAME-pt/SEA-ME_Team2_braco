# Developer Setup Guide (Sprint 0)

[![CI](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml)

This guide documents how to set up the development environment on developer hardware to interface with our robot platform (**Elephant Robotics myCobot 280 Jetson Nano**). It details system toolchains, the host Python and C++ tools, and how to verify the link to the arm.

> **Safety Notice:** Before powering on or executing any motion on the physical arm, review [Lab Safety Induction & Workstation Rules](safety.md). Ensure the robot is clamped to the workstation, the 350 mm perimeter is clear, and the arm is manually placed in the upright configuration ($0^\circ$) before engaging motor power.

---

## 1. Environment Specifications

The versions below represent the baseline software environment of the physical robot (NVIDIA Jetson Nano) alongside development workstation specifications:

| Component | Robot reports | Container provides | How obtained |
|---|---|---|---|
| OS | Ubuntu 20.04.6 LTS (`focal`) | Ubuntu 20.04 (`focal`) | `lsb_release -a` |
| Architecture | `aarch64` | `x86_64` or `aarch64`, host-dependent | `uname -m` |
| Compiler | GCC 8.4.0 | GCC 9.4 (Focal default) | `gcc --version` |
| Python | Python 3.8.10 | 3.8.x | `python3 --version` |
| ROS 2 | galactic | `ros-galactic-ros-base` | `printenv ROS_DISTRO` |
| CMake | 3.16.3 | 3.16.3 | `cmake --version` |
| Colcon | 0.17.0 | x | `pip3 show colcon-core` |

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

It creates `docker/.env` for this machine (PC or Jetson, detected), binds the commit message linters in `.githooks/`
so commit message standards are maintained, and runs `make doctor`. Then `make up` turns the braco system on
(see [docker/README.md](../docker/README.md)); `make` lists every command.

### 2.4 Host Python (the web page only)

Everything that talks to the arm runs in the containers. On the host, Python is only needed for the web page and its
tests; `make web` and `make ci` create `.venv` from `web/requirements*.txt` by themselves:

```bash
make web    # the page with mock arm data on http://localhost:8080 (no arm, no ROS, no Docker)
make ci     # lint and the web page tests, as the CI `host` job
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

Every pull request and every push to `main` runs [`.github/workflows/ci.yml`](../.github/workflows/ci.yml). No robot,
serial port or display is needed. Both jobs only call `make`, so a red check can be reproduced on a workstation with
the same commands:

| Job | What it runs | Reproduce locally |
|---|---|---|
| `host` | `make ci`: `make lint` (compose files for both machines, shellcheck, hadolint) and the web page tests (pytest `web/tests`, in a venv from `web/requirements*.txt`); creates `docker/.env` if missing | `make ci` |
| `ros` | Builds the Dockerfile's `base` stage (Ubuntu 26.04 + ROS 2 Lyrical), then `rosdep install`, `colcon build` and `colcon test` of `ros2_ws/src` (with `mycobot_cpp`) in a throw-away container | `make ci-ros` (builds the image the first time) |

* shellcheck and hadolint need no install: without a local binary, `make lint` runs the images pinned in
  `SHELLCHECK_IMAGE` and `HADOLINT_IMAGE` through docker, so it never skips a check that CI runs.
* Caches: pip (`actions/setup-python`), the image layers (BuildKit `type=gha`), and ccache + colcon `build/`
  (`actions/cache`, reset whenever `docker/` changes). A warm run only recompiles what changed.
* The arm64 Jetson image is not built in CI (QEMU is too slow); build it by hand with `make jetson-image`.
* `ci-ros` writes to `ros2_ws/build`, `install` and `log`, like `make build`, plus its ccache in `ros2_ws/.ccache`.

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
| *2026-10-22* | Root cleanup: `libs/cpp` moved to `ros2_ws/src/mycobot_cpp`; `tests/` (hello-world demos), `playground/`, `requirements.txt` and `.python-version` removed | Denionline |
