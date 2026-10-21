# Developer Setup Guide (Sprint 0)

[![CI](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml)

This guide documents how to set up the development environment on developer hardware to interface with our robot platform (**Elephant Robotics myCobot 280 Jetson Nano**). ROS 2, the build tools and the braco system run in Docker containers driven by the root `Makefile`; the host only needs Docker, `make` and, for the web page tests, Python 3.11. The guide also covers the hello-world test suites that talk to the arm directly over serial.

> **Safety Notice:** Before powering on or executing any motion on the physical arm, review [Lab Safety Induction & Workstation Rules](safety.md). Ensure the robot is clamped to the workstation, the 350 mm perimeter is clear, and the arm is manually placed in the upright configuration ($0^\circ$) before engaging motor power.

---

## 1. Environment Specifications

The Jetson Nano's own OS is old (Ubuntu 20.04, Python 3.8), so the code does not run on it directly: it runs in
containers built from `docker/Dockerfile` (Ubuntu 26.04 + ROS 2 Lyrical), the same base image on the Jetson and on
the PC. Details in [docker/README.md](../docker/README.md).

| Component | Jetson host (robot) | Containers (`dev`, `real`, `sim`) | How obtained |
|---|---|---|---|
| OS | Ubuntu 20.04.6 LTS (`focal`) | Ubuntu 26.04 | `lsb_release -a` |
| Architecture | `aarch64` | `x86_64` (PC) or `aarch64` (Jetson), host-dependent | `uname -m` |
| Compiler | GCC 8.4.0 | Ubuntu 26.04 default | `gcc --version` |
| Python | Python 3.8.10 | Ubuntu 26.04 default | `python3 --version` |
| ROS 2 | (none needed on the host) | Lyrical (`ros-base`; the PC image adds desktop, Gazebo, MoveIt 2) | `printenv ROS_DISTRO` |
| CMake | 3.16.3 | Ubuntu 26.04 default | `cmake --version` |

For the container column run the command through `make run`, e.g. `make run CMD='gcc --version'`.
The host Python venv used by `make ci` and `make web` is pinned to **Python 3.11** (`.python-version`).

---

## 2. Developer Hardware Software Installation

Developers work on a PC (`ROLE=pc`: simulation, RViz2, MoveIt 2) and on the Jetson Nano in the arm's base
(`ROLE=jetson`: the real arm). The two reach each other over ROS 2 DDS on the same network (same `ROS_DOMAIN_ID`
and RMW, set in `docker/.env`).

### 2.1 System Prerequisites & Packages

* **Docker Engine 24+**, the **Docker Compose v2.20+** plugin and **buildx** (BuildKit), with your user in the
  `docker` group. `make doctor` checks all three.
* `make`, `git` and `curl` (`make up` uses curl to wait for the web page).
* For the host checks and web tests (`make ci`, `make web`): **Python 3.11** with `venv`, plus `cmake` and a C++
  compiler for the hello-world build.

```bash
sudo apt-get update && sudo apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    make \
    python3.11 \
    python3.11-venv
```

If your distribution has no `python3.11` package, install it with `uv python install 3.11`, or point make at another
interpreter with `PYTHON=/path/to/python3.11`.

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

Commit messages must follow `<type>(<scope>): <description>`, with type one of
`fix feat test style docs build ops chore perf refactor` (checked by `.githooks/commit-msg`).

**On the Jetson**, `make doctor` also warns about what makes the Nano fail or crawl. Fix these before the first build:

| Warning | Fix |
|---|---|
| `nvgetty` holds `/dev/ttyTHS1` | `sudo systemctl disable --now nvgetty` |
| a desktop session is running (~1 GB of the 4 GB RAM) | `sudo systemctl set-default multi-user.target && sudo reboot` |
| power mode is not MAXN | `sudo nvpmodel -m 0` (needs the barrel-jack supply) |
| no swap (colcon runs out of memory) | `sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile` |

Building the Jetson image on the Nano is slow; it can be built on a PC instead and copied over:
`make jetson-image NANO=user@jetson-host`.

### 2.4 Day-to-day Commands

Run everything from the repo root. ROS commands run inside the `dev` container, where ROS and the workspace
(`ros2_ws`, mounted at `/ws`) are already sourced.

| Command | What it does |
|---|---|
| `make up` / `make down` | Turn the system on/off: the real arm on the Jetson, the simulation on the PC. Builds the image and the workspace only when needed. `make down PARK=1` parks the arm first. |
| `make status` / `make logs S=web` | Containers and the arm as the web page sees it / follow the log (`S=` filters: `web`, `arm_driver`, any pattern) |
| `make shell` | A shell in `dev` |
| `make run CMD='ros2 topic list'` | One command in `dev` |
| `make build` then `make restart` | `colcon build` of `ros2_ws/src` and `libs`, then restart the system to run it |
| `make test` | `colcon test` and the results (PC only; the Jetson builds without tests) |
| `make web` | The web page with mock arm data on http://localhost:8080 (no arm, ROS or Docker) |
| `make zero` / `make park` / `make temps` | Jetson, with the system up: see [arm-rest-mode.md](arm-rest-mode.md) |
| `make net-test` | ROS 2 talker/listener self-test (`net-talk` / `net-listen` across the PC and the Jetson) |

### 2.5 Python Software Installation

`make` manages the host virtual environment in `.venv` (Python 3.11): `make ci` installs `requirements.txt` and the web
test requirements, `make web` only the web page's. To use it for the hello-world script:

```bash
make ci                       # or only the arm packages: python3.11 -m venv .venv && .venv/bin/pip install -r requirements.txt
source .venv/bin/activate
```

* `pymycobot`: Elephant Robotics official Python SDK for kinematics, angle commands, and ATOM Matrix LED control.
* `pyserial`: Backend serial communication.
* `numpy`: Required by `pymycobot` (pinned to 1.24.4, which has no wheels past Python 3.11).

### 2.6 C++ Toolchain Installation

The ROS 2 packages and `libs/cpp` (`mycobot_cpp`, C++17) are built with colcon inside the containers (`make build`).
The hello-world demo builds on the host and uses standard POSIX serial communication (`<termios.h>`, `<fcntl.h>`,
`<unistd.h>`) and C++14 threads (`-lpthread`):

* **Compiler:** GCC/G++ $\ge 8.4$ (supports C++14 / C++17).
* **Build System:** CMake $\ge 3.16$ and GNU Make.
* The standalone header `tests/mycobot.hpp` is provided in the repository with zero external third-party library dependencies.

---

> **The hello-world demos open the serial port directly.** On the Jetson, run `make down` first: while the system
> is up, `arm_driver` owns `/dev/ttyTHS1` and must stay its only user (see [arm-rest-mode.md](arm-rest-mode.md)).
> Both demos power on the servos, so follow the pre-power steps in [safety.md](safety.md).

---

## 3. Python "Hello-World" Demo & Verification

A runnable Python verification script is located in `tests/hello_world.py`. It establishes communication with the robot controller, queries current joint telemetry, and triggers the Atom Matrix LED indicator.

### 3.1 Running the Python Demo

Ensure the virtual environment is activated, then run:

```bash
# Direct on the Jetson Nano:
python3 tests/hello_world.py

# Or on a workstation with a custom port:
python3 tests/hello_world.py --port /dev/ttyUSB0 --baud 1000000
```

### 3.2 Expected Output & Verification Criteria

Upon successful execution, the terminal should display:

```text
========================================
   myCobot 280 Python Hello World Demo   
========================================
Connecting to port: /dev/ttyTHS1 at 1000000 baud...
[INFO] Powering on servos...
[INFO] Querying current joint angles...
[OK] Received joint angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[INFO] Setting Atom Matrix LED to green (0, 255, 0)...

========================================
Hello, myCobot! Python communication verified successfully.
========================================
```

* **Pass Criteria:**
  1. No `SerialException` or permission errors.
  2. The Atom Matrix LED on the end-effector changes color to green.
  3. Joint angles are returned as a list of 6 float values.

> **Hardware Verification Status:** This demo has been tested and verified directly on the physical myCobot 280 Jetson Nano robot. Serial communication was established on `/dev/ttyTHS1` at 1,000,000 baud, servo motor torque engaged, joint telemetry retrieved successfully, and the Atom Matrix LED illuminated green.

---

## 4. C++ "Hello-World" Demo & Verification

The C++ demo suite (`tests/hello_world.cpp` and `tests/mycobot.hpp`) tests low-level serial packet transmission (`0xFE 0xFE` protocol frames) directly.

### 4.1 Direct Compilation with G++

You can compile the demo directly using `g++`:

```bash
g++ -O2 -std=c++14 tests/hello_world.cpp -lpthread -o tests/hello_world
```

### 4.2 Compilation with CMake

`tests/CMakeLists.txt` builds the same program out of source; CI builds it this way too (`make ci`). The binary
stays in the build folder (`CMAKE_RUNTIME_OUTPUT_DIRECTORY` is the binary dir), so the source tree is left clean:

```bash
cmake -S tests -B tests/build
cmake --build tests/build
```

### 4.3 Running the C++ Demo

Run the compiled executable (`tests/hello_world` with `g++`, `tests/build/hello_world` with CMake), optionally
passing the port and baud rate:

```bash
# On Jetson Nano (default: /dev/ttyTHS1 at 1000000):
./tests/build/hello_world

# Or on workstation / custom port and baud:
./tests/build/hello_world /dev/ttyUSB0 1000000
./tests/build/hello_world --port /dev/ttyUSB0 --baud 1000000
```

### 4.4 Expected Output & Verification Criteria

```text
========================================
   myCobot 280 C++ Hello World Demo     
========================================
Target serial port: /dev/ttyTHS1 (Baud: 1000000)
[OK] Serial connection established.
[INFO] Powering on servos...
[INFO] Reading current joint angles...
[OK] Received joint angles: [ J1: 0° J2: 0° J3: 0° J4: 0° J5: 0° J6: 0° ]

========================================
Hello, myCobot! C++ communication verified successfully.
========================================
```

* **Pass Criteria:**
  1. Serial port opens successfully in 8N1 raw mode at `B1000000`.
  2. Protocol request `0x20` is dispatched and parsed to valid joint coordinates.
  3. Clean zero-exit return code (`echo $?` returns `0`).

> **Hardware Verification Status:** This demo has been tested and verified directly on the physical myCobot 280 Jetson Nano robot. Serial connection to `/dev/ttyTHS1` opened at 1,000,000 baud, servo motors energized, and all 6 joint angles were read and parsed from the controller.

---

## 5. Continuous Integration (CI)

Every pull request and every push to `main` runs [`.github/workflows/ci.yml`](../.github/workflows/ci.yml). No robot,
serial port or display is needed. Both jobs only call `make`, so a red check can be reproduced on a workstation with
the same commands:

| Job | What it runs | Reproduce locally |
|---|---|---|
| `host` | `make ci`: host checks (venv with Python 3.11 from `.python-version`, imports, C++ hello-world build), `make lint` (compose files for both machines, shellcheck, hadolint) and the web page tests (pytest `web/tests`); creates `docker/.env` if missing | `make ci` |
| `ros` | Builds the Dockerfile's `base` stage (Ubuntu 26.04 + ROS 2 Lyrical), then `rosdep install`, `colcon build` and `colcon test` of `ros2_ws/src` and `libs` in a throw-away container | `make ci-ros` (builds the image the first time) |

* shellcheck and hadolint need no install: without a local binary, `make lint` runs the images pinned in
  `SHELLCHECK_IMAGE` and `HADOLINT_IMAGE` through docker, so it never skips a check that CI runs.
* Caches: pip (`actions/setup-python`), the image layers (BuildKit `type=gha`), and ccache + colcon `build/`
  (`actions/cache`, reset whenever `docker/` changes). A warm run only recompiles what changed.
* The arm64 Jetson image is not built in CI (QEMU is too slow); build it by hand with `make jetson-image`.
* `ci-ros` writes to `ros2_ws/build`, `install` and `log`, like `make build`, plus its ccache in `ros2_ws/.ccache`.

---

## 6. Troubleshooting & FAQ

| Problem | Cause | Solution |
|---|---|---|
| `Permission denied: '/dev/ttyTHS1'` | User not in `dialout` group | Run `sudo usermod -aG dialout $USER`, then log out and back in. |
| `Device or resource busy` | Another process is accessing UART (with the system up it is the `arm_driver` node in the `real` container, which holds the port exclusively) | Check active locks using `lsof /dev/ttyTHS1` or `fuser /dev/ttyTHS1`. Don't stop the driver just to run a script: use `make park`, `make zero` or `make temps` from the repo root (see [arm-rest-mode.md](arm-rest-mode.md)). |
| `[WARN] No response to get_angles query` | Base power switch off or wrong baud rate | Ensure the circular 60W power switch on the base is turned ON and baud rate is set to `1000000`. |
| Motors locked or resisting movement | Active closed-loop holding torque | Run torque release script (see [Safety Guide](safety.md#52-software-operational-stop--holding-torque-behavior)): `mc.release_all_servos()`. With the system up, `make park` releases the joints and keeps the driver monitoring temperatures. |

---

## 7. Changelog

| Date | Change | Author |
|---|---|---|
| *2026-10-09* | Initial setup draft | d-branco |
| *2026-10-09* | Added hardware installation guides, C++ & Python test suites, and CMake build support | d-branco |
| *2026-10-09* | Fixed CMake output path to tests/ directory and recorded successful physical robot test verification | d-branco |
| *2026-10-20* | Added the CI section (GitHub Action, issue #16) and its status badge | Petrichor |
| *2026-10-21* | New make commands (issue #54): `make setup`, `make ci`, `make ci-ros`, `make park` ... | Denionline |
| *2026-10-21* | Environment updated to the containers (Ubuntu 26.04, ROS 2 Lyrical), Docker prerequisites, Jetson preparation, day-to-day commands, make-managed venv, CMake output path and `--baud` of the C++ demo | Petrichor |
