# Developer Setup Guide (Sprint 0)

[![CI](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml)

This guide documents how to set up the development environment on developer hardware to interface with our robot platform (**Elephant Robotics myCobot 280 Jetson Nano**). It details system toolchains, Python and C++ dependencies, and step-by-step verification using provided hello-world test suites.

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

After cloning the repository, configure the project git hooks:

```bash
make init
```

This binds commit message linters in `.githooks/` to ensure commit message standards are maintained.

### 2.4 Python Software Installation

We recommend using a dedicated Python virtual environment for development:

1. Create and activate a virtual environment:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```

2. Install the required Python packages:
   ```bash
   pip install --upgrade pip
   pip install pymycobot opencv-python pyserial pytest
   ```

* `pymycobot`: Elephant Robotics official Python SDK for kinematics, angle commands, and ATOM Matrix LED control.
* `pyserial`: Backend serial communication.
* `opencv-python`: Captures and inspects frames from the tool-mounted USB camera.
* `pytest`: Unit testing framework.

### 2.5 C++ Toolchain Installation

Direct C++ robot control utilizes standard POSIX serial communication (`<termios.h>`, `<fcntl.h>`, `<unistd.h>`) and C++14 threads (`-lpthread`):

* **Compiler:** GCC/G++ $\ge 8.4$ (supports C++14 / C++17).
* **Build System:** CMake $\ge 3.16$ and GNU Make.
* The standalone header `tests/mycobot.hpp` is provided in the repository with zero external third-party library dependencies.

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

A minimal `CMakeLists.txt` is provided in the `tests/` directory:

```cmake
cmake_minimum_required(VERSION 3.16)
project(mycobot_tests CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output compiled executables directly to the tests directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Threads REQUIRED)

add_executable(hello_world hello_world.cpp)
target_include_directories(hello_world PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hello_world PRIVATE Threads::Threads)
```

Setting `CMAKE_RUNTIME_OUTPUT_DIRECTORY` ensures the generated binary is placed directly in `tests/hello_world` matching the direct `g++` compilation output, even when building inside an out-of-source `tests/build` folder.

To build with CMake:

```bash
mkdir -p tests/build
cd tests/build
cmake ..
cmake --build .
cd ../..
```

### 4.3 Running the C++ Demo

Run the compiled executable, optionally passing the port path:

```bash
# On Jetson Nano (default: /dev/ttyTHS1):
./tests/hello_world

# Or on workstation / custom port:
./tests/hello_world /dev/ttyUSB0
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
| `host` | `make env`, then `make host-check` (venv with Python 3.11 from `.python-version`, imports, C++ hello-world build), `make lint` (compose files, shellcheck, hadolint) and `make web-test` (pytest `web/tests`) | `make env` (once), then `make ci` |
| `ros` | Builds the Dockerfile's `base` stage (Ubuntu 26.04 + ROS 2 Lyrical), then `rosdep install`, `colcon build` and `colcon test` of `docker/ws/src` and `libs` in a throw-away container | `make ci-image ci-ros` |

* Install `shellcheck` locally: without it `make lint` skips that check, while CI always runs it. hadolint needs no
  install: without a local binary, `make lint` runs the image pinned in `HADOLINT_IMAGE` (also what CI uses).
* Caches: pip (`actions/setup-python`), the image layers (BuildKit `type=gha`), and ccache + colcon `build/`
  (`actions/cache`, reset whenever `docker/docker/` changes). A warm run only recompiles what changed.
* The arm64 Jetson image is not built in CI (QEMU is too slow); build it by hand with `make jetson-cross`.
* `ci-ros` writes to `docker/ws/build`, `install` and `log`, like `make ws-build`, plus its ccache in `docker/ws/.ccache`.

---

## 6. Troubleshooting & FAQ

| Problem | Cause | Solution |
|---|---|---|
| `Permission denied: '/dev/ttyTHS1'` | User not in `dialout` group | Run `sudo usermod -aG dialout $USER`, then log out and back in. |
| `Device or resource busy` | Another process is accessing UART (with the arm services up it is the `arm_driver` node in the `arm` container, which holds the port exclusively) | Check active locks using `lsof /dev/ttyTHS1` or `fuser /dev/ttyTHS1`. Don't stop the driver just to run a script: use `make arm-park`, `arm-zero` or `arm-temps` from the repo root (see [arm-rest-mode.md](arm-rest-mode.md)). |
| `[WARN] No response to get_angles query` | Base power switch off or wrong baud rate | Ensure the circular 60W power switch on the base is turned ON and baud rate is set to `1000000`. |
| Motors locked or resisting movement | Active closed-loop holding torque | Run torque release script (see [Safety Guide](safety.md#52-software-operational-stop--holding-torque-behavior)): `mc.release_all_servos()`. With the arm services up, `make arm-park` releases the joints and keeps the driver monitoring temperatures. |

---

## 7. Changelog

| Date | Change | Author |
|---|---|---|
| *2026-10-09* | Initial setup draft | d-branco |
| *2026-10-09* | Added hardware installation guides, C++ & Python test suites, and CMake build support | d-branco |
| *2026-10-09* | Fixed CMake output path to tests/ directory and recorded successful physical robot test verification | d-branco |
| *2026-10-20* | Added the CI section (GitHub Action, issue #16) and its status badge | Petrichor |
