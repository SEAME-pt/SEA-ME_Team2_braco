# Developer Setup Guide

How to set up your machine to work on braco (**myCobot 280 Jetson Nano**).

> **Safety:** before moving the real arm, read [safety.md](safety.md).

## 1. How it works

- **You do not install ROS 2 on your machine.** It is already inside a Docker container: a ready-made Linux with
  ROS 2 and every tool the project needs.
  - So everyone has exactly the same versions, on the PC and on the Jetson.
  - On your machine you only install Docker, `make` and `git`.
  - Why we chose this: [ADR-0001](../decisions/ADR-0001-ros2-in-docker-on-jetson-nano.md).
- **You never type `docker` commands.** You type `make <command>` in the repo folder, and `make` runs Docker for
  you.
  - Example: `make up` starts the containers, `make down` stops them.
  - Type `make` alone to see every command.
- **Your code stays on your machine.** The repo folder `ros2_ws/` is shared with the container (inside it, it is
  called `/ws`).
  - So you edit the files with your normal editor, and the container builds and runs them.
- **The PC and the Jetson run different containers:**

  | Machine | Container | What it is for |
  |---|---|---|
  | PC | `dev` | Your work: open a shell (`make shell`), build, test, RViz, Gazebo |
  | PC | `sim` | A simulated arm in Gazebo. Only if you ask: `make up SIM=1` |
  | Jetson | `real` | The real arm: the nodes that move it and read its temperatures |

- More about the containers: [docker/README.md](../../docker/README.md).

## 2. PC (Linux)

- **Install Docker Engine** with Compose 2.20 or newer: [docs.docker.com/engine/install](https://docs.docker.com/engine/install/).
- **Install the tools:**
  - `sudo apt-get install -y make git curl python3-venv x11-xserver-utils`
  - `python3-venv`: for the venv with the formatters. `x11-xserver-utils`: lets RViz and Gazebo open windows.
- **Run Docker without `sudo`:**
  - `sudo usermod -aG docker $USER`, then log out and back in.
- **GPU:** `make` uses an NVIDIA card by default.
  - NVIDIA: install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
  - Intel / AMD: add `GPU=dri` to the `make` commands. No GPU: `GPU=none` (Gazebo is slow).
- **Clone and prepare:**
  - `git clone git@github.com:SEAME-pt/SEA-ME_Team2_braco.git && cd SEA-ME_Team2_braco`
  - `make setup`: asks `dev or jetson?` → type **`dev`**.
  - It creates `docker/.env`, the venv and the git hooks, then runs `make doctor`: every line must be `[ ok ]`.
  - Never run `make` with `sudo`.
- **Start:**
  - `make up`: starts `dev`. The first time it builds the image (tens of minutes).
  - `make up SIM=1`: also the simulated arm.
- **Check:**
  - `make net-test` must print `PASS`.
  - `make shell` opens a shell in the container.

## 3. Jetson

- **Free the arm's serial port** (once): `sudo systemctl disable --now nvgetty`, then log out and back in.
- **Clone and prepare:**
  - Same `git clone` as on the PC.
  - `make setup` → type **`jetson`**.
  - Fix the `[warn]` lines too (desktop running, power mode not MAXN, no swap): they slow the Jetson down.
- **Get the image**, one of:
  - `make image` on the Jetson (slow);
  - `make jetson-image NANO=user@<jetson-ip>` on the PC (faster: builds it there and copies it).
- **Start:** `make up`. It waits until the arm answers, and never moves it.
- **Check:**
  - `make status`: the arm connected, six angles.
  - `make temps`: six temperatures.
  - `make web`, then `http://<jetson-ip>:8000`: live angles.

## 4. Before you push

- `make format`: fixes the C++ (clang-format) and Python (ruff) formatting.
- `make ci`: the same checks as GitHub (formatting, lint, web page tests).
- `make ci-ros`: if you changed ROS 2 code (builds and tests every package, like GitHub).
- The pre-push hook **blocks the push** if the formatting is wrong: `make format`, commit, push again.

## 5. Problems

- `permission denied` with `docker`: log out and back in after `usermod -aG docker`.
- `docker has no NVIDIA runtime`: install the NVIDIA Container Toolkit, or use `GPU=dri` / `GPU=none`.
- `[FAIL]` in `make doctor`: each line says how to fix it. More: [TROUBLESHOOTING.md](ROS2/TROUBLESHOOTING.md).
- Jetson, `Device or resource busy` on `/dev/ttyTHS1`: the port belongs to `arm_driver`. Use `make park`,
  `make zero`, `make temps`, never a script of your own.
- Jetson, the arm does not answer: turn on the round power switch on the base.
- Motors locked, resist by hand: `make park` frees the joints.
