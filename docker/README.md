# SEA:ME 2.0 - Arm class containers (Sprint 0 + Module 1)

Ubuntu 26.04 + ROS 2 Lyrical Luth for the myCobot 280 Jetson Nano, split across two machines:

```
 PC / workstation (x86_64)                        Jetson Nano in the arm base (aarch64)
 ┌───────────────────────────────┐   DDS (same    ┌───────────────────────────────┐
 │ pc-dev                        │   domain+RMW)  │ jetson-driver                 │
 │  RViz2 · MoveIt 2 · Gazebo    │ ◄────────────► │  pymycobot lifecycle driver   │
 │  ros2_control (sim) · QA tools│  host network  │  ros2_control + controllers   │
 └───────────────┬───────────────┘                └───────────────┬───────────────┘
                 └─────────────► stage `base` ◄───────────────────┘
          Ubuntu 26.04 · ROS 2 Lyrical (ros-base) · ros2_control · pymycobot · colcon defaults
```

One `docker/Dockerfile` with three stages. Both images are built `FROM base`, so ROS, ros2_control and
pymycobot are the same on both machines.

| Compose service | Dockerfile target | Runs on | Contains |
|---|---|---|---|
| `pc-dev` | `pc-dev` | PC | `ros-lyrical-desktop`, `ros-lyrical-ros-gz`, `ros-lyrical-moveit` **and** `ros-lyrical-moveit-servo` (a separate package), clang-tidy, cppcheck, lcov/gcovr, ruff, pre-commit, pytest-cov, **PlotJuggler 4**; optional gz_ros2_control, Foxglove bridge |
| `jetson-driver` | `jetson-driver` | Jetson | `base` only, plus serial tooling, real-time limits |

## Project layout

```
seame-arm-docker/
├── Makefile                 single entry point: run `make` to list everything
├── README.md
├── .env.example             copied to .env by `make env`
├── .gitignore
├── docker-compose.yml       at the root so plain `docker compose` finds it without -f
├── compose/
│   ├── nvidia.yml           optional GPU overrides (make up GPU=nvidia | dri)
│   └── dri.yml
├── tests/
│   └── talker-listener.sh   ROS 2 talker/listener test; mounted read-only into the containers
├── systemd/
│   └── seame-arm.service    boot unit for the arm services (make install-service)
└── docker/
    ├── Dockerfile           stages: base, jetson-driver, pc-dev
    ├── config/              colcon-defaults.yaml
    └── scripts/             entrypoint, env sourcing, smoke test, installers, apt helper
```

Created by `make` when needed, never shipped: `.env`, **`ws/src`** (your code, mounted at `/ws` on both machines),
`ws/bags` (rosbag recordings) and `dist/` (the archive from `make tar`). The workspace folders are created by
`make dirs`, and automatically by `make up` / `make run`, **as your user**. If Docker creates a missing bind-mount
folder itself it can end up owned by root, and then `colcon build` cannot write to it.

Every path the Makefile uses is defined in one block at its top (`project layout`), so moving a file means editing
one line there. `make doctor` checks that all project files are present, and `make lint` validates the compose files,
runs shellcheck on `docker/scripts/` and `tests/` and hadolint on the Dockerfile (each only if installed).

## Quick start

```bash
make env                 # creates .env with your UID/GID and group ids (ROLE=jetson on the Nano)
make doctor              # docker, compose >= 2.20, project files, .env, folders, display or serial port
make up                  # creates ws/src and ws/bags, builds the first time, then just starts
make smoke               # checks every piece below, including an RMW loopback
make shell
```

Without `make`:

```bash
cp .env.example .env                 # set COMPOSE_PROFILES=pc|jetson and USER_UID/USER_GID
mkdir -p ws/src ws/bags              # as YOUR user (or: make dirs), so Docker does not create them as root
xhost +local:                        # PC only: let containers draw on your display (Wayland desktops use XWayland)
docker compose up -d                 # builds only if the image is missing
docker compose exec pc-dev bash      # or jetson-driver
seame-smoke-test
```

**Prove the two machines see each other** (`make ping-echo` on one, `make ping-pub` on the other):

```bash
# Jetson:  docker compose exec jetson-driver ros_entrypoint.sh ros2 topic echo /seame_ping std_msgs/msg/String
# PC:      docker compose exec pc-dev ros_entrypoint.sh ros2 topic pub -r 2 /seame_ping std_msgs/msg/String "{data: hello}"
```

`docker compose exec <service> bash` gives you ROS and `./ws` sourced. A bare `docker compose exec <service> <command>`
skips the entrypoint, so ROS is **not** sourced - prefix it with `ros_entrypoint.sh` (as above), or use
`make exec CMD='...'` / `docker compose run --rm <service> <command>`.

## Make shortcuts

`make` wraps the commands above and picks the right service (`ROLE=pc` or `ROLE=jetson`, read from `COMPOSE_PROFILES`
in `.env`, or given on the command line). Run `make` for the full list.

```bash
make env / doctor / info    # setup, host and project checks, what the Makefile resolved
make dirs                   # create ws/src and ws/bags as your user (up and run do this automatically)
make lint                   # compose files + shellcheck + hadolint (the last two if installed)
make tar                    # pack the project into dist/seame-arm-docker.tar (no .env, ws/ or dist/)
make build / rebuild        # rebuild = --no-cache --pull, slow
make up / down / restart / ps / logs / shell
make talker-listener                                  # ROS 2 self-test: C++ talker + Python listener, PASS/FAIL
make talker  /  make listener                         # the two-machine version (talker on one, listener on the other)
make exec CMD='echo $ROS_DISTRO && ros2 topic list'   # in the running container; CMD reaches the shell exactly as typed
make run  CMD='colcon build'                          # throw-away container (CI style)
make ws-deps ws-build ws-test ws-clean                # rosdep / colcon on ws/ (--parallel-workers 2 on the Jetson)
make up GPU=nvidia          # or GPU=dri: adds compose/nvidia.yml or compose/dri.yml (PC only)
```

Jetson image built on the PC: `make binfmt` (once), `make jetson-cross`, `make jetson-ship NANO=user@nano.local`.
`make down` keeps your caches; `make clean` and `make distclean` delete volumes/images and ask first (`FORCE=1` skips
the prompt). In CI (`CI=true`) TTY allocation is turned off automatically.

## Plotting (PlotJuggler 4)

```bash
make shell                      # a terminal with ROS sourced (needed so the ROS 2 plugin finds its libraries)
plotjuggler4                    # opens on your display, like RViz2
```

- **Live topics:** pick the *ROS 2 Topic Subscriber* data source; it uses the same `ROS_DOMAIN_ID` and RMW as everything
  else, so topics from the Jetson show up too.
- **Recorded data:** `ros2 bag record -s mcap -o /ws/bags/run1 <topics>` (the MCAP storage plugin is installed) and open
  the bag in PlotJuggler. This is the easiest way to compare simulation against the real arm.
- A log line "Update check failed ... 403" at start-up is PlotJuggler asking GitHub for a newer release and being
  rate-limited; it is harmless.

## ROS 2 talker/listener test

The test from the ROS 2 install page (a **C++** talker and a **Python** listener, so both client libraries are
exercised), automated with a PASS/FAIL result. It runs the same way on the PC and on the Jetson.

```bash
make talker-listener        # self-test inside this container: run it on the PC, then on the Jetson
make talker                 # two-machine test, machine A: publish "Hello World: N" on /chatter (Ctrl-C stops it)
make listener               # two-machine test, machine B: PASS only if it hears the talker (and says who it was)
make talker-stop            # stop a talker that was left running without a terminal
```

| Test | What a PASS proves |
|---|---|
| `make talker-listener` | ROS 2 works **inside one container**: rclcpp + rclpy + type support + the selected RMW. It uses a private topic, so a talker elsewhere on the network cannot make it pass by accident. It also checks the message sequence (no gaps or duplicates) and that both nodes are visible in the ROS graph. |
| `make talker` on A + `make listener` on B | DDS discovery **across the network**: same `ROS_DOMAIN_ID` and RMW, multicast not blocked, firewall open. The listener names the publisher (`/talker_pc_dev` or `/talker_jetson_driver`) and tells you whether it ran on another role. Do it in both directions. |

Options go on the command line: `make listener COUNT=20 TIMEOUT=60 TOPIC=/arm_test` (messages to hear, seconds to wait,
topic; defaults 5 / 30 / `/chatter`). `COUNT=0` makes the listener run until Ctrl-C without a verdict. The script exits
0 on PASS, 1 on FAIL and 2 if it cannot run (a demo package or ROS is missing); `make` itself reports 2 for any
failing recipe. A FAIL prints what to check, in order: is a talker running, same domain and RMW, network, then what the
container can see.

Notes:

- `tests/` is mounted read-only at `/opt/seame/tests` in both containers: edit the script on the host and the next run
  uses it, no rebuild. The first `make up` after updating recreates the container to add the mount.
- The **Jetson image needs a rebuild** (`make build ROLE=jetson`; a small stage) because `ros-base` does not include the
  demo nodes. The PC image already has them through `ros-lyrical-desktop`. To try it before rebuilding:
  `make shell ROLE=jetson`, then `sudo apt-get update && sudo apt-get install -y ros-lyrical-demo-nodes-cpp ros-lyrical-demo-nodes-py`.
- Started in a normal terminal, Ctrl-C stops the nodes and leaves nothing running. A talker started *without* a terminal
  (CI, `make talker CI=true`, a closed window) survives the end of `docker exec`: use `make talker-stop`.

## Arm services (Jetson): driver, temperature, joystick, boot/shutdown routine

Everything from `ws/src/seame_arm` runs all the time on the Nano **inside ROS 2, in one container** (compose profile
`arm`, same `jetson-driver` image, no rebuild). `arm_routine.sh` is its entrypoint: it starts
`ros2 launch seame_arm arm.launch.py` and adds the zero/park routine around it:

```
arm_driver           ROS 2 node   the ONLY process that opens /dev/ttyTHS1 (libs/cpp MyCobot280)
                                  pub /joint_states (~20 Hz), /arm/servo_temperatures (every 2 s)
                                  sub /arm/command (STOP | JOINTS | ALL_ANGLES, latest wins), /arm/led
                                  srv /arm/power_on, /arm/release_servos, /arm/set_gripper
                                  srv /arm/set_read_only, pub /arm/read_only (rest mode, see below)
temperature_monitor  ROS 2 node   temperatures -> /diagnostics + LCD1602 (port of libs/cpp temps_lcd); respawned
joystick_teleop      ROS 2 node   joystick -> /arm/command (port of libs/cpp joystick_all_joints); respawned
braco web page       uvicorn      http://<jetson>:8000 (WEB_PORT), ARM_SOURCE=ros: a plain client of the topics/services above
arm_routine.sh       entrypoint   start: zero.py (all joints to 0) / stop: park.py (fold, release joints), then the page and the launch end
```

Launch arguments: `port`, `baud`, `lcd_bus`, `device` (filled from `.env`). Single nodes are no longer separate
containers; use ROS 2 tools on the running graph (`make exec CMD='ros2 node list'`, `ros2 param`, `ros2 service call`).
Logs of all nodes: `make arm-logs`.

- The temperature monitor depends only on the driver: it keeps updating whether the joystick is in use or not.
- The driver never moves or powers the arm by itself. If no new motion command arrives for 300 ms after a move,
  it sends `stop` (so a crashed or killed client cannot leave the arm running). Together with the step-ahead
  targets of the teleop, the arm coasts at most a few degrees.
- The teleop waits for the joystick and survives unplugging it (it stops the arm and opens the device again).
- `arm_routine.sh` replaces the old `~/scripts/zero.py` and `park.py`, which opened the serial port with pymycobot.
  The container has no restart policy: zero runs only when `make arm-up` or the boot unit starts it, and park runs
  whenever the container is stopped (`make arm-down`, `systemctl stop seame-arm`, a normal Jetson shutdown), before the
  launch stops the driver, so park can still reach the arm. Nothing runs on a power cut or `kill -9`, and
  `systemctl restart docker` stops every container at once, so park may fail then: use `make arm-down` first.
  Run them by hand with `make arm-park` / `make arm-zero` (no container has to be stopped).
- **Rest mode.** `park.py` ends by putting the driver in rest mode: motion, gripper and `power_on` are refused, but
  angles and temperatures keep being published, so a hot arm can cool down under watch (LCD and `/diagnostics` keep
  updating). Leave it with `make arm-resume` or `make arm-zero`. Details: [../docs/arm-rest-mode.md](../docs/arm-rest-mode.md).
- The driver opens the port with `TIOCEXCL`: a second program (non-root) gets `Device or resource busy` immediately.
- **The braco web page** ([../web/README.md](../web/README.md)) runs in this container too: the repository is mounted
  read-only at `/opt/braco` and `arm_routine.sh` starts uvicorn there (one process, `--workers 1`). It moves the arm
  only through `/arm/command` (bounded steps resent every 100 ms, so the 300 ms watchdog stops the arm if the page
  dies) and the driver's services. If the page crashes only a log line is printed; the arm keeps running. It needs
  fastapi/uvicorn in the image: **rebuild once** with `make build ROLE=jetson` (they moved into the `base` stage).
  Its ROS 2 > Demos tab uses the issue-18 demos: start them with `make arm-basics`.
- The C++ library in `../libs/cpp` is mounted read-only at `/opt/seame/libs` and built by colcon as `mycobot_cpp`;
  its own `Makefile` still builds the stand-alone programs.

```bash
make up ROLE=jetson && make ws-build     # build the workspace (includes ../libs) in the dev container
make arm-up                              # start the arm container; make arm-ps / make arm-logs
make arm-restart                         # after every make ws-build
make arm-web-restart                     # only the web page, after editing web/ (the arm is not zeroed or parked)
make arm-park                            # fold, release joints, rest mode (driver keeps reading temperatures)
make arm-temps                           # print the servo temperatures once
make arm-web-logs                        # only the web page's log lines; the page is http://<jetson>:8000
make arm-basics                          # ros2_basics demos (chatter, add_two_ints, fibonacci, lifecycle) for the page
make arm-resume                          # leave rest mode; make arm-zero also moves to zero; make arm-rest enters it
make install-service                     # start it at boot (systemd unit seame-arm), needs sudo
make exec CMD='ros2 topic echo /arm/servo_temperatures'
```

`make env` fills in `I2C_GID` and `INPUT_GID`; `LCD_I2C_BUS` and `JOYSTICK_DEVICE` in `.env` change the devices.
**Only one program can use the serial port:** run `make arm-down` before starting
`temps_lcd`, `joystick_all_joints` or any pymycobot script by hand. But prefer not to: `make arm-park`, `arm-zero` and
`arm-temps` go through the driver, and stopping the container also stops the temperature readings.

## The compatibility contract

| Piece | How it is kept consistent |
|---|---|
| OS | Both images start from `ubuntu:26.04`. |
| ROS install | Follows the ROS 2 Lyrical "Installing on Ubuntu" page: UTF-8 locale, Universe, `ros2-apt-source`, **`apt upgrade`**, then the `ros-lyrical-*` packages. |
| ROS apt source | `ros2-apt-source` **1.3.0** is pinned (verified to have a `resolute` build), so the build never asks GitHub's rate-limited API for "latest". `ROS_APT_SOURCE_VERSION=latest` resolves dynamically with a redirect fallback. |
| ROS | One `base` stage: one ROS 2 Lyrical install, one ros2_control, one pymycobot for both images. |
| Gazebo | Installed **only** via `ros-lyrical-ros-gz`: on Lyrical that is Gazebo **Jetty** from packages.ros.org (Harmonic and Ionic are source-only). Never `apt install gz-*` by hand. |
| RMW / DDS | `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` come from one `.env` field and must be copied to both machines. Default is Fast DDS; Cyclone DDS is installed too. Zenoh (`rmw_zenoh_cpp`) is installed if available but needs `ros2 run rmw_zenoh_cpp rmw_zenohd` - it is not a drop-in switch. |
| Networking | `network_mode: host` + `ipc: host` (multicast discovery across machines; Fast DDS shared memory between containers on one host). Allow UDP between the machines if a firewall is active. |
| Images | Built locally and tagged `seame/<service>:lyrical`. `pull_policy: never` stops Compose from looking those names up on Docker Hub; a missing image is built, an existing one is used as is. `make pull` fetches a prebuilt one from your own registry. |
| Build flags | `colcon-defaults.yaml` is identical in both images (symlink-install, RelWithDebInfo, ccache). On the Nano add `--parallel-workers 2`. |
| User IDs | The image creates user `dev` with your `USER_UID/USER_GID` (and removes Ubuntu's default `ubuntu` user at UID 1000) so `./ws` stays writable. If you pull a prebuilt image whose UID differs from yours, rebuild with your IDs. |
| Real-time | The unprivileged `dev` user gets SCHED_FIFO and locked memory from the `rtprio`/`memlock` ulimits in `docker-compose.yml` (the `cap_add` entries only help root/sudo). If rootless Docker refuses the ulimits, delete that block. |
| Python | System Python of Ubuntu 26.04 (3.14). `pymycobot` is pip-installed into it with `PIP_BREAK_SYSTEM_PACKAGES=1` so ROS nodes can import it. `ruff`/`pre-commit` live in their own virtualenv (they conflict with apt-owned `packaging`). |

## What has been verified, and what has not

Checked against the real thing:

- Every `ros-lyrical-*` package named in the Dockerfile is released for Lyrical in the rosdistro (MoveIt 2.15.2,
  ros_gz 3.0.10, gz_ros2_control 3.0.9, ros2_control 6.12, ...), and every Ubuntu package exists in the 26.04 archive.
- The `base`, `jetson-driver` and `pc-dev` stages were built with real Docker Compose on an Ubuntu 26.04 userland:
  locale, Universe, ros2-apt-source install, `apt upgrade`, `pymycobot` 4.0.7 on Python 3.14, user creation, QA tools.
  The two machines' services start without rebuilding, with the right user, volume ownership and environment.
- A real `pc-dev` build on a PC (amd64) completed, and `make smoke` passed except for the two items below.

Found by that real `make smoke` run:

- **`moveit_servo` was missing.** The `moveit` metapackage does not depend on it (its `package.xml` lists only
  moveit_core, planners, plugins, ros, setup_assistant and commander), so it needs its own
  `ros-lyrical-moveit-servo`, which the Dockerfile now installs. All 24 ROS dependencies of Servo are released.
  If your image predates this fix: `sudo apt-get update && sudo apt-get install ros-lyrical-moveit-servo` inside the
  container to test, then `make build && make up` to make it permanent (only the last image layer rebuilds).
- **`ros-lyrical-plotjuggler-ros` has no apt binary** (released in the rosdistro, but not built for 26.04). It is no longer
  used. The image installs **PlotJuggler 4** from its own repository instead
  (github.com/PlotJuggler/PlotJuggler, release `4.0.0`): the `.deb` is self-contained (bundles Qt6) and its
  ROS 2 Topic Subscriber extension has a `lyrical` build from version 1.1.1 on (the `.deb` bundles 1.0.2, which stops
  at humble/iron/jazzy/rolling, so 1.1.1 is installed over it). Both downloads are pinned by SHA-256 in the Dockerfile;
  `docker/scripts/install-plotjuggler.sh` explains how to upgrade. amd64 only, and a failure here never fails the build.
  Tested on Ubuntu 26.04: install, launch, extension kept (not overwritten) on first start, dispatch to `dist/lyrical`.

**Not** verified:

1. The arm64 / Jetson build. `pymycobot` pulls `bleak` -> `dbus-fast`; if no aarch64 wheel exists for Python 3.14 pip
   compiles it (build-essential and python3-dev are in the image). Optional packages that fail to install on one
   architecture print a warning block and are listed by `seame-smoke-test` instead of failing the build. If
   `gz_ros2_control` is missing, build it in `ws/src` from `ros-controls/gz_ros2_control` on the branch matching Lyrical.
2. **Live ROS 2 topics in PlotJuggler.** The plugin loads the Lyrical build and needs `librclcpp`, `librcl`,
   `librcutils` and `libtracetools`, which `ros-lyrical-ros-base` provides. I could not subscribe to real topics (no ROS
   repository where this was developed): `make smoke` checks that those libraries resolve and warns if not. Opening
   recorded bags does not need ROS at all.
3. **The talker/listener test against real ROS 2.** It was developed against a stand-in for the `ros2` command (same
   process structure as the real `ros2 run`, per-domain emulation of DDS, fault injection) and through the real Makefile,
   Compose and containers, but never against a real Lyrical install. It relies on the demo nodes' log line
   `I heard: [Hello World: N]` and on the layout of `ros2 topic info -v`; if a real run disagrees, `make exec CMD='ros2
   run demo_nodes_py listener'` shows the actual output to compare with.
4. **Serial port.** `/dev/ttyTHS1` at 1000000 baud is what I remember for the Jetson Nano version; confirm in the
   Elephant Robotics docs. On stock JetPack the UART is held by the serial console: on the host run
   `sudo systemctl disable --now nvgetty` and re-login before the container can open it.
5. **E-stop.** The class README does not say how it reaches the Jetson. If it is a GPIO line, uncomment the
   `/dev/gpiochip0` device in `docker-compose.yml` and have your lifecycle node read it.

## If the build fails

| Symptom | Cause and fix |
|---|---|
| `curl: (60) SSL certificate ... self-signed certificate in certificate chain` in step 2 | A network that re-signs HTTPS (some campus/corporate proxies). Add the proxy's CA certificate to the image (copy it to `/usr/local/share/ca-certificates/` and run `update-ca-certificates` before step 2). |
| `could not determine the ros-apt-source version` / 403 | Only with `ROS_APT_SOURCE_VERSION=latest` and no way to reach github.com. Use the pinned default. |
| `! Image seame/pc-dev:lyrical pull access denied ... repository does not exist` | Harmless and gone in this version. Compose used to try Docker Hub for `seame/pc-dev` (an image that only exists once you build it) before falling back to building. The compose file now has `pull_policy: never`, so it builds locally without asking a registry. If you still see it, your `docker-compose.yml` predates that line. |
| `toomanyrequests` pulling `ubuntu:26.04` | Docker Hub's anonymous pull limit; `docker login`, or wait. |
| Warning block "optional packages that could NOT be installed" | Not an error: those packages are optional and the build continues. `make smoke` lists them with a hint. |
| `PlotJuggler 4: not installed: ...` in the skipped list | The GitHub download failed or its checksum did not match; the image still works. Rebuild with network access, or on a non-amd64 host use Foxglove Studio via `foxglove_bridge` or `rqt_plot`. |
| `Unable to locate package ros-lyrical-...` | The ROS repo is not reachable (check `/etc/apt/sources.list.d/ros2.sources` and your network) or that binary does not exist yet for your arch. |
| `Cannot uninstall ... installed by debian` from pip | A pip dependency conflicts with an apt-owned package. Pin `PYMYCOBOT_VERSION`, or install the offending tool in its own virtualenv as done for `ruff`. |
| `error gathering device information while adding custom device "/dev/ttyTHS1"` | The Jetson service needs the serial device on the host; on a PC use `COMPOSE_PROFILES=pc`. |

## Jetson host: what to check before Module 1 hardware work

The Nano's stock JetPack 4.6 host is Ubuntu 18.04 with a 4.9 kernel and an old Docker Engine. Containers share
that kernel, so a 26.04 userland should be smoke-tested first:

```bash
docker run --rm ubuntu:26.04 bash -c 'uname -r && ls /dev/null && echo userland-ok'
```

- If that fails with odd `Operation not permitted` errors, the host's Docker/libseccomp is too old: upgrade Docker
  Engine (and `libseccomp2`) on the Nano, or use a newer community Ubuntu image for the Nano.
- Building on the Nano needs Docker Engine 24+ and Compose 2.20+. Otherwise build on the PC and ship the image:
  ```bash
  make binfmt                                            # one-time QEMU setup
  make jetson-cross                                      # slow: arm64 under emulation; tag <IMAGE_TAG>-arm64
  make jetson-ship NANO=user@nano.local                  # docker save | gzip | ssh docker load
  ```
  or push the image to a registry (`make push` on the PC, with `IMAGE_PREFIX=ghcr.io/<you>` in `.env` on both machines)
  and run `make pull` on the Nano. Prebuilt images work because `up` never rebuilds an image that exists, and it never
  pulls either (see `pull_policy: never` below).

## Module 2 and the Nano (TensorRT)

Not covered here on purpose. The original Jetson Nano's CUDA/cuDNN/TensorRT come from JetPack 4.6 (CUDA 10.2,
Python 3.6 bindings) and are injected from the host by the NVIDIA runtime; they cannot be used from a Python 3.14
userland on Ubuntu 26.04. When Module 2 starts, the realistic options are a separate L4T-based inference container
(dusty-nv `jetson-containers`) that exchanges images/detections with ROS 2 over DDS, or running inference on the PC and
measuring that trade-off - which the class README asks for anyway.

## Handy commands

```bash
docker compose build --no-cache pc-dev                  # refresh the PC image (the base layers too)
docker compose down                                     # stop containers (volumes with ccache/gz cache are kept)
docker compose run --rm pc-dev bash -c 'colcon build && colcon test && colcon test-result --verbose'
docker compose run --rm pc-dev ros2 launch urdf_tutorial display.launch.py model:=/ws/src/<pkg>/urdf/arm.urdf.xacro
docker build --target pc-dev -t seame/pc-dev:lyrical docker     # without compose
```
