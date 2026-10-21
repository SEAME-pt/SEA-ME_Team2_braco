# SEA:ME 2.0 - Arm class containers (braco)

Ubuntu 26.04 + ROS 2 Lyrical Luth for the myCobot 280 Jetson Nano. Each machine has **two containers**: `dev` for
the tools, and the braco system itself, `real` on the Jetson or `sim` on the PC (on the Jetson `dev` only exists
while a command runs).

```
 PC / workstation (x86_64)                        Jetson Nano in the arm base (aarch64)
 ┌───────────────────────────────┐   DDS (same    ┌───────────────────────────────┐
 │ dev  shell, build, test,      │   domain+RMW)  │ dev   shell, build, test      │
 │      RViz2 · MoveIt 2 · Gazebo│ ◄────────────► │ real  arm_driver, temperature,│
 │ sim  simulated arm + web page │  host network  │       joystick + web page     │
 └───────────────┬───────────────┘                └───────────────┬───────────────┘
        image braco-image/pc ─────────► stage `base` ◄───────── image braco-image/jetson
          Ubuntu 26.04 · ROS 2 Lyrical (ros-base) · ros2_control · pymycobot · colcon defaults
```

One `docker/Dockerfile` with three stages. Both images are built `FROM base`, so ROS, ros2_control and
pymycobot are the same on both machines. On each machine `dev` and the system use the **same image**, and the
system runs what `dev` built in `ros2_ws`. On the PC `dev` stays up; on the Jetson it never does: each `make build`,
`shell`, `test`, `run` or `net-*` is a throw-away `docker compose run --rm dev`, so normally only `real` runs there.

| Compose service | Profile | Image (Dockerfile target) | Runs | Contains |
|---|---|---|---|---|
| `dev` | (always) | `braco-image/pc` (`pc`) or `braco-image/jetson` (`jetson`), from `COMPOSE_PROFILES` | PC: `sleep infinity`, you `exec` into it. Jetson: one throw-away container per command | PC: `ros-lyrical-desktop`, `ros-lyrical-ros-gz`, `ros-lyrical-moveit` **and** `ros-lyrical-moveit-servo` (a separate package), clang-tidy, cppcheck, lcov/gcovr, ruff, pre-commit, pytest-cov, **PlotJuggler 4**; optional gz_ros2_control, Foxglove bridge. Jetson: `base` plus serial tooling and the demo nodes |
| `real` | `jetson` | `braco-image/jetson` | `arm_routine.sh`: the braco launch on the real arm + the web page | the serial port, the LCD's I2C bus, `/dev/input` |
| `sim` | `pc` | `braco-image/pc` | `arm_routine.sh` with `SIM=true`: the launch with `sim:=true` + the web page on Gazebo | your display (X11) |

`COMPOSE_PROFILES` in `.env` is the machine: exactly `pc` or `jetson`. It enables `sim` or `real` and picks the image
of `dev`.

## Project layout

```
mk/                          the Makefile's commands, one file per topic (the Makefile itself is at the repo root)
docker/
├── README.md
├── .env.example             copied to .env by `make setup` (or the first `make up`)
├── .gitignore
├── .dockerignore            keeps .env, compose files and tests out of the image build context
├── compose.yml              services dev, real, sim; found by plain `docker compose` without -f
├── compose/
│   ├── nvidia.yml           optional GPU overrides for dev and sim (make up GPU=nvidia | dri)
│   └── dri.yml
├── Dockerfile               stages: base, jetson, pc
├── config/                  colcon-defaults.yaml
├── scripts/                 entrypoint, env sourcing, smoke test, installers, apt helper
├── tests/
│   └── talker-listener.sh   ROS 2 talker/listener test; mounted read-only into dev
└── systemd/
    └── braco.service        boot unit for the system on the Jetson (make boot-on / boot-off)
```

The ROS 2 workspace is **`../ros2_ws`** (at the repo root), mounted at `/ws` in every container: `ros2_ws/src` holds
the packages, `ros2_ws/bags` the rosbag recordings. `make up` creates both folders **as your user**: if Docker
creates a missing bind-mount folder itself it can end up owned by root, and then `colcon build` cannot write to it.
Created by `make` when needed, never shipped: `.env`, `ros2_ws/bags`.

There is a single `Makefile`, at the **repo root**, split into `mk/*.mk`: run every `make` command from there. It
passes `--project-directory docker` to `docker compose`, so `.env`, `../ros2_ws` and `../libs` resolve exactly as
below. Every path it uses is defined in one block of `mk/config.mk`, so moving a file means editing one line there.

## Quick start

```bash
make up          # .env if missing, host checks, image if missing, colcon build if a source changed, start, wait, URL
make status      # containers, CPU/RAM, what the page sees (and tegrastats on the Jetson)
make shell       # a shell in dev, ROS and the workspace sourced
make down        # off (on the Jetson: make down PARK=1 parks the arm first)
```

`make up` is **idempotent**: run it again and it skips the image and the build, and leaves running containers alone.
After a source change it rebuilds (colcon only recompiles what changed) and restarts the system container. On the
Jetson it waits until the web page says the arm is connected (`UP_TIMEOUT=90`); on the PC it waits less, because the
simulated arm only connects once the Gazebo bringup (#34) exists. It ends by printing the page's URL.

Without `make` (run these from inside `docker/`):

```bash
cp .env.example .env                    # set COMPOSE_PROFILES=pc|jetson and USER_UID/USER_GID
mkdir -p ../ros2_ws/src ../ros2_ws/bags # as YOUR user, so Docker does not create them as root
docker compose build dev                # the machine's image
docker compose up -d dev
docker compose exec dev ros_entrypoint.sh colcon build --base-paths src /opt/seame/libs
docker compose up -d sim                # or real on the Jetson
docker compose exec dev bash
```

`docker compose exec dev bash` gives you ROS and the workspace (`/ws`) sourced. A bare `docker compose exec dev <command>`
skips the entrypoint, so ROS is **not** sourced - prefix it with `ros_entrypoint.sh` (as above), or use
`make run CMD='...'`.

## Make commands

`make` lists them all. The machine (`ROLE=pc` or `ROLE=jetson`) is read from `COMPOSE_PROFILES` in `.env`, or detected
(arm64 + Tegra marker = Jetson); `ROLE=...` on the command line overrides it.

| Group | Commands |
|---|---|
| System | `up`, `down` (`PARK=1`), `restart`, `status`, `logs` (`S=web`, `S=arm_driver`, any pattern) |
| Arm (Jetson) | `zero`, `park`, `rest`, `resume`, `temps` |
| Development | `shell`, `run CMD='...'`, `build` (`COLCON_ARGS=...`), `test`, `web` (mock page on :8080), `clean` |
| Setup | `setup`, `doctor`, `image` (`IMAGE_ARGS='--no-cache --pull'`), `boot-on`, `boot-off`, `jetson-image` (`NANO=user@host`), `reset` |
| Network | `net-test`, `net-talk`, `net-listen` |
| CI | `ci`, `ci-ros`, `lint` |

- `make up GPU=nvidia` (or `GPU=dri`) adds `compose/nvidia.yml` or `compose/dri.yml` for `dev` and `sim` (PC only);
  `COMPOSE_FILE` in `.env` does the same permanently.
- `make build` always runs colcon; `make up` only when something under `ros2_ws/src` or `libs` changed
  (on the Jetson, only the packages of the real system: see below).
  After `make build`, `make restart` (or the next `make up`) runs the new build.
- `make clean` deletes colcon's `build/ install/ log/` and the ROS logs (`~/.ros/log` on the host and in the
  containers). `make reset` also removes the containers, the cache volumes and the images, and asks first
  (`FORCE=1` skips the prompt).
- In CI (`CI=true`) or without a terminal, TTY allocation is turned off automatically.

**The Jetson never simulates.** Its image has no Gazebo, MoveIt or RViz (those are only in the `pc` stage), and
`make build` there only takes `braco_bringup` and what it depends on (driver, teleop, interfaces,
`mycobot_cpp`): `braco_description`, `braco_moveit_config` and
`braco_mission` are left out (`WS_PACKAGES` in `mk/config.mk`). If the real arm needs another package, add it to
`braco_bringup/package.xml` as an `exec_depend`; the opposite holds too: never make `braco_bringup` depend on a
simulation package (Gazebo, MoveIt), or the Jetson builds it again. The PC builds everything.

**Jetson Nano limits.** On the Jetson `make build` runs colcon with one package at a time
(`--parallel-workers 1`) and `MAKEFLAGS=-j2`, in `Release` without tests (`BUILD_TESTING=OFF`: `make test` runs on the
PC and in CI), and ccache is capped at 1 GB (`CCACHE_MAXSIZE` in `.env` overrides
it). `make doctor` warns when a desktop session is running (~1 GB of RAM), when the power mode is not MAXN (10 W),
and when there is no swap.

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
exercised), automated with a PASS/FAIL result. It runs in `dev`, the same way on the PC and on the Jetson.

```bash
make net-test               # self-test inside this machine: run it on the PC, then on the Jetson
make net-talk               # two-machine test, machine A: publish "Hello World: N" on /chatter (Ctrl-C stops it)
make net-listen             # two-machine test, machine B: PASS only if it hears the talker (and says who it was)
```

| Test | What a PASS proves |
|---|---|
| `make net-test` | ROS 2 works **inside one container**: rclcpp + rclpy + type support + the selected RMW. It uses a private topic, so a talker elsewhere on the network cannot make it pass by accident. It also checks the message sequence (no gaps or duplicates) and that both nodes are visible in the ROS graph. |
| `make net-talk` on A + `make net-listen` on B | DDS discovery **across the network**: same `ROS_DOMAIN_ID` and RMW, multicast not blocked, firewall open. The listener names the publisher (`/talker_pc_dev` or `/talker_jetson_driver`) and tells you whether it ran on another role. Do it in both directions. |

Options go on the command line: `make net-listen COUNT=20 TIMEOUT=60 TOPIC=/arm_test` (messages to hear, seconds to
wait, topic; defaults 5 / 30 / `/chatter`). `COUNT=0` makes the listener run until Ctrl-C without a verdict. The
script exits 0 on PASS, 1 on FAIL and 2 if it cannot run (a demo package or ROS is missing); `make` itself reports 2
for any failing recipe. A FAIL prints what to check, in order: is a talker running, same domain and RMW, network,
then what the container can see.

Notes:

- `tests/` is mounted read-only at `/opt/seame/tests` in `dev`: edit the script on the host and the next run uses it,
  no rebuild.
- `ros-base` does not include the demo nodes, so the Jetson image installs them in its own stage; the PC image has
  them through `ros-lyrical-desktop`.
- Started in a normal terminal, Ctrl-C stops the nodes and leaves nothing running. A talker started *without* a
  terminal (CI, a closed window) survives the end of `docker exec`: `make restart` or `make down` clears it.

## The system on the Jetson: driver, temperature, joystick, web page

Everything from `ros2_ws/src/braco_driver`, `braco_teleop` and `braco_bringup` runs all the time on the Nano **inside
ROS 2, in one container**, `real` (same `braco-image/jetson` image as `dev`, no build of its own). `arm_routine.sh` is its
entrypoint: it starts `ros2 launch braco_bringup braco.launch.py sim:=false` and the web page. It does **not** move
the arm at start or stop:

```
arm_driver           ROS 2 node   the ONLY process that opens /dev/ttyTHS1 (libs/cpp MyCobot280)
                                  pub /joint_states (~20 Hz), /arm/servo_temperatures (every 2 s)
                                  sub /arm/command (STOP | JOINTS | ALL_ANGLES, latest wins), /arm/led
                                  srv /arm/power_on, /arm/release_servos, /arm/set_gripper
                                  srv /arm/set_read_only, pub /arm/read_only (rest mode, see below)
temperature_monitor  ROS 2 node   temperatures -> /diagnostics + LCD1602 (port of libs/cpp temps_lcd); respawned
joystick_teleop      ROS 2 node   joystick -> /arm/command (port of libs/cpp joystick_all_joints); respawned
braco web page       uvicorn      http://<jetson>:8000 (WEB_PORT), ARM_SOURCE=ros: a plain client of the topics/services above
arm_routine.sh       entrypoint   start: the launch + the page / stop: the page, then the launch (no zero, no park)
```

Launch arguments: `port`, `baud`, `lcd_bus`, `device` (filled from `.env`). Use ROS 2 tools on the running graph from
`dev` (`make run CMD='ros2 node list'`, `ros2 param`, `ros2 service call`). Logs of all nodes: `make logs`; one node:
`make logs S=arm_driver`; only the page: `make logs S=web`.

- The temperature monitor depends only on the driver: it keeps updating whether the joystick is in use or not.
- The driver never moves or powers the arm by itself. If no new motion command arrives for 300 ms after a move,
  it sends `stop` (so a crashed or killed client cannot leave the arm running). Together with the step-ahead
  targets of the teleop, the arm coasts at most a few degrees.
- The teleop waits for the joystick and survives unplugging it (it stops the arm and opens the device again).
- Zero and park are **manual**: nothing moves the arm when the container starts or stops (`make up`, `make down`,
  `make restart`, `systemctl stop braco`, a Jetson shutdown). Run them with `make zero` / `make park` (no container
  has to be stopped), or `make down PARK=1` to park and then stop. Park the arm before shutting down, or it will drop
  when the servos lose power.
- **Rest mode.** `make park` (`ros2 run braco_bringup park`) ends by putting the driver in rest mode: motion, gripper and `power_on` are
  refused, but angles and temperatures keep being published, so a hot arm can cool down under watch (LCD and
  `/diagnostics` keep updating). Leave it with `make resume` or `make zero`; `make rest` enters it without parking.
  Details: [../docs/arm-rest-mode.md](../docs/arm-rest-mode.md).
- The driver opens the port with `TIOCEXCL`: a second program (non-root) gets `Device or resource busy` immediately.
- **The braco web page** ([../web/README.md](../web/README.md)) runs in this container too: the repository is mounted
  read-only at `/opt/braco` and `arm_routine.sh` starts uvicorn there (one process, `--workers 1`). It moves the arm
  only through `/arm/command` (bounded steps resent every 100 ms, so the 300 ms watchdog stops the arm if the page
  dies) and the driver's services. If the page crashes only a log line is printed; the arm keeps running.
- The C++ library in `../libs/cpp` is mounted read-only at `/opt/seame/libs` and built by colcon as `mycobot_cpp`;
  its own `Makefile` still builds the stand-alone programs.

```bash
make up                                  # build if needed, start real, wait for the arm, print the URL
make status                              # containers, CPU/RAM, the arm as the page sees it, 3 s of tegrastats
make park                                # fold, release joints, rest mode (driver keeps reading temperatures)
make temps                               # print the servo temperatures once
make resume                              # leave rest mode; make zero also moves to zero; make rest enters it
make boot-on                             # start the system at boot (systemd unit braco), needs sudo; make boot-off
make run CMD='ros2 topic echo /arm/servo_temperatures'
```

`make setup` fills in `I2C_GID` and `INPUT_GID`; `LCD_I2C_BUS` and `JOYSTICK_DEVICE` in `.env` change the devices.
**Only one program can use the serial port:** run `make down` before starting `temps_lcd`, `joystick_all_joints` or
any pymycobot script by hand. But prefer not to: `make park`, `zero` and `temps` go through the driver, and stopping
the container also stops the temperature readings.

## The system on the PC: simulation

`sim` runs the same `arm_routine.sh` with `SIM=true`: `braco.launch.py sim:=true` and the web page with
`ARM_SOURCE=gazebo` (http://localhost:8000). Until the Gazebo bringup (#34) and the model (#25) exist the launch
starts no node, so the page is up but shows the arm as not connected, and `make up` only warns about it.

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
| Images | Built locally and tagged `braco-image/pc:lyrical` and `braco-image/jetson:lyrical`. `pull_policy: never` stops Compose from looking those names up on Docker Hub; `make up` builds a missing image and uses an existing one as is (`make image` rebuilds it). |
| Build flags | `colcon-defaults.yaml` is identical in both images (symlink-install, RelWithDebInfo, ccache). On the Nano `make build` adds `--parallel-workers 1`, `MAKEFLAGS=-j2`, Release and no tests. |
| User IDs | The image creates user `dev` with your `USER_UID/USER_GID` (and removes Ubuntu's default `ubuntu` user at UID 1000) so `ros2_ws` stays writable. If you pull a prebuilt image whose UID differs from yours, rebuild with your IDs. |
| Real-time | The unprivileged `dev` user gets SCHED_FIFO and locked memory from the `rtprio`/`memlock` ulimits in `compose.yml` (the `cap_add` entries only help root/sudo). If rootless Docker refuses the ulimits, delete that block. |
| Python | System Python of Ubuntu 26.04 (3.14). `pymycobot` is pip-installed into it with `PIP_BREAK_SYSTEM_PACKAGES=1` so ROS nodes can import it. `ruff`/`pre-commit` live in their own virtualenv (they conflict with apt-owned `packaging`). |

## What has been verified, and what has not

Checked against the real thing:

- Every `ros-lyrical-*` package named in the Dockerfile is released for Lyrical in the rosdistro (MoveIt 2.15.2,
  ros_gz 3.0.10, gz_ros2_control 3.0.9, ros2_control 6.12, ...), and every Ubuntu package exists in the 26.04 archive.
- The `base`, `jetson` and `pc` stages (then named `jetson-driver` and `pc-dev`) were built with real Docker Compose on an Ubuntu 26.04 userland:
  locale, Universe, ros2-apt-source install, `apt upgrade`, `pymycobot` 4.0.7 on Python 3.14, user creation, QA tools.
  The two machines' services start without rebuilding, with the right user, volume ownership and environment.
- A real PC image build (amd64) completed, and the smoke test (`make run CMD=seame-smoke-test`) passed except for the two items below.

Found by that real smoke test run:

- **`moveit_servo` was missing.** The `moveit` metapackage does not depend on it (its `package.xml` lists only
  moveit_core, planners, plugins, ros, setup_assistant and commander), so it needs its own
  `ros-lyrical-moveit-servo`, which the Dockerfile now installs. All 24 ROS dependencies of Servo are released.
  If your image predates this fix: `sudo apt-get update && sudo apt-get install ros-lyrical-moveit-servo` inside the
  container to test, then `make image && make up` to make it permanent (only the last image layer rebuilds).
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
   `gz_ros2_control` is missing, build it in `ros2_ws/src` from `ros-controls/gz_ros2_control` on the branch matching Lyrical.
2. **Live ROS 2 topics in PlotJuggler.** The plugin loads the Lyrical build and needs `librclcpp`, `librcl`,
   `librcutils` and `libtracetools`, which `ros-lyrical-ros-base` provides. I could not subscribe to real topics (no ROS
   repository where this was developed): the smoke test checks that those libraries resolve and warns if not. Opening
   recorded bags does not need ROS at all.
3. **The talker/listener test against real ROS 2.** It was developed against a stand-in for the `ros2` command (same
   process structure as the real `ros2 run`, per-domain emulation of DDS, fault injection) and through the real Makefile,
   Compose and containers, but never against a real Lyrical install. It relies on the demo nodes' log line
   `I heard: [Hello World: N]` and on the layout of `ros2 topic info -v`; if a real run disagrees, `make run CMD='ros2
   run demo_nodes_py listener'` shows the actual output to compare with.
4. **Serial port.** `/dev/ttyTHS1` at 1000000 baud is what I remember for the Jetson Nano version; confirm in the
   Elephant Robotics docs. On stock JetPack the UART is held by the serial console: on the host run
   `sudo systemctl disable --now nvgetty` and re-login before the container can open it.
5. **E-stop.** The class README does not say how it reaches the Jetson. If it is a GPIO line, uncomment the
   `/dev/gpiochip0` device in `compose.yml` and have your lifecycle node read it.

## If the build fails

| Symptom | Cause and fix |
|---|---|
| `curl: (60) SSL certificate ... self-signed certificate in certificate chain` in step 2 | A network that re-signs HTTPS (some campus/corporate proxies). Add the proxy's CA certificate to the image (copy it to `/usr/local/share/ca-certificates/` and run `update-ca-certificates` before step 2). |
| `could not determine the ros-apt-source version` / 403 | Only with `ROS_APT_SOURCE_VERSION=latest` and no way to reach github.com. Use the pinned default. |
| `! Image braco-image/pc:lyrical pull access denied ... repository does not exist` | Harmless and gone in this version. Compose used to try Docker Hub for `braco-image/pc` (an image that only exists once you build it) before falling back to building. The compose file now has `pull_policy: never`, so it builds locally without asking a registry. If you still see it, your `compose.yml` predates that line. |
| `toomanyrequests` pulling `ubuntu:26.04` | Docker Hub's anonymous pull limit; `docker login`, or wait. |
| Warning block "optional packages that could NOT be installed" | Not an error: those packages are optional and the build continues. The smoke test (`make run CMD=seame-smoke-test`) lists them with a hint. |
| `PlotJuggler 4: not installed: ...` in the skipped list | The GitHub download failed or its checksum did not match; the image still works. Rebuild with network access, or on a non-amd64 host use Foxglove Studio via `foxglove_bridge` or `rqt_plot`. |
| `Unable to locate package ros-lyrical-...` | The ROS repo is not reachable (check `/etc/apt/sources.list.d/ros2.sources` and your network) or that binary does not exist yet for your arch. |
| `Cannot uninstall ... installed by debian` from pip | A pip dependency conflicts with an apt-owned package. Pin `PYMYCOBOT_VERSION`, or install the offending tool in its own virtualenv as done for `ruff`. |
| `error gathering device information while adding custom device "/dev/ttyTHS1"` | `real` needs the serial device (and the LCD's I2C bus) on the host: `make doctor` checks both. On a PC use `COMPOSE_PROFILES=pc`. |

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
  make jetson-image NANO=user@nano.local   # QEMU (registered once), slow arm64 build, docker save | gzip | ssh docker load
  ```
  The image is tagged `<IMAGE_TAG>-arm64` on the PC and gets its usual name on the Nano, so `make up` there uses it:
  `up` never rebuilds an image that exists, and it never pulls either (`pull_policy: never`).

## Module 2 and the Nano (TensorRT)

Not covered here on purpose. The original Jetson Nano's CUDA/cuDNN/TensorRT come from JetPack 4.6 (CUDA 10.2,
Python 3.6 bindings) and are injected from the host by the NVIDIA runtime; they cannot be used from a Python 3.14
userland on Ubuntu 26.04. When Module 2 starts, the realistic options are a separate L4T-based inference container
(dusty-nv `jetson-containers`) that exchanges images/detections with ROS 2 over DDS, or running inference on the PC and
measuring that trade-off - which the class README asks for anyway.

## Handy commands

Run from inside `docker/` (`COMPOSE_PROFILES` comes from `.env`):

```bash
docker compose build --no-cache dev                     # refresh this machine's image (the base layers too)
docker compose down                                     # stop the containers (volumes with ccache/gz cache are kept)
docker compose run --rm dev bash -c 'colcon build && colcon test && colcon test-result --verbose'
docker compose run --rm dev ros2 launch urdf_tutorial display.launch.py model:=/ws/src/<pkg>/urdf/arm.urdf.xacro
docker build --target pc -t braco-image/pc:lyrical docker     # without compose
```
