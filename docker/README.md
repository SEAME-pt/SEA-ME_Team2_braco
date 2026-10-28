# braco containers

Ubuntu 26.04 + ROS 2 Lyrical for the myCobot 280 Jetson Nano, in Docker.

## 1. Overview

- **Each machine has two containers:** `dev` for the tools, and the braco system (`real` on the Jetson, `sim` on
  the PC).
- **One `Dockerfile`, three stages:** `base` (ROS 2, ros2_control, pymycobot), then `pc` and `jetson` built on top
  of it. So ROS is the same on both machines.
- **On each machine, `dev` and the system use the same image.** The system runs what `dev` built in `ros2_ws`.
- **`COMPOSE_PROFILES` in `docker/.env` says which machine this is** (`pc` or `jetson`). It turns on `sim` or `real`
  and picks the image of `dev`.
- **The machines talk over DDS** (ROS 2), on the host network, with the same RMW.

| Container | Machine | Image | What it runs | Stays up? |
|---|---|---|---|---|
| `dev` | PC | `braco-image/pc` | Your tools: shell, build, tests, RViz2, MoveIt 2, Gazebo, PlotJuggler 4. `make web` runs the page here | Yes |
| `dev` | Jetson | `braco-image/jetson` | Build and tools | No: one throw-away container per command |
| `sim` | PC | `braco-image/pc` | The simulated arm (Gazebo). Only with `make up SIM=1` | While the simulation runs |
| `real` | Jetson | `braco-image/jetson` | The real arm (driver, temperatures, joystick, camera) and the page (`make web`) | Yes |

## 2. Files

| Path | What it is |
|---|---|
| `Makefile`, `mk/*.mk` (repo root) | Every command. Paths are defined once in `mk/config.mk` |
| `docker/compose.yml` | The services `dev`, `real`, `sim` |
| `docker/compose/nvidia.yml`, `dri.yml` | GPU overrides for the PC |
| `docker/Dockerfile` | Stages `base`, `jetson`, `pc` |
| `docker/.env.example` | Copied to `docker/.env` by `make setup` |
| `docker/config/` | `colcon-defaults.yaml` |
| `docker/scripts/` | Entrypoint, environment, smoke test, installers |
| `docker/tests/talker-listener.sh` | The ROS 2 network test (mounted read-only in `dev`) |
| `docker/systemd/braco.service` | Starts the system at boot on the Jetson (`make boot-on`) |
| `ros2_ws/` | The workspace, mounted at `/ws` in every container (`src/` = packages, `bags/` = recordings) |

- `make` creates `ros2_ws/src` and `ros2_ws/bags` **as your user**. If Docker created them, they would belong to root
  and `colcon build` could not write to them.
- `make doctor` checks that every project file is there. `make lint` checks the compose files, the scripts
  (shellcheck) and the Dockerfile (hadolint).

## 3. Quick start

```bash
make up          # creates .env if missing, checks the host, builds the image and the code if needed, starts
make status      # containers, CPU/RAM, what the page sees
make shell       # a shell in dev, ROS ready
make down        # stops everything (Jetson: make down PARK=1 parks the arm first)
```

- **`make up` is safe to run again:** it skips what is already done.
- After a code change it rebuilds (only what changed) and restarts the system.
- On the Jetson, and on the PC with `SIM=1`, it waits until the arm publishes `/joint_states` (90 s max).

**Without `make`** (from inside `docker/`):

```bash
cp .env.example .env                    # set COMPOSE_PROFILES, USER_UID, USER_GID
mkdir -p ../ros2_ws/src ../ros2_ws/bags # as your user
docker compose build dev
docker compose up -d dev
docker compose exec dev ros_entrypoint.sh colcon build --base-paths src
docker compose up -d sim                # or real on the Jetson
docker compose exec dev bash            # ROS ready
```

- `docker compose exec dev <command>` alone does **not** load ROS: prefix it with `ros_entrypoint.sh`, or use
  `make run CMD='...'`.

## 4. Make commands

`make` alone lists them all.

| Group | Commands |
|---|---|
| System | `up` (`SIM=1` on the PC), `down` (`PARK=1`), `restart`, `status`, `logs` (`S=web`, `S=arm_driver`, any word) |
| Web page | `web` (`MOCK=1`: fake arm data), `web-stop` |
| Arm (Jetson) | `zero`, `park`, `rest`, `resume`, `temps`, `bench` |
| Development | `shell`, `run CMD='...'`, `build` (`COLCON_ARGS=...`), `test`, `sim-gui`, `clean` |
| Setup | `setup`, `role`, `doctor`, `image` (`IMAGE_ARGS='--no-cache --pull'`), `boot-on`, `boot-off`, `jetson-image` (`NANO=user@host`), `re`, `fclean` |
| Network | `net-test`, `net-talk`, `net-listen` |
| CI | `ci`, `web-test`, `ci-image`, `ci-ros`, `cppcheck`, `tidy`, `coverage`, `lint`, `format`, `format-check` ([CI guide](../docs/guides/ci.md)) |

**Options**

| Option | What it does |
|---|---|
| `ROLE=pc` / `ROLE=jetson` | Forces the machine (normally read from `.env`, or detected) |
| `GPU=dri` | Intel/AMD graphics on the PC (default: NVIDIA, `compose/nvidia.yml`) |
| `GPU=none` | No GPU on the PC. `COMPOSE_FILE` in `.env` sets the GPU permanently |
| `FORCE=1` | Skips the "are you sure?" of `re` and `fclean` |

**Build and clean**

- `make build` always runs colcon. `make up` builds only if something in `ros2_ws/src` changed.
- After `make build`, run `make restart` (or `make up`) to use the new build.
- `make clean`: deletes `build/ install/ log/` and the ROS logs.
- `make re`: also removes this machine's containers, volumes and image, then rebuilds everything. Never touches the
  other machine's image.
- `make fclean`: removes **every** Docker container, image, volume and network on the machine, **other projects'
  too**. Rebuilds nothing.

**The Jetson**

- **It never simulates:** no Gazebo, MoveIt or RViz in its image.
- **It builds only what the real arm needs:** `braco_bringup` and its dependencies (driver, teleop, camera,
  interfaces, `mycobot_cpp`).
  - A new package the real arm needs: add it to `braco_bringup/package.xml` as `exec_depend`.
  - Never make `braco_bringup` depend on a simulation package, or the Jetson builds it too.
- **It builds slowly on purpose** (4 GB of RAM): one package at a time, 2 compiler jobs, `Release`, no tests (they run
  on the PC and in CI), ccache max 1 GB (`CCACHE_MAXSIZE`).

## 5. The `real` container (Jetson)

- **Runs** `arm_routine.sh` → `ros2 launch braco_bringup braco.launch.py sim:=false`. The nodes:
  [architecture §3 and §4](../docs/architecture/architecture.md#3-nodes-and-devices).
- **Never moves the arm** when it starts or stops. Park before turning off (`make down PARK=1`), or the arm drops.
- **Stops when the launch ends** (for example if `arm_driver` loses the port). `arm_driver` is not restarted on
  purpose: a driver fault must be seen.
- **Settings from `.env`:**

  | Variable | What it sets |
  |---|---|
  | `MYCOBOT_PORT`, `MYCOBOT_BAUD` | Serial port of the arm (`/dev/ttyTHS1`, `1000000`) |
  | `LCD_I2C_BUS` | I2C bus of the LCD |
  | `JOYSTICK_DEVICE` | The joystick |
  | `CAMERA_DEVICE` | The camera, a name under `/dev` (`video0`, or `v4l/by-id/...` to survive renumbering) |
  | `I2C_GID`, `INPUT_GID`, `VIDEO_GID` | Device groups, filled by `make setup`. Missing: no LCD or joystick (`make doctor` warns) |

- **Plug in later:** the joystick and the camera can be plugged in after the container started (the host's `/dev` is
  mounted at `/run/hostdev`).
- **One owner of the serial port:** `arm_driver`. A second program gets `Device or resource busy`. Use `make park`,
  `make zero`, `make temps`; for a script of your own, `make down` first.
- **The web page** (`make web`) runs here, from the repo mounted read-only at `/opt/braco`.
  See [web/README.md](../web/README.md).
- **Inspect:** `make run CMD='ros2 node list'`, `make logs S=arm_driver`, `make logs S=web`.

## 6. The `sim` container (PC)

- **Starts only with** `make up SIM=1` (`make restart SIM=1` recreates it; `make down` stops it).
- **Runs** the same launch with `sim:=true`: Gazebo (no window), the model, the controllers. Stops when Gazebo ends.
- **ROS domain 77** (the PC), never 42 (the real arm): both publish `/joint_states`, and on one domain they would
  mix. To reach the real arm from `dev`: `ROS_DOMAIN_ID=42 ros2 ...`
- **Its own Gazebo** (`GZ_PARTITION=braco-sim`). A second Gazebo in `dev` on domain 77 fights it: stop `sim`, or
  `export ROS_DOMAIN_ID=78` in that shell.
- **Gazebo window:** `make sim-gui`, or `SIM_GUI=true` in `.env` + `make restart SIM=1`.
- **Web page:** `make web` (runs in `dev`), http://localhost:8000.
- Step by step: [Run the simulation](../ros2_ws/src/braco_description/README.md#run-the-simulation-step-by-step).

## 7. Tools

### PlotJuggler 4 (PC)

```bash
make shell
plotjuggler4
```

- **Live topics:** data source *ROS 2 Topic Subscriber* (same domain and RMW, so the Jetson's topics show up too).
- **Recordings:** `ros2 bag record -s mcap -o /ws/bags/run1 <topics>`, then open the bag. The easiest way to compare
  the simulation with the real arm.
- `Update check failed ... 403` at start: harmless.

### ROS 2 network test

| Command | Where | A PASS proves |
|---|---|---|
| `make net-test` | Each machine | ROS 2 works in the container (C++ talker + Python listener) |
| `make net-talk` + `make net-listen` | One on each machine, then swap | The two machines see each other (same domain and RMW, multicast, firewall) |

- Options: `make net-listen COUNT=20 TIMEOUT=60 TOPIC=/arm_test` (defaults 5 / 30 / `/chatter`; `COUNT=0` runs until
  Ctrl-C).
- A FAIL prints what to check, in order.
- `docker/tests/` is mounted read-only: edit the script on the host, no rebuild.
- A talker started without a terminal keeps running: `make restart` or `make down` stops it.

## 8. How both machines stay the same

| Piece | How |
|---|---|
| OS | Both images start from `ubuntu:26.04` |
| ROS | One `base` stage: the same ROS 2 Lyrical, ros2_control and pymycobot for both |
| ROS apt source | `ros2-apt-source` pinned to 1.3.0 (`ROS_APT_SOURCE_VERSION=latest` to resolve it at build time) |
| Gazebo | Only through `ros-lyrical-ros-gz` (Gazebo Jetty). Never `apt install gz-*` by hand |
| RMW / DDS | `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` from `.env`, the same on both machines. Default Fast DDS; Cyclone installed too |
| Network | `network_mode: host` + `ipc: host`. With a firewall, allow UDP between the machines |
| Images | Built locally, never pulled (`pull_policy: never`). `make up` builds a missing one; `make image` rebuilds it |
| Build flags | Same `colcon-defaults.yaml` (symlink-install, RelWithDebInfo, ccache); the Jetson adds its limits |
| User | User `dev` with your `USER_UID`/`USER_GID`, so `ros2_ws` stays writable |
| Real-time | `rtprio`/`memlock` ulimits in `compose.yml`. Rootless Docker refuses them? Delete that block |
| Python | Ubuntu's Python 3.14; `pymycobot` installed into it. `ruff`/`pre-commit` in their own venv |

## 9. Not verified yet

- **Live ROS 2 topics in PlotJuggler** (recordings work).
- **E-stop:** how it reaches the Jetson is unknown. If it is a GPIO line, uncomment `/dev/gpiochip0` in `compose.yml`.

## 10. If the build fails

| Error | Fix |
|---|---|
| `curl: (60) SSL certificate ... self-signed` | A network proxy re-signs HTTPS: add its CA certificate to the image |
| `could not determine the ros-apt-source version` / 403 | Use the pinned version (not `latest`) |
| `pull access denied ... braco-image/pc` | Your `compose.yml` is old: update it (`pull_policy: never`) |
| `toomanyrequests` pulling `ubuntu:26.04` | Docker Hub limit: `docker login`, or wait |
| "optional packages that could NOT be installed" | Not an error; `make run CMD=seame-smoke-test` lists them |
| `PlotJuggler 4: not installed` | Download failed; the image still works. Rebuild with network, or use `rqt_plot` |
| `Unable to locate package ros-lyrical-...` | ROS repo not reachable, or the package does not exist for your CPU |
| `Cannot uninstall ... installed by debian` | A pip/apt conflict: pin `PYMYCOBOT_VERSION` |

## 11. Jetson host

The Jetson's host is old (JetPack 4.6, Ubuntu 20.04, kernel 4.9), and the containers share its kernel. **Verified:**
the `jetson` image (Ubuntu 26.04 + ROS 2 Lyrical) runs the whole `real` system on it: the arm, the temperatures, the
joystick, the camera and the LCD.

- **After reflashing or updating the host,** check that the userland still runs:

  ```bash
  docker run --rm ubuntu:26.04 bash -c 'uname -r && echo userland-ok'
  ```

  - `Operation not permitted` errors: Docker (or `libseccomp2`) is too old. Update it.
- **Building on the Jetson needs** Docker 24+ and Compose 2.20+. Otherwise build on the PC and copy it:
  `make jetson-image NANO=user@<jetson-ip>`.

## 12. Module 2 (TensorRT)

- Not covered here: the Jetson's CUDA/TensorRT (JetPack 4.6) cannot be used from this Ubuntu 26.04 container.
- Options when Module 2 starts:
  - a separate inference container (dusty-nv `jetson-containers`) talking to ROS 2 over DDS;
  - or inference on the PC (measure the trade-off).

## 13. Handy commands

From inside `docker/`:

```bash
docker compose build --no-cache dev                     # rebuild this machine's image from zero
docker compose down                                     # stop (the cache volumes are kept)
docker compose run --rm dev bash -c 'colcon build && colcon test && colcon test-result --verbose'
docker build --target pc -t braco-image/pc:lyrical docker     # without compose
```
