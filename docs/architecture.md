# braco: ROS 2 architecture

How the braco software is split into ROS 2 packages, which process owns what, and how the pieces talk to each
other on the real arm (Jetson Nano) and in simulation (PC). Deliverable of
[issue #30](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/30).

Why ROS 2 runs in Docker, and on which distro: [ADR-0001](ADR-0001-ros2-in-docker-on-jetson-nano.md).
The containers and the `make` commands: [docker/README.md](../docker/README.md). ROS 2 concepts:
[ROS2/README.md](ROS2/README.md).

---

## 1. Principles

1. **One owner per device.** `arm_driver` is the only process that opens the myCobot 280 serial port
   (`/dev/ttyTHS1`, 1 Mbaud). The joystick, the web page and the zero/park programs reach the arm through its
   topics and services, so each of them can start, stop or crash without touching the port.
2. **The driver never moves the arm by itself.** Starting or stopping the system does not zero, park or power on
   the arm; those are explicit commands (`make zero`, `make park`, `make down PARK=1`).
3. **Safety lives next to the hardware.** Joint limits, the table margin, the motion watchdog, rest mode and
   the control owner are enforced in `arm_driver`, not in its clients: a buggy client cannot bypass them.
4. **Faults are visible.** `arm_driver` is not respawned: if it dies the launch shuts down (an `OnProcessExit`
   handler in `braco.launch.py`), `arm_routine.sh` ends and the container stops. Clients that only read or
   command (`temperature_monitor`, `joystick_teleop`) are respawned.
5. **One client moves the arm at a time.** The page, the joystick, zero/park and the mission take the control
   from `arm_driver` before moving; motion from anyone else is ignored, STOP never is
   ([ADR-0002](ADR-0002-controle-unico.md)).
6. **Same interface for the real arm and the simulation.** Everything above the driver uses `/joint_states`
   and `braco_interfaces`, so the mission and planning layers run unchanged on both (#37).
7. **Logic without ROS where possible.** The serial protocol, the joystick reader, the LCD and the teleop logic
   live in `mycobot_cpp` (`libs/cpp`), plain C++ that is unit-tested without a ROS graph; so do the control
   owner and the auto-rest decision of `braco_driver` (`include/braco_driver/`).

---

## 2. Workspace layout

colcon builds two base paths together: `ros2_ws/src` and `libs` (`make build` passes
`--base-paths src /opt/seame/libs`; flags in [docker/config/colcon-defaults.yaml](../docker/config/colcon-defaults.yaml)).

```
ros2_ws/src/
  braco_interfaces/      msg/            messages shared by every node
  braco_driver/          src/            arm_driver, temperature_monitor        (real arm)
  braco_teleop/          src/ test/      joystick_teleop                        (real arm)
  braco_bringup/         launch/ config/ braco.launch.py (sim:=true|false), braco.yaml,
                         scripts/ src/   arm_routine.sh, zero and park
  braco_description/                     URDF/xacro, meshes, RViz          (placeholder, #25)
  braco_moveit_config/                   SRDF, kinematics, limits          (placeholder, #40)
  braco_mission/                         pick-and-place, pen drawing       (placeholder, #43)
libs/cpp/                                mycobot_cpp: serial protocol, joystick, LCD, teleop logic (no ROS)
```

| Package | Kind | Role | Status |
|---|---|---|---|
| [`mycobot_cpp`](../libs/cpp) | C++ library | myCobot 280 serial driver, joint limits and poses (`mycobot.hpp`), joystick, LCD1602, teleop logic. Also builds the stand-alone tools (no ROS) | Done |
| [`braco_interfaces`](../ros2_ws/src/braco_interfaces/README.md) | Interfaces | `ArmCommand`, `JointTarget`, `ServoTemperatures`, `TakeControl` | Done |
| [`braco_driver`](../ros2_ws/src/braco_driver/README.md) | Nodes | `arm_driver` (owns the port and the control), `temperature_monitor` (LCD + `/diagnostics` + rest on overheat) | Done |
| [`braco_teleop`](../ros2_ws/src/braco_teleop/README.md) | Node | `joystick_teleop`: jogs the arm through `arm_driver` | Done |
| [`braco_bringup`](../ros2_ws/src/braco_bringup/README.md) | Launch + config | `braco.launch.py`, `braco.yaml`, the container entrypoint, `zero` / `park` | Done (`sim:=true` reserved) |
| [`braco_description`](../ros2_ws/src/braco_description/README.md) | Model | URDF of the myCobot 280 | Placeholder, #25 |
| [`braco_moveit_config`](../ros2_ws/src/braco_moveit_config/README.md) | Config | MoveIt 2 for the arm | Placeholder, #40 |
| [`braco_mission`](../ros2_ws/src/braco_mission/README.md) | Nodes | Mission state machine, MoveIt 2 client | Placeholder, #43 |

Every package has its own README with its topics, parameters and how to run it by hand.

### Dependencies between packages

```
                 mycobot_cpp (libs/cpp)        braco_interfaces
                    │      │      │              │     │     │
                    ▼      ▼      ▼              │     │     │
           braco_driver  braco_teleop  ◄─────────┘     │     │
                 ▲            ▲                        │     │
                 │ exec       │ exec                   │     │
                 └──── braco_bringup ◄─────────────────┘     │
                       (zero/park link mycobot_cpp too)      │
                                                             │
 braco_description ──► braco_moveit_config ──► braco_mission ┘ (planned)
```

`braco_bringup` lists every package the **real** system needs as an `exec_depend`. The Jetson builds only
`--packages-up-to braco_bringup`, so a simulation-only package (Gazebo, MoveIt) must never become a dependency of
`braco_bringup`, or the Nano starts building it (see `WS_PACKAGES` in [mk/config.mk](../mk/config.mk)).

---

## 3. Deployment

```
 PC  (COMPOSE_PROFILES=pc)                                Jetson Nano  (COMPOSE_PROFILES=jetson)
 ┌──────────────────────────────────────┐                 ┌───────────────────────────────────────────┐
 │ dev  shell, colcon build/test,       │                 │ real  arm_routine.sh                      │
 │      RViz2, PlotJuggler, rqt         │    DDS          │   ros2 launch braco_bringup               │
 │                                      │   domain 42     │       braco.launch.py sim:=false          │
 │ sim  arm_routine.sh                  │ ◄─────────────► │     arm_driver ── /dev/ttyTHS1 ── arm     │
 │   braco.launch.py sim:=true          │   Fast DDS,     │     temperature_monitor ── /dev/i2c-0 LCD │
 │     (Gazebo arm, #34)                │   host network  │     joystick_teleop ── /dev/input/js0     │
 │   web page :8000 (ARM_SOURCE=gazebo) │                 │   web page :8000 (ARM_SOURCE=ros)         │
 │   MoveIt 2, mission (planned)        │                 │                                           │
 └──────────────────────────────────────┘                 │ dev  throw-away (run --rm): build, zero,  │
                                                          │      park                                 │
                                                          └───────────────────────────────────────────┘
```

- Both machines run the **same image family** (`braco-image/pc`, `braco-image/jetson`, ROS 2 Lyrical on
  Ubuntu 26.04) and the same workspace, bind-mounted at `/ws`. `dev` builds; the system container (`real` or
  `sim`) runs what `dev` built.
- **Heavy work stays on the PC.** `move_group` needs 300 to 600 MB and a full core while planning, so MoveIt 2
  and the mission run on the PC and reach the arm on the Jetson through DDS. The Jetson runs only the nodes that
  need its devices.
- `make up` starts the system container; `arm_routine.sh` runs the launch and the web page, stops both on
  `docker stop`, and ends the container when the launch ends.

---

## 4. The ROS 2 graph (real arm, `sim:=false`)

```
                                 /joint_states (20 Hz)
                       ┌─────────────────────────────────────────┬──────────────────┐
                       │                                         ▼                  ▼
  ┌──────────────┐     │      /arm/command (ArmCommand)   ┌────────────────┐  ┌──────────┐
  │  arm_driver  │─────┘ ◄─────────────────────────────── │ joystick_teleop│  │ web page │
  │              │       ◄─────────────────────────────── └────────────────┘  │ (rclpy)  │
  │ serial port  │       /arm/led (ColorRGBA)                    │            └──────────┘
  │ control owner│ ◄─────────────────────────────────────────────┘                │
  │ watchdog     │       /arm/take_control  (joystick, page, zero/park)           │
  │ limits       │ ◄──────────────────────────────────────────────────────────────┤
  │ rest mode    │       /arm/power_on, /arm/set_gripper  (joystick, page)        │
  │              │ ◄──────────────────────────────────────────────────────────────┘
  └──────────────┘       /arm/release_servos, /arm/set_read_only  (page, zero/park)
     │   │   │
     │   │   └── /arm/control_owner (String, latched) ───────────────► joystick, page
     │   └────── /arm/read_only (Bool, latched) ─────────────────────► page
     │
     └── /arm/servo_temperatures (every 2 s) ──► temperature_monitor ──► LCD1602
                                                   │        │
                                                   │        └── /diagnostics ──► page (levels, ROS health)
                                                   └── /arm/set_read_only true (a servo at 65 °C)
```

| Name | Type | Publisher / server | Users |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` (rad, `joint1..joint6`) | `arm_driver` | teleop, page, zero/park, MoveIt (planned) |
| `/arm/command` | `braco_interfaces/ArmCommand` (with `source`) | teleop, page, zero/park | `arm_driver` (the owner's motion, anyone's STOP) |
| `/arm/led` | `std_msgs/ColorRGBA` | teleop, page | `arm_driver` |
| `/arm/servo_temperatures` | `braco_interfaces/ServoTemperatures` | `arm_driver` | `temperature_monitor`, page |
| `/arm/read_only` | `std_msgs/Bool`, latched | `arm_driver` | page |
| `/arm/control_owner` | `std_msgs/String`, latched (`""` = free) | `arm_driver` | teleop, page |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `temperature_monitor` | page |
| `/arm/take_control` | `braco_interfaces/TakeControl` | `arm_driver` | teleop, page, zero/park |
| `/arm/power_on`, `/arm/release_servos` | `std_srvs/Trigger` | `arm_driver` | teleop, page, zero/park |
| `/arm/set_gripper`, `/arm/set_read_only` | `std_srvs/SetBool` | `arm_driver` | teleop, page, zero/park, `temperature_monitor` |

### Commanding the arm

- `ArmCommand` carries `STOP`, `JOINTS` (one target per joint) or `ALL_ANGLES` (six angles, one speed). The
  newest command replaces any pending one ("latest wins"), so a slow serial link never builds a queue.
- Clients stream short moves a few degrees ahead of the angle they read on `/joint_states`, and only while that
  feedback is fresh. If the client dies, the arm coasts very little.
- **Watchdog:** when motion commands stop for `command_timeout_ms` (300 ms), `arm_driver` stops the arm.
- **Limits:** angles are clamped to `JOINT_LIMITS` (`mycobot.hpp`); poses that put the wrist or tool below the
  table margin are refused.
- **Rest mode** (`/arm/set_read_only true`): motion, gripper and power on are refused; angles and temperatures
  keep being published. `temperature_monitor` enters it by itself when a servo reaches 65 °C, once per hot
  episode; leaving it is manual (`make resume`). See [arm-rest-mode.md](arm-rest-mode.md) and
  [ADR-0003](ADR-0003-repouso-por-temperatura.md).

### Who moves the arm

One control owner at a time, kept by `arm_driver` ([ADR-0002](ADR-0002-controle-unico.md)):

| | |
|---|---|
| Owner | `web`, `joystick`, `script` (zero/park) or `mission`, the `source` of its `ArmCommand`s; `""` = free |
| Take | `/arm/take_control`: succeeds when free, already ours, or with `force` (drops the old owner's pending motion and stops the arm) |
| Motion | obeyed only from the owner; ignored (and logged) from anyone else |
| STOP | always obeyed, from anyone |
| Lose | `release`, forced away, or `control_idle_s` (5 s) without motion |

| Client | Takes the control | Gives it back |
|---|---|---|
| Web page | with the page lease ("Take control"), renewed every second while the lease lasts; **409** while the joystick or zero/park has it | when the lease ends |
| Joystick | first stick / X / A / B with the control free; from another owner only by **holding button 8 for 1 s** | 5 s after the last move |
| zero / park | forced, before moving | at the end, also on failure |

---

## 5. Launch: `braco.launch.py`

One launch file for the whole system, in `braco_bringup`:

```bash
ros2 launch braco_bringup braco.launch.py              # sim:=false, the real arm (default)
ros2 launch braco_bringup braco.launch.py sim:=true    # the simulated arm
ros2 launch braco_bringup braco.launch.py --show-args
```

| Argument | Default | Used by |
|---|---|---|
| `sim` | `false` | `false`: `arm_driver`, `temperature_monitor`, `joystick_teleop`. `true`: the Gazebo arm (#34); until then it starts nothing and logs why |
| `port`, `baud` | `/dev/ttyTHS1`, `1000000` | `arm_driver` |
| `lcd_bus` | `/dev/i2c-0` | `temperature_monitor` |
| `device` | `/dev/input/js0` | `joystick_teleop` |

The **wiring** (devices) comes from the launch arguments, filled by `docker/compose.yml` from `docker/.env`; the
**tuning** (rates, watchdog, timeouts) comes from [`config/braco.yaml`](../ros2_ws/src/braco_bringup/config/braco.yaml).

---

## 6. Simulation and the planned upper layers

```
 braco_mission ──► MoveIt 2 (braco_moveit_config) ──► trajectory ──► real arm: arm_driver
   (#43)                (#40)                                     └─► sim:  Gazebo + gz_ros2_control (#34)
                          ▲
                          └── model: braco_description (#25)
```

- `braco_description` holds the URDF; its joint limits must match `JOINT_LIMITS` in `mycobot.hpp`, and its joint
  names are the ones on `/joint_states` (`joint1..joint6`) on both the real arm and the simulation.
- `sim:=true` will start Gazebo with that model, publishing the same `/joint_states`.
- The mission talks only to MoveIt 2, never to `arm_driver` or the serial port, so the same code runs on both.
  How MoveIt's trajectories reach `arm_driver` on the real arm is decided with #37 and #40.

---

## 7. Build and test

```bash
make build    # colcon build of ros2_ws/src and libs, in the dev container
make test     # colcon test: gtest (teleop logic) and the ament lint checks (PC only)
make restart  # run what was just built
```

- PC: everything, `RelWithDebInfo`, with tests.
- Jetson: only `--packages-up-to braco_bringup`, `Release`, `BUILD_TESTING=OFF`, one package at a time (4 GB RAM).
- CI (`make ci-ros`) builds and tests the workspace in the `braco-image/ci` image on every pull request.

---

## 8. Adding a package

1. Name it `braco_<role>` and put it in `ros2_ws/src`. Code with no ROS dependency goes in `libs/cpp`.
2. `package.xml` format 3, Apache-2.0, maintainer set; `ament_lint_auto` as a test dependency.
3. A `README.md` with what it does, its topics/services/parameters and how to run it by hand.
4. Needed on the real arm? Add it as an `exec_depend` of `braco_bringup` and start it from `braco.launch.py`
   under `sim:=false`. Simulation or planning only? Keep it out of `braco_bringup`'s dependencies.
5. New message or service? Put it in `braco_interfaces`, never in the package that uses it.
6. Update the tables of this document.
