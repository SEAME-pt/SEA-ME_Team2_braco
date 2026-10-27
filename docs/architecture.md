# braco architecture

The official description of the braco software **as it is implemented today**: how it is split into ROS 2
packages, which process owns what, how the pieces talk to each other on the real arm (Jetson Nano) and in
simulation (PC), where each file lives, how the system is started and what it costs on the Jetson. Started as
[issue #30](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/30).

This is the Markdown version of [architecture.html](architecture.html), which has the same sections with drawn
figures. **Both files are kept in sync**: a change to one goes to the other in the same PR.

> **This document follows the code.** Every pull request that changes a node, a topic, a service, a parameter, a
> package, a folder, a `make` command or the deployment updates the matching section here in the same PR (the map is
> in [section 13](#13-changing-the-architecture)), and moves the item in the
> [status](#12-status-against-the-target-architecture) between *pending*, *differs* and *done*. If the code and this
> document disagree, the code wins and this document is the bug.
>
> Last checked against the code: **2026-10-23** · [ADR-0001](ADR-0001-ros2-in-docker-on-jetson-nano.md) ·
> [ADR-0002](ADR-0002-unique-control.md) · [ADR-0003](ADR-0003-rest-per-temp.md)

**Contents**

1. [Principles](#1-principles)
2. [The system at a glance](#2-the-system-at-a-glance)
3. [Scenarios](#3-scenarios)
4. [ROS 2 graph and control](#4-ros-2-graph-and-control-real-arm-simfalse)
5. [Packages and folders](#5-packages-and-folders)
6. [Deployment](#6-deployment)
7. [Launch and parameters](#7-launch-and-parameters)
8. [Make commands](#8-make-commands)
9. [CPU and memory on the Jetson](#9-cpu-and-memory-on-the-jetson)
10. [Simulation and the planned upper layers](#10-simulation-and-the-planned-upper-layers)
11. [Build and test](#11-build-and-test)
12. [Status against the target architecture](#12-status-against-the-target-architecture)
13. [Changing the architecture](#13-changing-the-architecture)

---

## 1. Principles

- **One owner per device.** `arm_driver` is the only process that opens the myCobot 280 serial port
  (`/dev/ttyTHS1`, 1 Mbaud). Everyone else reaches the arm through its topics and services, so each client can
  start, stop or crash without touching the port. The page has no direct serial mode. Likewise `camera` is the
  only process that opens the eye-in-hand USB camera; the page (and later vision) subscribes to its frames.
- **The driver never moves the arm by itself.** Starting or stopping the system does not zero, park or power on the
  arm; those are explicit commands (`make zero`, `make park`, `make down PARK=1`). The one automatic move is the
  park on overheat, run by `temperature_monitor` ([ADR-0003](ADR-0003-rest-per-temp.md)).
- **Safety lives next to the hardware.** Joint limits, the table margin, the watchdog, rest mode and the control
  owner are enforced in `arm_driver`, not in its clients: a buggy client cannot bypass them.
- **Faults are visible.** `arm_driver` is not respawned: if it dies the launch shuts down, `arm_routine.sh` ends and
  the container stops. `temperature_monitor`, `joystick_teleop` and `camera` are respawned; the arm never depends
  on the camera, whose node waits for it when it is unplugged.
- **One client moves the arm at a time.** The page, the joystick, zero/park and the mission take the control from
  `arm_driver` before moving; motion from anyone else is ignored, STOP never is
  ([ADR-0002](ADR-0002-unique-control.md)).
- **One source for each fact.** The thermal level is computed once, by `temperature_monitor`, and published on
  `/diagnostics`; the page shows it and does not recompute it. The control owner is kept once, by `arm_driver`.
- **Same interface for real and simulated.** Everything above the driver uses `/joint_states` and
  `braco_interfaces`, so the mission and planning layers run unchanged on both (#37).
- **Logic without ROS where possible.** Serial protocol, joystick, LCD and teleop logic live in `mycobot_cpp`
  (`ros2_ws/src/mycobot_cpp`); the control owner and the auto-rest decision are plain headers in `braco_driver/include`. All
  unit-tested without a ROS graph.

---

## 2. The system at a glance

```
 ┌────────────────────────────────────────────────────────────────────┐
 │ arm_driver                 the only process that opens the UART    │      UART 1 Mbaud     ┌─────────────┐
 │                                                                    │ ◄───────────────────► │ myCobot 280 │
 │ GATES OF A COMMAND        PUBLISHES                RECEIVES        │                       │ /dev/ttyTHS1│
 │ 1 STOP always passes      /joint_states            /arm/command    │                       └─────────────┘
 │ 2 rest mode               /arm/servo_temperatures  /arm/led        │
 │ 3 control owner           /arm/read_only           SERVICES        │
 │ 4 watchdog 300 ms         /arm/servo_voltages      power_on        │
 │                           /arm/control_owner       set_gripper     │
 │                                                    set_read_only   │
 │                                                    release_servos  │
 │                                                    take_control    │
 └──────────────────────────────────┬─────────────────────────────────┘
                                    │                                      ┌───────────────────────┐
                                    │                                      │ camera                │  V4L2   ┌──────────────┐
                                    │                                      │ owns the USB camera   │ ◄────── │ USB camera   │
                                    │                                      │ SENDS /camera/        │         │ /dev/video0  │
                                    │                                      │  image_raw/compressed │         └──────────────┘
                                    │                                      └───────────┬───────────┘
 ═══════════════════════ ROS 2 · DDS · domain 42 ════════════════════════════════════════════════════════
      │                       │                        │                          ┊
 ┌────┴──────────────┐ ┌──────┴────────────────┐ ┌─────┴───────────────────┐ ┌┄┄┄┴┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┐
 │ braco_web         │ │ joystick_teleop       │ │ temperature_monitor     │ ┊ braco_mission (planned)   ┊
 │ rclpy in uvicorn  │ │ reopens js0 if        │ │ the thermal authority   │ ┊ #43, same contract        ┊
 │ SENDS             │ │ unplugged             │ │ SENDS                   │ ┊ SENDS                     ┊
 │  /arm/command,led │ │ SENDS                 │ │  /diagnostics           │ ┊  srv take_control         ┊
 │  srv take_control │ │  /arm/command, /led   │ │  srv set_read_only      │ ┊  FollowJointTrajectory    ┊
 │  srv power_on, …  │ │  srv take_control     │ │   at 65 °C (after park) │ ┊ READS                     ┊
 │ READS             │ │  srv set_gripper,     │ │ READS                   │ ┊  /joint_states            ┊
 │  /joint_states    │ │      power_on         │ │  /arm/servo_temperatures│ ┊  /diagnostics             ┊
 │  /control_owner   │ │ READS                 │ │                         │ └┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┘
 │  /diagnostics     │ │  /joint_states        │ │                         │
 │  /camera/…/compr. │ │  /control_owner       │ │                         │
 └────┬──────────────┘ └──────┬────────────────┘ └─────┬───────────────────┘
      │ HTTP · WS · MJPEG     │                        │ I2C
      │ :8000                 │                        │
   browsers             joystick /dev/input/js0    LCD1602 /dev/i2c-0
```

The clients do not know each other: each talks only to the graph, and the safety rules sit in one place, the gates
of `arm_driver`. A motion command passes them in order: STOP is always obeyed; in rest mode motion is refused;
motion from anyone but the control owner is ignored (and logged); and when motion commands stop for 300 ms the
watchdog stops the arm.

---

## 3. Scenarios

**An operator moves the arm from the page**

1. The user takes the control on the page. With `ARM_SOURCE=ros` the page lease calls `take_control("web")`.
2. `arm_driver` publishes `/arm/control_owner = web`. The joystick sees it and dims its LED.
3. The page streams `ALL_ANGLES` a few degrees ahead of the real angle, only while `/joint_states` is fresh.
4. If the page dies, the watchdog stops the arm within 300 ms. After 5 s without motion the owner goes back to free.

**Someone picks up the joystick**

1. With the control free, the first stick / X / A / B move calls `take_control("joystick")`.
2. If the page has the control, the joystick forces it by **holding button 8 for 1 s**: whoever stands next to the
   arm has priority.
3. `arm_driver` changes the owner, drops the page's pending motion and stops the arm. The page shows "joystick in
   control"; taking the control from the page fails with **409** until it is free again.
4. The joystick calls `power_on` on connect only once it has the control.

**A servo gets hot**

1. `arm_driver` publishes `/arm/servo_temperatures` (and `/arm/servo_voltages`) every 2 s, also in rest mode.
2. `temperature_monitor` classifies normal (up to 54 °C), warm (55–64 °C) or hot (65 °C or more), publishes
   `/diagnostics` and shows it on the LCD.
3. At hot, once per hot episode, it runs `park` (`park_on_hot`): the arm folds, the joints are released so the
   servos cool, and the arm ends in rest mode, also when park fails. The episode ends only when every joint is
   below 60 °C (hysteresis), so a reading that jitters around 65 °C does not park the arm again. With `park_on_hot: false` it only calls
   `set_read_only(true)`. In rest mode `arm_driver` drops the pending motion and refuses motion, gripper and power
   on, but keeps reading.
4. The page shows the level from `/diagnostics`. When the arm has cooled down, someone runs `make resume`
   ([arm-rest-mode.md](arm-rest-mode.md)).

---

## 4. ROS 2 graph and control (real arm, `sim:=false`)

| Name | Type | Publisher / server | Users |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` (rad, `joint1..joint6`), 20 Hz | `arm_driver` | teleop, page, zero/park, MoveIt (planned) |
| `/arm/command` | `braco_interfaces/ArmCommand` (with `source`) | teleop, page, zero/park | `arm_driver` (the owner's motion, anyone's STOP) |
| `/arm/led` | `std_msgs/ColorRGBA` | teleop, page | `arm_driver` |
| `/arm/servo_temperatures` | `braco_interfaces/ServoTemperatures`, every 2 s | `arm_driver` | `temperature_monitor`, page |
| `/arm/servo_voltages` | `braco_interfaces/ServoVoltages` (V per joint, `valid`), every 2 s | `arm_driver` | page (panel and history chart) |
| `/arm/read_only` | `std_msgs/Bool`, latched | `arm_driver` | page |
| `/arm/control_owner` | `std_msgs/String`, latched (`""` = free) | `arm_driver` | teleop, page |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `temperature_monitor` | page (levels, ROS health) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` (JPEG, up to 15 fps, sensor-data QoS) | `camera` | page (MJPEG on Arm and Control), vision (planned) |
| `/arm/take_control` | `braco_interfaces/TakeControl` | `arm_driver` | teleop, page, zero/park |
| `/arm/power_on`, `/arm/release_servos` | `std_srvs/Trigger` | `arm_driver` | teleop, page, zero/park |
| `/arm/set_gripper`, `/arm/set_read_only` | `std_srvs/SetBool` | `arm_driver` | teleop, page, zero/park, `temperature_monitor` |

### Commanding the arm

- **Latest wins.** `ArmCommand` carries `STOP`, `JOINTS` (one target per joint) or `ALL_ANGLES` (six angles, one
  speed). The newest command replaces any pending one, so a slow serial link never builds a queue. Clients stream
  short moves a few degrees ahead of the angle they read on `/joint_states`, and only while that feedback is fresh:
  if the client dies, the arm coasts very little.
- **Watchdog and limits.** When motion commands stop for `command_timeout_ms` (300 ms), `arm_driver` stops the arm.
  Angles are clamped to `JOINT_LIMITS` (`mycobot.hpp`); poses that put the wrist or tool below the table margin are
  refused.
- **Rest mode.** `/arm/set_read_only true`: motion, gripper and power on are refused; angles and temperatures keep
  being published. `temperature_monitor` enters it by itself when a servo reaches 65 °C, once per hot episode
  (which ends below 60 °C), after parking the arm (`park_on_hot`); leaving it is manual (`make resume`). See [arm-rest-mode.md](arm-rest-mode.md) and
  [ADR-0003](ADR-0003-rest-per-temp.md).

### Who moves the arm ([ADR-0002](ADR-0002-unique-control.md))

| | |
|---|---|
| **Owner** | `web`, `joystick`, `script` (zero/park) or `mission`, the `source` of its `ArmCommand`s; `""` = free |
| **Take** | `/arm/take_control`: succeeds when free, already ours, or with `force` (drops the old owner's pending motion and stops the arm) |
| **Motion** | obeyed only from the owner; ignored (and logged) from anyone else |
| **STOP** | always obeyed, from anyone |
| **Lose** | `release`, forced away, or `control_idle_s` (5 s) without motion |

| Client | Takes the control | Gives it back |
|---|---|---|
| Web page | with the page lease ("Take control"), renewed every second while the lease lasts; **409** while the joystick or zero/park has it | when the lease ends |
| Joystick | first stick / X / A / B with the control free; from another owner only by **holding button 8 for 1 s** | 5 s after the last move |
| zero / park | forced, before moving | at the end, also on failure |

---

## 5. Packages and folders

colcon builds one workspace, `ros2_ws/src` (mounted at `/ws` in every container; `make build` passes
`--base-paths src`; flags in [colcon-defaults.yaml](../docker/config/colcon-defaults.yaml)). Every
package has its own README with its topics, parameters and how to run it by hand.

| Package | Kind | Role | Status |
|---|---|---|---|
| [`mycobot_cpp`](../ros2_ws/src/mycobot_cpp/README.md) | C++ library | myCobot 280 serial driver, joint limits and poses (`mycobot.hpp`), joystick, LCD1602, teleop logic; also the stand-alone tools (no ROS) | done |
| [`braco_interfaces`](../ros2_ws/src/braco_interfaces/README.md) | Interfaces | `ArmCommand`, `JointTarget`, `ServoTemperatures`, `ServoVoltages`, `TakeControl` | done |
| [`braco_driver`](../ros2_ws/src/braco_driver/README.md) | Nodes | `arm_driver` (owns the port and the control), `temperature_monitor` (LCD, `/diagnostics`, rest on overheat) | done |
| [`braco_teleop`](../ros2_ws/src/braco_teleop/README.md) | Node | `joystick_teleop`: jogs the arm through `arm_driver` | done |
| [`braco_camera`](../ros2_ws/src/braco_camera/README.md) | Node (Python) | `camera`: the eye-in-hand USB camera as JPEG frames | done |
| [`braco_bringup`](../ros2_ws/src/braco_bringup/README.md) | Launch + config | `braco.launch.py`, `braco.yaml`, the container entrypoint, `zero` / `park` | done, `sim:=true` reserved |
| [`braco_description`](../ros2_ws/src/braco_description/README.md) | Model + tools | URDF of the myCobot 280 with G-base, camera flange and gripper; `gazebo.launch.py` (Gazebo viewer); `sim_to_real.py` (checks against the real arm) | done (#25) |
| [`braco_moveit_config`](../ros2_ws/src/braco_moveit_config/README.md) | Config | MoveIt 2 for the arm | placeholder #40 |
| [`braco_mission`](../ros2_ws/src/braco_mission/README.md) | Nodes | Mission state machine, MoveIt 2 client | placeholder #43 |

### Repository layout

```
SEA-ME_Team2_braco/
├── Makefile                  # the one entry point
├── mk/                       # config system arm dev setup net ci
├── ros2_ws/src/              # ONE colcon workspace: everything that gets built
│   ├── mycobot_cpp/          # no ROS: serial protocol, joystick, LCD, teleop logic
│   │   ├── includes/  mycobot.hpp  joystick.hpp  lcd_display.hpp
│   │   │              display.hpp  teleop_logic.hpp
│   │   └── srcs/      library + stand-alone tools (zero, park, pid_tool, temps_lcd…)
│   ├── braco_interfaces/
│   │   ├── msg/  ArmCommand  JointTarget  ServoTemperatures  ServoVoltages
│   │   └── srv/  TakeControl
│   ├── braco_driver/
│   │   ├── src/  arm_driver.cpp  temperature_monitor.cpp
│   │   ├── include/braco_driver/  control.hpp  auto_rest.hpp
│   │   └── test/ test_control.cpp
│   ├── braco_teleop/
│   │   ├── src/  joystick_teleop.cpp
│   │   └── test/ test_teleop_logic.cpp
│   ├── braco_camera/
│   │   └── scripts/ camera.py         # USB camera -> /camera/image_raw/compressed
│   ├── braco_bringup/
│   │   ├── launch/  braco.launch.py     # sim:=true|false
│   │   ├── config/  braco.yaml          # tuning of every node
│   │   ├── scripts/ arm_routine.sh      # container entrypoint
│   │   └── src/     zero.cpp  park.cpp  arm_moves.cpp
│   ├── braco_description/
│   │   ├── urdf/    braco.urdf.xacro  mycobot_280  adaptive_gripper  braco.gazebo
│   │   ├── meshes/  config/  launch/ gazebo.launch.py
│   │   └── scripts/ braco_arm.py  park.py  move_arm_example.py  sim_to_real.py
│   ├── braco_moveit_config/    # placeholder #40
│   └── braco_mission/          # placeholder #43
├── scripts/  bench.sh        # bench test of control and auto-rest (make bench)
├── web/                      # the braco page, a ROS 2 client
│   ├── server/   app.py  arm.py  camera.py  ros.py  session.py …
│   ├── static/
│   └── tests/
├── docker/
│   ├── Dockerfile  compose.yml  .env.example
│   ├── compose/  nvidia.yml  dri.yml
│   ├── config/  scripts/  tests/
│   └── systemd/  braco.service
├── docs/
│   ├── architecture.html  architecture.md   # this document, two formats
│   ├── ADR-000N-*.md  safety.md  protocol_commands.md
│   └── arm-rest-mode.md  dev-setup.md  ROS2/  inventory/
├── .github/  .githooks/       # CI workflow, PR template, commit-msg and pre-push hooks
└── .clang-format  .ruff.toml  requirements-dev.txt   # formatters (make format), host dev venv
```

The root holds only the six folders above and the `Makefile`: everything colcon builds is in `ros2_ws/src`, everything
else has one generic folder.

### Naming rules

- **Package = `braco_` + role.** One word after the prefix: `driver`, `teleop`, `mission`, `bringup`. The prefix
  avoids collisions with ROS packages and marks what belongs to the project.
- **One name per thing.** File, executable and node share the name: `arm_driver.cpp` runs as `arm_driver`. Topics
  and services of the arm live under `/arm/*`.
- **Outside ROS, generic folders:** `web`, `docker`, `docs`, `mk`, `scripts`. Anyone opening the repo understands them
  without a legend.
- **`mycobot_cpp` keeps its name.** The library is the robot's protocol, not the product. It is a colcon package of
  the workspace like the others, but depends on nothing from ROS.
- **Infrastructure carries the braco brand:** compose project `braco`, images `braco-image/pc` and
  `braco-image/jetson`, systemd unit `braco.service`.

### Dependencies between packages

```
       mycobot_cpp            braco_interfaces
         │      ╲            ╱       │
         ▼       ╲          ╱        ▼
   braco_driver   ╲────────╱   braco_teleop          braco_description ┄► braco_moveit_config ┄► braco_mission
         ▲          ╲    ╱           ▲                       (planned · simulation and planning only ·
         │ exec_depend ╲╱ exec_depend │                        never a dependency of braco_bringup)
         └──────── braco_bringup ─────┘
                         │ exec_depend
                         ▼
                    braco_camera  (Python, depends only on ROS packages)
```

`braco_bringup` lists every package the **real** system needs as an `exec_depend`. The Jetson builds only
`--packages-up-to braco_bringup`, so a simulation-only package (Gazebo, MoveIt) must never become a dependency of
`braco_bringup`, or the Nano starts building it (`WS_PACKAGES` in [mk/config.mk](../mk/config.mk)).

---

## 6. Deployment

```
 PC  (COMPOSE_PROFILES=pc · braco-image/pc)            Jetson Nano  (COMPOSE_PROFILES=jetson)
 ┌──────────────────────────────────────┐              ┌───────────────────────────────────────────┐
 │ dev  stays up                        │              │ real  arm_routine.sh · braco.service      │
 │   shell · colcon build / test        │              │   braco.launch.py sim:=false              │
 │   RViz2 · PlotJuggler · rqt          │    DDS       │     arm_driver ──── /dev/ttyTHS1 ── arm   │
 │                                      │  domain 42   │     temperature_monitor ── /dev/i2c-0 LCD │
 │ sim  arm_routine.sh                  │ ◄──────────► │     joystick_teleop ── /dev/input/js0     │
 │                                      │              │     camera ── /dev/video0 USB camera      │
 │   braco.launch.py sim:=true          │  Fast DDS,   │   web page :8000  ARM_SOURCE=ros          │
 │     Gazebo arm (#34, reserved)       │  host network│   launch ends → container stops           │
 │   web page :8000  ARM_SOURCE=gazebo  │              │                                           │
 │                                      │              │ dev  throw-away, run --rm                 │
 │ MoveIt 2 · braco_mission (planned)   │              │   build · zero · park                     │
 └──────────────────────────────────────┘              └───────────────────────────────────────────┘
```

Both machines run the same image family (ROS 2 Lyrical on Ubuntu 26.04) and the same workspace, bind-mounted at
`/ws`. `dev` builds; the system container (`real` or `sim`) runs what `dev` built.

- **Heavy work stays on the PC.** `move_group` needs 300–600 MB and a full core while planning, so MoveIt 2 and the
  mission run on the PC and reach the arm on the Jetson through DDS. The Jetson runs only the nodes that need its
  devices.
- `make up` starts the system container; `arm_routine.sh` runs the launch and the web page side by side, stops both
  on `docker stop`, and ends the container when the launch ends.
- The page's arm source: `ARM_SOURCE=ros` (real), `ARM_SOURCE=gazebo` (sim, PC only) or `ARM_MOCK=1` (`make web`,
  no arm, no ROS, no Docker). There is no mode that opens the serial port.
- Gazebo's own transport stays on the machine: `compose.yml` sets `GZ_IP=127.0.0.1` (override in `.env`). With the
  host network it otherwise picks an interface by itself and, behind a VPN or a firewall, its processes never find
  each other. ROS 2 (DDS) is not affected. Run a simulation on its own `ROS_DOMAIN_ID` (e.g. 77) when the real arm
  is on the network: both publish `/joint_states`.

---

## 7. Launch and parameters

```bash
ros2 launch braco_bringup braco.launch.py              # sim:=false, the real arm (default)
ros2 launch braco_bringup braco.launch.py sim:=true    # the simulated arm
ros2 launch braco_bringup braco.launch.py --show-args
```

The **wiring** (devices) comes from the launch arguments, filled by `docker/compose.yml` from `docker/.env`; the
**tuning** comes from [config/braco.yaml](../ros2_ws/src/braco_bringup/config/braco.yaml).

| Argument | Default | Used by |
|---|---|---|
| `sim` | `false` | `false`: `arm_driver`, `temperature_monitor`, `joystick_teleop`, `camera`. `true`: the Gazebo arm (#34); until then it starts nothing and logs why |
| `port`, `baud` | `/dev/ttyTHS1`, `1000000` | `arm_driver` |
| `lcd_bus` | `/dev/i2c-0` | `temperature_monitor` |
| `device` | `/dev/input/js0` | `joystick_teleop` |
| `camera` | `/dev/video0` | `camera`; the `real` container passes `/run/hostdev/$CAMERA_DEVICE` (host `/dev` mounted there, so the camera can be plugged in later) |

| Node | Parameter | Value | Meaning |
|---|---|---|---|
| `arm_driver` | `state_rate_hz` | 20.0 | `/joint_states` rate |
| | `temp_period_s` | 2.0 | servo temperature and voltage read period |
| | `command_timeout_ms` | 300 | watchdog |
| | `gripper_speed` | 50 | 1..100 |
| | `read_only` | false | start in rest mode |
| | `control_idle_s` | 5.0 | the owner loses the control after this long without motion |
| `temperature_monitor` | `stale_timeout_s` | 5.0 | no temperatures for this long: "no feedback" on the LCD |
| | `auto_rest_on_hot` | true | a servo at 65 °C puts the arm to rest, once per episode |
| | `park_on_hot` | true | ... by running park (fold, release the joints); false: rest mode only |
| `joystick_teleop` | `state_timeout_ms` | 150 | older `/joint_states`: no jogging |
| | `debug` | false | log every joystick event |
| | `force_button`, `force_hold_s` | 8, 1.0 | hold to take the control from another owner |
| `camera` | `width`, `height` | 640, 480 | asked of the camera |
| | `fps` | 15.0 | the most it publishes |
| | `jpeg_quality` | 80 | only for cameras without MJPG (MJPG frames are passed on as they come) |
| | `reopen_s` | 2.0 | without a camera: try again this often |

---

## 8. Make commands

> **One command turns the system on: `make up`.** On the Jetson it starts the real arm (driver, temperature,
> joystick, page); on the PC the simulation and the page. It can be repeated: each step (`.env`, checks, image,
> build) is skipped when it is already done, so `make up` after a code change only rebuilds what changed. No step
> moves the arm. `make` alone lists every command; the machine (`ROLE=pc|jetson`) comes from `COMPOSE_PROFILES` in
> `docker/.env` or is detected (arm64 + Tegra = Jetson).

**System** · [mk/system.mk](../mk/system.mk)

| Command | What it does |
|---|---|
| `make up` | Turn the system on, wait for the page and print its URL |
| `make down` | Turn it off, without moving the arm (`PARK=1` parks the arm first) |
| `make restart` | Restart the system container, then like `up` |
| `make status` | Containers, CPU/RAM per container, the arm as the page sees it; a few seconds of `tegrastats` on the Jetson |
| `make logs` | Follow the system's log (`S=web`, `S=arm_driver`, `S=temperature_monitor`, `S=joystick_teleop`, `S=camera`) |

**Arm** · [mk/arm.mk](../mk/arm.mk) · Jetson

| Command | What it does |
|---|---|
| `make zero` | Leave rest mode and move to the zero pose |
| `make park` | Fold the arm, release the joints, enter rest mode |
| `make rest` | Rest mode on: motion refused, still reading |
| `make resume` | Rest mode off |
| `make temps` | Print the servo temperatures once |
| `make bench` | Bench test of control and auto-rest ([scripts/bench.sh](../scripts/bench.sh)); asks before moving the arm, appends to `bench-report.log` |

**Development** · [mk/dev.mk](../mk/dev.mk)

| Command | What it does |
|---|---|
| `make shell` | Shell in `dev` with ROS sourced |
| `make run CMD='…'` | Run a command in `dev` |
| `make build` | colcon build of `ros2_ws/src` |
| `make test` | colcon test and the results (PC only) |
| `make web` | The page with mock data on :8080, no ROS, no Docker |
| `make clean` | Delete build/ install/ log/ and `~/.ros/log` |

**Setup** · [mk/setup.mk](../mk/setup.mk) · once per machine

| Command | What it does |
|---|---|
| `make setup` | `.env`, git hooks, folders, then `doctor` |
| `make doctor` | Check docker, files, devices; on the Jetson also desktop, power mode and swap |
| `make image` | (Re)build this machine's image |
| `make boot-on` / `boot-off` | Start at boot (systemd `braco`) / undo it |
| `make jetson-image` | Build the arm64 image on the PC (`NANO=user@host` also copies it) |
| `make reset` | Clean + remove containers, volumes and images |

**Network** · [mk/net.mk](../mk/net.mk)

| Command | What it does |
|---|---|
| `make net-test` | Talker + listener on this machine, PASS/FAIL |
| `make net-talk` | Publish on one machine… |
| `make net-listen` | …and check on the other |

**CI** · [mk/ci.mk](../mk/ci.mk) · same as GitHub

| Command | What it does |
|---|---|
| `make ci` | The `host` job: format check, lint, page tests |
| `make ci-ros` | The `ros` job: rosdep, build, test in a clean image |
| `make ci-image` | Build that job's image |
| `make lint` | Compose files, ruff check, shellcheck, hadolint |
| `make format` | clang-format (C++) and ruff (Python); `format-check` only checks (pre-push hook) |

Each machine has two compose services from the same image: `dev`, the tools container used by `shell`, `run` and
`build` (on the Jetson a throw-away `run --rm`), and the system itself, `real` on the Jetson or `sim` on the PC.

---

## 9. CPU and memory on the Jetson

> **Estimates, not measurements.** They assume a 4 GB Jetson Nano (4 Cortex-A57 cores, RAM shared with the GPU), no
> desktop and the 10 W power mode, and come from what these processes usually take on aarch64 and from the rates in
> the code: state at 20 Hz, temperatures every 2 s, joystick at 50 Hz, page at 5 Hz. Replace them with numbers from
> `make status` (end of this section).

| ~0.25 GB | ~0.2 core | ~1.6 GB |
|---|---|---|
| RAM of the braco system on: driver, temperature, joystick, launch and page | Average CPU in operation, about 5% of the 4 cores; ~0.4 with the page's ROS tab open | Peak of `colcon build` with 2 g++ running: the moment with the highest out-of-memory risk |

Estimated memory of a 4 GB Nano (`█` system L4T + Docker, `▓` braco on, `▒` MoveIt, `░` colcon build, `·` free;
one character ≈ 100 MB):

```
                         0         1 GB      2 GB      3 GB      4 GB
                         |         |         |         |         |
Operation      ~0.95 GB  ███████▓▓▓·······························   free ~3.1 GB
+ MoveIt here  ~1.45 GB  ███████▓▓▓▒▒▒▒▒··························   free ~2.6 GB  (not planned: MoveIt runs on the PC)
During a build ~2.55 GB  ███████▓▓▓░░░░░░░░░░░░░░░░···············   free ~1.5 GB  (~0.9 GB with a desktop)
                                             ↑ limit of a 2 GB Nano
```

In operation braco uses little. The risk is the build: two `g++` compiling `rclcpp` code pass 1.5 GB together, which
on a 2 GB Nano or with the desktop on leads to the OOM killer or to swapping on the SD card.

| Process | RAM | Avg CPU (1 core = 100%) | What weighs |
|---|---|---|---|
| `arm_driver` | 15–25 MB | 4–8% | Serial loop with 10 ms pauses, angles at 20 Hz, temperatures and voltages every 2 s; mostly waiting on the UART |
| `temperature_monitor` | 10–20 MB | <1% | One message every 2 s and one I2C write to the LCD |
| `joystick_teleop` | 10–20 MB | 1–3% | 50 Hz timer; publishes only while a stick is off-centre |
| `camera` (Python + OpenCV) | 60–90 MB | 3–8% (estimate) | 15 fps 640×480; MJPG frames are not decoded. The page relays them as MJPEG, ~0.8 MB/s per open tab with the camera |
| `ros2 launch` (Python) | 50–70 MB | <1% | Only supervises its children |
| page (uvicorn + rclpy) | 100–150 MB | 5–15% | 5 Hz telemetry per open tab; with the ROS tab open, graph introspection every 2 s (20–40%) |
| Docker + containerd | 80–120 MB | 1–2% | Outside the container, always on |
| **Total today** | **~330–490 MB** | **~15–38%** | About 4–10% of the Nano's CPU |
| *`robot_state_publisher` (planned)* | *15–25 MB* | *1–2%* | *TF from the URDF* |
| *`move_group` (planned, PC)* | *300–600 MB* | *<2% idle* | *100% of a core for 0.1–2 s per plan: the heaviest process, kept on the PC* |
| *`braco_mission` (planned, PC)* | *60–100 MB* | *<5%* | *State machine, mostly waiting on MoveIt* |
| `colcon build` | 0.6–1 GB per g++ | 100% × jobs | 10–25 min from scratch on the Nano, 1–3 min incremental |

| | Mitigation | Where |
|---|---|---|
| **Build without running out of RAM** | On the Jetson: one package at a time, `MAKEFLAGS=-j2`, Release without tests. `make doctor` warns without swap and suggests a 4 GB swapfile. Or build on the PC with `make jetson-image` | `mk/config.mk` · `mk/setup.mk` |
| **No desktop on the Jetson** | The Ubuntu desktop takes 0.5–1 GB that nobody uses. `make doctor` warns and suggests `sudo systemctl set-default multi-user.target` | `mk/setup.mk` |
| **10 W power mode** | In 5 W mode the Nano runs 2 cores at a lower clock, halving the headroom. `make doctor` warns when not in MAXN (`sudo nvpmodel -m 0`) | `mk/setup.mk` |
| **SD card limits** | `CCACHE_MAXSIZE=1G` instead of the 5 GB default; `make clean` also deletes `~/.ros/log`, which otherwise grows without limit | `docker/compose.yml` · `mk/dev.mk` |
| **Replace the estimates with measurements** *(pending)* | `make status` shows `docker stats` per container and a few seconds of `tegrastats`. Measure in three moments (arm idle, joystick in use, page's ROS tab open) and put the numbers in the table above | `mk/system.mk` |

---

## 10. Simulation and the planned upper layers

```
 braco_mission ┄┄► MoveIt 2 ──trajectory──┬──► real arm: arm_driver    (trajectory path decided with #37)
     #43           braco_moveit_config #40 └┄┄► sim: Gazebo + gz_ros2_control  #34
                        ▲
                        ┊
                   braco_description  URDF
```

The mission talks only to MoveIt 2 (and takes the control as `mission`), never to the serial port, so the same code
runs on the real arm and in simulation.

- `braco_description` holds the URDF; its joint limits must match `JOINT_LIMITS` in `mycobot.hpp`, and its joint
  names are the ones on `/joint_states` (`joint1..joint6`) on both the real arm and the simulation.
- `sim:=true` will start Gazebo with that model, publishing the same `/joint_states`. Until then
  `braco_description`'s `gazebo.launch.py` shows it in Gazebo with `gz_ros2_control` (`/arm_controller`,
  `FollowJointTrajectory`), and `scripts/braco_arm.py` runs the same moves there and, with `--real`, on the arm.
- How MoveIt's trajectories reach `arm_driver` on the real arm is decided with #37 and #40.

---

## 11. Build and test

```bash
make build    # colcon build of ros2_ws/src, in the dev container
make test     # colcon test: gtest (teleop logic, control owner, auto rest) and ament lint (PC only)
make restart  # run what was just built
make ci       # the `host` CI job: format check, lint and the web page tests, no ROS
make ci-ros   # the `ros` CI job: rosdep, build and test in a clean image
```

- **PC:** everything, `RelWithDebInfo`, with tests.
- **Jetson:** only `--packages-up-to braco_bringup`, `Release`, `BUILD_TESTING=OFF`, one package at a time.
- **CI** ([ci.yml](../.github/workflows/ci.yml)) runs both jobs on every pull request.

---

## 12. Status against the target architecture

The target was agreed before the ROS 2 migration: single serial owner, one control owner, one thermal authority, the
`braco_*` layout, one `make up`. This is where the gap between that target and the code is tracked; update it in the
PR that closes or opens a gap.

### Done

| Item | Detail |
|---|---|
| **Single serial owner** | Only `arm_driver` opens the UART; the page has no direct serial mode |
| **One control owner** | `ArmCommand.source`, `TakeControl.srv`, latched `/arm/control_owner`; motion only from the owner, STOP from anyone ([ADR-0002](ADR-0002-unique-control.md)) |
| **Clients follow the owner** | Page lease = control `web`; joystick takes it free or forced (button 8 for 1 s), `power_on` only as owner; zero/park force and release |
| **One thermal authority** | `temperature_monitor` publishes the level on `/diagnostics`; the page reads it |
| **Rest on overheat** | Once per hot episode (from 65 °C until every joint is below 60 °C), manual resume ([ADR-0003](ADR-0003-rest-per-temp.md)) |
| **braco layout** | `ros2_ws/src` with `braco_interfaces`, `braco_driver`, `braco_teleop`, `braco_camera`, `braco_bringup`; one launch and one `braco.yaml`; a README in every package, `mycobot_cpp` included |
| **braco infrastructure** | `docker/Dockerfile`, `compose.yml` (project `braco`), `braco-image/*`, `braco.service` |
| **One Makefile** | Split in `mk/`, idempotent `make up`, ~30 public commands |
| **Lean repository root** | `docker`, `docs`, `mk`, `ros2_ws`, `scripts` (operator scripts run by `make`), `web` and the `Makefile` (plus the formatter configs and `requirements-dev.txt`); `mycobot_cpp` in `ros2_ws/src` (one colcon base path, no extra mount); `tests/`, `playground/` and the root `requirements.txt` removed |
| **Jetson guards** | `doctor` checks, ccache limit, ROS log cleanup, `status` with `docker stats` and `tegrastats` |
| **Validated URDF** | `braco_description`: joint names, limits, zero pose, joint directions and FK (within 1.5 mm of `get_coords()`) checked on the real arm; measured joint velocities; table check in `mycobot_cpp` uses the same kinematics ([report](urdf-sim-to-real.md), #25) |

### Differs from the target (accepted for now)

| Target | Today | Why / next step |
|---|---|---|
| `mycobot_cpp/{include,src,tools}`, CMake only | `ros2_ws/src/mycobot_cpp/{includes,srcs}` with a Makefile next to CMake | Folder rename not done yet; low value until the library changes |
| `teleop_logic.hpp` in `braco_teleop/include/` | in `mycobot_cpp/includes/` | Kept ROS-free in the library; the test lives in `braco_teleop/test/` |
| `braco_moveit` | `braco_moveit_config` | MoveIt's own convention; decide when #40 starts |
| zero / park / arm_moves as Python scripts | C++ programs in `braco_bringup/src/` | They reuse the `mycobot_cpp` poses and limits |
| ADRs in `docs/adr/NNNN-*.md` | `docs/ADR-NNNN-*.md` | The page's Docs tab groups them by this prefix |
| `examples/ros2_basics_*` and `make demos` | removed | The learning demos are no longer part of the system |
| The page as an `ExecuteProcess` inside the launch | started by `arm_routine.sh` next to the launch | Optional; a matter of taste, not of correctness |

### Pending

| Item | Issue |
|---|---|
| `sim:=true`: Gazebo + `gz_ros2_control` with the same `/joint_states` | #34 |
| How trajectories reach `arm_driver` on the real arm (`FollowJointTrajectory` or equivalent) | #37 |
| `braco_moveit_config`: SRDF, kinematics, limits, controllers | #40 |
| `braco_mission`: pick-and-place, pen drawing, taking the control as `mission` | #43 |
| Measured CPU/RAM on the Jetson replacing the estimates of [section 9](#9-cpu-and-memory-on-the-jetson) | — |

---

## 13. Changing the architecture

### Adding a package

1. Name it `braco_<role>` and put it in `ros2_ws/src`. Code with no ROS dependency goes in `mycobot_cpp`.
2. `package.xml` format 3, Apache-2.0, maintainer set; `ament_lint_auto` as a test dependency.
3. A `README.md` with what it does, its topics/services/parameters and how to run it by hand.
4. Needed on the real arm? `exec_depend` of `braco_bringup`, started from `braco.launch.py` under `sim:=false`.
   Simulation or planning only? Keep it out of `braco_bringup`.
5. New message or service? In `braco_interfaces`, never in the package that uses it.

### Keeping this document current

In the same PR as the code change, in **both** `architecture.md` and `architecture.html`:

| You changed | Update |
|---|---|
| a node, topic, service or message | sections 2 and 4, and the package README |
| who may move the arm, a safety gate, rest mode | sections 1, 3 and 4 (and an ADR if it is a new policy) |
| a parameter in `braco.yaml` or a launch argument | section 7 |
| a package, a folder or a file name | section 5 |
| a container, image, `.env` key or the start-up script | section 6 |
| a `make` command | section 8 |
| something that changes CPU/RAM on the Jetson, or a measurement | section 9 |
| anything listed in section 12 | move it between *pending*, *differs* and *done* |

Then update the "Last checked against the code" date at the top.

---

Why ROS 2 runs in Docker: [ADR-0001](ADR-0001-ros2-in-docker-on-jetson-nano.md) · Containers and commands:
[docker/README.md](../docker/README.md) · ROS 2 concepts: [ROS2/README.md](ROS2/README.md)
