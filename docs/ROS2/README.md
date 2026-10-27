# ROS 2 for the braco team

This folder is the team's reference for ROS 2 (Robot Operating System 2). The top of this README tells you **how braco uses ROS 2**: which nodes run, and the topics and services they use to talk to each other.

ROS 2 runs in Docker: **ROS 2 Lyrical Luth on Ubuntu 26.04**, on both the PC and the Jetson Nano. The reasons are in [ADR-0001](../ADR-0001-ros2-in-docker-on-jetson-nano.md), and the container setup is in [docker/README.md](../../docker/README.md).

---

## Start here

### What to read first

| Step | Read | Why |
|---|---|---|
| 1 | [ROS 2 in braco](#ros-2-in-braco) (just below) | The nodes, the topics and services between them, and how the system starts. Read this first even if you already know ROS 2 |
| 2 | Reference: [4. The computation graph](#4-the-computation-graph), [5. Nodes](#5-nodes), [7. Topics](#7-topics-publish--subscribe), [9. Services](#9-services-request--response), [11. Choosing between topic, service and action](#11-choosing-between-topic-service-and-action) | The five ideas you need to follow the diagram. Skip if you know ROS 2 |
| 3 | [`braco_driver`](../../ros2_ws/src/braco_driver/README.md) and [`braco_teleop`](../../ros2_ws/src/braco_teleop/README.md) READMEs | What each node does, its safety rules (watchdog, rest mode) and how to run it by hand |
| 4 | [safety.md](../safety.md) | **Before you move the real arm** |
| 5 | [INSTRUCTIONS.md](INSTRUCTIONS.md) | Hands-on: write and run your own publisher, subscriber, service, action and lifecycle nodes |
| Later | [CHEATSHEET.md](CHEATSHEET.md), [TROUBLESHOOTING.md](TROUBLESHOOTING.md), the [Reference](#reference-ros-2-concepts) | Look things up when you need them |

### A first command to try

On any machine (PC or Jetson), no arm needed:

```bash
make net-test
```

It starts this machine's dev container, runs a C++ talker and a Python listener, and prints **PASS** or **FAIL**. A PASS means ROS 2, the container and DDS discovery work here.

Then, with the workspace built (`make build`), look at the message every motion goes through:

```bash
make run CMD='ros2 interface show braco_interfaces/msg/ArmCommand'
```

On the Jetson with the system running (`make up`), read the arm without moving it:

```bash
make temps                          # one message from /arm/servo_temperatures (safe, works in rest mode)
make run CMD='ros2 node list'       # expect /arm_driver, /temperature_monitor, /joystick_teleop, /camera, /braco_web
```

The web page shows the same thing without a terminal: its health panel says whether the four arm nodes (`arm_driver`, `temperature_monitor`, `joystick_teleop`, `camera`) and the five `arm_driver` services are present, and its graph explorer lists every node, topic and service and can echo any topic (read-only).

---

## ROS 2 in braco

### The idea

**`arm_driver` is the only process that opens the myCobot 280 serial port.** Everything else (the joystick, the zero and park routines, the web page, the `make` targets) is a client of its topics and services. Any client can start, stop or crash without touching the port, and the driver's watchdog stops the arm if motion commands stop arriving for 300 ms. Only one client moves the arm at a time: the control owner, kept by `arm_driver` ([ADR-0002](../ADR-0002-unique-control.md)); STOP is obeyed from anyone. Likewise `camera` is the only process that opens the USB camera.

### Node diagram

```
 CLIENTS (send commands)                  THE DRIVER                    WATCHERS

 ┌───────────────────┐                  ┌──────────────┐  /arm/servo_   ┌─────────────────────┐
 │ joystick_teleop   │──┐               │              │  temperatures  │ temperature_monitor │
 │ braco_teleop      │  │               │              │ ─────────────► │ braco_driver        │
 └───────────────────┘  │ /arm/command  │  arm_driver  │                └───┬─────────────┬───┘
 ┌───────────────────┐  │ /arm/led      │ braco_driver │                    │ I2C         │ /diagnostics
 │ arm_zero/arm_park │──┼─────────────► │              │                    ▼             ▼
 │ braco_bringup     │  │ services ═══► │              │                 LCD1602     braco_web, rqt
 └───────────────────┘  │               │              │
 ┌───────────────────┐  │ ◄──────────── │              │
 │ braco_web         │──┤ /joint_states │              │
 │ web page (Python) │  │ /arm/read_only│              │
 └───────────────────┘  │               └──────┬───────┘
 ┌───────────────────┐  │                      │ UART /dev/ttyTHS1, 1 Mbaud
 │ make rest/resume  │──┘                      ▼
 │ make temps        │                    myCobot 280
 └───────────────────┘

  ───► topic    ═══► service call (/arm/take_control, /arm/power_on, /arm/release_servos, /arm/set_gripper,
                                    /arm/set_read_only)
  Not drawn: /arm/servo_voltages and /arm/control_owner (arm_driver → braco_web, joystick_teleop) and the
  camera node (USB camera → /camera/image_raw/compressed → braco_web).
```

All of it runs on the Jetson, in the `real` container. The PC sees the same topics over DDS (same `ROS_DOMAIN_ID`, from `docker/.env`), so you can inspect the arm from the PC with `ros2 topic echo`, rqt or PlotJuggler.

### Nodes

| Node | Package | Started by | Job |
|---|---|---|---|
| `arm_driver` | `braco_driver` | `braco.launch.py` (not respawned: a driver fault must be seen) | Owns the serial port. Publishes angles, temperatures and voltages, executes commands from the control owner, watchdog, rest mode |
| `temperature_monitor` | `braco_driver` | `braco.launch.py` (respawned) | Servo temperatures to the LCD1602 and `/diagnostics` (warm from 55 °C, hot from 65 °C); at hot it runs `park` once per episode ([ADR-0003](../ADR-0003-rest-per-temp.md)) |
| `joystick_teleop` | `braco_teleop` | `braco.launch.py` (respawned) | Joystick to joint jogging, gripper, homing and speed modes. Survives the joystick being unplugged |
| `camera` | `braco_camera` | `braco.launch.py` (respawned) | Owns the eye-in-hand USB camera; publishes its JPEG frames. Waits for the camera when it is unplugged |
| `arm_zero` | `braco_bringup` | `make zero` (runs once) | Leaves rest mode, powers on, moves all joints to 0° |
| `arm_park` | `braco_bringup` | `make park` / `make down PARK=1` (runs once) | Folds the arm, releases the joints, enters rest mode |
| `braco_web` | `web/server` (Python, `rclpy` inside the uvicorn web server; not a ROS package) | `arm_routine.sh` | The web page's single ROS node, shared by the arm controls, the health panel and the graph explorer, spun by a `MultiThreadedExecutor` (see [14. Executors](#14-executors-callbacks-and-threading)). A plain client of `arm_driver` like the joystick: a pose or joint move is a stream of `ALL_ANGLES` steps at most 15° ahead of the real angles, resent every 100 ms, the same as `arm_zero` and `arm_park` |

### Topics

| Topic | Type | Published by | Subscribed by | Notes |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | `arm_driver` | `joystick_teleop`, `arm_zero`, `arm_park`, `braco_web` | 20 Hz, radians, `joint1`..`joint6`. Clients stop moving the arm if it gets old (150 ms for the joystick, 500 ms for the zero/park routines and the web page) |
| `/arm/command` | `braco_interfaces/msg/ArmCommand` | `joystick_teleop`, `arm_zero`, `arm_park`, `braco_web` | `arm_driver` | `STOP`, `JOINTS` (only the joystick sends these) or `ALL_ANGLES`. **Latest wins**: a new command replaces any pending one |
| `/arm/led` | `std_msgs/msg/ColorRGBA` | `joystick_teleop`, `braco_web` | `arm_driver` | LED colour of the arm (the joystick shows its speed mode) |
| `/arm/servo_temperatures` | `braco_interfaces/msg/ServoTemperatures` | `arm_driver` | `temperature_monitor`, `braco_web`, `make temps` | Every 2 s, also in rest mode. `valid = false` when the arm did not answer |
| `/arm/servo_voltages` | `braco_interfaces/msg/ServoVoltages` | `arm_driver` | `braco_web` | Every 2 s, also in rest mode. `valid = false` when the arm did not answer |
| `/arm/read_only` | `std_msgs/msg/Bool` | `arm_driver` | `braco_web` | Rest mode on/off. **Latched** (`TRANSIENT_LOCAL`, depth 1) on both sides: a late subscriber gets the current value. A `VOLATILE` publisher would not match this subscriber (see [8. QoS](#8-quality-of-service-qos)) |
| `/arm/control_owner` | `std_msgs/msg/String` | `arm_driver` | `joystick_teleop`, `braco_web` | Who may move the arm (`web`, `joystick`, `script`, `mission`; `""` = free). **Latched** like `/arm/read_only` |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `temperature_monitor` | `braco_web` (health panel), rqt | One status per joint (OK / WARN / ERROR), `STALE` when temperatures stop arriving |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `camera` | `braco_web` | JPEG, up to 15 fps, sensor-data QoS |

### Services

All five are served by `arm_driver`.

| Service | Type | Called by | What it does |
|---|---|---|---|
| `/arm/take_control` | `braco_interfaces/srv/TakeControl` | `joystick_teleop`, `arm_zero` / `arm_park` (forced), `braco_web` (the page lease) | Become the control owner (`source`, `force`) or give it back (`release`) ([ADR-0002](../ADR-0002-unique-control.md)) |
| `/arm/power_on` | `std_srvs/srv/Trigger` | `joystick_teleop` (when a joystick connects), `arm_zero`, `braco_web` ("wake up", after leaving rest mode) | Locks the joints. Refused in rest mode |
| `/arm/release_servos` | `std_srvs/srv/Trigger` | `arm_park`, `braco_web` (Park, once the pose is reached) | Frees the joints (the arm can be moved by hand). Drops pending motion |
| `/arm/set_gripper` | `std_srvs/srv/SetBool` | `joystick_teleop` (A / B), `braco_web` | `true` closes, `false` opens. Refused in rest mode |
| `/arm/set_read_only` | `std_srvs/srv/SetBool` | `arm_zero` (false), `arm_park` (true), `make rest` / `make resume`, `braco_web` (rest on/off, wake up) | Rest mode: motion, gripper and power on are refused, angles and temperatures keep being published |

braco uses **no actions** yet. Long motions (zero, park) are short programs that stream `/arm/command` and watch `/joint_states` until the pose is reached. Actions arrive with MoveIt 2; see [10. Actions](#10-actions-long-running-goals).

### Packages

All in [`ros2_ws/src/`](../../ros2_ws/src/).

| Package | Status | Contains |
|---|---|---|
| [`braco_interfaces`](../../ros2_ws/src/braco_interfaces/README.md) | In use | `ArmCommand`, `JointTarget`, `ServoTemperatures`, `ServoVoltages`, `TakeControl` |
| [`braco_driver`](../../ros2_ws/src/braco_driver/README.md) | In use | `arm_driver`, `temperature_monitor` (on top of `mycobot_cpp`) |
| [`braco_teleop`](../../ros2_ws/src/braco_teleop/README.md) | In use | `joystick_teleop`; the jogging logic without ROS and its tests |
| [`braco_camera`](../../ros2_ws/src/braco_camera/README.md) | In use | `camera` (Python) |
| [`braco_bringup`](../../ros2_ws/src/braco_bringup/README.md) | In use | `braco.launch.py`, `config/braco.yaml`, `arm_routine.sh`, `zero`, `park` |
| [`braco_description`](../../ros2_ws/src/braco_description/README.md) | In use (PC), #25 | URDF model, meshes, Gazebo viewer (`gazebo.launch.py`), sim-to-real checks |
| [`braco_moveit_config`](../../ros2_ws/src/braco_moveit_config/README.md) | Empty | MoveIt 2 configuration (runs on the PC) |
| [`braco_mission`](../../ros2_ws/src/braco_mission/README.md) | Empty | Tasks (pick-and-place, drawing) as a state machine over MoveIt 2 |

### How the system starts

```
make up ──► `real` container ──► arm_routine.sh ──┬──► ros2 launch braco_bringup braco.launch.py sim:=false
                                                  │        ├── arm_driver
                                                  │        ├── temperature_monitor
                                                  │        ├── joystick_teleop
                                                  │        └── camera
                                                  └──► web page (uvicorn, port WEB_PORT)
```

- **Parameters**: tuning values (rates, watchdog, timeouts) are in [`braco_bringup/config/braco.yaml`](../../ros2_ws/src/braco_bringup/config/braco.yaml); the wiring (`port`, `baud`, `lcd_bus`, `device`, `camera`) comes from launch arguments that the compose file fills from `docker/.env`.
- **The arm never moves on start or stop.** Zero and park are manual (`make zero`, `make park`, `make down PARK=1`).
- If the launch dies (for example the driver lost the port), the container stops.
- `sim:=true` (the `sim` container on the PC) is reserved for the Gazebo arm (#34) and starts no node yet. The web page still starts there, with `ARM_SOURCE=gazebo`: it reads `/joint_states` and sends moves as `trajectory_msgs/msg/JointTrajectory` on `/joint_trajectory_controller/joint_trajectory` (the `ros2_control` controller Gazebo will provide), so it only waits until #34 lands.

### ROS 2 features braco does not use yet

The [Reference](#reference-ros-2-concepts) covers more than the code uses today. Not used yet on the real arm: actions, lifecycle nodes (`arm_driver` is a plain `rclcpp::Node`), tf2 and the URDF, `ros2_control`, MoveIt 2. The URDF, `gz_ros2_control` and Gazebo are used only by `braco_description`'s viewer on the PC (#25); the `sim:=true` bringup is #34.

---

## The documents in this folder

| Document | What it covers |
|---|---|
| [README.md](README.md) (this file) | How braco uses ROS 2 (top), then the concepts: nodes, topics, QoS, services, actions, parameters, lifecycle, launch, DDS, workspaces, tooling |
| [INSTRUCTIONS.md](INSTRUCTIONS.md) | Step by step: install ROS 2 (in our Docker setup or natively), create a workspace, write and run publisher/subscriber, service, action and lifecycle nodes in C++ and Python, inspect them |
| [CHEATSHEET.md](CHEATSHEET.md) | One-page command reference (`ros2`, `colcon`, `rosdep`, the project `make` targets) |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Common errors and how to fix them |

---

# Reference: ROS 2 concepts

General ROS 2 theory, for when you need it. Section numbers stay stable so links from the other documents keep working.

1. [What ROS 2 is (and is not)](#1-what-ros-2-is-and-is-not)
2. [Why ROS 2 and not ROS 1](#2-why-ros-2-and-not-ros-1)
3. [Distributions](#3-distributions)
4. [The computation graph](#4-the-computation-graph)
5. [Nodes](#5-nodes)
6. [Interfaces: messages, services and actions](#6-interfaces-messages-services-and-actions)
7. [Topics (publish / subscribe)](#7-topics-publish--subscribe)
8. [Quality of Service (QoS)](#8-quality-of-service-qos)
9. [Services (request / response)](#9-services-request--response)
10. [Actions (long-running goals)](#10-actions-long-running-goals)
11. [Choosing between topic, service and action](#11-choosing-between-topic-service-and-action)
12. [Parameters](#12-parameters)
13. [Managed (lifecycle) nodes](#13-managed-lifecycle-nodes)
14. [Executors, callbacks and threading](#14-executors-callbacks-and-threading)
15. [Names, namespaces and remapping](#15-names-namespaces-and-remapping)
16. [Launch files](#16-launch-files)
17. [The middleware: DDS, RMW and discovery](#17-the-middleware-dds-rmw-and-discovery)
18. [Time and clocks](#18-time-and-clocks)
19. [Coordinate frames: tf2](#19-coordinate-frames-tf2)
20. [Packages, workspaces and the build system](#20-packages-workspaces-and-the-build-system)
21. [Tools](#21-tools)
22. [The ecosystem we will use for the arm](#22-the-ecosystem-we-will-use-for-the-arm)
23. [How this maps to our project](#23-how-this-maps-to-our-project)
24. [Glossary](#24-glossary)
25. [Further reading](#25-further-reading)

---

## 1. What ROS 2 is (and is not)

ROS 2 is **not an operating system**. It is a set of libraries, tools and conventions for building robot software as a collection of small, independent programs that talk to each other. It runs on top of a normal OS (Ubuntu in our case).

It gives you:

- **A communication layer**: programs (called *nodes*) exchange typed data over *topics*, *services* and *actions*, on one machine or across a network, with no extra code for the networking.
- **Standard data types**: messages for sensor data, poses, joint states, images and so on (`sensor_msgs`, `geometry_msgs`, ...), so packages written by different people fit together.
- **Build and packaging tools**: `colcon`, `ament`, `rosdep`.
- **Developer tools**: command-line introspection (`ros2 ...`), visualisation (RViz2, rqt), recording and replay (`rosbag2`).
- **A large ecosystem**: motion planning (MoveIt 2), hardware abstraction and control (ros2_control), simulation (Gazebo), coordinate transforms (tf2), navigation (Nav2) and more.

What it does **not** give you: the robot's behaviour. You still write the nodes that read the camera, decide where to pick, and drive the arm. ROS 2 is the plumbing and the toolbox around them.

### The core idea in one picture

```
 ┌──────────────┐   /image_raw (topic)    ┌──────────────┐   /target_pose (topic)   ┌──────────────┐
 │ camera_node  │ ──────────────────────► │ vision_node  │ ───────────────────────► │ planner_node │
 └──────────────┘                         └──────────────┘                          └──────┬───────┘
                                                                                           │ /move_arm (action)
                                                                                           ▼
                                          /joint_states (topic)                     ┌──────────────┐
                                    ◄────────────────────────────────────────────── │ arm_driver   │ ── UART ── myCobot
                                                                                    └──────────────┘
```

Each box is a separate process (or a component inside one process). Each can be written in C++ or Python, started, stopped, replaced or moved to another machine without changing the others, as long as the names and data types stay the same.

---

## 2. Why ROS 2 and not ROS 1

ROS 1 (last release: Noetic, end-of-life May 2025) had a central **master** process (`roscore`). Every node registered with it, and if it died, discovery stopped. ROS 2 was rewritten to remove the main limitations of ROS 1:

| Topic | ROS 1 | ROS 2 |
|---|---|---|
| Discovery | Central `roscore` (single point of failure) | Decentralised, peer-to-peer via DDS: no master |
| Transport | Custom TCPROS/UDPROS | Industry standard DDS (or Zenoh), pluggable via the RMW layer |
| Reliability control | Little | Per-topic **QoS** policies (reliable/best-effort, history, durability, deadlines) |
| Real-time | Not designed for it | Designed to allow real-time-safe code paths (`rclcpp`, memory pre-allocation, executors) |
| Node state | Ad hoc | Standard **lifecycle** (managed) nodes |
| Multiple robots / networks | Hard | Domain IDs, namespaces, discovery ranges |
| Security | None | SROS2: authentication, encryption, access control (DDS-Security) |
| Python | Python 2, later 3 | Python 3 only |
| Build | `catkin` | `ament` + `colcon` |
| Platforms | Linux | Linux, Windows, macOS, RTOS (micro-ROS) |

The Jetson Nano's host still has ROS 1 Noetic installed next to ROS 2 Galactic in `/opt/ros`. **Never source both in the same shell**: their environment variables conflict.

---

## 3. Distributions

A ROS 2 **distribution** ("distro") is a versioned set of packages released together, named alphabetically with an adjective and a turtle species. Each distro targets specific Ubuntu versions; the binary packages only exist for those.

| Distro | Released | Target Ubuntu | Support | Relevance to us |
|---|---|---|---|---|
| Galactic Geochelone | 2021 | 20.04 | EOL 2022 | Installed natively on the Nano; vendor `mycobot_ros2` packages target it |
| Humble Hawksbill | 2022 | 22.04 | LTS, to 2027 | Very common in tutorials |
| Jazzy Jalisco | 2024 | 24.04 | LTS, to 2029 | |
| Kilted Kaiju | 2025 | 24.04 | Short-term | |
| **Lyrical Luth** | 2026 | **26.04** | **LTS** | **Our target, in Docker** |
| Rolling Ridley | continuous | latest | Rolling | Development branch, not for robots |

Rules worth remembering:

- **Even years are LTS** (5 years of support); odd years are short-term (about 1.5 years).
- Packages from different distros **do not mix**. A package built for Galactic must be rebuilt (and sometimes ported) for Lyrical.
- Nodes from different distros can *sometimes* talk over the network, but message definitions and middleware defaults change between releases, so it is not guaranteed. In this project both machines use the same image base so this never comes up.
- The active distro is in the environment variable `ROS_DISTRO` (`echo $ROS_DISTRO` prints `lyrical` in our containers).
- Tutorials written for Humble or Jazzy mostly work on Lyrical, but check the docs for the distro you run: some APIs change (for example `ament_target_dependencies` is deprecated in favour of `target_link_libraries`).
- Differences we hit on Lyrical while testing the Sprint 0 examples: Python loggers have `warning()` but no `warn()`; there is no bare `rqt_graph` command (use `ros2 run rqt_graph rqt_graph`); CMake warns below `cmake_minimum_required(VERSION 3.10)`; `ros2 pkg create` also generates `py.typed` and a `test_mypy.py` linter. Details and fixes in [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

---

## 4. The computation graph

The **ROS graph** is the network of all ROS 2 elements that are running and can see each other at a given moment: nodes, and the topics, services and actions that connect them. It is dynamic: nodes appear and disappear at runtime, and connections form automatically when a publisher and a subscriber with the same name, type and compatible QoS exist.

You can look at the graph at any time:

```bash
ros2 node list          # all nodes
ros2 topic list -t      # all topics with their types
ros2 service list       # all services
ros2 action list        # all actions
ros2 run rqt_graph rqt_graph   # a picture of it (PC only, needs a display)
```

---

## 5. Nodes

A **node** is the unit of computation: one participant in the graph, usually responsible for one job (drive the arm, read the camera, detect objects). A node can:

- publish and subscribe to topics,
- offer and call services,
- offer and call actions,
- declare and read parameters,
- run timers.

Nodes are written with a **client library**:

| Language | Library | Typical use |
|---|---|---|
| C++ | `rclcpp` | Drivers, control loops, anything performance- or timing-sensitive |
| Python | `rclpy` | Prototyping, orchestration, vision with OpenCV/numpy, scripts |

Both sit on top of the same C library (`rcl`), so a C++ node and a Python node interoperate with no extra work.

**One process can host many nodes.** In C++ this is done with *components* (nodes compiled as shared libraries and loaded into a container process), which allows **intra-process communication**: messages are passed as pointers without serialisation. Useful later for camera images; not needed for Sprint 0.

Every node has a **name**, which must be unique in the graph. Two nodes with the same name cause warnings and confusing behaviour in tools.

---

## 6. Interfaces: messages, services and actions

All data in ROS 2 is strongly typed. The types are described in small text files called **interfaces**, written in the ROS Interface Definition Language (IDL), and code is generated from them for C++ and Python at build time.

### Messages (`.msg`) — used by topics

```text
# geometry_msgs/msg/Point.msg
float64 x
float64 y
float64 z
```

### Services (`.srv`) — a request and a response, separated by `---`

```text
# example_interfaces/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Actions (`.action`) — goal, result and feedback, separated by `---`

```text
# Fibonacci.action
int32 order               # goal
---
int32[] sequence          # result
---
int32[] partial_sequence  # feedback
```

### Field types

| Kind | Examples |
|---|---|
| Built-in | `bool`, `byte`, `char`, `int8`...`int64`, `uint8`...`uint64`, `float32`, `float64`, `string`, `wstring` |
| Arrays | `int32[]` (unbounded), `float64[6]` (fixed), `int32[<=10]` (bounded) |
| Other messages | `geometry_msgs/Pose pose`, `std_msgs/Header header` |
| Constants | `uint8 MODE_IDLE=0` |
| Defaults | `float64 speed 0.5` |

### Common packages

| Package | Contains |
|---|---|
| `std_msgs` | Primitive wrappers (`String`, `Bool`, `Float64`, `Header`). Fine for tests; prefer meaningful types in real code |
| `geometry_msgs` | `Point`, `Pose`, `PoseStamped`, `Twist`, `Transform` |
| `sensor_msgs` | `JointState`, `Image`, `CameraInfo`, `Imu` |
| `trajectory_msgs` | `JointTrajectory` (what arm controllers consume) |
| `std_srvs` | `Trigger`, `SetBool`, `Empty` |
| `example_interfaces` | `AddTwoInts`, `Fibonacci`: **for examples and learning only** |
| `action_msgs`, `lifecycle_msgs` | Plumbing used by actions and lifecycle nodes |

Custom interfaces live in their **own package** (convention: `<project>_interfaces`), built with CMake even if all your nodes are Python. [INSTRUCTIONS.md](INSTRUCTIONS.md#6-create-a-custom-interfaces-package) shows how.

Fully qualified type names look like `package/kind/Name`, for example `sensor_msgs/msg/JointState` or `example_interfaces/srv/AddTwoInts`. Inspect any type with:

```bash
ros2 interface show sensor_msgs/msg/JointState
ros2 interface list | grep my_package
```

---

## 7. Topics (publish / subscribe)

A **topic** is a named, typed channel for a **continuous stream of data**. Nodes **publish** messages to it and other nodes **subscribe** to it.

```
            /joint_states (sensor_msgs/msg/JointState)
 publisher ─────────────────────────────────────────────► subscriber A
                                     └──────────────────► subscriber B
```

Properties:

- **Many-to-many**: any number of publishers and subscribers on one topic.
- **Anonymous and decoupled**: the publisher does not know who (if anyone) listens, and does not wait for them.
- **Asynchronous, fire-and-forget**: no reply, no acknowledgement at application level.
- **Typed**: a topic has exactly one type. A publisher and a subscriber with different types on the same name do not connect.
- A subscriber receives messages in a **callback** function, executed by the node's executor.

Use topics for: sensor streams (images, joint states), periodic status, commands sent at a fixed rate (velocity, joint targets streamed by a controller).

---

## 8. Quality of Service (QoS)

QoS policies control **how** messages are delivered on a topic. Every publisher and subscriber has a QoS profile, and **a connection is only made if the two profiles are compatible**. This is one of the most common reasons for "my subscriber receives nothing".

### The policies

| Policy | Options | Meaning |
|---|---|---|
| **History** | `KEEP_LAST` / `KEEP_ALL` | Keep only the last *N* samples, or all of them (bounded by resource limits) |
| **Depth** | integer (e.g. 10) | *N* for `KEEP_LAST`: the queue size |
| **Reliability** | `RELIABLE` / `BEST_EFFORT` | Retransmit until delivered, or send once and accept losses |
| **Durability** | `VOLATILE` / `TRANSIENT_LOCAL` | New subscribers get nothing old, or the publisher keeps the last *depth* samples and sends them to late joiners ("latched" topic in ROS 1 terms) |
| **Deadline** | duration | Expected maximum period between messages; violations raise an event |
| **Lifespan** | duration | Messages older than this are dropped and not delivered |
| **Liveliness** | `AUTOMATIC` / `MANUAL_BY_TOPIC` | How a publisher shows it is still alive |
| **Lease duration** | duration | How long without a liveliness signal before the publisher is considered dead |

### Predefined profiles

| Profile | Settings | Typical use |
|---|---|---|
| Default (depth 10) | `KEEP_LAST 10`, `RELIABLE`, `VOLATILE` | Most topics |
| `SensorDataQoS` | `KEEP_LAST 5`, `BEST_EFFORT`, `VOLATILE` | High-rate sensors (camera, IMU) where fresh data beats complete data |
| `ParametersQoS` / `ServicesQoS` | `RELIABLE`, larger depth | Used internally |
| "Latched" (custom) | `KEEP_LAST 1`, `RELIABLE`, `TRANSIENT_LOCAL` | Values that change rarely and late joiners must see: robot description, map, current mode |

### Compatibility rules (Request vs Offered)

The **subscriber requests** a level of service and the **publisher offers** one. The connection is made only if what is offered is at least as strong as what is requested.

| Publisher offers | Subscriber requests | Connected? |
|---|---|---|
| `RELIABLE` | `RELIABLE` | Yes |
| `RELIABLE` | `BEST_EFFORT` | Yes (subscriber gets best effort) |
| `BEST_EFFORT` | `BEST_EFFORT` | Yes |
| `BEST_EFFORT` | `RELIABLE` | **No** |
| `TRANSIENT_LOCAL` | `VOLATILE` | Yes |
| `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL` | Yes |
| `VOLATILE` | `TRANSIENT_LOCAL` | **No** |

The same logic applies to deadline (offered period must be ≤ requested), liveliness and lease duration.

When QoS is incompatible, both sides get an "incompatible QoS" event and a warning is logged. Check with:

```bash
ros2 topic info -v /my_topic      # shows the QoS of every endpoint
```

### Practical guidance for the arm

| Data | Suggested QoS | Why |
|---|---|---|
| Camera images | `SensorDataQoS` | A late frame is useless; do not let retransmissions pile up |
| `/joint_states` | Default reliable, depth 10 | Small, frequent, everyone expects it reliable |
| Commands to the arm | Reliable, small depth (1 to 10) | Do not lose commands, but do not execute a backlog of stale ones |
| Robot description, current mode | Reliable + `TRANSIENT_LOCAL`, depth 1 | Late joiners need the current value |

The issue for Sprint 0 asks for publishers and subscribers **with explicit QoS**: always build a `QoSProfile` in code instead of relying on the integer shortcut (`create_publisher(..., 10)`), so the choice is visible and reviewed. Examples are in [INSTRUCTIONS.md](INSTRUCTIONS.md#7-publisher-and-subscriber-with-explicit-qos).

---

## 9. Services (request / response)

A **service** is a **synchronous-style, one-to-one call**: a client sends a request and the server sends back exactly one response.

```
 client ──── request (a=2, b=3) ───► server
        ◄─── response (sum=5) ─────
```

Properties:

- One **server** per service name (several servers with the same name is undefined behaviour); any number of clients.
- The response comes once. There is no progress feedback and no cancellation.
- Calls should be **quick**. A service that takes seconds blocks its caller and often its own executor.
- In code, prefer **asynchronous calls** (`call_async` in Python, `async_send_request` in C++). Calling a service synchronously from inside a callback on a single-threaded executor **deadlocks**, because the executor that should process the response is busy waiting for it.

Use services for: queries ("what is the current gripper state?"), short commands that either succeed or fail quickly ("power on the servos", "set LED colour", "switch to free-move mode"), triggering a computation.

---

## 10. Actions (long-running goals)

An **action** is for **tasks that take time**, report progress and can be cancelled. It is built from three services and two topics under the hood:

```
 action client                                   action server
      │ ── send_goal (service) ─────────────────────► │  accept / reject
      │ ◄──────────────────────── feedback (topic) ── │  0%, 20%, 40%...
      │ ── cancel_goal (service, optional) ─────────► │
      │ ── get_result (service) ────────────────────► │
      │ ◄──────────────────────────── result ──────── │  succeeded / aborted / canceled
      │ ◄──────────────────────── status (topic) ──── │
```

Goal lifecycle:

```
              accept
  (sent) ──────────────► ACCEPTED ──► EXECUTING ──► SUCCEEDED
     │                                   │   │
     │ reject                            │   └──► ABORTED  (server gave up)
     ▼                                   ▼
  REJECTED                     CANCELING ──► CANCELED
```

Properties:

- The server decides whether to **accept or reject** each goal (for example, reject a target outside the workspace).
- It publishes **feedback** during execution.
- The client can **cancel**; the server decides how to stop safely.
- The final **result** carries a status: succeeded, aborted or canceled.

Use actions for: moving the arm to a pose, executing a trajectory, a pick-and-place sequence, any behaviour that lasts more than a moment. MoveIt 2 and `ros2_control`'s `joint_trajectory_controller` use actions (`FollowJointTrajectory`).

---

## 11. Choosing between topic, service and action

| Question | Topic | Service | Action |
|---|---|---|---|
| Continuous stream of data? | **Yes** | No | No |
| Need a reply? | No | **Yes, one** | **Yes, at the end** |
| Takes long (> ~1 s)? | n/a | Avoid | **Yes** |
| Progress feedback? | n/a | No | **Yes** |
| Cancelable? | n/a | No | **Yes** |
| Number of receivers | Many | One server | One server |
| Arm example | `/joint_states`, `/image_raw` | `/power_on`, `/get_gripper_state` | `/move_to_pose`, `/pick` |

Rule of thumb: **data → topic, quick question/command → service, task → action.**

---

## 12. Parameters

**Parameters** are a node's configuration values: settable at startup and (optionally) changeable at runtime. They belong to a node; there is no global parameter server as in ROS 1.

- Types: `bool`, `int`, `double`, `string`, `byte[]`, and arrays of `bool`/`int`/`double`/`string`.
- A node must **declare** each parameter (name, default, optional descriptor with description, range, read-only flag) before using it.
- Set them from the command line (`--ros-args -p port:=/dev/ttyTHS1`), from YAML files (`--params-file config.yaml`), from launch files, or at runtime (`ros2 param set`).
- A node can register a callback to **validate or reject** changes at runtime.

```yaml
# config/arm_driver.yaml
arm_driver:
  ros__parameters:
    port: /dev/ttyTHS1
    baud: 1000000
    max_speed: 30
```

```bash
ros2 param list /arm_driver
ros2 param get /arm_driver max_speed
ros2 param set /arm_driver max_speed 20
ros2 param dump /arm_driver > arm_driver.yaml
```

For our arm, parameters are the place for the serial port, baud rate, speed limits and joint limits, so nothing hardware-specific is hard-coded.

---

## 13. Managed (lifecycle) nodes

A normal node starts doing its job the moment it is created. A **managed node** (lifecycle node) follows a **standard state machine**, so an external manager or a person can bring it up in a controlled order, pause it, and shut it down cleanly. This matters for hardware drivers: you want to open the serial port, check the arm, and only then start publishing and accepting commands.

### States and transitions

```
                          ┌───────────────┐
              create ───► │ Unconfigured  │ ◄──────────────┐
                          └──────┬────────┘                │
                      configure  │       ▲ cleanup         │
                                 ▼       │                 │
                          ┌───────────────┐                │
                          │   Inactive    │                │
                          └──────┬────────┘                │
                       activate  │       ▲ deactivate      │
                                 ▼       │                 │
                          ┌───────────────┐                │
                          │    Active     │                │
                          └───────────────┘                │
                                                           │
   shutdown (from Unconfigured, Inactive or Active) ──► Finalized ──► destroyed
   error in any transition ──► ErrorProcessing ──► on_error() ──┘ (success → Unconfigured, failure → Finalized)
```

**Primary states:** `Unconfigured`, `Inactive`, `Active`, `Finalized`.
**Transition states** (temporary, while your callback runs): `Configuring`, `CleaningUp`, `Activating`, `Deactivating`, `ShuttingDown`, `ErrorProcessing`.

### What to do in each callback

| Callback | Transition | Typical work for an arm driver | Returns |
|---|---|---|---|
| `on_configure` | Unconfigured → Inactive | Read parameters, open the serial port, create publishers/subscribers/timers, check firmware version | `SUCCESS` / `FAILURE` / `ERROR` |
| `on_activate` | Inactive → Active | Power on servos, activate lifecycle publishers, start the control timer | |
| `on_deactivate` | Active → Inactive | Stop motion, stop publishing; keep the port open | |
| `on_cleanup` | Inactive → Unconfigured | Close the port, destroy publishers and timers | |
| `on_shutdown` | any → Finalized | Release everything, leave the hardware safe | |
| `on_error` | ErrorProcessing → Unconfigured / Finalized | Try to recover; return `SUCCESS` to go back to Unconfigured | |

Return value meaning: `SUCCESS` completes the transition, `FAILURE` stays in the previous state, `ERROR` (or an exception) goes to `ErrorProcessing`.

### Lifecycle publishers

A lifecycle node creates **lifecycle publishers**. They only send messages while the node is `Active`; in other states `publish()` is a no-op (with a warning). This guarantees that an inactive driver never emits stale data.

### Controlling them

Every lifecycle node offers standard services (`<node>/change_state`, `<node>/get_state`, `<node>/get_available_transitions`, ...) and publishes transitions on `<node>/transition_event`. The CLI wraps them:

```bash
ros2 lifecycle nodes                     # which nodes are managed
ros2 lifecycle get /arm_driver           # current state
ros2 lifecycle list /arm_driver          # available transitions from here
ros2 lifecycle set /arm_driver configure
ros2 lifecycle set /arm_driver activate
ros2 lifecycle set /arm_driver deactivate
```

In launch files, transitions can be triggered automatically, and `nav2_lifecycle_manager`-style managers can bring up many nodes in order. `ros2_control` hardware interfaces follow the same lifecycle states, which is why the driver for our arm is planned as a lifecycle node (see [docker/README.md](../../docker/README.md)). Today `arm_driver` is still a plain node (see [ROS 2 in braco](#ros-2-in-braco)).

---

## 14. Executors, callbacks and threading

ROS 2 code is **callback driven**: you register functions (subscription callbacks, timer callbacks, service handlers) and an **executor** calls them when there is work.

- `rclcpp::spin(node)` / `rclpy.spin(node)` runs the default executor until shutdown.
- **SingleThreadedExecutor**: one callback at a time. Simple and safe; a slow callback delays all others.
- **MultiThreadedExecutor**: callbacks can run in parallel threads.
- **Callback groups** control what may run in parallel:
  - *MutuallyExclusive* (default): callbacks in the group never overlap.
  - *Reentrant*: callbacks in the group may run concurrently (even the same one).

Rules that prevent most bugs:

1. **Never block in a callback** (no `sleep`, no long loops, no synchronous service call). Use timers and asynchronous calls.
2. If a node must call a service and wait for the answer inside a callback, use a **MultiThreadedExecutor** and put the client in a **separate callback group**.
3. Protect shared state when using multiple threads (mutex in C++; in Python the GIL helps but logical races are still possible).
4. Action server execute callbacks are long by design; in Python they run in the executor, so long actions need a `MultiThreadedExecutor` if the node must keep serving other callbacks.

---

## 15. Names, namespaces and remapping

Names for nodes, topics, services, actions and parameters follow filesystem-like rules:

| Name written in code | Node namespace `/arm` | Resolves to |
|---|---|---|
| `joint_states` (relative) | `/arm` | `/arm/joint_states` |
| `/joint_states` (absolute) | `/arm` | `/joint_states` |
| `~/status` (private) | `/arm`, node `driver` | `/arm/driver/status` |

Use **relative names** in code so the same node can run several times under different namespaces (two arms, two cameras).

Anything can be renamed at startup without touching the code (**remapping**):

```bash
ros2 run demo_nodes_cpp talker --ros-args \
  -r __node:=my_talker \          # rename the node
  -r __ns:=/robot1 \              # put it in a namespace
  -r chatter:=/robot1/news        # remap a topic
```

Allowed characters: letters, digits, `_` and `/`. Names cannot start with a digit, and **cannot contain `-`** (a frequent mistake when deriving names from hostnames).

---

## 16. Launch files

Starting a system node by node in many terminals does not scale. **Launch files** start many nodes at once, with their parameters, remappings and namespaces, and can include other launch files.

ROS 2 launch files can be written in **Python** (most powerful), **XML** or **YAML**:

```python
# launch/basics.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='ros2_basics_cpp', executable='talker', name='talker',
             parameters=[{'publish_rate_hz': 2.0}]),
        Node(package='ros2_basics_py', executable='listener', name='listener',
             remappings=[('chatter', 'chatter')]),
    ])
```

```xml
<!-- launch/basics.launch.xml -->
<launch>
  <node pkg="ros2_basics_cpp" exec="talker" name="talker">
    <param name="publish_rate_hz" value="2.0"/>
  </node>
  <node pkg="ros2_basics_py" exec="listener" name="listener"/>
</launch>
```

```bash
ros2 launch ros2_basics_py basics.launch.py
ros2 launch ros2_basics_py basics.launch.py --show-args   # list launch arguments
```

Launch also supports **arguments** (`DeclareLaunchArgument`), **conditions** (`IfCondition`), **events** (react when a node exits or a lifecycle transition happens) and **lifecycle nodes** (`LifecycleNode` with automatic `configure`/`activate`).

---

## 17. The middleware: DDS, RMW and discovery

### Layers

```
 ┌──────────────────────────────────────────┐
 │ Your node (C++ / Python)                 │
 ├────────────────────┬─────────────────────┤
 │ rclcpp             │ rclpy               │   client libraries
 ├────────────────────┴─────────────────────┤
 │ rcl (C)                                  │   common logic: names, parameters, lifecycle...
 ├──────────────────────────────────────────┤
 │ rmw (ROS middleware interface)           │   an abstract API
 ├──────────────┬──────────────┬────────────┤
 │ rmw_fastrtps │ rmw_cyclone  │ rmw_zenoh  │   implementations (selectable)
 │ (Fast DDS)   │ (Cyclone DDS)│ (Zenoh)    │
 └──────────────┴──────────────┴────────────┘
              UDP / shared memory / TCP
```

**DDS** (Data Distribution Service) is an industry standard for publish/subscribe used in defence, aviation and automotive. ROS 2 maps topics, services and actions onto DDS. The **RMW** layer hides which DDS vendor is used, so you can switch implementation with an environment variable and no code change.

| Variable | Our value | Meaning |
|---|---|---|
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | Which middleware to use. Cyclone DDS (`rmw_cyclonedds_cpp`) is also installed in our images |
| `ROS_DOMAIN_ID` | `42` | Logical network partition (0 to 101 is safe). Only nodes with the **same** domain ID see each other |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET` | How far discovery reaches: `OFF`, `LOCALHOST`, `SUBNET`, `SYSTEM_DEFAULT` |
| `ROS_STATIC_PEERS` | unset | Extra IPs to contact directly when multicast is blocked (Wi-Fi networks often block it) |

**All machines that must talk to each other need the same `ROS_DOMAIN_ID` and, in practice, the same `RMW_IMPLEMENTATION`.** In our project both come from the `.env` file of the Docker setup.

### Discovery

There is no master. When a node starts, its DDS participant **announces itself via UDP multicast** on the local network. Other participants answer, exchange their lists of endpoints (topics, types, QoS), and matching publishers and subscribers connect directly to each other. Consequences:

- Two machines on the same network and domain see each other automatically.
- If multicast is blocked (campus Wi-Fi, some routers), discovery fails; set `ROS_STATIC_PEERS` or use a wired link.
- Firewalls must allow UDP between the machines.
- Docker containers need `network_mode: host` (we use it) for multicast to work across machines, and `ipc: host` for shared-memory transport between containers on the same host.
- With **Zenoh** (`rmw_zenoh_cpp`) discovery goes through a router (`ros2 run rmw_zenoh_cpp rmw_zenohd`) instead of multicast. It is installed in our images but is not a drop-in switch.

### The ros2 CLI daemon

`ros2` commands use a background **daemon** that caches discovery information so the CLI responds quickly. If `ros2 node list` shows stale or missing nodes, restart it: `ros2 daemon stop; ros2 daemon start`.

---

## 18. Time and clocks

ROS 2 has three clock types:

| Clock | Source | Use |
|---|---|---|
| `SYSTEM_TIME` | Wall clock (can jump with NTP) | Logging, timestamps for humans |
| `STEADY_TIME` | Monotonic clock | Measuring durations, timeouts |
| `ROS_TIME` (default for nodes) | Wall clock, **or** the `/clock` topic when `use_sim_time:=true` | Everything that must also work in simulation and bag replay |

When running with Gazebo or replaying a bag, set the parameter `use_sim_time:=true` on every node so they follow the simulated clock. Timestamps in message headers (`header.stamp`) should come from `node.get_clock().now()`.

Across machines (PC + Jetson), keep clocks synchronised (NTP / `chrony`), otherwise timestamps and tf2 lookups disagree.

---

## 19. Coordinate frames: tf2

A robot arm has many coordinate frames: the base, each link, the gripper, the camera, the object to pick. **tf2** keeps track of the transforms between them over time.

- Transforms are published on `/tf` (dynamic: joints moving) and `/tf_static` (fixed: camera mounted on the gripper).
- The frames form a **tree** (each frame has exactly one parent).
- Any node can ask "where is `object` in `base_link` coordinates, at time *t*?" and tf2 chains the transforms.
- `robot_state_publisher` reads the robot model (URDF) and `/joint_states`, and publishes the transforms of all links.

```bash
ros2 run tf2_ros tf2_echo base_link link6        # print a transform continuously
ros2 run tf2_tools view_frames                   # write frames.pdf with the tree
```

For our eye-in-hand camera, tf2 is what converts a detection in camera pixels/coordinates into a target pose for the arm.

---

## 20. Packages, workspaces and the build system

### Packages

A **package** is the unit of organisation and distribution: a folder with a `package.xml` manifest, containing nodes, libraries, launch files, configs or interfaces.

| Build type | Language | Files |
|---|---|---|
| `ament_cmake` | C++ (also required for interface packages) | `package.xml`, `CMakeLists.txt`, `src/`, `include/` |
| `ament_python` | Python | `package.xml`, `setup.py`, `setup.cfg`, `resource/<pkg>`, `<pkg>/` |

`package.xml` declares the name, version, license, maintainers and **dependencies** (`<depend>`, `<build_depend>`, `<exec_depend>`, `<test_depend>`). Dependencies are used by `colcon` to order the build and by `rosdep` to install system packages.

### Workspaces and overlays

A **workspace** is a folder with a `src/` directory containing packages:

```
ws/
├── src/        # your packages (source, version controlled)
├── build/      # intermediate build files      ─┐
├── install/    # built packages + setup scripts  ├─ generated by colcon, never committed
└── log/        # build logs                    ─┘
```

Workspaces are **layered**:

- The **underlay** is the ROS installation itself, activated with `source /opt/ros/lyrical/setup.bash`.
- Your workspace is an **overlay**, activated with `source install/setup.bash` after building. Packages in the overlay take precedence over the underlay.

Our containers source both automatically (`docker/scripts/env.sh`). After a build, run `source /ws/install/setup.bash` again (or open a new shell) to see new packages and executables.

### colcon, ament and rosdep

| Tool | Role |
|---|---|
| `colcon` | Builds all packages in a workspace in dependency order (`colcon build`), runs their tests (`colcon test`) |
| `ament` | The build system *inside* each package: CMake macros (`ament_cmake`) or setuptools integration (`ament_python`), plus linters (`ament_lint`) |
| `rosdep` | Reads `package.xml` dependencies and installs the matching apt/pip packages |

Our images configure `colcon` with `symlink-install`, `RelWithDebInfo` and `ccache` (`docker/config/colcon-defaults.yaml`). With `--symlink-install`, edits to Python files and launch/config files take effect without rebuilding.

---

## 21. Tools

| Tool | Command | What it is for |
|---|---|---|
| ros2 CLI | `ros2 <verb> ...` | Inspect and interact with everything: `node`, `topic`, `service`, `action`, `param`, `interface`, `lifecycle`, `bag`, `launch`, `run`, `pkg`, `doctor` |
| rqt_graph | `ros2 run rqt_graph rqt_graph` | Picture of nodes and topics |
| rqt | `rqt` | Plugin GUI: topic monitor, service caller, parameter editor, console, plot |
| RViz2 | `rviz2` | 3D visualisation: robot model, tf frames, camera images, point clouds, planned trajectories |
| rosbag2 | `ros2 bag record / play / info` | Record topics to a file and replay them later (MCAP storage is installed) |
| PlotJuggler 4 | `plotjuggler4` | Plot any numeric field over time, live or from bags (PC image) |
| ros2 doctor | `ros2 doctor --report` | Checks the setup and network, prints a report |
| Gazebo | via `ros_gz` | Physics simulation |

GUI tools (rqt, RViz2, Gazebo, PlotJuggler) are only in the **PC image** (`braco-image/pc`); the Jetson image is headless. The Jetson's topics are visible from the PC over the network, so you inspect the arm from the PC.

---

## 22. The ecosystem we will use for the arm

| Component | Purpose | Status in our images |
|---|---|---|
| **URDF / xacro** | XML description of the robot's links, joints, limits and geometry | `xacro`, `robot_state_publisher` installed |
| **robot_state_publisher** | Publishes tf from the URDF and `/joint_states` | Installed |
| **ros2_control** | Hardware abstraction: a *hardware interface* talks to the real arm (or the simulator), *controllers* (e.g. `joint_trajectory_controller`) compute commands. The same controllers run in simulation and on the real robot | Installed on both images |
| **MoveIt 2** | Motion planning, inverse kinematics, collision checking, and MoveIt Servo for real-time jogging | PC image |
| **Gazebo (Jetty, via `ros_gz`)** | Simulated arm with physics, with `gz_ros2_control` | PC image |
| **pymycobot** | Vendor Python library for the myCobot serial protocol | Installed on both images |
| **mycobot_ros2** (vendor) | Elephant Robotics' ROS 2 packages, built for Galactic | Not installed; may need porting to Lyrical (see ADR-0001) |

---

## 23. How this maps to our project

What runs **today** is described in [ROS 2 in braco](#ros-2-in-braco) at the top of this file. This section is the Sprint 0 plan: where the project is heading, not what the code does now (for example, `arm_driver` is a plain node in C++ over `mycobot_cpp`, not yet a lifecycle node).

```
 PC (dev + sim containers)                          Jetson Nano (real container)
 ┌────────────────────────────────────┐   DDS,      ┌────────────────────────────────────┐
 │ RViz2 · MoveIt 2 · Gazebo          │   domain 42 │ arm driver (lifecycle node,        │
 │ planning, visualisation, QA        │ ◄─────────► │   pymycobot over /dev/ttyTHS1)     │
 │ rqt_graph · PlotJuggler · bags     │   Fast DDS  │ ros2_control + controllers         │
 └────────────────────────────────────┘             │ camera node                        │
                                                    └────────────────────────────────────┘
```

- **Topics**: `/joint_states` from the driver, `/image_raw` from the camera, detections from the vision node.
- **Services**: quick arm commands (power on/off, gripper state, free-move mode).
- **Actions**: moving to a pose, following a trajectory, the pick-and-place task.
- **Lifecycle**: the arm driver, so the serial port is opened in `configure` and motion is only possible in `active`.
- **Parameters**: port, baud, speed and joint limits.
- **QoS**: `SensorDataQoS` for images, reliable for joint states and commands.

Sprint 0 ([issue #18](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/18)) builds each of these patterns once in isolation; [INSTRUCTIONS.md](INSTRUCTIONS.md) walks through all of them.

---

## 24. Glossary

| Term | Meaning |
|---|---|
| **Action** | Long-running, cancelable request with feedback and a result |
| **ament** | Build system used inside ROS 2 packages |
| **colcon** | Command-line tool that builds and tests a workspace |
| **Component** | A node compiled as a shared library, loadable into a shared process |
| **DDS** | Data Distribution Service, the standard pub/sub middleware under ROS 2 |
| **Discovery** | Automatic process by which nodes find each other on the network |
| **Distro** | Versioned release of ROS 2 (Galactic, Humble, Lyrical...) |
| **Domain ID** | Number that isolates groups of nodes on the same network |
| **Executor** | Object that runs a node's callbacks |
| **Interface** | A `.msg`, `.srv` or `.action` type definition |
| **Launch file** | Python/XML/YAML file that starts and configures many nodes |
| **Lifecycle node** | Node with the standard Unconfigured/Inactive/Active/Finalized states |
| **Node** | One participant in the ROS graph, doing one job |
| **Overlay / underlay** | A workspace layered on top of / below another |
| **Package** | Unit of code organisation with a `package.xml` |
| **Parameter** | Named configuration value owned by a node |
| **QoS** | Quality of Service policies that control delivery on topics |
| **Remapping** | Renaming topics/nodes/namespaces at startup without code changes |
| **RMW** | ROS Middleware interface; the pluggable layer over DDS/Zenoh |
| **rosdep** | Tool that installs the system dependencies of packages |
| **Service** | One request, one response |
| **tf2** | Library that tracks coordinate frames over time |
| **Topic** | Named, typed publish/subscribe channel |
| **URDF** | XML robot description (links, joints, limits) |

---

## 25. Further reading

- Official documentation (select the Lyrical version in the top-left menu): <https://docs.ros.org/en/lyrical/>
  - Concepts: <https://docs.ros.org/en/lyrical/Concepts.html>
  - Tutorials (beginner CLI, client libraries, intermediate): <https://docs.ros.org/en/lyrical/Tutorials.html>
  - About QoS settings: <https://docs.ros.org/en/lyrical/Concepts/Intermediate/About-Quality-of-Service-Settings.html>
- Managed nodes design article: <https://design.ros2.org/articles/node_lifecycle.html>
- Lifecycle demo (C++ and Python): <https://github.com/ros2/demos/tree/rolling/lifecycle>
- ROS 2 design articles (why things are the way they are): <https://design.ros2.org/>
- ros2_control: <https://control.ros.org/>
- MoveIt 2: <https://moveit.picknik.ai/>
- REP 2004 (package quality) and REP 144 (package naming): <https://ros.org/reps/rep-0000.html>
