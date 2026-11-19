# braco architecture

How the braco software is built today: layers, nodes, topics, packages and deployment. If the code and this document disagree, the code wins. CPU and memory: [resources.md](resources.md). Timing: [timing-budget.md](timing-budget.md).

Last checked against the code: **2026-11-19**.

**Contents:** [Principles](#1-principles) · [Layers](#2-layers) · [Nodes and devices](#3-nodes-and-devices) · [Topics and services](#4-topics-and-services) · [Control of the arm](#5-control-of-the-arm) · [Packages](#6-packages) · [Deployment](#7-deployment) · [Launch and parameters](#8-launch-and-parameters) · [MoveIt 2, simulation and the real arm](#9-moveit-2-simulation-and-the-real-arm) · [Status](#10-status) · [Changing the architecture](#11-changing-the-architecture)

---

## 1. Principles

- **One owner per device.** Only `arm_driver` opens the arm's serial port (`/dev/ttyTHS1`, 1 Mbaud); only `camera` opens the USB camera. Everyone else uses their topics and services.
- **The driver never moves the arm by itself.** Zero, park and power on are explicit commands. The one automatic move is the park on overheat ([ADR-0003](../decisions/ADR-0003-rest-per-temp.md)).
- **Safety lives next to the hardware.** Joint limits, table margin, watchdog, rest mode and control owner are enforced in `arm_driver`, not in clients.
- **One client moves the arm at a time.** Clients take the control before moving; STOP is always obeyed ([ADR-0002](../decisions/ADR-0002-unique-control.md)).
- **Faults are visible.** `arm_driver` is not respawned: if it dies, the container stops. `temperature_monitor`, `joystick_teleop` and `camera` are respawned.
- **One source for each fact.** The thermal level comes from `temperature_monitor` (`/diagnostics`); the control owner from `arm_driver`.
- **Same interface for real and simulated.** Everything above the hardware abstraction uses the same names and types. Only layer 3 changes with `sim:=true|false`.
- **Logic without ROS where possible.** Protocol, joystick, LCD and teleop logic live in `mycobot_cpp`. Control owner and auto-rest are plain headers in `braco_driver/include`. All unit-tested without ROS.
- **Every node that moves the arm is a lifecycle node.** *Target, not implemented yet* ([Status](#10-status)).

---

## 2. Layers

Each layer talks only to the one below it. Layers 1–2 are identical in simulation and on the real arm; layers 3–5 switch with `sim:=true|false`.

| Layer | Role | Real arm (`sim:=false`, Jetson) | Simulation (`sim:=true`, PC) |
|---|---|---|---|
| **1 Mission and clients** | Decide what the arm does; take the control before moving | `braco_web`, `joystick_teleop`, `temperature_monitor`, `zero` / `park`; `braco_mission` and MoveIt 2 (`move_group`, on the PC) | same nodes |
| **2 ROS 2 middleware** | The contract: names and types | DDS domain 22; `/joint_states`, `/arm/command`, `/arm/*`, `/diagnostics`; `FollowJointTrajectory` on `/arm_controller` | same names, each teammate's domain (20–29); `FollowJointTrajectory` on `/arm_controller` |
| **3 Hardware abstraction** | Device access and safety | `arm_driver`, `camera`, `mycobot_cpp`; `ros2_control_node` with `braco_hardware` | `gz_ros2_control` (`controller_manager`, `arm_controller`, `joint_state_broadcaster`, `gripper_action_controller`) |
| **4 Servo controller** | Drives the servo bus | Atom (ESP32, firmware 7.3) | Gazebo physics |
| **5 Sensors and actuators** | What moves and senses | 6 servos, gripper; camera, joystick, LCD1602 wired to the Jetson | simulated `joint1..joint6`, gripper, wrist camera |

---

## 3. Nodes and devices

```
                 ┌───────────────────────────────────────────┐
                 │ arm_driver   gates: 1 STOP · 2 rest ·     │  UART 1 Mbaud   ┌─────────────┐
                 │              3 owner · 4 watchdog 300 ms  │ ◄─────────────► │ myCobot 280 │
                 └─────────────────────┬─────────────────────┘                 └─────────────┘
                                       │                       ┌────────┐  V4L2  ┌────────────┐
                                       │                       │ camera │ ◄───── │ USB camera │
                                       │                       └───┬────┘        └────────────┘
 ══════════════════════════ ROS 2 · DDS · domain 22 ═══════════════╧══════════════════
        │                         │                          │
 ┌──────┴──────┐          ┌───────┴─────────┐       ┌────────┴────────────┐
 │ braco_web   │          │ joystick_teleop │       │ temperature_monitor │
 └──────┬──────┘          └───────┬─────────┘       └────────┬────────────┘
   HTTP/WS :8000          /dev/input/js0               I2C LCD1602
```

Also on the real arm (`sim:=false`), with no device of their own: `robot_state_publisher` (the URDF on `/robot_description`, `/tf` from `/joint_states`) and `ros2_control_node` with the `braco_hardware` plugin, which reads `/joint_states` from `arm_driver` and sends `arm_controller`'s targets on `/arm/command` as `mission` (ADR-0004), and `arm_controller` on it (no `joint_state_broadcaster`).

A motion command passes the `arm_driver` gates in order:

1. **STOP** is always obeyed.
2. **Rest mode** refuses motion.
3. **Owner:** motion from anyone but the control owner is ignored.
4. **Watchdog:** 300 ms without commands stops the arm.

---

## 4. Topics and services

| Name | Type | From | Used by |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState`, 20 Hz nominal (18.7 measured; ~14 under continuous commands, [timing](timing-budget.md)) | `arm_driver` | teleop, page, zero/park, `braco_hardware` (in `ros2_control_node`), `robot_state_publisher` |
| `/robot_description` | `std_msgs/String`, latched (the URDF, `real_arm:=true`) | `robot_state_publisher` | `ros2_control_node` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` | MoveIt, RViz, Foxglove |
| `/arm/command` | `braco_interfaces/ArmCommand` (`STOP`, `JOINTS`, `ALL_ANGLES`, `HALT`, `source`) | clients, `braco_hardware` | `arm_driver` |
| `/arm/led` | `std_msgs/ColorRGBA` | teleop, page | `arm_driver` |
| `/arm/servo_temperatures` | `braco_interfaces/ServoTemperatures`, 2 s | `arm_driver` | `temperature_monitor`, page |
| `/arm/servo_voltages` | `braco_interfaces/ServoVoltages`, 2 s | `arm_driver` | page |
| `/arm/read_only` | `std_msgs/Bool`, latched | `arm_driver` | page |
| `/arm/control_owner` | `std_msgs/String`, latched (`""` = free) | `arm_driver` | teleop, page, `braco_hardware` |
| `/arm/stopped` | `std_msgs/Bool`, latched: STOP latch | `arm_driver` | teleop, page, zero/park, `braco_hardware` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `temperature_monitor` | page |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage`, ≤15 fps, frame `camera_optical_frame` | `camera` (sim: Gazebo's wrist camera, `ros_gz_image`) | page |
| `/arm/take_control` | `braco_interfaces/TakeControl` | `arm_driver` | teleop, page, zero/park, `move_to_pose` (`real:=true`) |
| `/arm/power_on` | `std_srvs/Trigger` | `arm_driver` | teleop, page, zero/park |
| `/arm/release_servos` | `std_srvs/Trigger` | `arm_driver` | page, zero/park |
| `/arm/release_stop` | `std_srvs/Trigger` | `arm_driver` | page, `make resume` |
| `/arm/set_gripper`, `/arm/set_read_only` | `std_srvs/SetBool` | `arm_driver` | clients, `temperature_monitor` |
| `/move_to_pose` (action) | `braco_interfaces/MoveToPose` (`PoseStamped` goal; result: pose error from TF) | `move_to_pose` (`braco_mission`) | command line |
| `/move_cartesian` (action) | `braco_interfaces/MoveCartesian` (waypoints; result: planned `fraction`, line deviation, error at the last waypoint) | `move_to_pose` (`braco_mission`) | command line, `draw_line.py`; later pen drawing |

- **Latest wins:** a new command replaces the pending one, so commands never queue up. Clients stream short moves only while `/joint_states` is fresh.
- **Limits:** angles are clamped to `JOINT_LIMITS` (`mycobot.hpp`). Poses below the table margin are refused.

---

## 5. Control of the arm

Details: [ADR-0002](../decisions/ADR-0002-unique-control.md) (owner), [ADR-0003](../decisions/ADR-0003-rest-per-temp.md) (overheat), [ADR-0005](../decisions/ADR-0005-stop-latch.md) (STOP latch).

| Rule | |
|---|---|
| Owner | `web`, `joystick`, `script` (zero/park) or `mission`; `""` = free |
| Take | `/arm/take_control`: if free, already owned, or `force` (stops the arm) |
| Motion | only from the owner |
| STOP | always, from anyone; latches and frees the control until `/arm/release_stop` (from 3 s after it) |
| HALT | routine stop, from anyone; no latch, the owner keeps the control |
| Lose | `release`, forced away, 5 s without motion, or a STOP |

| Client | Takes | Releases |
|---|---|---|
| Web page | with its lease; **409** while another client owns the arm | when the lease ends |
| Joystick | first input if free; **hold button 8 for 1 s** to force | 5 s after the last move |
| zero / park | forced | at the end, also on failure |

**Overheat:** at ≥ 65 °C, `temperature_monitor` runs `park` once per episode: the arm folds, the joints are released and rest mode turns on. The episode ends when every joint is below 60 °C. Resuming is manual (`make resume`, [overheating guide](../guides/overheating.md)).

---

## 6. Packages

One colcon workspace, `ros2_ws/src`, mounted at `/ws`. Each package has a README.

| Package | Role | Status |
|---|---|---|
| [`mycobot_cpp`](../../ros2_ws/src/mycobot_cpp/README.md) | Serial protocol, joint limits, joystick, LCD, teleop logic (no ROS) | done |
| [`braco_interfaces`](../../ros2_ws/src/braco_interfaces/README.md) | `ArmCommand`, `JointTarget`, `ServoTemperatures`, `ServoVoltages`, `TakeControl`, `MoveToPose` and `MoveCartesian` (actions) | done |
| [`braco_driver`](../../ros2_ws/src/braco_driver/README.md) | `arm_driver`, `temperature_monitor` | done |
| [`braco_hardware`](../../ros2_ws/src/braco_hardware/README.md) | ros2_control plugin of the real arm: joints from `arm_driver`'s `/joint_states`, targets to `/arm/command` as `mission` (ADR-0004) | reads and writes |
| [`braco_teleop`](../../ros2_ws/src/braco_teleop/README.md) | `joystick_teleop` | done |
| [`braco_camera`](../../ros2_ws/src/braco_camera/README.md) | `camera` (Python) | done |
| [`braco_bringup`](../../ros2_ws/src/braco_bringup/README.md) | `braco.launch.py`, `braco.yaml`, entrypoint, `zero` / `park` | done |
| [`braco_description`](../../ros2_ws/src/braco_description/README.md) | Robot model: URDF/xacro, meshes, `controllers.yaml` and `controllers_real.yaml` (no Gazebo, so also on the Jetson) | done |
| [`braco_simulation`](../../ros2_ws/src/braco_simulation/README.md) | Gazebo launch and the scripts that move the simulated arm (PC only) | done |
| [`braco_moveit_config`](../../ros2_ws/src/braco_moveit_config/README.md) | MoveIt 2 config (SRDF, kinematics, joint limits, controllers), demo with mock hardware, launches on the Gazebo arm and the real arm | done |
| [`braco_mission`](../../ros2_ws/src/braco_mission/README.md) | `move_to_pose`: a Cartesian pose goal and straight lines through MoveIt 2 (C++); later the mission state machine | `move_to_pose` done |

```
mycobot_cpp   braco_interfaces
     │    ╲   ╱        │
     ▼     ╲ ╱         ▼
braco_driver ╳   braco_teleop
     ▲      ╱ ╲        ▲
     └─ braco_bringup ─┘
            │
     ┌──────┼───────────────┐
     ▼      ▼               ▼
braco_camera braco_hardware braco_description ◄┄ braco_simulation, braco_moveit_config ┄► braco_mission
                                                 (simulation/planning only: never a dependency of braco_bringup)
```

`braco_hardware`, `braco_mission` and `braco_simulation` also depend on `braco_interfaces` (not drawn).

`braco_bringup` declares as `exec_depend` everything the real arm needs. The Jetson builds only `--packages-up-to braco_bringup`, so a simulation package must never be one of its dependencies. The robot model (`braco_description`) has no Gazebo, so the real arm can depend on it; the Gazebo launch lives in `braco_simulation`, which it never depends on.

**Naming:** packages are named `braco_<role>`. File, executable and node share one name. Arm topics and services live under `/arm/*`.

---

## 7. Deployment

| | PC | Jetson Nano |
|---|---|---|
| Image | `braco-image/pc` | `braco-image/jetson` |
| `dev` | stays up: shell, build, test, RViz2, PlotJuggler, web page, `make moveit` (`move_group` for the real arm) | throw-away (`run --rm`): build, shell |
| System | `sim` (optional, `SIM=1`): Gazebo + controllers + `move_group` (`SIM_MOVEIT=true`, the default) | `real`: `arm_driver`, `temperature_monitor`, `joystick_teleop`, `camera`, `robot_state_publisher`, `ros2_control_node`; `make zero` / `park` / `resume` run in it |
| ROS domain | per teammate, 20–29 (27 if unlisted; 28 the `braco_hardware` test, 29 the MoveIt launch test) | 22 |
| Page source | `ARM_SOURCE=gazebo` | `ARM_SOURCE=ros` |

- Same image family (ROS 2 Lyrical, Ubuntu 26.04) and workspace on both machines. `dev` builds; the system container runs what was built.
- **Heavy work stays on the PC:** MoveIt 2 and the mission run there and reach the arm over DDS.
- `make up` starts the system container. If the launch ends, the container stops. `make web` starts the page in the running container.
- Each PC uses its teammate's domain (`PC_ROS_DOMAIN_ID`, set by `make setup` from the GitHub login, see the
  [table](../../docker/README.md#ros-domain-per-teammate)) so its `/joint_states` never mix with the real arm's or
  another PC's. To reach the arm from the PC: `ROS_DOMAIN_ID=22 ros2 ...`.
  All are in the 0–101 range that is safe on Linux
  ([ROS 2 docs, *The ROS_DOMAIN_ID*](https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/nodes/About-Domain-ID.rst)).
- Gazebo transport is kept local (`GZ_IP=127.0.0.1`).
- The Jetson also hosts a self-hosted CI runner, `jetson-braco` (user `gh-runner`, label `braco-arm`), outside the containers. It runs only the manual `hil` job, which never opens the serial port: [HIL runner](../guides/ci.md#hil-runner).

---

## 8. Launch and parameters

```bash
ros2 launch braco_bringup braco.launch.py              # real arm (sim:=false)
ros2 launch braco_bringup braco.launch.py sim:=true    # simulation
```

Devices come from launch arguments (filled from `docker/.env`); tuning comes from [braco.yaml](../../ros2_ws/src/braco_bringup/config/braco.yaml).

| Argument | Default |
|---|---|
| `sim` / `gui` | `false` / `false` |
| `moveit` / `rviz` (`sim:=true` only) | `false` / `false` |
| `port`, `baud` | `/dev/ttyTHS1`, `1000000` |
| `lcd_bus` | `/dev/i2c-0` |
| `device` (joystick) | `/dev/input/js0` |
| `camera` | `/dev/video0` |

Key parameters: `command_timeout_ms` 300 (watchdog), `control_idle_s` 5.0, `state_rate_hz` 20, `auto_rest_on_hot` / `park_on_hot` true, `force_button` 8 / `force_hold_s` 1.0.

---

## 9. MoveIt 2, simulation and the real arm

The mission plans through MoveIt 2 and never opens the serial port, so the same code runs on both arms. Only what
sits under `arm_controller` changes: Gazebo in simulation, `arm_driver` on the real arm
([ADR-0004](../decisions/ADR-0004-trajectories-through-arm-driver.md)).

```
Simulation: PC, sim container, the teammate's domain (20–29)

 braco_mission ──goal──► move_group ──FollowJointTrajectory──► arm_controller ──► gz_ros2_control ──► Gazebo
  move_to_pose           (MoveIt 2)

Real arm (ADR-0004): move_group on the PC, the arm on the Jetson, both on domain 22

                 PC                    ║                         Jetson
 braco_mission ──goal──► move_group    ║
  real:=true             (MoveIt 2,    ║
  (use_control)           scaling 0.1) ║
     │                       │         ║  FollowJointTrajectory
     │                       └─────────╫──► arm_controller ──► BracoSystem (braco_hardware)
     │                                 ║                            │ ALL_ANGLES, source "mission",
     │                                 ║                            ▼ 25 Hz while mission owns the arm
     └── /arm/take_control("mission") ─╫───────────────────────► arm_driver ──UART──► myCobot 280
                                       ║                gates: STOP · rest · owner · watchdog 300 ms
```

**Planning (both arms)**

- `move_to_pose` (`braco_mission`) takes a `PoseStamped` on the `/move_to_pose` action, plans the `tcp` with `MoveGroupInterface` (position only as a fallback), executes on `arm_controller` and reports the pose error read from TF. The same node serves `/move_cartesian`: straight lines of the `tcp` through waypoints with `computeCartesianPath`, re-timed (TOTG) at the 0.1 velocity scaling of ADR-0004; nothing moves unless the whole path is planned (`fraction` 1.0 by default). Its steps are plain C++ behind an `ArmBackend` interface, unit tested without ROS: [braco_mission](../../ros2_ws/src/braco_mission/README.md).
- `braco_moveit_config` plans for the group `arm` (joint1..joint6, for `tcp`) and sends to `arm_controller`
  (`FollowJointTrajectory`), the controller name of `braco_description/config/controllers.yaml`. Its joint limits are
  `JOINT_LIMITS`; its named states `home`/`park` are `ZERO_ANGLES`/`PARK_ANGLES` of `mycobot.hpp`. Velocity and
  acceleration scaling default to 0.1 (`joint_limits.yaml`, ADR-0004). Its demo runs the same controllers on
  `mock_components` (no Gazebo, no arm): [Run the demo](../../ros2_ws/src/braco_moveit_config/README.md#run-the-demo).
- Every mesh declares the COLLADA unit 1 and the URDF scales the millimetre ones: MoveIt ignores the unit and would
  otherwise check collisions on meshes 1000x too big.
- The URDF (`braco_description`) uses the joint names of `/joint_states` and the limits of `JOINT_LIMITS`.
- Sim and real arm share one controller file (`braco_description/config/controllers.yaml`): `arm_controller` on both,
  `joint_state_broadcaster` and `gripper_action_controller` only in simulation. Only the hardware plugin in the URDF
  differs: `GazeboSimSystem` in simulation, `braco_hardware/BracoSystem` on the real arm (`real_arm:=true`). On the
  real arm the spawner also loads `controllers_real.yaml`, which adds the velocity command to `arm_controller` for
  the servo speed (the MoveIt mock and Gazebo accept one command interface per joint).
- A goal the arm does not reach fails: `arm_controller` aborts it when a joint falls more than 0.3 rad behind the trajectory (`PATH_TOLERANCE_VIOLATED`) or is more than 0.07 rad from its last point 0.5 s after its end (`GOAL_TOLERANCE_VIOLATED`), and MoveIt returns `CONTROL_FAILED`, so the mission sees a blocked or stalled arm (`constraints` in `controllers.yaml`).
- Before `move_group` starts, the domain check runs and a FAIL stops the launch: `--target sim` in simulation, `--target real` on the real arm. A teammate's shell on the wrong domain, or a second arm on 22, never gets a trajectory.

**Simulation**

- `braco.launch.py sim:=true moveit:=true` adds `move_group` once `arm_controller` is active, on simulated time. The `sim` container runs it by default (`SIM_MOVEIT`); RViz opens from `dev` (`make rviz-gui`) and reaches it over DDS: [Run on the Gazebo arm](../../ros2_ws/src/braco_moveit_config/README.md#run-on-the-gazebo-arm).
- `sim:=true` runs in the lab bench scene (`braco_simulation/worlds/lab_bench.sdf`). Its `world` frame stays the table surface: only the spawn pose places the arm, so TF, MoveIt and the table-clearance check do not change. MoveIt plans around the table: `sim.launch.py` puts the table tops of the same scene into its planning scene (`braco_moveit_config/scripts/lab_bench_scene.py`), so a pose in the table is refused before the arm moves: [The lab bench](../../ros2_ws/src/braco_simulation/README.md#the-lab-bench).
- The gripper moves: `gazebo.launch.py` builds the model with `gripper_actuated:=true` and adds `gripper_action_controller` (`/gripper_action_controller/gripper_cmd`, `ParallelGripperCommand`), outside MoveIt's `arm` group. The world runs Bullet Featherstone, the physics engine that enforces the gripper's five mimic joints. On the real arm the fingers stay fixed: the driver has no gripper state.
- The wrist camera renders too: a camera sensor at `camera_link` (`braco.gazebo.xacro`), 640×480 at 15 Hz like the real one, rendered by the world's Sensors system. `gazebo.launch.py` bridges it to the real camera's topic, `/camera/image_raw/compressed` (plus `/camera/image_raw` and `/camera/camera_info`), in `camera_optical_frame`, so the page shows it unchanged. Its field of view (60°) is not measured yet.
- How to run it: [Run the simulation](../../ros2_ws/src/braco_simulation/README.md#run-it).

**Real arm** ([ADR-0004](../decisions/ADR-0004-trajectories-through-arm-driver.md), checklist: [safety.md §9](../guides/safety.md#9-first-moveit-2-motion-on-the-real-arm))

- On the Jetson, `braco.launch.py sim:=false` (`make up`) starts `ros2_control_node` with `BracoSystem` and spawns `arm_controller`. `joint_state_broadcaster` is not spawned: `arm_driver` keeps publishing `/joint_states`.
- On the PC (the Jetson image has no MoveIt), on the arm's domain: `make moveit` runs `braco_moveit_config`'s `real.launch.py` (domain check, then `move_group` and RViz): [Run on the real arm](../../ros2_ws/src/braco_moveit_config/README.md#run-on-the-real-arm). `move_to_pose.launch.py real:=true` runs the mission with the real arm's model, the system clock and `use_control`: [Run it on the real arm](../../ros2_ws/src/braco_mission/README.md#run-it-on-the-real-arm).
- **Control:** with `use_control`, `braco_mission` calls `take_control("mission")` without `force` before each goal; refused → nothing is sent, nothing is queued. It releases the control after the result, also on failure. The plugin never takes the control.
- **Sending:** `BracoSystem` `write()` sends `arm_controller`'s target as `ALL_ANGLES` with `source: "mission"` at 25 Hz, at the servo speed of the fastest joint's planned velocity ([braco_hardware](../../ros2_ws/src/braco_hardware/README.md#servo-speed)), only while `arm_controller` is active and `mission` owns the arm, so trajectories pass the same gates as every client. Once the target stops changing it keeps sending it for 2 s so the arm gets there, then sends nothing: 300 ms later the watchdog stops the arm (already there), and a control `braco_mission` could not release is freed after 5 s. `arm_driver` clamps to `JOINT_LIMITS`; the plugin does not.
- **Stopping:** while `arm_controller` is commanding, a STOP from any client (seen on the latched `/arm/stopped`), a forced take (page or joystick) or the park on overheat makes `write()` return `DEACTIVATE`: `arm_controller` stops, the goal is aborted and the plugin goes inactive. After a STOP the latch ([ADR-0005](../decisions/ADR-0005-stop-latch.md)) refuses `take_control("mission")` until `make resume`, so no goal runs before a person releases it. `braco_mission` turns `BracoSystem` and `arm_controller` back on at its next goal, right after taking the control (no new configure). By hand: [braco_hardware](../../ros2_ws/src/braco_hardware/README.md#when-it-sends).
- **Timing** ([timing-budget.md](timing-budget.md)): setpoints reach the arm at ~14 Hz and motion starts ~200 ms after a command.
- **Resources** ([resources.md](resources.md)): `move_group` and the mission run on the PC, so the Jetson adds only `robot_state_publisher` and `ros2_control_node` with `arm_controller`. The Jetson figures are still estimates.

---

## 10. Status

**Done:** single serial owner · one control owner · STOP latch ([ADR-0005](../decisions/ADR-0005-stop-latch.md)) · one thermal authority · rest on overheat · `braco_*` layout with READMEs · one Makefile (`mk/`) · simulation · validated URDF ([report](../hardware/urdf-sim-to-real.md)) · trajectories through `arm_driver` ([ADR-0004](../decisions/ADR-0004-trajectories-through-arm-driver.md)) · `braco_moveit_config`: MoveIt 2 config, Gazebo and real-arm launches · `braco_mission`: `/move_to_pose` and `/move_cartesian`, control held as `mission` on the real arm · `braco_hardware` `SystemInterface`: `read()` and `write()` · `arm_controller` on the real arm · domain check before MoveIt · timing budget ([timing-budget.md](timing-budget.md)).

**Differs from the target (accepted):**

| Target | Today |
|---|---|
| `mycobot_cpp/{include,src}` | `{includes,srcs}` |
| `teleop_logic.hpp` in `braco_teleop` | in `mycobot_cpp` (stays ROS-free) |
| `braco_moveit` | `braco_moveit_config` (MoveIt convention) |
| Executables named like their nodes | `zero` / `park` run as `arm_zero` / `arm_park` |
| Lifecycle motion nodes | plain nodes |
| Dedicated e-stop node | software STOP on `/arm/command`, latched in `arm_driver` until `make resume` (gate 1, [ADR-0005](../decisions/ADR-0005-stop-latch.md)); no wired e-stop |

**Pending:**
- `arm_driver` as a lifecycle node
- `braco_mission` beyond `move_to_pose` and `move_cartesian`: the mission state machine
- Measured CPU/RAM on the Jetson ([resources.md](resources.md))

---

## 11. Changing the architecture

**New package:**
1. Name it `braco_<role>` and put it in `ros2_ws/src`. ROS-free code goes in `mycobot_cpp`.
2. Add a README with its topics, services and parameters.
3. New messages or services go in `braco_interfaces`.
4. Needed on the real arm: add it as an `exec_depend` of `braco_bringup`. Simulation or planning only: keep it out.

**Keeping this document current:** in the same PR as the code change, update the matching section and the "Last checked" date:

| Change | Section |
|---|---|
| A node, or what it runs on | §2 Layers, §3 Nodes and devices |
| A topic, service or action | §4 Topics and services |
| Control owner, STOP or overheat rules | §5 Control of the arm |
| A package, a folder, or a dependency between packages | §6 Packages |
| Containers, images, ROS domains, `make` commands | §7 Deployment |
| Launch arguments or parameters | §8 Launch and parameters |
| MoveIt 2, simulation or the real-arm path | §9 MoveIt 2, simulation and the real arm |
| Something done, pending or differing from the target | §10 Status |

---

Why Docker: [ADR-0001](../decisions/ADR-0001-ros2-in-docker-on-jetson-nano.md) · Containers: [docker/README.md](../../docker/README.md) · ROS 2: [guides/ROS2](../guides/ROS2/README.md)
