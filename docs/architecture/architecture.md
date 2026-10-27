# braco architecture

How the braco software is built today: layers, nodes, topics, packages and deployment. If the code and this document disagree, the code wins. CPU and memory: [resources.md](resources.md).

Last checked against the code: **2026-10-27**.

**Contents:** [Principles](#1-principles) · [Layers](#2-layers) · [Nodes and devices](#3-nodes-and-devices) · [Topics and services](#4-topics-and-services) · [Control of the arm](#5-control-of-the-arm) · [Packages](#6-packages) · [Deployment](#7-deployment) · [Launch and parameters](#8-launch-and-parameters) · [Simulation and planned layers](#9-simulation-and-planned-layers) · [Status](#10-status) · [Changing the architecture](#11-changing-the-architecture)

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
| **1 Mission and clients** | Decide what the arm does; take the control before moving | `braco_web`, `joystick_teleop`, `temperature_monitor`, `zero` / `park`; planned: `braco_mission`, MoveIt 2 | same nodes |
| **2 ROS 2 middleware** | The contract: names and types | DDS domain 42; `/joint_states`, `/arm/command`, `/arm/*`, `/diagnostics` | same names, domain 77; `FollowJointTrajectory` on `/arm_controller` |
| **3 Hardware abstraction** | Device access and safety | `arm_driver`, `camera`, `mycobot_cpp` | `gz_ros2_control` (`controller_manager`, `arm_controller`, `joint_state_broadcaster`) |
| **4 Servo controller** | Drives the servo bus | Atom (ESP32, firmware 7.3) | Gazebo physics |
| **5 Sensors and actuators** | What moves and senses | 6 servos, gripper; camera, joystick, LCD1602 wired to the Jetson | simulated `joint1..joint6`, gripper |

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
 ══════════════════════════ ROS 2 · DDS · domain 42 ═══════════════╧══════════════════
        │                         │                          │
 ┌──────┴──────┐          ┌───────┴─────────┐       ┌────────┴────────────┐
 │ braco_web   │          │ joystick_teleop │       │ temperature_monitor │
 └──────┬──────┘          └───────┬─────────┘       └────────┬────────────┘
   HTTP/WS :8000          /dev/input/js0               I2C LCD1602
```

A motion command passes the `arm_driver` gates in order:

1. **STOP** is always obeyed.
2. **Rest mode** refuses motion.
3. **Owner:** motion from anyone but the control owner is ignored.
4. **Watchdog:** 300 ms without commands stops the arm.

---

## 4. Topics and services

| Name | Type | From | Used by |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState`, 20 Hz | `arm_driver` | teleop, page, zero/park |
| `/arm/command` | `braco_interfaces/ArmCommand` (`STOP`, `JOINTS`, `ALL_ANGLES`, `source`) | clients | `arm_driver` |
| `/arm/led` | `std_msgs/ColorRGBA` | teleop, page | `arm_driver` |
| `/arm/servo_temperatures` | `braco_interfaces/ServoTemperatures`, 2 s | `arm_driver` | `temperature_monitor`, page |
| `/arm/servo_voltages` | `braco_interfaces/ServoVoltages`, 2 s | `arm_driver` | page |
| `/arm/read_only` | `std_msgs/Bool`, latched | `arm_driver` | page |
| `/arm/control_owner` | `std_msgs/String`, latched (`""` = free) | `arm_driver` | teleop, page |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `temperature_monitor` | page |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage`, ≤15 fps | `camera` | page |
| `/arm/take_control` | `braco_interfaces/TakeControl` | `arm_driver` | teleop, page, zero/park |
| `/arm/power_on`, `/arm/release_servos` | `std_srvs/Trigger` | `arm_driver` | teleop, page, zero/park |
| `/arm/set_gripper`, `/arm/set_read_only` | `std_srvs/SetBool` | `arm_driver` | clients, `temperature_monitor` |

- **Latest wins:** a new command replaces the pending one, so commands never queue up. Clients stream short moves only while `/joint_states` is fresh.
- **Limits:** angles are clamped to `JOINT_LIMITS` (`mycobot.hpp`). Poses below the table margin are refused.

---

## 5. Control of the arm

Details: [ADR-0002](../decisions/ADR-0002-unique-control.md) (owner), [ADR-0003](../decisions/ADR-0003-rest-per-temp.md) (overheat).

| Rule | |
|---|---|
| Owner | `web`, `joystick`, `script` (zero/park) or `mission`; `""` = free |
| Take | `/arm/take_control`: if free, already owned, or `force` (stops the arm) |
| Motion | only from the owner |
| STOP | always, from anyone |
| Lose | `release`, forced away, or 5 s without motion |

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
| [`braco_interfaces`](../../ros2_ws/src/braco_interfaces/README.md) | `ArmCommand`, `JointTarget`, `ServoTemperatures`, `ServoVoltages`, `TakeControl` | done |
| [`braco_driver`](../../ros2_ws/src/braco_driver/README.md) | `arm_driver`, `temperature_monitor` | done |
| [`braco_teleop`](../../ros2_ws/src/braco_teleop/README.md) | `joystick_teleop` | done |
| [`braco_camera`](../../ros2_ws/src/braco_camera/README.md) | `camera` (Python) | done |
| [`braco_bringup`](../../ros2_ws/src/braco_bringup/README.md) | `braco.launch.py`, `braco.yaml`, entrypoint, `zero` / `park` | done |
| [`braco_description`](../../ros2_ws/src/braco_description/README.md) | URDF, Gazebo launch, sim-to-real check | done |
| [`braco_moveit_config`](../../ros2_ws/src/braco_moveit_config/README.md) | MoveIt 2 config | placeholder |
| [`braco_mission`](../../ros2_ws/src/braco_mission/README.md) | Mission state machine | placeholder |

```
mycobot_cpp   braco_interfaces
     │    ╲   ╱        │
     ▼     ╲ ╱         ▼
braco_driver ╳   braco_teleop       braco_description ┄► braco_moveit_config ┄► braco_mission
     ▲      ╱ ╲        ▲            (simulation/planning only: never a dependency of braco_bringup)
     └─ braco_bringup ─┘
            │
            ▼
      braco_camera
```

`braco_bringup` declares as `exec_depend` everything the real arm needs. The Jetson builds only `--packages-up-to braco_bringup`, so a simulation package must never be one of its dependencies.

**Naming:** packages are named `braco_<role>`. File, executable and node share one name. Arm topics and services live under `/arm/*`.

---

## 7. Deployment

| | PC | Jetson Nano |
|---|---|---|
| Image | `braco-image/pc` | `braco-image/jetson` |
| `dev` | stays up: shell, build, test, RViz2, PlotJuggler, web page | throw-away (`run --rm`): build, zero, park |
| System | `sim` (optional, `SIM=1`): Gazebo + controllers | `real`: `arm_driver`, `temperature_monitor`, `joystick_teleop`, `camera` |
| ROS domain | 77 | 42 |
| Page source | `ARM_SOURCE=gazebo` | `ARM_SOURCE=ros` |

- Same image family (ROS 2 Lyrical, Ubuntu 26.04) and workspace on both machines. `dev` builds; the system container runs what was built.
- **Heavy work stays on the PC:** MoveIt 2 and the mission run there and reach the arm over DDS.
- `make up` starts the system container. If the launch ends, the container stops. `make web` starts the page in the running container.
- The PC uses domain 77 so its `/joint_states` never mix with the real arm's. To reach the arm from the PC: `ROS_DOMAIN_ID=42 ros2 ...`.
- Gazebo transport is kept local (`GZ_IP=127.0.0.1`).

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
| `port`, `baud` | `/dev/ttyTHS1`, `1000000` |
| `lcd_bus` | `/dev/i2c-0` |
| `device` (joystick) | `/dev/input/js0` |
| `camera` | `/dev/video0` |

Key parameters: `command_timeout_ms` 300 (watchdog), `control_idle_s` 5.0, `state_rate_hz` 20, `auto_rest_on_hot` / `park_on_hot` true, `force_button` 8 / `force_hold_s` 1.0.

---

## 9. Simulation and planned layers

```
braco_mission ┄► MoveIt 2 ──trajectory──┬──► real: arm_driver   (path to decide)
   planned                              └──► sim: Gazebo + gz_ros2_control
```

- The mission talks only to MoveIt 2, so the same code runs on real and simulated arms.
- The URDF (`braco_description`) uses the joint names of `/joint_states` and the limits of `JOINT_LIMITS`.
- `sim:=true` runs the controllers of `braco_description/config/controllers.yaml`, the set planned for the real arm. On the real arm only the URDF's hardware plugin changes (a braco `SystemInterface`, planned).
- How to run it: [Run the simulation](../../ros2_ws/src/braco_description/README.md#run-the-simulation-step-by-step).

---

## 10. Status

**Done:** single serial owner · one control owner · one thermal authority · rest on overheat · `braco_*` layout with READMEs · one Makefile (`mk/`) · simulation (#34) · validated URDF ([report](../hardware/urdf-sim-to-real.md), #25).

**Differs from the target (accepted):**

| Target | Today |
|---|---|
| `mycobot_cpp/{include,src}` | `{includes,srcs}` |
| `teleop_logic.hpp` in `braco_teleop` | in `mycobot_cpp` (stays ROS-free) |
| `braco_moveit` | `braco_moveit_config` (MoveIt convention) |
| Executables named like their nodes | `zero` / `park` run as `arm_zero` / `arm_park` |
| Lifecycle motion nodes | plain nodes |
| Dedicated e-stop node | STOP on `/arm/command` (gate 1) |

**Pending:**
- `arm_driver` as a lifecycle node
- E-stop node
- ros2_control `SystemInterface` for the real arm, and how trajectories reach `arm_driver`
- `braco_moveit_config`
- `braco_mission`
- Measured CPU/RAM ([resources.md](resources.md))

---

## 11. Changing the architecture

**New package:**
1. Name it `braco_<role>` and put it in `ros2_ws/src`. ROS-free code goes in `mycobot_cpp`.
2. Add a README with its topics, services and parameters.
3. New messages or services go in `braco_interfaces`.
4. Needed on the real arm: add it as an `exec_depend` of `braco_bringup`. Simulation or planning only: keep it out.

**Keep this document current:** in the same PR as the code change, update the matching section and the "Last checked" date.

---

Why Docker: [ADR-0001](../decisions/ADR-0001-ros2-in-docker-on-jetson-nano.md) · Containers: [docker/README.md](../../docker/README.md) · ROS 2: [guides/ROS2](../guides/ROS2/README.md)
