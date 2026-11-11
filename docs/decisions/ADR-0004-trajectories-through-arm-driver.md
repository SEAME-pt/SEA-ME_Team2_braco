---
status: accepted
date: 2026-11-03
deciders: all
---

# MoveIt trajectories reach the real arm through arm_driver

## Context

MoveIt 2 sends `FollowJointTrajectory` goals to `arm_controller` (`JointTrajectoryController`, 100 Hz). In simulation, `gz_ros2_control` moves the joints. On the real arm, `arm_driver` is the only process that opens the serial port, and it holds every safety rule: the STOP, rest, owner and watchdog gates, the control lease (ADR-0002) and rest on overheat (ADR-0003). It also publishes `/joint_states` at 20 Hz (18.7 Hz measured idle, ~14 Hz under continuous commands, [timing budget](../architecture/timing-budget.md)) and stops publishing when angle feedback fails, which clients read as "stale".

**Question:** how does a trajectory from `arm_controller` reach the real arm without bypassing `arm_driver`'s safety rules, and who holds the control, publishes `/joint_states` and stops the trajectory?

## Drivers

- One serial owner: `arm_driver`
- Trajectories pass the same gates as every other client (STOP, rest, owner, watchdog, `JOINT_LIMITS`)
- STOP on `/arm/command` halts a running trajectory
- An overheating servo still parks the arm (ADR-0003)
- Exactly one `/joint_states` publisher (#97)
- Sim and real use the same controllers; only the hardware plugin changes
- The web page and joystick keep working

## Options

| Option | Pros | Cons |
|---|---|---|
| **`SystemInterface` that talks to `arm_driver` over topics** | All gates, the lease and `park` still apply; page and joystick unchanged; only the hardware plugin differs from sim | One extra hop; the controller does not see a refused command by itself; ROS subscriptions inside the plugin |
| `SystemInterface` that owns the serial port | No extra hop; plain ros2_control layout | `arm_driver` cannot run: no gates, lease, STOP from `/arm/command`, `park`, page or joystick; every safety rule rewritten in the plugin |
| Bridge node from `arm_controller` to `/arm/command` | No ROS code inside a hardware plugin; gates still apply | New process; real arm runs mock hardware, unlike sim; controller state is commanded, not measured |

## Decision

**`SystemInterface` that talks to `arm_driver` over topics**, because it is the only option that keeps every safety rule of ADR-0002 and ADR-0003 in one place, `arm_driver`.

| | Rule |
|---|---|
| Serial port | `arm_driver` only. The plugin never opens it |
| `read()` | Positions from the latest `/joint_states` (radians), velocities from it or 0. No message yet → `ERROR` |
| `write()` | Publishes `ALL_ANGLES` on `/arm/command` with `source: "mission"` at `state_rate_hz` (20 Hz; `arm_driver` sends at most ~14 of them per second, the rest are overwritten, [measured](../architecture/timing-budget.md)) while the controller is active, **even when the target has not changed**: the watchdog stops the arm after `command_timeout_ms` (300 ms) without a motion command, which would leave it short of the last point. Angles are clamped by `arm_driver` (`JOINT_LIMITS`) |
| Control | `braco_mission` calls `take_control("mission")` **without `force`** before each goal and releases it after the result, also on failure (like zero / park). Refused → the goal is not sent; nothing is queued. The plugin never calls `/arm/take_control` |
| Lost control or STOP | The plugin reads `/arm/control_owner` and `/arm/command`. While `mission` holds the control, a STOP or another client taking the control makes `write()` return `ERROR`, so ros2_control stops `arm_controller` and the goal is aborted. Covers a STOP from any client, a forced take by the page or joystick, and `park` on overheat. While the owner is free or another client, `write()` sends nothing and returns `OK`: nothing went wrong |
| Safety commands | STOP and entering rest need no control: `mission` may send them at any time. To end its own trajectory it cancels the goal. `mission` never leaves rest mode (ADR-0003) |
| Other services | `power_on`, `set_gripper`, `release_servos`: only while `mission` holds the control (ADR-0002) |
| `/joint_states` | `arm_driver` stays the only publisher. `joint_state_broadcaster` is not spawned with `sim:=false` (#97) |
| Speed | MoveIt velocity and acceleration scaling ≤ 0.1 on the real arm (#101) |

## Consequences

- ✅ One serial owner and one place for safety rules; trajectories obey the same gates as the page and joystick
- ✅ STOP, a forced take and `park` on overheat all stop a running trajectory
- ✅ `/joint_states` keeps its rate, its consumers and its stale-state signal
- ✅ Sim and real share `arm_controller`; MoveIt and the mission do not know which arm they drive
- ❌ One extra hop (plugin → topic → `arm_driver` → serial); 100 Hz controller setpoints are thinned to ~14 Hz measured (20 Hz nominal: `COMMAND_GAP` bounds the serial loop), and motion starts ~200 ms after a command ([timing budget](../architecture/timing-budget.md), #90)
- ❌ After an `ERROR`, the hardware component and `arm_controller` must be activated again before the next goal (#101)
- ❌ A goal sent straight to `/arm_controller/follow_joint_trajectory` (e.g. from RViz) needs `take_control("mission")` first, as a bare `ros2 topic pub` does today
- ⚠️ `joint_state_broadcaster` runs in sim but not on the real arm, so the controller sets differ by one

## Confirmation

- gtest of the plugin against the fake arm (`PtyArm`): `read()` maps `/joint_states`; `write()` publishes `ALL_ANGLES` with `source: mission`; `ERROR` on STOP and on owner ≠ `mission` (#87, #101)
- `ros2 topic info /joint_states` with `sim:=false`: publisher count 1 (#97)
- Bench (`make bench`), velocity scaling ≤ 0.1: a trajectory moves the arm; STOP from the page halts it; forcing from the joystick halts it; `take_control("mission")` is refused while the page owns the arm

## Links

- Issues [#86](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/86), [#87](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/87), [#97](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/97), [#101](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/101), [#90](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/90) (timing); parent [#5](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/5)
- [ADR-0002](ADR-0002-unique-control.md), [ADR-0003](ADR-0003-rest-per-temp.md), [Architecture §9](../architecture/architecture.md)
- Revisit if: the extra hop makes trajectories visibly jerky on the bench, or a tool needs `joint_state_broadcaster` on the real arm
