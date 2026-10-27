---
status: accepted
date: 2026-10-21
deciders: all
---

# One control owner for the arm, decided by arm_driver

## Context

`arm_driver` is the only process that opens the serial port. The web page, joystick, zero/park scripts and (later) the mission all command the arm through `/arm/command`. The driver obeyed the newest command ("latest wins"). As a result, the page and the joystick could take turns commanding the arm 10–50 times a second each.

**Question:** how do we make sure only one client moves the arm at a time, while STOP still works from anywhere?

## Drivers

- Two clients must never command the arm at once
- STOP must always work, from any client
- Enforced next to the hardware, not by the clients
- A person at the arm can take over on purpose, but not by brushing the stick
- A crashed or idle owner must not lock the arm forever
- No new process

## Options

| Option | Pros | Cons |
|---|---|---|
| **Owner in `arm_driver`** (`source` field + `/arm/take_control`) | The driver sees every command and already enforces safety; one field, one service, one topic | The driver grows a little |
| Separate arbiter node | `arm_driver` stays thin | One more process and hop; clients can bypass it |
| Coordination in the clients | No driver change | Every client must know every other; one buggy client breaks it |

## Decision

**Owner in `arm_driver`**, because it already sees every command and is where safety rules live.

| | Rule |
|---|---|
| Owner | One of `web`, `joystick`, `script`, `mission`, or `""` (free). Latched on `/arm/control_owner` |
| Take | `/arm/take_control`: succeeds if free, already owned, or `force`. Forcing stops the arm |
| Motion | `JOINTS` / `ALL_ANGLES` obeyed only from the owner. Motion never takes control by itself |
| STOP | Always obeyed, from any source |
| Release | `release: true`, forced take by another client, or 5 s without motion (`control_idle_s`) |

Clients:

- **Joystick:** first real input takes a free control. Hold **button 8 for 1 s** to force it. Without control, it never sends STOP.
- **Web page:** its lease calls `take_control("web")`. It answers **409** while another client owns the arm and renews every second.
- **zero / park:** force-take before moving, release at the end (also on failure).
- `power_on`, `set_gripper`, `release_servos` carry no `source`. Clients call them only while they own the control; the driver does not check.

Implemented in `ControlLease` ([control.hpp](../../ros2_ws/src/braco_driver/include/braco_driver/control.hpp), no ROS).

## Consequences

- ✅ Clients can no longer fight over the arm
- ✅ STOP works from everywhere
- ✅ One rule, one place, unit tested
- ✅ An idle or crashed owner loses control after 5 s
- ❌ A bare `ros2 topic pub /arm/command` is ignored until `/arm/take_control` is called with the same `source`
- ❌ The page makes one service call per second while it holds the lease
- ⚠️ The three `std_srvs` services rely on client discipline

## Confirmation

- `braco_driver/test/test_control.cpp`: take, refuse, force, idle release, owner-only release
- `web/tests/` (`test_session.py`, `test_ros_arm.py`, `test_server.py`): lease, 409, keep-alive, lease ends on forced take
- Bench (`make bench`): stick ignored while the page moves; button 8 takes over; free after 5 s; STOP from the page; `make zero` / `make park` with the joystick connected

## Links

- Issue [#55](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/55)
- [Architecture](../architecture/architecture.md), [ADR-0003](ADR-0003-rest-per-temp.md), [hot-arm guide](../guides/hot-arm/README.md)
- Revisit if: a client we do not control calls `power_on` / `set_gripper` / `release_servos`, or a wrong call is seen on the bench
