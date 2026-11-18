---
status: rejected
date: 2026-11-18
deciders: all
---

# MoveIt trajectories reach the Atom as a few waypoints, not as streamed setpoints

## Context

Today a MoveIt trajectory reaches the arm as hundreds of micro-targets: `arm_controller` (JTC) interpolates at 100 Hz, `BracoSystem` publishes at 50 Hz and `arm_driver` sends about 14 per second (ADR-0004, [timing budget](../architecture/timing-budget.md)). The Atom runs in fresh mode 1 (read on 2026-11-18): every new `SEND_ANGLES` interrupts the current move and is planned again from zero, so a trajectory becomes many short accelerate–brake moves. On #93 the same mechanism made the joystick jitter. A single long `SEND_ANGLES` is the smoothest move the firmware can make, and OMPL's path has only a few vertices before time parameterization makes it dense.

**Question:** should a planned trajectory reach the Atom as its few path vertices, each a full firmware move, instead of a stream of setpoints?

## Drivers

- Smooth motion on the real arm (#93)
- Every safety rule stays in `arm_driver`: STOP, owner, rest, watchdog, joint limits (ADR-0002, ADR-0004, ADR-0005)
- A STOP must halt the arm at once, also in the middle of a multi-waypoint move
- The joystick keeps fresh mode 1: the latest target wins
- The path between waypoints stays the one MoveIt checked for collisions

## Options

| Option | Pros | Cons |
|---|---|---|
| Stream setpoints (today) | Follows the planned timing; works with the standard JTC; the watchdog bounds coasting | Hundreds of firmware replans per move: jitter |
| **Waypoints in fresh mode 1, the next sent on arrival** | Firmware-smooth segments; STOP and watchdog work as today; no mode switch | A short stop at each waypoint; needs our own `FollowJointTrajectory` controller |
| Waypoints queued in fresh mode 0 | Firmware chains the segments with no host gap | The mode is global (the joystick needs 1); queued moves run with no command, so STOP must empty the queue (unproven); a stop at each waypoint anyway (no blending) |

## Decision

**Keep streamed setpoints (today, ADR-0004)**, because on the real arm the stream was as smooth as a single long move, waypoints were worse, and fresh mode 0 is unsafe.

Measured on 2026-11-18 with `waypoint_test.py` (speed 15, J1 +40° and J2 +20° and back, one run per way):

| Way | cv J1 | cv J2 | Stops on the way | Time |
|---|---|---|---|---|
| stream (a target every ~70 ms, open loop in time) | 0.04–0.06 | 0.07–0.10 | 0 | 2.1 s |
| single `SEND_ANGLES` | 0.06–0.08 | 0.04–0.07 | 0 | 1.8 s |
| 3 waypoints, fresh mode 1, next on arrival | 0.12–0.19 | ~1.0 | 2 (J2) | 1.8 s |

- **Fresh mode 0 fails `stop`:** STOP (0x29) at J1 = 7.0° ended only the current move; the Atom ran the rest of the queue to 38.4° (31.4° after the STOP). A STOP would not stop the arm.
- **Waypoints stop at every waypoint:** the Atom moves each joint at the same angular speed without synchronising arrival, so J2 (shorter travel) reaches each waypoint first and waits for J1.
- **Streaming is not the cause of jitter:** what made the joystick jitter (#93) was a target computed from ~200 ms old feedback (measured + lead), not the number of commands. A target that follows the clock, as the JTC does, is smooth.

## Consequences

- ✅ No new controller: sim and real keep sharing `arm_controller` (ADR-0004)
- ✅ The Atom stays in fresh mode 1; STOP, watchdog and coast bound unchanged
- ❌ Jitter in cartesian trajectories (#141) still needs its own cause: measure the real MoveIt pipeline (`/arm/command` and `/joint_states` during `draw_line.py`)
- ⚠️ Nothing in `arm_driver` checks the fresh mode: a tool that sets it to 0 would make STOP unsafe

## Confirmation

`scripts/timing/waypoint_test.py`, on the real arm with `arm_driver` stopped:

1. `stop`: fresh mode 0, three waypoints queued, STOP in the first one. PASS only if the arm stays within 3° of where the STOP came and is not moving 3 s later. FAIL rules mode 0 out.
2. `smooth`: J1 +40° and J2 +20° and back, four ways (stream, single, wp-mode1, wp-mode0). Accept if wp-mode1 has a cv of velocity at most half of stream's and no more stops than waypoints.
3. The fresh mode is back to 1 after every run (the script reads it back).

## Links

- #93; [ADR-0004](ADR-0004-trajectories-through-arm-driver.md), [ADR-0005](ADR-0005-stop-latch.md), [timing budget](../architecture/timing-budget.md)
- Revisit if: a firmware with blending or synchronised joints appears, or STOP empties the queue in fresh mode 0
