---
status: proposed
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

**Pending the measurements below.** Expected: waypoints in fresh mode 1, because it keeps every safety rule as it is and only changes how a goal is cut. Mode 0 is chosen only if `stop` passes and it is clearly smoother.

- A trajectory is cut at the vertices of the planned path (not the time-parameterized points); at most a handful per goal
- Each segment is one `SEND_ANGLES` with a speed from the segment's planned duration
- The next waypoint is sent when the arm is within tolerance of the current one, or when `IS_MOVING` turns false
- `arm_driver` keeps the serial port and the gates; the new piece is a controller that replaces `arm_controller` on the real arm only

## Consequences

- ✅ Segments as smooth as the firmware allows; far fewer serial commands per goal
- ✅ STOP, owner, rest and limits unchanged (mode 1)
- ❌ A brief stop at each intermediate waypoint (the Atom has no blending)
- ❌ Sim and real no longer share `arm_controller`: the simulation keeps the JTC
- ❌ The watchdog needs a new rule: a long segment runs with no new command by design

## Confirmation

`scripts/timing/waypoint_test.py`, on the real arm with `arm_driver` stopped:

1. `stop`: fresh mode 0, three waypoints queued, STOP in the first one. PASS only if the arm stays within 3° of where the STOP came and is not moving 3 s later. FAIL rules mode 0 out.
2. `smooth`: J1 +40° and J2 +20° and back, four ways (stream, single, wp-mode1, wp-mode0). Accept if wp-mode1 has a cv of velocity at most half of stream's and no more stops than waypoints.
3. The fresh mode is back to 1 after every run (the script reads it back).

## Links

- #93; [ADR-0004](ADR-0004-trajectories-through-arm-driver.md), [ADR-0005](ADR-0005-stop-latch.md), [timing budget](../architecture/timing-budget.md)
- Revisit if: the measurements show stream close to the other ways, or a firmware with blending appears
