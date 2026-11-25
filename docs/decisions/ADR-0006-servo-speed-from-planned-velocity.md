---
status: proposed
date: 2026-11-20
deciders: all
---

# The servo speed on the real arm comes from the planned velocity

## Context

`BracoSystem::write()` ([ADR-0004](ADR-0004-trajectories-through-arm-driver.md)) sent every `ALL_ANGLES` at a fixed servo speed 50, ignoring the velocity MoveIt planned (at most ~0.26 rad/s at scaling 0.1).

**Question:** which servo speed does `write()` send with each trajectory point?

## Drivers

- The arm follows the plan's timing, at any MoveIt scaling
- No magic number to keep in sync with the joint limits
- Sim and the MoveIt mock unchanged

## Options

| Option | Pros | Cons |
|---|---|---|
| **Speed from the planned velocity** | Follows the plan at any scaling | `arm_controller` claims a velocity command, on the real arm only |
| Fixed speed 50 (before) | Simple | Ignores the plan; falls behind above scaling ~0.3 |
| Speed 100 always | Never falls behind | Reaches each point early and waits: stop-and-go |

## Decision

**Speed from the planned velocity**, because only the plan knows how fast each joint must go.

- `arm_controller` commands `position` and `velocity` on the real arm (`controllers_real.yaml`)
- Speed = largest `|velocity| / URDF max velocity × 100`, × 1.5, rounded up, clamped to [`min_speed` 5, 100]
- `on_init` refuses a joint without a velocity command or a velocity limit
- `write()` publishes at 25 Hz (was 20): faster than `arm_driver` sends, ~14 Hz at `COMMAND_GAP` 50 ms and ~20 Hz at 30 ms (tested by the team while tuning smooth motion; the code keeps 50 ms), so `arm_driver` always finds a fresh target and never waits; the extra ones are dropped by design (latest wins)

**Why × 1.5, with the scaling ≤ 0.1 of ADR-0004:** the two set different things.

| | Sets | Example, 130 mm line |
|---|---|---|
| Scaling 0.1 (ADR-0004) | The plan: where each joint must be, and when | ~10.7 % of the max velocity |
| Servo speed (× 1.5) | The servo's ceiling to reach each target it receives | 16 |

The arm starts moving ~200 ms after a command ([timing budget](../architecture/timing-budget.md)), so it is always a little behind the plan. At × 1.0 it would never catch up. At × 1.5 it may go faster to catch up, but the targets move at the plan's pace, so it never overtakes the plan: the motion stays at the scaling. Measured: the same 130 mm line took 4.27 s at speed 50 and 4.43 s at 16.

## Consequences

- ✅ A higher MoveIt scaling raises the speed by itself
- ❌ Real arm and sim claim different command interfaces
- ⚠️ No measurable change in accuracy or jerks at ~14 Hz (see Confirmation)

## Confirmation

- `test_speed_for_velocity.cpp`, `test_braco_system.cpp`
- Real arm, 2026-11-20, `draw_line.py` 130 mm: speed 50 vs 5 → 16 → 5; duration 4.27 vs 4.43 s; end error 15.8 vs 15.4 mm

## Links

- [ADR-0004](ADR-0004-trajectories-through-arm-driver.md), [braco_hardware README](../../ros2_ws/src/braco_hardware/README.md#servo-speed)
- Revisit if: the arm falls behind the plan, or `COMMAND_GAP` changes
