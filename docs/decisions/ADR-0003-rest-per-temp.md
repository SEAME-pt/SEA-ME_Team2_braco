---
status: accepted
date: 2026-10-21
deciders: all
---

# An overheating servo puts the arm to rest by itself

## Context

`temperature_monitor` reads the six servo temperatures (`/arm/servo_temperatures`, every 2 s). It shows them on the LCD and publishes `/diagnostics`: warm from 55 °C, hot from 65 °C, just below the servos' own protection (~70 °C). A hot servo only showed a warning, so clients could keep loading it until its protection dropped the joint. `arm_driver` already has a rest mode (`/arm/set_read_only`) that refuses motion but keeps publishing temperatures.

**Question:** who decides that a hot arm must rest, and how does it come back?

## Drivers

- Stop a hot servo before its own protection drops a joint
- Nobody needs to be watching
- Resuming is a person's decision, and is not overruled
- A temperature hovering at the limit must not trigger again on every crossing
- Keep `arm_driver` simple

## Options

| Option | Pros | Cons |
|---|---|---|
| **`temperature_monitor` parks the arm, once per hot episode** | Levels already computed there; driver stays simple; a person can resume | A safety action depends on a second node |
| `arm_driver` rests by itself | No other node needed | Temperature policy in the serial loop; duplicates the monitor |
| Only warn | No change | The arm works until a joint drops |

## Decision

**`temperature_monitor` parks the arm, once per hot episode**, because it already computes the level and the driver already has rest mode.

- **Trigger:** the first reading ≥ 65 °C (`TEMP_HOT_C`) starts an episode and acts **once**. The episode ends when every joint is < 60 °C (`AutoRest::REARM_BELOW_C`). The 5 °C hysteresis stops jitter around the limit from re-parking the arm.
- **Action:** runs `park` (fold, release the joints, rest mode on). The joints must be released, because a servo that keeps holding its pose does not cool down. Falls back to `/arm/set_read_only true` if `park` fails. Retries on the next hot reading if the request fails.
- **Shown:** log (joint and temperature), LCD `REST: SERVO HOT`, the page shows "resting".
- **Resume:** never automatic. `make resume` or *Wake up* on the page. Allowed during the episode (to park a hot arm on purpose).
- **Parameters** (`braco_bringup/config/braco.yaml`): `auto_rest_on_hot`, `park_on_hot`, `park_command`.
- One definition of "hot" (`mycobot.hpp`). The page reads it from `/diagnostics`.

Implemented in `AutoRest` ([auto_rest.hpp](../../ros2_ws/src/braco_driver/include/braco_driver/auto_rest.hpp), no ROS).

## Consequences

- ✅ A hot servo stops being loaded with nobody watching
- ✅ Temperatures keep being published in rest mode, so cooling is visible
- ✅ The released servos actually cool
- ✅ The person decides when to resume
- ❌ A move in progress is replaced by the park move (the hot servo moves once more)
- ❌ Depends on `temperature_monitor` running (respawned by the launch). The servos' own protection stays the last line

## Confirmation

- `braco_driver/test/test_control.cpp`: once per episode, 64/65 °C jitter is one episode, new episode only below 60 °C, retry after failure
- Bench (`make bench`, step 5): simulated hot servo → arm parks, joints released, rest on, LCD warns; `make resume` → motion accepted

## Links

- Issue [#55](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/55); hysteresis added after PR #64 review (2026-10-23)
- [ADR-0002](ADR-0002-unique-control.md), [arm-rest-mode.md](../guides/arm-rest-mode.md), [safety.md](../guides/safety.md)
