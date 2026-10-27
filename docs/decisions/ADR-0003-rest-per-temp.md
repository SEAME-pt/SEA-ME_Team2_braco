---
status: "accepted"
date: 2026-10-21
decision-makers: all
consulted: all
informed: all
---

# An overheating servo puts the arm to rest by itself

## Context and Problem Statement

The myCobot 280 servos heat up under load. `temperature_monitor` reads the six
temperatures that `arm_driver` publishes (`/arm/servo_temperatures`, every 2 s),
shows them on the LCD1602 and publishes `/diagnostics`: normal, warm from 55 °C
(`TEMP_WARM_C`), hot from 65 °C (`TEMP_HOT_C`), a few degrees below the servos'
own protection (~70 °C).

Until now a hot servo only turned the LCD red ("SERVO HOT! J4 / 66 C - rest arm")
and logged an error. Nothing stopped the arm: the joystick or the page could keep
loading the hot servo until its protection cut in, which drops the joint.

`arm_driver` already has a rest mode (`/arm/set_read_only`, [arm-rest-mode.md](arm-rest-mode.md)):
it stops the arm and refuses motion, gripper and power on, while it keeps
publishing angles and temperatures.

Question: who decides that a hot arm must rest, and how does it come back?

## Decision Drivers

* A hot servo must stop working before its own protection drops a joint
* Nobody may need to be watching the LCD or the page for that to happen
* Coming back must be a person's decision (they know whether the arm cooled down)
* No fight with a person who resumes on purpose (for example to park a hot arm)
* A servo hovering at the limit (1 °C steps, one reading every 2 s) must not trigger
  the reaction again on every upward crossing
* Keep `arm_driver` a serial bridge with simple, local rules

## Considered Options

* `temperature_monitor` parks the arm and puts it to rest once per hot episode
* `arm_driver` enters rest mode itself when it reads a hot temperature
* Only warn (LCD, page, log), as before

## Decision Outcome

Chosen option: "`temperature_monitor` parks the arm and puts it to rest once per hot
episode", because it already computes the level for the LCD and `/diagnostics`,
and the driver already has the rest mode it needs.

* **When:** the first reading with any joint at `TEMP_HOT_C` (65 °C) or above
  starts an *episode* and puts the arm to rest, **once**. The episode ends only
  when every joint is below `AutoRest::REARM_BELOW_C` (60 °C); the next hot
  reading after that starts a new one. The 5 °C gap is hysteresis: readings that
  jitter around the limit (65, 64, 65, …) stay one episode, so `park` does not run
  again on every crossing. Ending the episode at 65 °C instead would re-park a
  cooling arm (briefly leaving rest mode and re-energising servos that sagged
  from the park pose) and take the control away from a person who resumed on
  purpose, every time the reading crossed the limit. The decision is `AutoRest` (`braco_driver/include/braco_driver/auto_rest.hpp`,
  no ROS, unit tested). If the request cannot be sent (driver not reachable) or is
  refused, the next hot reading tries again.
* **Seen:** the log says which joint and how hot; the LCD shows
  `REST: SERVO HOT` / `J4 66 C cooling`; the page shows "resting" (from
  `/arm/read_only`) and the hot level (from `/diagnostics`).
* **Back:** never automatic. A person runs `make resume` (or Wake up on the page)
  when the arm has cooled down. Resuming during the same episode is allowed and
  not overruled: it is how a hot arm is parked on purpose.
* **What it does:** it runs `park` (`ros2 run braco_bringup park`, a child process): the arm
  folds, the servos are released and rest mode is on. A hot arm that was only held still
  (rest mode alone keeps the joints energised, holding the pose) does not cool; releasing
  the joints does. `park` takes the control by force, so it wins over the page and the
  joystick. If `park` cannot start or exits with an error, the monitor falls back to
  `/arm/set_read_only true`, so the arm is never left without protection.
* **Switch:** parameter `auto_rest_on_hot` (default `true`) turns the reaction off;
  `park_on_hot` (default `true`) chooses park over rest mode only; `park_command` is the
  command line to run. All in `braco_bringup/config/braco.yaml`.

The page no longer computes temperature levels from its own copy of the limits:
on the real arm they come from `/diagnostics`, so there is one definition of
"hot" (`mycobot.hpp`). The copy in `web/server/arm.py` is only used by the mock.

### Consequences

* Good, because a hot servo stops being loaded without anybody watching
* Good, because rest mode was already there and keeps the temperatures coming, so cooling is watched
* Good, because the person decides when to resume, and is not overruled while they handle the arm
* Good, because the servos are released and can cool, even if the arm was standing still
* Bad, because a move in progress is replaced by the park move at 65 °C (a hot servo moves
  once more, to the park pose, instead of holding where it was)
* Bad, because it depends on `temperature_monitor` running (it is respawned by
  the launch). The servos' own protection remains the last line

### Confirmation

* `test/test_control.cpp` (gtest): rest once per episode, readings jittering
  between 64 and 65 °C stay one episode, a new episode only after cooling below
  60 °C, retry after a failed request
* Bench (Jetson, `make bench`, step 5): a simulated hot servo (a fake `ServoTemperatures`, or a lower
  limit) → the arm parks, the joints are released, rest mode on, LCD warns, `make resume` → motion accepted again

## Pros and Cons of the Options

### temperature_monitor, once per episode

* Good, because the level is already computed there, and the driver stays simple
* Good, because "once per episode" lets a person resume on purpose
* Bad, because it adds a dependency on a second node for a safety action

### arm_driver enters rest by itself

* Good, because it needs no other node
* Bad, because the driver gets temperature policy (levels, episodes) next to the serial loop
* Bad, because it would duplicate what `temperature_monitor` computes for the LCD and `/diagnostics`

### Only warn

* Good, because nothing changes
* Bad, because the arm keeps working until a joint drops

## More Information

* Issue [#55](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/55)
* The reaction was first only `/arm/set_read_only true`; it now runs `park` (fold, release the
  joints, rest mode) and falls back to `set_read_only` (see "What it does" above).
* 2026-10-23: the episode end moved from "below 65 °C" to "below 60 °C" (hysteresis),
  found in the review of PR #64. The decision itself is unchanged.
* Related: [ADR-0002](ADR-0002-unique-control.md) (one control owner), [safety.md](safety.md)
