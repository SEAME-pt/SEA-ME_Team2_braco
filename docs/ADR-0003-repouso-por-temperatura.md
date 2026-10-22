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
* Keep `arm_driver` a serial bridge with simple, local rules

## Considered Options

* `temperature_monitor` calls `/arm/set_read_only true` once per hot episode
* `arm_driver` enters rest mode itself when it reads a hot temperature
* Only warn (LCD, page, log), as before

## Decision Outcome

Chosen option: "`temperature_monitor` calls `/arm/set_read_only true` once per hot
episode", because it already computes the level for the LCD and `/diagnostics`,
and the driver already has the rest mode it needs.

* **When:** the first reading with any joint at `TEMP_HOT_C` (65 °C) or above
  starts an *episode* and puts the arm to rest, **once**. The episode ends when
  every joint is below `TEMP_HOT_C` again; the next hot reading starts a new one.
  The decision is `AutoRest` (`braco_driver/include/braco_driver/auto_rest.hpp`,
  no ROS, unit tested). If the request cannot be sent (driver not reachable) or is
  refused, the next hot reading tries again.
* **Seen:** the log says which joint and how hot; the LCD shows
  `REST: SERVO HOT` / `J4 66 C cooling`; the page shows "resting" (from
  `/arm/read_only`) and the hot level (from `/diagnostics`).
* **Back:** never automatic. A person runs `make resume` (or Wake up on the page)
  when the arm has cooled down. Resuming during the same episode is allowed and
  not overruled: it is how a hot arm is parked on purpose.
* **Switch:** parameter `auto_rest_on_hot` (default `true`) in
  `braco_bringup/config/braco.yaml`.

The page no longer computes temperature levels from its own copy of the limits:
on the real arm they come from `/diagnostics`, so there is one definition of
"hot" (`mycobot.hpp`). The copy in `web/server/arm.py` is only used by the mock.

### Consequences

* Good, because a hot servo stops being loaded without anybody watching
* Good, because rest mode was already there and keeps the temperatures coming, so cooling is watched
* Good, because the person decides when to resume, and is not overruled while they handle the arm
* Bad, because a move in progress is cut at 65 °C, wherever the arm is (it holds
  its position; it does not drop)
* Bad, because it depends on `temperature_monitor` running (it is respawned by
  the launch). The servos' own protection remains the last line

### Confirmation

* `test/test_control.cpp` (gtest): rest once per episode, a new episode after
  cooling below 65 °C, retry after a failed request
* Bench (Jetson, `make bench`, step 5): a simulated hot servo (a fake `ServoTemperatures`, or a lower
  limit) → rest mode on, LCD warns, `make resume` → motion accepted again

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
* Related: [ADR-0002](ADR-0002-controle-unico.md) (one control owner), [safety.md](safety.md)
