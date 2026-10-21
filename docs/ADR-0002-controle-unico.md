---
status: "accepted"
date: 2026-10-21
decision-makers: all
consulted: all
informed: all
---

# One control owner for the arm, decided by arm_driver

## Context and Problem Statement

`arm_driver` is the only process that opens the myCobot 280 serial port
([architecture](architecture.md), principle 1). Everything else reaches the arm
through `/arm/command` and the `/arm/*` services: the web page, the joystick
(`joystick_teleop`), the zero/park programs (`make zero`, `make park`) and, later,
the mission layer.

Until now `arm_driver` obeyed the newest command, whoever sent it ("latest
wins"). The page had its own control lease, but it only arbitrated between
browser tabs: the joystick knew nothing about it. With the page moving the arm to
a pose, a touch on the stick sent joint targets that replaced the page's, and the
two clients took turns commanding the arm, 10 to 50 times a second each.

Question: how do we make sure only one client moves the arm at a time, without
losing the ability to stop it from anywhere?

## Decision Drivers

* Safety: two clients must never command the arm at the same time
* STOP must always work, from any client, whoever is in control
* The rule must be enforced next to the hardware (principle 3), not by clients
  that may be buggy, old or not running
* A person standing at the arm with the joystick must be able to take over on
  purpose, but not by brushing the stick
* An owner that crashes or walks away must not keep the arm locked forever
* Small change: no new process, the clients we have keep working the same way

## Considered Options

* Owner in `arm_driver`, named by a `source` field in `ArmCommand` and taken with
  a `/arm/take_control` service
* A separate arbiter node (a multiplexer in front of `arm_driver`)
* Keep "latest wins" and coordinate in the clients (the page asks the joystick to
  pause, and so on)

## Decision Outcome

Chosen option: "Owner in `arm_driver`", because the driver already sees every
command and is the place where safety rules live; it needs one message field, one
service and one latched topic.

The rule, implemented by `ControlLease` (`braco_driver/include/braco_driver/control.hpp`,
no ROS, unit tested):

| | |
|---|---|
| Who | One owner: `web`, `joystick`, `script` (zero/park) or `mission`. `""` = free. Published latched on `/arm/control_owner` (`std_msgs/String`). |
| Taking | `/arm/take_control` (`braco_interfaces/srv/TakeControl`): succeeds when the control is free, already ours, or `force` is set. Forcing drops the previous owner's pending motion and stops the arm. |
| Motion | `ArmCommand` with `JOINTS` / `ALL_ANGLES` is obeyed only from the owner's `source`. Anything else is ignored (logged, throttled). Taking the control is always explicit: motion never takes a free control by itself. |
| STOP | Always obeyed, from any source. |
| Losing it | `release: true` by the owner, a forced take by another client, or `control_idle_s` (5 s) without motion from the owner. |

The clients:

* **Joystick:** with the control free, the first real input (a stick out of its
  dead zone, X, A/B) takes it. While another client has it, the sticks are
  ignored and the LED is dimmed; holding **button 8 for 1 s** (`force_button`,
  `force_hold_s`) forces it. Letting go does not release it: the driver's idle
  timeout does. `power_on` on connect waits until the joystick has the control. A
  joystick without the control never sends STOP (unplugging it must not cut
  someone else's move).
* **Web page:** the page lease *is* the page's claim on the arm. Taking the lease
  calls `take_control("web")` (no force) and answers **409** while the joystick or
  a script has the arm; every lease tick renews it, so an idle operator keeps the
  arm for the whole page lease (60 s) and the joystick needs the button to take
  over. When the control is forced away, the lease ends and a running move stops
  streaming.
* **zero / park:** `take_control("script", force=true)` before moving (a person
  ran the command on purpose) and `release` at the end, also on failure.

### power_on, set_gripper and release_servos

These services are `std_srvs` types and carry no `source`, so `arm_driver` cannot
tell who calls them. We chose the simplest option: **the clients call them only
while they own the control** (the joystick after it has it, the page while it
holds the lease, zero/park after `take_control`), and the driver does not check.
Giving them a `source` means a new service type for each, and every client
changing at once, for clients we write ourselves. Revisit this if a client we do
not control appears (a mission from another team, a tool) or if a wrong call is
ever seen on the bench.

### Consequences

* Good, because two clients can no longer take turns commanding the arm
* Good, because STOP still works from the page, the joystick and the scripts, whoever is in control
* Good, because the rule lives in one place, next to the serial port, and is unit tested
* Good, because a crashed or idle owner loses the control after 5 s
* Good, because the page shows who has the arm ("the joystick is in control")
* Bad, because every client must now take the control before moving: a bare
  `ros2 topic pub /arm/command ...` is ignored until someone calls
  `/arm/take_control` with the same `source`
* Bad, because the page renews the control once a second while its lease lasts
  (one service call per second)
* Neutral, because `power_on`, `set_gripper` and `release_servos` rely on the
  clients' discipline (above)

### Confirmation

* `test/test_control.cpp` (gtest, `make test`): free is taken; another source
  without force is refused; force changes the owner; idle release; only the owner releases
* `web/tests/test_session.py`, `test_ros_arm.py`, `test_server.py`: the lease asks
  the driver, 409 while the joystick has it, keep-alive, the lease ends when the
  control is forced away, the page sends no STOP after losing it
* Bench (Jetson, `make up`): page moving + stick ignored; hold button 8 → the
  joystick takes over and the page shows it and answers 409; let go 5 s → free;
  STOP from the page with the joystick in control; `make zero` / `make park` with the joystick connected

## Pros and Cons of the Options

### Owner in arm_driver

* Good, because the driver already sees every command and enforces the other safety rules
* Good, because no new process: one field, one service, one topic
* Bad, because the driver grows a little

### A separate arbiter node

* Good, because `arm_driver` stays a thin serial bridge
* Bad, because one more process on the Nano and one more hop for every command
* Bad, because a client that publishes straight to `arm_driver` bypasses it

### Coordination in the clients

* Good, because nothing changes in the driver
* Bad, because every client must know every other one, and one old or buggy
  client breaks the rule: exactly what principle 3 forbids

## More Information

* Issue [#55](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/55)
* Related: [ADR-0003](ADR-0003-repouso-por-temperatura.md) (rest mode on overheat),
  [arm-rest-mode.md](arm-rest-mode.md)
