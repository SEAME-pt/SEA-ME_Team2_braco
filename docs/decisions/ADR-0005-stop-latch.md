---
status: accepted
date: 2026-11-10
deciders: all
---

# A STOP latches in arm_driver until a person releases it

## Context

There is no hardwired e-stop, so STOP on `/arm/command` is the emergency stop. `arm_driver` stopped the arm once and dropped pending motion, but the owner kept the control ([ADR-0002](ADR-0002-unique-control.md)): a client that resent its target moved the arm again. On the real arm (PR #124), a STOP from the terminal did not stop a move the page streamed. The same `STOP` was also sent for routine stops (stick released, move timed out).

**Question:** how does a STOP from any client keep the arm stopped until a person decides to go on?

## Drivers

- A STOP from anyone holds, whatever the other clients send
- Only a deliberate act releases it, and the release never moves the arm
- An immediate reflex release must not undo it
- Routine stops (joystick, timeouts) must not freeze the arm
- Enforced next to the hardware, not by the clients

## Options

| Option | Pros | Cons |
|---|---|---|
| **Latch in `arm_driver` with its own release, after a minimum hold** | Holds for every client; one place; zero/park/auto-park cannot undo it | Routine stops need their own command type (`HALT`) |
| STOP enters rest mode | Reuses existing refusals | zero, park, `temperature_monitor`, the page and scripts all leave rest mode, then move |
| Timed lockout, then motion accepted again | No release step | A client still streaming moves the arm when the timer ends |
| Every streaming client stops itself | No driver change | One buggy client breaks it (the failure seen in PR #124) |

## Decision

**Latch in `arm_driver` with its own release, after a minimum hold**, because only the driver sees every client and can hold the arm whatever they send.

| | Rule |
|---|---|
| STOP (`0`) | Stops the arm, drops pending motion, frees the control, latches. Published on `/arm/stopped` (latched) |
| While latched | Refused: motion, `take_control` (even `force`), `power_on`, `set_gripper`, `release_servos`, `set_read_only false` |
| Release | `/arm/release_stop` (`make resume`, the page's Release STOP), refused for `stop_hold_s` (3 s) after the last STOP. Moves nothing; the control stays free |
| Second STOP | Keeps the latch and restarts the hold |
| HALT (`3`) | Routine stop: stops the arm, no latch, the owner keeps the control. Used by the joystick, zero/park, `braco_arm.py`, the page's move timeout |

- The page's release is open to anyone, like STOP: nobody can hold the control during a latch.
- `make zero` never releases a STOP: its `take_control` is refused, so it fails. Run `make resume` first.

## Consequences

- ✅ A STOP from any client holds while another one streams
- ✅ zero, park and the auto-park cannot undo it: their `take_control` is refused
- ✅ Releasing never moves the arm
- ❌ After any STOP someone must release it (`make resume` or the page) before the arm moves again
- ❌ A new `ArmCommand` type: every client must use `HALT` for routine stops
- ⚠️ Anyone on the page can release a STOP

## Confirmation

- `braco_driver/test/test_arm_driver.cpp`: latch, refusals (even `force`), second STOP, hold, release moves nothing, `HALT` does not latch; `test_control.cpp`: `ControlLease::clear()`
- `test_joystick_teleop.cpp`: the LED turns yellow while latched; `test_arm_moves.cpp`: zero/park give up a move at once
- `web/tests/`: the page cancels its move on a latch, refuses motion, release inside/after the hold
- Bench (`make bench`, step 3): STOP from the terminal while `web` streams, `force` refused, release refused inside the hold and accepted after it, the arm still after the release

## Links

- Issue [#126](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/126), found in PR #124
- [ADR-0002](ADR-0002-unique-control.md), [ADR-0004](ADR-0004-trajectories-through-arm-driver.md), [safety guide](../guides/safety.md)
- Revisit if: a hardwired e-stop is added, or the 3 s hold gets in the way on the bench
