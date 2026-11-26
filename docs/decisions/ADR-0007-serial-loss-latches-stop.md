---
status: accepted
date: 2026-11-25
deciders: all
---

# A lost serial link latches a STOP in arm_driver

## Context

`arm_driver` is the only process on the serial link to the arm (`/dev/ttyTHS1`, wired inside the base). When the
link dies, the Atom is expected to finish the last target it received and hold it with the motors on. `arm_driver`
used to log serial errors and carry on. Its watchdog and every STOP sent `stop` into the dead link, and the clients
kept streaming targets. When the link came back, the next target in the stream moved the arm, with no person
deciding it. Found while preparing the fault rehearsal of #142.

**Question:** what does `arm_driver` do when the arm stops answering, and when it answers again?

## Drivers

- The arm must not move by itself when the link comes back
- Nobody can release a STOP for an arm the driver cannot see
- A missed reply now and then is normal on the real arm and must not stop it
- One rule in one place, like the STOP latch ([ADR-0005](ADR-0005-stop-latch.md))

## Options

| Option | Pros | Cons |
|---|---|---|
| **Latch a STOP after `feedback_timeout_s` without valid angles** | Reuses the latch: every client already handles it. Nothing moves on reconnect until a person releases it | `make resume` is needed after every link loss, even a short one |
| Exit `arm_driver`, so the launch shuts down | The fault is impossible to miss | The arm is left on its last target with no STOP sent when the link returns. `make up` is needed |
| Only log, as before | No change | A stream resumes the motion the moment the link is back |
| Drop motion while lost, accept it again on reconnect | No release step | Same as above: the arm moves on reconnect without a decision |

## Decision

**Latch a STOP after `feedback_timeout_s` without valid angles**, because the latch is the one state every client
already stops for, and only a person can end it.

| | Rule |
|---|---|
| Lost | No valid angles for `feedback_timeout_s` (1 s, `braco.yaml`) since the last valid ones, or since the port opened. Checked on every pass of the serial loop, also after a serial error |
| On loss | A STOP from `feedback`: latched, pending motion dropped, control freed (ADR-0005) |
| While lost | `/arm/release_stop` is refused: "no angle feedback from the arm" |
| Back | The first valid angles send `stop` again (the first one may have gone nowhere). The latch holds |
| Release | `make resume` once the angles are back, as after any STOP |
| Off | `feedback_timeout_s: 0` turns the check off (tests of the dead port itself) |
| Reads while lost | After two angle reads in a row, or one temperature or voltage read, without a reply, temperatures and voltages are published invalid without reading the port, until valid angles come back |

- Detection takes about 1.3 s after the loss, at most about 2.5 s (a link already dead at start). On a dead link an
  angle read waits ~0.6 s for its timeout and a temperature or voltage read ~2 s, so the check runs after each read
  and the slow reads are skipped while the arm does not answer. Without that, the latch came ~4.4 s after the loss.
- The motion that was running when the link died is not stopped by this. The Atom finishes its last target, as it
  would with any lost STOP. A slow speed bounds what it can do.

## Consequences

- ✅ When the link comes back, the arm stays still until a person releases the STOP
- ✅ `/arm/stopped` tells the page, the joystick and `braco_mission` at once: they already handle a latch
- ✅ A port that fails every write (`EIO`) is caught too, not only a silent arm
- ✅ One missed angle reply, normal on the real arm, does not skip a temperature read
- ❌ After a link loss someone must run `make resume`, even if it lasted 1 s
- ❌ The move in progress when the link died still completes on the Atom: no software can stop it then
- ⚠️ An arm that misses replies for more than 1 s in normal use would latch: watch for it on the bench

## Confirmation

- `braco_driver/test/test_arm_driver.cpp`: `FeedbackLossLatchesAStopUntilTheLinkIsBackAndAPersonReleases` (silent arm
  mid-move: latch, release refused, stream ignored, stop sent again on reconnect, nothing moves until release, take
  and a new command), `ClosedPortLatchesAStopOnItsOwn`, `FeedbackTimeoutZeroLeavesTheLatchOff`,
  `DeadLinkLatchesWithinAboutASecondOfTheTimeout` (latch under 2.5 s after the loss, temperatures still published
  invalid)
- Fault rehearsal (`make fault`): the results table in the [safety guide](../guides/safety.md#10-faults-rehearsed)

## Links

- Issue [#142](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/142), parent [#139](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/139)
- [ADR-0002](ADR-0002-unique-control.md), [ADR-0004](ADR-0004-trajectories-through-arm-driver.md), [ADR-0005](ADR-0005-stop-latch.md)
- Revisit if: the real arm misses replies for more than `feedback_timeout_s` in normal use, or a hardwired e-stop is added
