# `controller.py`: Code Explanation

Joint-space joystick teleoperation for the **myCobot 280 Jetson Nano**, communicating through `pymycobot` over `/dev/ttyTHS1` at 1 Mbaud.

This document follows the data from the gamepad device to the servos, then lists the hardware pitfalls and the bench checks to run before trusting the script.

---

## 1. The core trick: velocity control built from position commands

The myCobot firmware mostly accepts *position* goals ("go to angle θ at speed s"). It has no clean "move at this velocity until told to stop" command. The script gets velocity-like behaviour anyway:

1. **Stick pushed:** send the joint a goal at its mechanical limit in that direction, using `send_angle(joint, ±LIMIT, speed)`.
2. The joint starts travelling toward that distant goal at the requested speed.
3. **Stick released:** call `mc.stop()`, which interrupts the motion wherever the joint is.

Holding the stick means "head for the end stop," and releasing it means "brake." Changing the speed mid-motion re-sends the same distant goal with a new speed.

> ⚠️ **Key consequence:** the default failure mode is *full travel to the limit*.
> If the `stop()` never arrives, the joint keeps moving until it reaches its limit or hits something. That can happen because of a lost serial byte, a frozen process, `kill -9`, or an exception outside the `try` block. This is the most important property of the script.

---

## 2. Configuration constants

| Constant | Value | Meaning |
|---|---|---|
| `SERIAL_PORT` / `BAUD_RATE` | `/dev/ttyTHS1`, `1000000` | Jetson UART connected to the arm's Atom controller |
| `JOYSTICK_DEVICE` | `/dev/input/js0` | Linux joystick API device |
| `SPEED_CAP` | 30 | Global speed cap: no arm motion faster than 30 % (firmware units, 1–100) |
| `MIN_SPEED` / `MAX_SPEED` | 10 / 30 (`SPEED_CAP`) | Speed range for analog sticks |
| `SPEED_STEP` | 5 | Quantization step for stick speeds (levels: 10, 15, 20, 25, 30) |
| `BUMPER_SPEED` | 30 (`SPEED_CAP`) | Fixed speed for joint 6 on L1/R1 |
| `HOME_SPEED` | 30 (`SPEED_CAP`) | Speed of the "all joints to 0" move (Y button) |
| `HOME_WAIT` | 8.0 s | Blocking wait after the home command (longer because the move is slower) |
| `GRIPPER_SPEED` | 50 | Gripper open/close speed (separate actuator, not affected by `SPEED_CAP`) |
| `AXIS_THRESHOLD` | 16000 | Deadzone (about 49 % of stick travel) |
| `COMMAND_GAP` | 0.05 s | Pause after each serial command so the firmware isn't overrun |
| `LIMITS` | `[None, 160, 130, 140, 120, 135, 170]` | Per-joint target magnitude in degrees, indexed by joint number (index 0 unused) |

### Control mapping

| Input | Joint | Notes |
|---|---|---|
| Left stick X (axis 0) | J1 base yaw | Direction inverted |
| Left stick Y (axis 1) | J2 shoulder pitch | |
| Right stick (axis 3) | J4 wrist pitch | Inverted |
| Right stick (axis 4) | J5 wrist yaw | Inverted |
| D-pad Y (axis 7) | J3 elbow | Digital input, always full speed (see §8) |
| L1 / R1 (buttons 6 / 7) | J6 tool rotation | Fixed speed; pressing both cancels |
| A (button 0) | Gripper close | |
| B (button 1) | Gripper open | |
| Y (button 3) | All joints → 0 | Blocking, 4 s |

> The code comments and the docstring disagree about which right-stick axis is vertical. Axis and button numbers depend on the controller and driver, so check them with `--debug` (§9).

---

## 3. Stage 1: Reading the joystick (`read_events`)

The script skips pygame and SDL and reads the Linux joystick API directly. Each event the kernel delivers is an 8-byte C struct:

```c
struct js_event {
    __u32 time;    // timestamp, ms
    __s16 value;   // axis position or button state
    __u8  type;    // 0x01 button, 0x02 axis, OR'd with 0x80 for init events
    __u8  number;  // axis / button index
};
```

`EVENT_FORMAT = "IhBB"` unpacks exactly these fields.

**Non-blocking drain.** The device is opened with `O_NONBLOCK`, so `os.read` raises `BlockingIOError` when the buffer is empty. `read_events` uses that to drain *every* pending event in one call. Sticks generate many events per second; handling only one per loop would make the arm fall further and further behind your hand.

**Init events.** When the device opens, the kernel sends synthetic events describing the current state, flagged with `0x80` (`TYPE_INIT`). The script discards them. It therefore assumes all controls are at rest at startup, so don't hold a stick while launching it.

**Sign convention.** Axis values range from −32767 to +32767, and pushing a stick *up* gives a *negative* Y value. This explains some of the `-1` direction factors in the mapping.

---

## 4. Stage 2: Converting deflection to speed (`calculate_speed`)

The function applies a deadzone, then a linear map, then quantization:

$$
\text{ratio} = \frac{|v| - 16000}{32767 - 16000}, \qquad
s = 15 + \text{ratio}\cdot(100 - 15)
$$

The result is rounded to the nearest multiple of 10 and clamped to `[MIN_SPEED, MAX_SPEED]`.

**Why quantize?** Each new speed value produces a new `send_angle` on the serial bus. Without quantization, a slightly unsteady hand would flood the bus with nearly identical commands. With it, there are only about nine speed levels, and a resend happens only when you cross a band boundary.

**Details that are easy to miss:**

- The **deadzone is 16000, almost half the stick's travel**. The first half of each push does nothing, and the whole speed range is squeezed into the outer half. Most controllers are clean beyond roughly 4000–6000.
- `MIN_SPEED = 15` never actually occurs. Python rounds 1.5 to 2, so the slowest real speed is **20**.

---

## 5. Stage 3: Deciding what should be moving (`wanted_motion`)

This function is pure: controller state goes in, and a dictionary comes out, with no side effects.

```python
{joint_number: (direction, speed)}    # e.g. {1: (-1, 40), 4: (1, 70)}
```

- For each mapped axis: `direction_out = configured_direction if value > 0 else -configured_direction`.
- Joint 6 is handled separately on L1/R1 at `BUMPER_SPEED`. If both bumpers are pressed, they cancel.

Keeping this function pure is the best design decision in the file. You can unit-test it without the arm, and it carries over almost unchanged into a ROS 2 teleop node.

---

## 6. Stage 4: Reconciling desired and current motion (main loop)

The loop keeps two dictionaries with the same shape:

| Dict | Meaning |
|---|---|
| `wanted` | What the controller is asking for right now |
| `moving` | What the script last commanded the arm to do |

It sends serial commands only when `wanted != moving`, which keeps bus traffic low.

### Reconciliation steps

**Step A: handle released or reversed joints.** If any joint in `moving` is no longer wanted, or is wanted in the opposite direction, the script calls `mc.stop()` and clears `moving`.

> **Subtlety:** `stop()` halts *every* joint, not only the released one.
> Suppose you are moving J1 and J2 and you let go of J2. Everything stops, `moving` is cleared, and in the same pass J1 is treated as newly engaged and restarted. The behaviour is correct, but you'll see a brief **stutter on J1** each time you release any other control. The protocol has no per-joint stop.

**Step B: start or update joints.** For each wanted joint that is new, or whose speed band changed, the script sends

```python
mc.send_angle(joint, direction * LIMITS[joint], spd)
```

and then sleeps `COMMAND_GAP` (50 ms).

**Latency cost.** The command gaps add up. Changing three axes in one pass adds about 150 ms before the joystick is read again.

**Loop rate.** There is a 20 ms sleep at the end of each pass, so the loop polls at roughly 50 Hz when nothing changes.

---

## 7. Buttons and shutdown

### Home (`Y`)

1. `mc.stop()`
2. `send_angles([0]*6, 100)`
3. **Blocking `sleep(4)`.** The script does not check whether the arm actually arrived.
4. Drain the joystick events that queued up and clear all state, so a stick pushed during homing doesn't trigger motion afterward.

The zero pose is the arm pointing straight up. At speed 100 this is a fast, sweeping move, so keep the space above the arm clear.

### Gripper (`A` / `B`)

`set_gripper_state(1, 50)` closes the gripper and `set_gripper_state(0, 50)` opens it. Both are fire-and-forget calls with no gap afterward.

### Shutdown path

- `SIGTERM` and `SIGHUP` are converted into `KeyboardInterrupt`. Ctrl+C, closing the SSH session, and `systemctl stop` therefore all reach the same `finally` block.
- A joystick disconnect raises `OSError` from `os.read`, which is also caught.
- `finally` calls `mc.stop()`, waits 0.2 s, prints the angles, and closes the device.
- `get_angles()` can return `-1` or `[]` if the serial link is unhealthy. Treat it as information only; it does not confirm the arm stopped.

---

## 8. Platform pitfalls

| Pitfall | Effect | Mitigation |
|---|---|---|
| **Controller layout varies** | With the `xpad` driver, axis 3 is usually right-stick X and axis 4 right-stick Y. LB/RB are usually buttons 4/5, and 6/7 are Back/Start. | Check with `jstest` / `--debug` and fix the mapping tables. |
| **D-pad is digital** | J3 only ever sees 0 or ±32767, so `calculate_speed` always returns 100. The elbow becomes the fastest joint and the one most likely to drive the wrist into the table. | Give J3 a fixed, moderate speed like `BUMPER_SPEED`. |
| **`LIMITS` covers joint ranges, not collisions** | Commands aren't rejected, but nothing prevents the gripper from hitting the base, table or fixtures. | Planning scene and collision checking (MoveIt / Servo). |
| **`nvgetty` holds `/dev/ttyTHS1`** | On many JetPack images the serial console uses the port, causing garbage replies or `get_angles() == -1`. | `systemctl status nvgetty`; disable it if active. |
| **Servo backlash** | A reversed joint takes up gear slack before the link moves. The stop-on-reverse logic makes this more visible. | Expect about 0.5–1° of dead motion on reversal; use visual feedback where precision matters. |
| **Limit-targeting failure mode** | A lost `stop()` means travel to the end stop. | Step-ahead targets or a dead-man watchdog (§10). |
| **Blocking home** | During the 4 s sleep the script ignores the controller. | Poll `is_moving()` / angles instead of a fixed sleep. |

---

## 9. Bench verification

**Controller layout (no arm needed):**

```bash
sudo apt install joystick
jstest /dev/input/js0                       # live axis/button numbers
python3 controller.py --debug      # same, through the script's parser
```

**Serial link health:**

```bash
ls -l /dev/ttyTHS1 && groups                # check that you have access (dialout or equivalent)
systemctl status nvgetty                    # must not be running
python3 -c "from pymycobot import MyCobot280 as M; import time; m=M('/dev/ttyTHS1',1000000); time.sleep(1); print(m.get_angles())"
```

**First live run checklist:**

1. Move one joint at a time with small stick deflection.
2. Keep a hand near the hardwired E-stop.
3. Test the dangerous cases deliberately:
   - release the stick mid-motion,
   - unplug the controller mid-motion,
   - press Ctrl+C mid-motion.
4. Each must stop the arm within a fraction of a second. If any doesn't, fix it before going further.

---

## 10. A safer pattern and the path to ROS 2

**Step-ahead targets.** Instead of targeting the end stop, re-command a target a small distance ahead of the current angle on every loop tick, scaled by speed. If commands stop arriving, the joint coasts a few degrees and stops by itself, with no `stop()` needed. This acts as a built-in dead-man switch, and it also removes the cross-joint stutter from §6.

**Native jogging.** Recent `pymycobot` versions provide `mc.jog_angle(joint, direction, speed)`. Check your firmware version and confirm that `stop()` is reliably honoured before depending on it.

**ROS 2 equivalent.** The step-ahead idea is how **MoveIt Servo** works:

```
joy_node ──sensor_msgs/Joy──▶ teleop node ──JointJog / TwistStamped──▶ MoveIt Servo ──▶ ros2_control ──▶ myCobot hardware interface
                              (≈ wanted_motion)                         (collision + singularity checks)
```

`wanted_motion` maps almost directly onto the teleop node. Servo adds what this script lacks: short-horizon goals, collision checking against the planning scene, joint-limit and singularity handling, and a command timeout that stops the arm when input goes quiet.
