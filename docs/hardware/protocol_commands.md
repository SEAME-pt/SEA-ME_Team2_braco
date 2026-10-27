# myCobot 280 Jetson Nano: Protocol Commands Used

Documents the serial commands used to connect to the arm, read joint angles, and command a slow joint move. Everything goes through the `pymycobot` library (`MyCobot280` class), which wraps the arm's serial protocol.

## 1. Connection

| Parameter | Value |
|---|---|
| Serial port | `/dev/ttyTHS1` |
| Baud rate | `1000000` (1 Mbaud) |
| Library | `pymycobot` (`MyCobot280`; `get_angles.py` falls back to `MyCobot` on older versions) |
| Joystick input | `/dev/input/js0` (controller script only) |
| Display | Waveshare LCD1602 RGB on I2C bus 0 (`0x3E` LCD, `0x60` RGB backlight) |

```python
from pymycobot import MyCobot280
mc = MyCobot280("/dev/ttyTHS1", 1000000)
time.sleep(1)      # let the link settle
mc.power_on()      # energise the servos
time.sleep(0.5)
```

**Note:** `/dev/ttyTHS1` is the Jetson's hardware UART (the arm's internal serial link), not a USB-serial adapter. If the arm is ever driven from another computer over USB, the port will appear as `/dev/ttyUSB*` or `/dev/ttyACM*`; confirm with `ls /dev/tty*` before and after plugging in.

**Permissions:** if the port can't be opened, run with `sudo` or add the user to the `dialout` group (`sudo usermod -aG dialout $USER`, then log out and back in).

## 2. Commands used

| # | `pymycobot` call | Purpose | Used in |
|---|---|---|---|
| 1 | `MyCobot280(port, baud)` | Open the serial link | both scripts |
| 2 | `mc.power_on()` | Power/lock the servos so they hold position | both scripts |
| 3 | `mc.release_all_servos()` | Release all servos so the arm can be posed by hand | `get_angles.py` |
| 4 | `mc.get_angles()` | Read the six joint angles (degrees) | both scripts |
| 5 | `mc.get_coords()` | Read Cartesian pose `[X, Y, Z, RX, RY, RZ]` (optional, `--coords`) | `get_angles.py` |
| 6 | `mc.send_angle(joint, angle, speed)` | Move a single joint to an absolute angle | `controller_display.py` |
| 7 | `mc.send_angles([6 angles], speed)` | Move all joints to target angles (used for homing) | `controller_display.py` |
| 8 | `mc.stop()` | Stop the current motion immediately | `controller_display.py` |
| 9 | `mc.set_color(r, g, b)` | Set the arm's LED (indicates speed mode) | `controller_display.py` |
| 10 | `mc.set_gripper_state(state, speed)` | Close (`1`) / open (`0`) the gripper | `controller_display.py` |

### Parameter reference

| Parameter | Range / meaning |
|---|---|
| `joint` | `1` to `6` |
| `angle` | Degrees. Software limits used in the controller script (±): J1 160, J2 130, J3 140, J4 120, J5 135, J6 170 |
| `speed` | `1` to `100` (percent of maximum) |
| `get_angles()` return | List of 6 floats, or an error value (see below) |

## 3. Reading the joint angles (`get_angles.py`)

Procedure: lock the arm with `power_on()`, then read with `get_angles()`. `get_angles()` can occasionally return a non-list value (e.g. `-1` or `None`) when a reply is missed, so the script retries up to 5 times with a 0.1 s pause and accepts only a list of exactly 6 numbers.

```python
def read_value(getter, retries=5):
    for _ in range(retries):
        value = getter()
        if isinstance(value, list) and len(value) == 6:
            return value
        time.sleep(0.1)
    return None
```

Command sequence per recorded pose (Enter toggles):

1. `release_all_servos()` — arm goes limp; pose it by hand.
2. `power_on()` — lock the arm in the new pose (0.5 s settle).
3. `get_angles()` (and optionally `get_coords()`) — read and store.

Run:

```bash
python3 get_angles.py             # angles only
python3 get_angles.py --coords    # angles + X, Y, Z, RX, RY, RZ
```

Recorded poses are printed at exit in a form that can be pasted straight into `mc.send_angles(...)`.

**Caution:** releasing the servos removes holding torque. Support the arm by hand before pressing Enter to release, because gravity will move it.

## 4. Slow single-joint move (`controller_display.py`)

The move was commanded from the joystick controller script, which sends one `send_angle` command per active joint each loop (about every 70 ms including the 0.05 s command gap), targeting a point slightly "ahead" of the current angle (the *lead*). The arm keeps moving toward that rolling target while the stick is held; releasing the stick calls `stop()`.

### Slow mode parameters

| Setting | Value |
|---|---|
| Mode name | `slow` (`SPEED_MODES[0]`; see the check below about `START_MODE`) |
| Max speed | 12 % |
| Minimum speed | 3 % |
| Max lead | 5.0° |
| Min lead | 1.0° |
| Bumper (J6) speed | 8 % |
| LED colour | `(0, 0, 255)` |

> **Check:** `START_MODE = 1` indexes `SPEED_MODES[1]`, which is **medium** (max 30 %), not slow (index 0). If the move was meant to start in slow mode, change `START_MODE` to `0` or press the speed button (button 9) to cycle to the blue LED (slow) before moving.

### Command pattern while jogging

```python
angles = mc.get_angles()                     # read current pose
speed = MIN_SPEED + ratio * (max_speed - MIN_SPEED)
lead  = LEAD_MIN_DEG + ratio * (lead_max - LEAD_MIN_DEG)
target = angles[joint - 1] + direction * lead
mc.send_angle(joint, round(target, 1), speed)   # move one joint
...
mc.stop()                                       # on stick release
```

`ratio` (0 to 1) comes from the stick deflection after a dead zone of 5000/32767 and an exponential curve (`STICK_EXPO = 1.8`), so small deflections give very low speeds.

### Safety behaviours built into the script

- **Joint limits:** `shape_near_limit()` scales speed down within 25° of a limit (floor 15 % of the commanded speed) and refuses motion within 0.3° of it.
- **Hold on idle joints:** joints not being moved are re-commanded to their held angle at 20 % speed if they drift more than 0.8°.
- **Bad feedback:** if `get_angles()` fails while jogging, the script calls `stop()`.
- **Exit:** Ctrl+C, SIGTERM, SIGHUP, or joystick disconnect all end with `mc.stop()` and a final angle read.
- **Homing (button 3):** sends `send_angles(targets, 60)` with a 15° lead toward all-zero; any stick input aborts, and 3 bad reads in a row abort.

## 5. Pre-move checklist

1. Work cell clear of people and objects within the arm's full reach.
2. E-stop (or power switch) within reach of the operator.
3. Confirm slow mode (blue LED) before touching the sticks.
4. Read and note the starting angles with `get_angles.py`.
5. Command a small, short move first and keep the stick deflection light.
6. After the move, record the end angles and confirm the arm is stopped and locked.

## 6. Wire-level protocol (reference)

`pymycobot` builds each command as a framed serial packet:

```
0xFE 0xFE  <length>  <command id>  <data...>  0xFA
```

Command IDs below are from memory of the pymycobot protocol table and should be verified against the installed library version (e.g. `pymycobot/generate.py` or the `ProtocolCode` class) before being relied on:

| Call | Command ID |
|---|---|
| `power_on` | `0x10` |
| `release_all_servos` | `0x13` |
| `get_angles` | `0x20` |
| `send_angle` | `0x21` |
| `send_angles` | `0x22` |
| `get_coords` | `0x23` |
| `stop` | `0x29` |
| `set_gripper_state` | `0x66` |
| `set_color` | `0x6A` |

Angles are sent as 16-bit signed integers in hundredths of a degree. Using the library calls above is recommended over hand-built packets.
