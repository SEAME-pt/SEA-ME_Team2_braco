# myCobot 280 Jetson Nano: serial protocol

How braco talks to the arm over serial. The code is in
[`mycobot_cpp`](../../ros2_ws/src/mycobot_cpp/README.md) (`includes/mycobot.hpp`, `srcs/mycobot.cpp`): plain C++,
no Python, no ROS. It speaks the same protocol as Elephant Robotics' `pymycobot`.

- **Only one program opens the port: `arm_driver`.** Everything else (joystick, web page, `make zero`/`park`/`temps`)
  goes through its ROS 2 topics and services.
- The stand-alone tools of `mycobot_cpp` (`zero`, `park`, `temps_lcd`, `pid_tool`...) open the port themselves:
  only with the system down (`make down`).

## 1. Connection

| Setting | Value |
|---|---|
| Port | `/dev/ttyTHS1` (the Jetson's UART, wired to the arm inside the base) |
| Baud rate | `1000000` |
| Format | 8N1, raw (no echo, no line processing) |
| Exclusive | Opened with `TIOCEXCL`: a second program gets `Device or resource busy` |
| Retries | Queries retry up to 5 times (3 for temperatures, voltages, servo data), 100 ms apart |

- `nvgetty` must be off on the Jetson, or it holds the port: `sudo systemctl disable --now nvgetty`.
- From another computer through a USB adapter the port is `/dev/ttyUSB*` or `/dev/ttyACM*` (`dmesg | grep tty`).

## 2. Frame format

```
0xFE 0xFE <len> <command> <data...> 0xFA
```

- `len` = number of data bytes + 2 (the command byte and the footer).
- Numbers are 16-bit signed, big-endian.

| Value | Unit | Sent as |
|---|---|---|
| Angle | degrees | int16 × 100 |
| X, Y, Z | mm | int16 × 10 |
| RX, RY, RZ | degrees | int16 × 100 |
| Speed | 1..100 (percent) | one byte |
| Voltage | V | one byte, steps of 0.1 V |

## 3. Commands

Codes from `pymycobot`'s `ProtocolCode`. The column "arm_driver" marks what the running system uses.

| Code | C++ method | What it does | arm_driver |
|---|---|---|---|
| `0x10` | `powerOn()` | Turns the motors on (locks the joints) | yes |
| `0x11` | `powerOff()` | Turns the motors off | |
| `0x12` | `isPowerOn()` | Are the motors on? | |
| `0x13` | `releaseAllServos()` | Frees the joints (move them by hand) | yes |
| `0x20` | `getAngles()` | Reads the six angles (12 bytes) | yes |
| `0x21` | `sendAngle(joint, angle, speed)` | Moves one joint | yes |
| `0x22` | `sendAngles(angles, speed)` | Moves all six joints | yes |
| `0x23` | `getCoords()` | Reads the tool pose X, Y, Z, RX, RY, RZ (12 bytes) | |
| `0x29` | `stop()` | Stops the motion at once | yes |
| `0x2B` | `isMoving()` | Is the arm moving? | |
| `0x66` | `setGripperState(state, speed)` | Gripper: `1` closes, `0` opens | yes |
| `0x6A` | `setColor(r, g, b)` | LED colour on the arm's top | yes |
| `0x52` / `0x53` | `getServoData()` / `setPid()` | Reads / writes servo settings (PID). Configuration only (`pid_tool`) | |
| `0xE3` | `getVoltages()` | Six servo supply voltages | yes |
| `0xE5` | `getTemps()` | Six servo temperatures (°C) | yes |

- `0x52`, `0x53`, `0xE3` and `0xE5` are marked **"verify"** in `mycobot.hpp`: check them against your pymycobot
  version (`grep -nE "= 0x" .../pymycobot/common.py`) before relying on new uses.
- Motion is **fire-and-forget**: the arm does not confirm a move. Read `getAngles()` to know where it is.

## 4. Safety built into the library

- **Joint limits** (`JOINT_LIMITS`, degrees): angles outside them are clamped to the nearest limit.

  | J1 | J2 | J3 | J4 | J5 | J6 |
  |---|---|---|---|---|---|
  | ±166 | ±138 | ±149 | ±149 | -153 .. 158 | ±178 |

  - The firmware's limits minus 1-2°, so a joint never rests on its end stop. `braco_description`'s URDF uses the
    same values (a test checks it).
- **Table clearance:** a pose whose lowest point (wrist or tool tip) is under 20 mm above the table is **not sent**
  (`minHeightMm()`).
- **Bad angles** (NaN, infinite) or a joint outside 1..6 throw an error.
- **PID writes** are bounded: P and D at most 1.5 × factory, I at most 8, and every value is read back.

## 5. Fixed values

| Name | Value | Used by |
|---|---|---|
| `ZERO_ANGLES` | `0, 0, 0, 0, 0, 0` | `make zero` |
| `PARK_ANGLES` | `0, 131, -148, 146, 4.5, 0` (folded, wrist 160 mm above the table) | `make park` |
| `MOVE_SPEED` | `30` | every zero / park move |
| `POSE_TOLERANCE_DEG` | `2.0` | when a pose counts as reached |
| `TEMP_WARM_C` / `TEMP_HOT_C` | `55` / `65` °C (the servos protect themselves at about 70) | `temperature_monitor` |
| `FACTORY_PID` | J1 `15/0/20`, J2-J3 `32/0/8`, J4-J6 `25/0/25` (P/I/D) | `pid_tool` |

## 6. Other devices on the Jetson

| Device | Where | Used by |
|---|---|---|
| LCD1602 RGB (Waveshare) | I2C bus 0: `0x3E` LCD, `0x60` backlight | `temperature_monitor`, `temps_lcd` |
| Joystick | `/dev/input/js0` | `joystick_teleop`, `joystick_all_joints` |
