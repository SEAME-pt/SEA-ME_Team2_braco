# braco_driver

The nodes that sit next to the hardware of the real arm. Built on `mycobot_cpp` (`libs/cpp`).

## arm_driver

The **only** process that opens the myCobot 280 serial port (`/dev/ttyTHS1`, 1 Mbaud). Every other node, the
web page and the zero/park scripts reach the arm through its topics and services, so they can start, stop and
crash independently while the port keeps a single owner. It never powers the arm on or moves it by itself.

| | Name | Type |
|---|---|---|
| pub | `/joint_states` | `sensor_msgs/JointState` (radians, `joint1..joint6`, 20 Hz) |
| pub | `/arm/servo_temperatures` | `braco_interfaces/ServoTemperatures` (every 2 s, also in rest mode) |
| pub | `/arm/read_only` | `std_msgs/Bool`, latched: rest mode on/off |
| sub | `/arm/command` | `braco_interfaces/ArmCommand` |
| sub | `/arm/led` | `std_msgs/ColorRGBA` |
| srv | `/arm/power_on`, `/arm/release_servos` | `std_srvs/Trigger` |
| srv | `/arm/set_gripper`, `/arm/set_read_only` | `std_srvs/SetBool` |

Safety: angles are clamped to the soft limits of `mycobot.hpp`; a **watchdog** stops the arm when motion commands
stop for `command_timeout_ms` (300 ms); in **rest mode** motion, gripper and power on are refused while angles and
temperatures keep being published. Not respawned on purpose: a driver fault must be seen.

## temperature_monitor

Reads `/arm/servo_temperatures` (never the port), shows them on the LCD1602 (`/dev/i2c-0`) and publishes
`/diagnostics` per joint: normal, warm from 55 °C, hot from 65 °C. Shows "no feedback" when nothing arrives for
`stale_timeout_s`.

## Run

Both start with the whole system: `make arm-up` on the Jetson (`braco_bringup/launch/braco.launch.py`).
Parameters are in `braco_bringup/config/braco.yaml`. By hand, in the dev container:

```bash
ros2 run braco_driver arm_driver --ros-args -p port:=/dev/ttyTHS1
ros2 run braco_driver temperature_monitor
```
