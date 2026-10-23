# braco_driver

The nodes that sit next to the hardware of the real arm. Built on `mycobot_cpp` (`../mycobot_cpp`).

## arm_driver

The **only** process that opens the myCobot 280 serial port (`/dev/ttyTHS1`, 1 Mbaud). Every other node, the
web page and the zero/park scripts reach the arm through its topics and services, so they can start, stop and
crash independently while the port keeps a single owner. It never powers the arm on or moves it by itself.

| | Name | Type |
|---|---|---|
| pub | `/joint_states` | `sensor_msgs/JointState` (radians, `joint1..joint6`, 20 Hz) |
| pub | `/arm/servo_temperatures` | `braco_interfaces/ServoTemperatures` (every 2 s, also in rest mode) |
| pub | `/arm/read_only` | `std_msgs/Bool`, latched: rest mode on/off |
| pub | `/arm/control_owner` | `std_msgs/String`, latched: who may move the arm (`""` = free) |
| sub | `/arm/command` | `braco_interfaces/ArmCommand` |
| sub | `/arm/led` | `std_msgs/ColorRGBA` |
| srv | `/arm/take_control` | `braco_interfaces/TakeControl` |
| srv | `/arm/power_on`, `/arm/release_servos` | `std_srvs/Trigger` |
| srv | `/arm/set_gripper`, `/arm/set_read_only` | `std_srvs/SetBool` |

**Control owner** ([ADR-0002](../../../docs/ADR-0002-unique-control.md)): one client moves the arm at a time.
`/arm/take_control` gives the control to a `source` (`web`, `joystick`, `script`, `mission`) when it is free,
already that source's, or with `force` (which drops the previous owner's pending motion and stops the arm).
Motion (`JOINTS`, `ALL_ANGLES`) is obeyed only from the owner; **STOP always**, from anyone. The owner loses
the control with `release`, when another client forces it, or after `control_idle_s` (5 s) without motion.
`power_on`, `set_gripper` and `release_servos` carry no source: clients call them only while they own the
control. The rule is `ControlLease` (`include/braco_driver/control.hpp`), tested by `test/test_control.cpp`.

```bash
ros2 topic echo /arm/control_owner
ros2 service call /arm/take_control braco_interfaces/srv/TakeControl "{source: script, force: true}"
ros2 service call /arm/take_control braco_interfaces/srv/TakeControl "{source: script, release: true}"
```

Safety: angles are clamped to `JOINT_LIMITS` of `mycobot.hpp` and poses that put the wrist or tool below the table margin are refused; a **watchdog** stops the arm when motion commands
stop for `command_timeout_ms` (300 ms); in **rest mode** motion, gripper and power on are refused while angles and
temperatures keep being published. Not respawned on purpose: a driver fault must be seen (the launch shuts
down when it exits, and the container stops).

## temperature_monitor

Reads `/arm/servo_temperatures` (never the port), shows them on the LCD1602 (`/dev/i2c-0`) and publishes
`/diagnostics` per joint: normal, warm from 55 °C, hot from 65 °C. Shows "no feedback" when nothing arrives for
`stale_timeout_s`.

**Rest on overheat** ([ADR-0003](../../../docs/ADR-0003-rest-per-temp.md)): the first reading with a
joint at 65 °C or above runs `park` (fold, release the joints so the servos cool, rest mode on; parameter
`park_on_hot`, default `true`), once per episode (the episode ends when every joint is
below 60 °C, so a reading that jitters around 65 °C does not park it again), logs it (if `park` cannot run or fails, it calls `/arm/set_read_only true` instead) and shows `REST: SERVO HOT` / `J4 66 C cooling` on the LCD. It never leaves rest mode:
`make resume` does. Parameter `auto_rest_on_hot` (default `true`); the decision is `AutoRest`
(`include/braco_driver/auto_rest.hpp`), tested by `test/test_control.cpp`.

## Run

Both start with the whole system: `make up` on the Jetson (`braco_bringup/launch/braco.launch.py`).
Parameters are in `braco_bringup/config/braco.yaml`. By hand, in the dev container:

```bash
ros2 run braco_driver arm_driver --ros-args -p port:=/dev/ttyTHS1
ros2 run braco_driver temperature_monitor
```
