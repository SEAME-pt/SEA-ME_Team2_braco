# braco_teleop

`joystick_teleop`: drives the arm from a joystick (`/dev/input/js0`) through `arm_driver`, never the serial port.
It waits for the joystick, survives unplugging (stops the arm and retries every second) and only jogs with
fresh `/joint_states`: every target is a few degrees ahead of the real angle, so the arm coasts very little if
anything dies.

| Input | Action |
|---|---|
| Left stick X / Y | joint 1 (base, inverted) / joint 2 (shoulder) |
| D-pad Y | joint 3 (elbow) |
| Right stick Y / X | joint 4 / joint 5 |
| L1 / R1 | joint 6 (tool rotation), exclusive while held |
| A / B | close / open the gripper |
| X | home all joints to 0 (click again, or touch a stick, to abort) |
| Button 9 | cycle the speed mode: blue → green → purple → white → red (LED colour) |

| | Name |
|---|---|
| sub | `/joint_states` |
| pub | `/arm/command`, `/arm/led` |
| client | `/arm/power_on` (when a joystick connects), `/arm/set_gripper` |

The jogging logic (dead zone, speed modes, slow-down near the soft limits, homing steps) is in
`include/braco_teleop/teleop_logic.hpp`, without ROS, and tested by `test/test_teleop_logic.cpp` (`make ws-test`).

Starts with the whole system (`make arm-up`); by hand: `ros2 run braco_teleop joystick_teleop`.
