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
| Button 8, held 1 s | take the control from the web page or zero/park |

| | Name |
|---|---|
| sub | `/joint_states`, `/arm/control_owner` |
| pub | `/arm/command` (`source: joystick`), `/arm/led` |
| client | `/arm/take_control`, `/arm/power_on` (when a joystick connects, once it has the control), `/arm/set_gripper` |

**Control** ([ADR-0002](../../../docs/ADR-0002-unique-control.md)): only the control owner moves the arm. With
the control free, the first stick move (or X, A, B) takes it. While the web page or zero/park has it, the
joystick is ignored and the LED is dimmed; hold **button 8 for 1 s** (`force_button`, `force_hold_s`) to take
it anyway. Letting go does not release it: `arm_driver` frees it 5 s after the last move. A joystick without
the control never sends STOP, so unplugging it does not cut somebody else's move.

The jogging logic (dead zone, speed modes, slow-down near the soft limits, homing steps, the held button) is in
`ros2_ws/src/mycobot_cpp/includes/teleop_logic.hpp` (shared with the stand-alone `joystick_all_joints`), without ROS, and tested by `test/test_teleop_logic.cpp` (`make test`).

Starts with the whole system (`make up`); by hand: `ros2 run braco_teleop joystick_teleop`.
