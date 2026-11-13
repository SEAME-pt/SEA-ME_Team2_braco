# braco_interfaces

Messages, services and actions of the braco arm. Every node that commands or watches the myCobot 280 uses these types.

| Type | Used on | Meaning |
|---|---|---|
| `msg/ArmCommand` | `/arm/command` | Motion for `arm_driver`: `STOP`, `JOINTS` (one target per joint) or `ALL_ANGLES` (six angles at one speed), plus `source` (`web`, `joystick`, `script`, `mission`). The newest command replaces any pending one. Only the control owner's motion is obeyed; `STOP` always is. |
| `msg/JointTarget` | inside `ArmCommand` | One joint goal: joint 1..6, angle in degrees, speed 1..100. |
| `msg/ServoTemperatures` | `/arm/servo_temperatures` | The six servo temperatures in °C; `valid = false` when the arm did not answer. |
| `msg/ServoVoltages` | `/arm/servo_voltages` | The six servo supply voltages in V; `valid = false` (and all zero) when the arm did not answer. |
| `srv/TakeControl` | `/arm/take_control` | Become the control owner (`source`, `force`) or give it back (`release`). Answers `success`, the `owner` after the call (`""` = free) and a `message`. |
| `action/MoveToPose` | `/move_to_pose` (`braco_mission`) | Move the `tcp` to a `PoseStamped` through MoveIt 2. An orientation of all zeros asks for the position only; `position_only_fallback` retries with the position when the full pose has no plan. Result: `success`, the position and orientation error measured from TF after the move, `position_only`, `message`. Feedback: the `stage`. |
| `action/MoveCartesian` | `/move_cartesian` (`braco_mission`) | Move the `tcp` in straight lines from where it is through `waypoints` (one `header` frame for all) with MoveIt 2 `computeCartesianPath`. A waypoint orientation of all zeros keeps the start orientation; `eef_step_m` and `min_fraction` at 0 take the node's defaults (0.005 m, 1.0). Nothing moves when less than `min_fraction` of the path is planned. Result: `success`, `fraction`, `max_deviation_m` of the planned path from the lines, the position and orientation error at the last waypoint, `message`. Feedback: the `stage`. |

```bash
ros2 interface show braco_interfaces/msg/ArmCommand
ros2 topic echo --once /arm/servo_temperatures     # on the Jetson, with the arm running
ros2 service call /arm/take_control braco_interfaces/srv/TakeControl "{source: script, force: true}"
```

Python: `from braco_interfaces.msg import ArmCommand`. C++: `#include "braco_interfaces/msg/arm_command.hpp"`,
then link `${braco_interfaces_TARGETS}`.
