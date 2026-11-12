# braco_interfaces

Messages and services of the braco arm. Every node that commands or watches the myCobot 280 uses these types.

| Type | Used on | Meaning |
|---|---|---|
| `msg/ArmCommand` | `/arm/command` | Motion for `arm_driver`: `STOP`, `JOINTS` (one target per joint), `ALL_ANGLES` (six angles at one speed) or `HALT`, plus `source` (`web`, `joystick`, `script`, `mission`). The newest command replaces any pending one. Only the control owner's motion is obeyed; `STOP` and `HALT` always are. `STOP` latches until `/arm/release_stop`, `HALT` (routine stop) does not ([ADR-0005](../../../docs/decisions/ADR-0005-stop-latch.md)). |
| `msg/JointTarget` | inside `ArmCommand` | One joint goal: joint 1..6, angle in degrees, speed 1..100. |
| `msg/ServoTemperatures` | `/arm/servo_temperatures` | The six servo temperatures in °C; `valid = false` when the arm did not answer. |
| `msg/ServoVoltages` | `/arm/servo_voltages` | The six servo supply voltages in V; `valid = false` (and all zero) when the arm did not answer. |
| `srv/TakeControl` | `/arm/take_control` | Become the control owner (`source`, `force`) or give it back (`release`). Answers `success`, the `owner` after the call (`""` = free) and a `message`. |
| `action/MoveToPose` | `/move_to_pose` (`braco_mission`) | Move the `tcp` to a `PoseStamped` through MoveIt 2. An orientation of all zeros asks for the position only; `position_only_fallback` retries with the position when the full pose has no plan. Result: `success`, the position and orientation error measured from TF after the move, `position_only`, `message`. Feedback: the `stage`. |

```bash
ros2 interface show braco_interfaces/msg/ArmCommand
ros2 topic echo --once /arm/servo_temperatures     # on the Jetson, with the arm running
ros2 service call /arm/take_control braco_interfaces/srv/TakeControl "{source: script, force: true}"
```

Python: `from braco_interfaces.msg import ArmCommand`. C++: `#include "braco_interfaces/msg/arm_command.hpp"`,
then link `${braco_interfaces_TARGETS}`.
