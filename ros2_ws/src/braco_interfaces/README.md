# braco_interfaces

Messages of the braco arm. Every node that commands or watches the myCobot 280 uses these types.

| Type | Used on | Meaning |
|---|---|---|
| `msg/ArmCommand` | `/arm/command` | Motion for `arm_driver`: `STOP`, `JOINTS` (one target per joint) or `ALL_ANGLES` (six angles at one speed). The newest command replaces any pending one. |
| `msg/JointTarget` | inside `ArmCommand` | One joint goal: joint 1..6, angle in degrees, speed 1..100. |
| `msg/ServoTemperatures` | `/arm/servo_temperatures` | The six servo temperatures in °C; `valid = false` when the arm did not answer. |

```bash
ros2 interface show braco_interfaces/msg/ArmCommand
ros2 topic echo --once /arm/servo_temperatures     # on the Jetson, with the arm running
```

Python: `from braco_interfaces.msg import ArmCommand`. C++: `#include "braco_interfaces/msg/arm_command.hpp"`,
then link `${braco_interfaces_TARGETS}`.
