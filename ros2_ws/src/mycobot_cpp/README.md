# mycobot_cpp

The myCobot 280 without ROS: serial protocol, joint limits and poses, joystick, LCD1602 and teleop logic. It is a
colcon package like the others (so `braco_driver` and `braco_teleop` link it), but depends on nothing from ROS and
publishes no topics.

| Header (`includes/`) | What it holds |
|---|---|
| `mycobot.hpp` | Serial protocol ([protocol_commands.md](../../../docs/hardware/protocol_commands.md)), `JOINT_LIMITS`, `ZERO_ANGLES` / `PARK_ANGLES`, the table margin |
| `joystick.hpp` | `input::Joystick`: reads `/dev/input/js*` events |
| `lcd_display.hpp` | `ui::LcdDisplay`: the LCD1602 over I2C |
| `display.hpp` | `ui::TerminalDisplay`: full-screen terminal status |
| `teleop_logic.hpp` | Jogging logic shared by `joystick_teleop`, `joystick_all_joints` and `joystick_display`; tested in `braco_teleop/test/` |

`braco_description` must keep its joint limits equal to `JOINT_LIMITS`.

**Use from another package:**

```cmake
find_package(mycobot_cpp REQUIRED)
target_link_libraries(my_node mycobot_cpp::mycobot_cpp)
```

**Stand-alone tools** (`srcs/`, built by the `Makefile` next to `CMakeLists.txt`, not by colcon): `zero`, `park`,
`joystick_all_joints`, `joystick_display`, `pid_tool`, `temps_lcd`. They open the serial port themselves, so run
`make down` first. Prefer `make zero`, `make park` and `make temps`, which go through `arm_driver`.

```bash
cd ros2_ws/src/mycobot_cpp && make && ./temps_lcd
```

Their devices can be changed with environment variables (the tests point them at fakes):

| Variable | Default | Used by |
|---|---|---|
| `MYCOBOT_PORT` | `/dev/ttyTHS1` | every tool |
| `MYCOBOT_JOYSTICK` | `/dev/input/js0` | `joystick_all_joints`, `joystick_display` |
| `MYCOBOT_LCD_BUS` | `/dev/i2c-0` | `temps_lcd` |

**Tests** (`test/`, run by `make test` and `make coverage`): `test_serial` and `test_tools` use `PtyArm`
(`test/pty_arm.hpp`), a fake arm on a pseudo-terminal that answers like the real one. It is installed with the
headers, so other packages' tests can use it too: `#include "pty_arm.hpp"` and link `util`.
