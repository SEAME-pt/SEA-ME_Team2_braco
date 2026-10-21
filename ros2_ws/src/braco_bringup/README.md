# braco_bringup

Starts the braco system. Nothing here talks to the hardware: it only decides which nodes run, with which
parameters.

| File | What it does |
|---|---|
| `launch/braco.launch.py` | The whole system. `sim:=false` (default): `arm_driver`, `temperature_monitor`, `joystick_teleop`. When `arm_driver` exits, the launch shuts down (so the container stops). `sim:=true`: reserved for the simulated arm (Gazebo #34, model from `braco_description` #25); for now it starts nothing and says so. |
| `config/braco.yaml` | Tuning parameters of the nodes (rates, watchdog, control idle time, auto rest on overheat, the joystick's force button). The wiring (`port`, `baud`, `lcd_bus`, `device`) comes from the launch arguments, filled from `docker/.env`. |
| `scripts/arm_routine.sh` | Entrypoint of the system containers (`real`, `sim`): the launch plus the braco web page. Stops both on `docker stop`; if the launch dies, the container stops. |
| `src/zero.cpp`, `src/park.cpp` | Move the arm to the zero pose / fold it, release the joints and enter rest mode. C++ ROS 2 nodes, the counterparts of `libs/cpp/srcs/zero.cpp` and `park.cpp`: same poses (`mycobot::ZERO_ANGLES`, `PARK_ANGLES`) and speed, but through `arm_driver` (`src/arm_moves.cpp`), never the port. They take the control forced (`source: script`, from the page or the joystick) and give it back at the end, also on failure. |

```bash
make up                # on the Jetson: the `real` container runs arm_routine.sh
make zero              # ros2 run braco_bringup zero inside it
make park              # ros2 run braco_bringup park inside it

# by hand, in the dev container
ros2 launch braco_bringup braco.launch.py                       # real arm
ros2 launch braco_bringup braco.launch.py port:=/dev/ttyUSB0    # other serial port
ros2 launch braco_bringup braco.launch.py --show-args
```

The arm is never zeroed or parked by starting or stopping the system: both are manual.
