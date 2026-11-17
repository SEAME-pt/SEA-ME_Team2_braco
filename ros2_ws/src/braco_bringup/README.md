# braco_bringup

Starts the braco system. Nothing here talks to the hardware: it only decides which nodes run, with which
parameters.

| File | What it does |
|---|---|
| `launch/braco.launch.py` | The whole system. `sim:=false` (default): `arm_driver`, `temperature_monitor`, `joystick_teleop`, `camera`, and ros2_control (`robot_state_publisher` + `ros2_control_node`) with `braco_hardware` and the `arm_controller` spawner (no `joint_state_broadcaster`: `arm_driver` publishes `/joint_states`, #97). When `arm_driver` exits, the launch shuts down (so the container stops). `sim:=true` (PC only, the `sim` container): the simulated arm, `braco_simulation`'s `gazebo.launch.py` (Gazebo, the model, `joint_state_broadcaster` + `arm_controller`); `gui:=true` opens the Gazebo window. When Gazebo exits, the launch shuts down. `moveit:=true` runs `braco_moveit_config`'s `sim.launch.py` instead, which adds `move_group`; `rviz:=true` also opens RViz. |
| `config/braco.yaml` | Tuning parameters of the nodes (rates, watchdog, control idle time, auto rest on overheat, the joystick's force button, the camera's size and rate). The wiring (`port`, `baud`, `lcd_bus`, `device`, `camera`) comes from the launch arguments, filled from `docker/.env`. |
| `scripts/arm_routine.sh` | Entrypoint of the system containers (`real`, `sim`): the launch only (the web page is `make web`). Stops it on `docker stop`; if the launch dies, the container stops. |
| `src/zero.cpp`, `src/park.cpp` | Move the arm to the zero pose / fold it, release the joints and enter rest mode. C++ ROS 2 nodes, the counterparts of `mycobot_cpp/srcs/zero.cpp` and `park.cpp`: same poses (`mycobot::ZERO_ANGLES`, `PARK_ANGLES`) and speed, but through `arm_driver` (`src/arm_moves.cpp`), never the port. They take the control forced (`source: script`, from the page or the joystick) and give it back at the end, also on failure. |

```bash
make up                # on the Jetson: the `real` container runs arm_routine.sh
make zero              # ros2 run braco_bringup zero inside it
make park              # ros2 run braco_bringup park inside it

# by hand, in the dev container
ros2 launch braco_bringup braco.launch.py                       # real arm
ROS_DOMAIN_ID=78 ros2 launch braco_bringup braco.launch.py sim:=true gui:=true   # the simulated arm, with a window
ROS_DOMAIN_ID=78 ros2 launch braco_bringup braco.launch.py sim:=true moveit:=true gui:=true rviz:=true   # with MoveIt too
ros2 launch braco_bringup braco.launch.py port:=/dev/ttyUSB0    # other serial port
ros2 launch braco_bringup braco.launch.py --show-args
```

The arm is never zeroed or parked by starting or stopping the system: both are manual.

`braco_description` (the robot model, no Gazebo) is a dependency: the real arm's ros2_control needs its URDF.
`braco_simulation` and `braco_moveit_config` are not: they need Gazebo and MoveIt, which only the PC image has, and the
Jetson builds only up to `braco_bringup`. `sim:=true` looks them up when it runs. How to launch the simulation and
command a joint: [Run the simulation](../braco_simulation/README.md#run-it).
