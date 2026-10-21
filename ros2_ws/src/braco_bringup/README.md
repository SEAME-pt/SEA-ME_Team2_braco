# braco_bringup

Starts the braco system. Nothing here talks to the hardware: it only decides which nodes run, with which
parameters.

| File | What it does |
|---|---|
| `launch/braco.launch.py` | The whole system. `sim:=false` (default): `arm_driver`, `temperature_monitor`, `joystick_teleop`. `sim:=true`: reserved for the simulated arm (Gazebo #34, model from `braco_description` #25); for now it starts nothing and says so. |
| `config/braco.yaml` | Tuning parameters of the nodes (rates, watchdog, timeouts). The wiring (`port`, `baud`, `lcd_bus`, `device`) comes from the launch arguments, filled from `docker/.env`. |
| `scripts/arm_routine.sh` | Entrypoint of the system containers (`real`, `sim`): the launch plus the braco web page. Stops both on `docker stop`; if the launch dies, the container stops. |
| `scripts/zero.py`, `park.py` | Move the arm to the zero pose / fold it, release the joints and enter rest mode. Through `arm_driver` (`arm_moves.py`), never the port. |

```bash
make up                # on the Jetson: the `real` container runs arm_routine.sh
make zero              # ros2 run braco_bringup zero.py inside it
make park              # ros2 run braco_bringup park.py inside it

# by hand, in the dev container
ros2 launch braco_bringup braco.launch.py                       # real arm
ros2 launch braco_bringup braco.launch.py port:=/dev/ttyUSB0    # other serial port
ros2 launch braco_bringup braco.launch.py --show-args
```

The arm is never zeroed or parked by starting or stopping the system: both are manual.
