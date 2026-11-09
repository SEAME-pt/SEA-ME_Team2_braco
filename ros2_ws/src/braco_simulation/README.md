# braco_simulation

The braco arm in Gazebo, on the PC. It uses the robot model of
[`braco_description`](../braco_description/README.md) and the same controllers as the real arm.

- Runs on the **PC only**: the Jetson has no Gazebo and never builds this package
- `braco_bringup` starts it with `sim:=true` (the `sim` container, `make up SIM=1`)

## What is in it

| File | What it does |
|---|---|
| `launch/gazebo.launch.py` | Starts Gazebo, puts the arm in it and starts the controllers |
| `scripts/move_joint.py` | Moves one joint (`move_joint.py 2 30`: joint2 to 30°) |
| `scripts/move_arm_example.py` | A few poses, written once: copy it to make your own |
| `scripts/park.py` | The park pose |
| `scripts/sim_to_real.py` | Compares the model with the real arm: `directions`, `fk`, `velocity` |
| `scripts/braco_arm.py` | The helper the scripts above use (Gazebo, then `--real` on the arm) |

## How it fits

| Piece | Where it comes from |
|---|---|
| The robot model (URDF) | `braco_description`, with `gazebo:=true` |
| The controllers (`arm_controller`, `joint_state_broadcaster`) | `braco_description/config/controllers.yaml` |
| The hardware plugin | `GazeboSimSystem` (on the real arm: `braco_hardware`) |
| `/joint_states` | `joint_state_broadcaster` (on the real arm: `arm_driver`) |

It runs on **ROS domain 77**, never the real arm's 42: both publish `/joint_states`.

## Run it

On the PC, from the repo root:

| Step | Command | What you get |
|---|---|---|
| 1. Start | `make up SIM=1` | Gazebo running in the `sim` container (no window) |
| 2. Watch | `make web` → http://localhost:8000, or `make sim-gui` | The arm in the web page, or a Gazebo window |
| 3. Move | in `make shell`: `ros2 run braco_simulation move_joint.py 2 30` | joint2 goes to 30° |
| 4. Check | `ros2 control list_controllers` | `arm_controller` and `joint_state_broadcaster` active |
| 5. Stop | `make down` | |

More moves, in `make shell`:

```bash
ros2 run braco_simulation move_arm_example.py      # a few poses
ros2 run braco_simulation park.py                  # the park pose
ros2 run braco_simulation park.py --real           # then the same on the real arm (asks first)

# one move by hand: six angles in radians, reached in 2 s
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.3, -0.3, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"
```

Your own moves: copy `scripts/move_arm_example.py` to `scripts/my_test.py`, edit its `MOVES` list and run
`python3 /ws/src/braco_simulation/scripts/my_test.py` (no rebuild). Add `--real --speed 15` to run them on the arm
afterwards: it takes the control as `script` and gives it back at the end
([ADR-0002](../../../docs/decisions/ADR-0002-unique-control.md)).

A second Gazebo by hand (to work on the model), on its own domain so it does not fight the `sim` container:

```bash
export ROS_DOMAIN_ID=78
ros2 launch braco_simulation gazebo.launch.py      # gui:=false: no window
```

## When something goes wrong

| What you see | Fix |
|---|---|
| Slow, or "Gazebo is taking too long to start" | No GPU in use. Restart with `make down && make up SIM=1` |
| No window, `could not connect to display` | See [TROUBLESHOOTING → GUI tools](../../../docs/guides/ROS2/TROUBLESHOOTING.md#gui-tools-rqt-rqt_graph-rviz2-gazebo) |
| `Controller already loaded`, `Moved backwards in time` | Two simulations on one domain. Give your own Gazebo another `ROS_DOMAIN_ID` |
| `arm_controller not found on ROS domain ...` | The shell is on another domain than the simulation |
| The `sim` container stopped | Gazebo exited: `make logs`, then `make up SIM=1` |
| The model looks old after a URDF change | Gazebo reads the model at start: launch it again |
