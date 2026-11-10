# braco_simulation

The braco arm in Gazebo, on the PC, standing on a copy of the lab bench. It uses the robot model of
[`braco_description`](../braco_description/README.md) and the same controllers as the real arm.

- Runs on the **PC only**: the Jetson has no Gazebo and never builds this package
- `braco_bringup` starts it with `sim:=true` (the `sim` container, `make up SIM=1`)
- The `sim` container also runs MoveIt (`move_group`) by default (`SIM_MOVEIT=true`):
  [Run on the Gazebo arm](../braco_moveit_config/README.md#run-on-the-gazebo-arm)

## What is in it

| File | What it does |
|---|---|
| `launch/gazebo.launch.py` | Starts Gazebo with the lab bench, puts the arm on the table and starts the controllers |
| `launch/gazebo_gui.launch.py` | Only the Gazebo window, on a simulation already running (`make gaze-gui`) |
| `config/gazebo_gui.config` | The Gazebo window's first view, close to the arm |
| `worlds/lab_bench.sdf` | The scene: floor, light, the two tables of the real bench; Bullet physics and the camera's Sensors system |
| `models/lab_table/` | One table of the bench (static, the top collides) |
| `scripts/move_joint.py` | Moves one joint (`move_joint.py 2 30`: joint2 to 30°) |
| `scripts/move_arm_example.py` | A few poses, written once: copy it to make your own |
| `scripts/park.py` | The park pose |
| `scripts/sim_to_real.py` | Compares the model with the real arm: `directions`, `fk`, `velocity` |
| `scripts/braco_arm.py` | The helper the scripts above use (Gazebo, then `--real` on the arm) |
| `scripts/gz_track_arm.sh` | Makes the Gazebo window keep looking at the arm |

## How it fits

| Piece | Where it comes from |
|---|---|
| The robot model (URDF) | `braco_description`, with `gazebo:=true` |
| The controllers (`arm_controller`, `joint_state_broadcaster`, `gripper_action_controller`) | `braco_description/config/controllers.yaml` (the gripper one is simulation only) |
| The hardware plugin | `GazeboSimSystem` (on the real arm: `braco_hardware`) |
| `/joint_states` | `joint_state_broadcaster` (on the real arm: `arm_driver`) |
| `/camera/image_raw/compressed` | The wrist camera sensor, through `ros_gz_image` (on the real arm: `camera`) |

It runs on **your ROS domain** (`PC_ROS_DOMAIN_ID` in `docker/.env`, one per teammate, 20–29:
[table](../../../docker/README.md#ros-domain-per-teammate)), never the real arm's 22: both publish `/joint_states`.

## Run it

On the PC, from the repo root:

| Step | Command | What you get |
|---|---|---|
| 1. Start | `make up SIM=1` | Gazebo (no window) and MoveIt in the `sim` container |
| 2. Watch | `make web` → http://localhost:8000 | The 3D arm in the web page; the Control tab moves it |
| | `make gaze-gui`, `make rviz-gui`, `make sim-gui` | The Gazebo window, RViz with MoveIt, or both. Closing one leaves the simulation running |
| 3. Move | in `make shell`: `ros2 run braco_simulation move_joint.py 2 30` | joint2 goes to 30° |
| 4. Check | `ros2 control list_controllers` | `arm_controller`, `joint_state_broadcaster` and `gripper_action_controller` active |
| 5. Stop | `make down` | |

Use the GPU: without one (`GPU=none`) Gazebo draws on the CPU and is slow or freezes. Pass `SIM=1` on every
`make up` / `make restart` that should keep the simulation.

More moves, in `make shell`:

```bash
ros2 run braco_simulation move_arm_example.py      # a few poses
ros2 run braco_simulation park.py                  # the park pose
ros2 run braco_simulation park.py --real           # then the same on the real arm (asks first)

# one move by hand: six angles in radians, reached in 2 s
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.3, -0.3, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# the gripper (radians of its master joint gripper_controller: -0.74 closed .. 0.15 open)
ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/ParallelGripperCommand \
  "{command: {name: [gripper_controller], position: [-0.74]}}"

# the wrist camera (or view /camera/image_raw in RViz, rqt_image_view)
ros2 topic hz /camera/image_raw/compressed
```

Your own moves: copy `scripts/move_arm_example.py` to `scripts/my_test.py`, edit its `MOVES` list and run
`python3 /ws/src/braco_simulation/scripts/my_test.py` (no rebuild). Add `--real --speed 15` to run them on the arm
afterwards: it takes the control as `script` and gives it back at the end
([ADR-0002](../../../docs/decisions/ADR-0002-unique-control.md)). Check first that the PC sees the arm:
`ROS_DOMAIN_ID=22 ros2 node list` shows `/arm_driver`.

A second Gazebo by hand (to work on the model), on its own domain so it does not fight the `sim` container:

```bash
export ROS_DOMAIN_ID=78     # not 22 (the real arm), not 20-29 (the teammates' sim containers)
ros2 launch braco_simulation gazebo.launch.py      # gui:=false: no window
```

## The lab bench

The scene copies the real bench: two white tables side by side with a narrow gap, the arm at the rear edge of one of
them, next to the gap, facing the table top.

| Point | What to know |
|---|---|
| Gazebo's origin | On the floor, under the gap, at the rear edge. x along the gap, y across it, z up. The top is at 0.75 m |
| The model's `world` frame | Stays the table surface: TF, MoveIt and the table-clearance check do not change. Only the spawn pose places the arm |
| The pedestal | Grey, only the look of `g_base` with `gazebo:=true`, as tall as the real mount (30 mm) |
| Collisions | The table stops the arm in Gazebo and `arm_controller` ends the goal as failed (`PATH_TOLERANCE_VIOLATED` or `GOAL_TOLERANCE_VIOLATED`; through MoveIt, `CONTROL_FAILED`): within 0.3 rad of the trajectory while moving, within 0.07 rad of the last point 0.5 s after it (`constraints` in `controllers.yaml`) |
| MoveIt knows the table | `braco_moveit_config`'s `sim.launch.py` adds the table tops to the planning scene (`lab_bench_scene.py`, read from `lab_bench.sdf` and `lab_table/model.sdf`): MoveIt refuses a goal in the table (`GOAL_STATE_INVALID`) or plans around it. Their top is the table surface, not the real arm's 20 mm margin (`TABLE_MARGIN_MM`): `arm_driver` still refuses that part |
| The gripper | Moves only here (`gripper_actuated:=true`, `gripper_action_controller`); Bullet Featherstone enforces its five mimic joints. On the real arm the fingers stay fixed |
| Wrist camera | A sensor at `camera_link`, 640×480 at 15 Hz like the real one, on the real camera's topics, so the web page shows it as is. Field of view (60°) `UNMEASURED`. Renders on the GPU, also with `gui:=false` |
| One simulation per domain | `gazebo.launch.py` (and `sim.launch.py`) refuses to start when a `/controller_manager` already runs (e.g. the `sim` container): use that one, or `make down` first |
| Unmeasured | The table and pedestal sizes and the arm's position are provisional (`UNMEASURED` in the files) |

Move the arm without editing a file (`braco.launch.py sim:=true` passes the same arguments on):

```bash
ros2 launch braco_simulation gazebo.launch.py robot_x:=0.2 robot_y:=0.3 robot_yaw:=0.5   # metres, radians
ros2 launch braco_simulation gazebo.launch.py world:=empty.sdf robot_z:=0                # the empty world
```

## When something goes wrong

| What you see | Fix |
|---|---|
| Slow, or "Gazebo is taking too long to start" | No GPU in use (`glxinfo -B` says `llvmpipe`). Restart with `make down && make up SIM=1` |
| No window, `could not connect to display` | See [TROUBLESHOOTING → GUI tools](../../../docs/guides/ROS2/TROUBLESHOOTING.md#gui-tools-rqt-rqt_graph-rviz2-gazebo) |
| Stuck on `Requesting list of world names` | `GZ_IP` must be `127.0.0.1` (set by `docker/compose.yml`): recreate the containers |
| `Controller already loaded`, `Moved backwards in time` | Two simulations on one domain. Give your own Gazebo another `ROS_DOMAIN_ID` |
| `arm_controller not found on ROS domain ...` | The shell is on another domain than the simulation |
| `/joint_states` with `gripper_controller` or values like `1e-11` on domain 22 | That is a simulation, not the real arm: someone runs Gazebo on 22 |
| `Command of at least one joint is out of limits` | The joint limiter slowing a too-fast move; the move still finishes |
| The `sim` container stopped | Gazebo exited: `make logs`, then `make up SIM=1` |
| The model looks old after a URDF change | Gazebo reads the model at start: launch it again |
