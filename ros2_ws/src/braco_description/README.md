# braco_description

Model of the myCobot 280 with the adaptive gripper and a camera mount, for the braco stack: links, joints,
limits, meshes and the TF frames the rest of the system uses. Issue **#25**. The validation against the real
arm (zero pose, joint directions, forward kinematics) and the open measurements are in
[`docs/hardware/urdf-sim-to-real.md`](../../../docs/hardware/urdf-sim-to-real.md).

| Folder | Content |
|---|---|
| `urdf/braco.urdf.xacro` | The whole robot: `world` -> G-base -> arm -> `tool0` -> gripper, `tcp`, camera frames |
| `urdf/mycobot_280.urdf.xacro` | The arm (macro): `base_link`, `link1..link6`, `joint1..joint6` |
| `urdf/adaptive_gripper.xacro` | The gripper (macro), fingers fixed open or with the vendor linkage |
| `urdf/braco.gazebo.xacro` | `gz_ros2_control` on `joint1..joint6`, added only with `gazebo:=true` |
| `config/gazebo_gui.config` | The Gazebo window: the first view of the camera, close to the arm |
| `config/controllers.yaml` | The ros2_control controllers: `arm_controller` (joint trajectory, sim and real arm) and `joint_state_broadcaster` (sim and MoveIt demo, not the real arm: ADR-0004) |
| `meshes/` | Visual and collision meshes of Elephant Robotics (`LICENSE.elephantrobotics`) |
| `worlds/lab_bench.sdf` | The default scene: floor, light and the two tables of the real bench |
| `models/lab_table/` | One table of the bench (static, the top collides) |
| `launch/gazebo.launch.py` | The simulated arm: Gazebo (gz sim), the scene, the model and the controllers. Included by `braco_bringup`'s `sim:=true` |
| `scripts/braco_arm.py` | Runs a list of moves in Gazebo, then (`--real`) the same moves on the real arm |
| `scripts/move_joint.py` | Moves one joint of the simulated arm (`move_joint.py 2 30`: joint2 to 30°) |
| `scripts/move_arm_example.py` | Example: a few poses, written once, run in Gazebo and on the arm |
| `scripts/park.py` | The park pose (`PARK_ANGLES` of `mycobot.hpp`), in Gazebo and on the arm |
| `launch/gazebo_gui.launch.py` | Only the Gazebo window, on a simulation already running (`make gaze-gui`) |
| `scripts/gz_track_arm.sh` | Turns on Track in the Gazebo window, so the camera keeps looking at the arm |
| `scripts/sim_to_real.py` | Issue #25 checks against the real arm: `directions`, `fk`, `velocity` |
| `test/test_urdf.py` | Keeps the URDF and the code in sync |

## The simulation (issue #34)

```
 braco_bringup  braco.launch.py sim:=true            (the `sim` container, ROS domain PC_ROS_DOMAIN_ID)
   └─ braco_description  gazebo.launch.py
        gz sim (lab_bench.sdf) ── spawns braco.urdf.xacro gazebo:=true, on the table
          └─ gz_ros2_control (hardware: GazeboSimSystem, joint1..joint6, position)
               ├─ joint_state_broadcaster  ──► /joint_states
               └─ arm_controller           ◄── /arm_controller/follow_joint_trajectory  (action)
                                           ◄── /arm_controller/joint_trajectory         (topic, web page)
        robot_state_publisher ──► /tf        ros_gz_bridge ──► /clock
```

The controllers are `config/controllers.yaml`. On the real arm only the URDF `<hardware>` plugin changes (planned: a
braco `SystemInterface` over `arm_driver`, #87), so MoveIt, the mission and the page see the same `arm_controller`.
`joint_state_broadcaster` is not used on the real arm: there `arm_driver` is the only `/joint_states` publisher
([ADR-0004](../../../docs/decisions/ADR-0004-trajectories-through-arm-driver.md), #97).
The validation of the model against the real arm is [`docs/hardware/urdf-sim-to-real.md`](../../../docs/hardware/urdf-sim-to-real.md).

## The lab bench

The scene mirrors the real bench: two white tables joined along their long sides with a narrow gap, the arm at the rear
edge (a short side) of one of them, next to the gap, facing the table top.

- **Gazebo's origin** is on the floor, under the gap, at the rear edge. x points from the rear edge along the gap, y
  across it, z up. The top is at 0.75 m.
- **The model's `world` frame stays the table surface**, so TF, MoveIt and the table-clearance check see no change. Only
  the spawn pose puts the arm in the scene.
- **The pedestal** (grey) is only the look of `g_base` with `gazebo:=true`. It has the
  height of the measured mount (`g_base_height`, 30 mm), so the kinematics do not change. The plain model keeps the mesh.
- **Collisions:** the table top stops the arm in Gazebo. `arm_controller` then ends the goal as failed
  (`PATH_TOLERANCE_VIOLATED` or `GOAL_TOLERANCE_VIOLATED`; through MoveIt, `CONTROL_FAILED`): every joint must stay
  within 0.3 rad of the trajectory while moving and end within 0.07 rad of its last point, 0.5 s after it
  (`constraints` in `config/controllers.yaml`). MoveIt does not know the table, so it can plan through it.
- **One simulation per ROS domain:** `gazebo.launch.py` (and `sim.launch.py`) refuses to start when a
  `/controller_manager` already runs, e.g. the `sim` container. Started anyway, its controllers would not start and
  its Gazebo server would be left running. Use the running one, or `make down` first.
- **Unmeasured:** the sizes of the table and the pedestal and the arm's position are provisional, marked `UNMEASURED` in
  `models/lab_table/model.sdf`, `worlds/lab_bench.sdf`, `braco.urdf.xacro` and `gazebo.launch.py`.

Move the arm without editing a file (`braco.launch.py sim:=true` passes the same arguments on):

```bash
ros2 launch braco_description gazebo.launch.py robot_x:=0.2 robot_y:=0.3 robot_yaw:=0.5   # metres, radians
ros2 launch braco_description gazebo.launch.py world:=empty.sdf robot_z:=0                # the empty world
```

In the `sim` container these are not set: to try another position, stop it and use a dev shell (step 4 below).

## Run the simulation, step by step

On the **PC** (Gazebo only runs there; the Jetson image has none). Commands in `make` run on the host from the repo
root; the others in a container shell (`make shell`).

**1. Start the simulation, with the GPU.** A plain `make up` on the PC starts only `dev`; `SIM=1` adds `sim`.

```bash
make up SIM=1               # with the NVIDIA GPU (the PC default; GPU=dri for Intel/AMD, GPU=none = CPU, slow)
```

Every `make` command on the PC uses the NVIDIA GPU (`make doctor` checks the NVIDIA container runtime). Without a GPU
Gazebo draws on the CPU (`llvmpipe`) and is slow or freezes. Pass `SIM=1` on every `make up` / `make restart` that
should keep the simulation.

That is the simulation: the `sim` container runs `braco.launch.py sim:=true` (Gazebo headless, the model, the
controllers and, with `SIM_MOVEIT=true`, the default, `move_group`). `make up SIM=1` waits until it publishes
`/joint_states`. To plan in RViz: [Run on the Gazebo arm](../braco_moveit_config/README.md#run-on-the-gazebo-arm).
It runs on **your ROS domain** (`PC_ROS_DOMAIN_ID` in `docker/.env`, set by `make setup`: see the
[table](../../../docker/README.md#ros-domain-per-teammate); also the PC's `dev`), never the real arm's 22:
both publish `/joint_states`. If Gazebo exits, the container stops, like `real` when `arm_driver` exits.

**2. Watch it.** Either:

- the **web page**: `make web`, then http://localhost:8000: the 3D arm follows the simulation, and the Control tab
  moves it; or
- the **windows**: `make gaze-gui` opens Gazebo on that running simulation, `make rviz-gui` opens RViz with MoveIt, and
  `make sim-gui` opens both. They run in the background: closing a window ends only that window, the simulation keeps
  running. The Gazebo window starts about 0.65 m from the arm, tracking it: the camera stays where it is and turns to
  keep looking at the arm. To change the first view, edit `camera_pose` in `config/gazebo_gui.config`. Or `SIM_GUI=true` in `docker/.env`, then `make restart SIM=1`: the `sim` container opens Gazebo itself. Do
  **not** run `gazebo.launch.py` in `dev` for this: that is a second simulation (step 4).

**3. Command a joint.** In a dev shell (already on the simulation's domain, your `PC_ROS_DOMAIN_ID`):

```bash
make shell
source /ws/install/setup.bash

ros2 run braco_description move_joint.py 2 30       # joint2 to +30 degrees in 2 s (--seconds to change)
ros2 run braco_description move_arm_example.py      # a few poses, waits for each move
ros2 run braco_description park.py                  # the park pose

# or one move by hand: six angles in radians, reached in 2 s
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.3, -0.3, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

ros2 topic echo /joint_states --field position      # the joint angles (radians)
ros2 control list_controllers                       # joint_state_broadcaster and arm_controller: active
```

**4. Gazebo by hand instead (optional).** To work on the model (a URDF or mesh change needs a new launch), run
a second Gazebo yourself in `dev`, with a window. `dev` is on the `sim` container's domain (yours), so either stop
`sim` first (`make down`, then `make shell`) or give your Gazebo another domain as below. Two simulations on one domain
fight: `Controller already loaded`, `no controller with this name exists`, `Moved backwards in time`.

```bash
make shell
source /ws/install/setup.bash
export ROS_DOMAIN_ID=78     # not 22 (the real arm), not 20-29 (the teammates' sim containers)
                            # (each new shell starts on your PC_ROS_DOMAIN_ID again: export it there too)
ros2 launch braco_description gazebo.launch.py      # gui:=false: no window
```

The scripts of step 3 work the same with `export ROS_DOMAIN_ID=78`.

**5. Your own moves:** copy `scripts/move_arm_example.py` (e.g. to `scripts/my_test.py`), edit its `MOVES` list
(six angles in degrees, seconds in Gazebo) and run it straight from the source folder, no rebuild:

```bash
python3 /ws/src/braco_description/scripts/my_test.py
```

**6. Then on the real arm (optional).** With the Jetson up (`make up` there), `--real` runs the same moves on the
arm after Gazebo, and asks first. Like `make zero` / `make park`, the script takes the arm's control as `script`
(forced: it takes it from the web page or the joystick, [ADR-0002](../../../docs/decisions/ADR-0002-unique-control.md)) and
gives it back when it ends. Check first that the PC sees the arm: `ROS_DOMAIN_ID=22 ros2 node list` shows
`/arm_driver`.

```bash
python3 /ws/src/braco_description/scripts/my_test.py --real --speed 15
```

**7. Stop:** `make down` (the `sim` container), or Ctrl-C in the terminal of a Gazebo started by hand. A `parameter_bridge ... exit code -11` on the way out is harmless.

### When something goes wrong

| What you see | Why / fix |
|---|---|
| `Package 'braco_description' not found` | Not built or not sourced: see [TROUBLESHOOTING](../../../docs/guides/ROS2/TROUBLESHOOTING.md#environment-and-commands) |
| No window, or Gazebo aborts with `Authorization required ... could not connect to display :0` | The display refused the container: see [TROUBLESHOOTING → GUI tools](../../../docs/guides/ROS2/TROUBLESHOOTING.md#gui-tools-rqt-rqt_graph-rviz2-gazebo) |
| Slow, or "Gazebo is taking too long to start" | No GPU: `glxinfo -B \| grep "OpenGL renderer"` says `llvmpipe`. Restart with `make down && make up SIM=1` |
| Stuck on `Requesting list of world names` | Gazebo's processes cannot find each other: `GZ_IP` must be `127.0.0.1` (set by `docker/compose.yml`; recreate the containers after pulling it) |
| `Moved backwards in time`, `Controller already loaded`, `no controller with this name exists` | Two simulations on one `ROS_DOMAIN_ID`: `gazebo.launch.py` in `dev` while the `sim` container runs (both on your `PC_ROS_DOMAIN_ID`). For a window on `sim`: `make sim-gui`. For a second Gazebo: step 4 |
| `arm_controller not found on ROS domain ...` | The shell is not on the simulation's domain: a `dev` shell is on your `PC_ROS_DOMAIN_ID` (the `sim` container); for your own Gazebo, `export` its domain |
| `/joint_states` values like `1e-11` on domain 22 | That is a simulation, not the real arm: someone runs Gazebo on 22. Give it its own domain |
| The `sim` container stopped | Gazebo exited: `make logs`, then `make up` |
| `Command of at least one joint is out of limits` | The joint limiter capping a move faster than the measured joint speed; the move still finishes |
| The model looks old after a change to the URDF or meshes | Gazebo reads the model at launch: Ctrl-C and launch again |

## Commands in short

```bash
# The simulation is the sim container (make up SIM=1); its windows, from the host:
make sim-gui      # Gazebo and RViz (only one: make gaze-gui, make rviz-gui)
# a second Gazebo by hand, on its own domain (gui:=false for server only)
ROS_DOMAIN_ID=78 ros2 launch braco_description gazebo.launch.py
# move it there (radians, reached in 2 s)
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.3, -0.3, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# moves in Gazebo (running, same ROS_DOMAIN_ID: the sim container and dev are on your PC_ROS_DOMAIN_ID), then --real: the same on the real arm
ros2 run braco_description move_arm_example.py
ros2 run braco_description park.py --real       # asks before the real arm moves; --yes skips
# Gazebo and the real arm must be on different ROS domains (both publish /joint_states):
# Gazebo on this shell's ROS_DOMAIN_ID, the real arm on --real-domain (default 22)

# fingers with the vendor linkage (one slider): gripper_actuated:=true
# check the model by hand
xacro urdf/braco.urdf.xacro > /tmp/braco.urdf && check_urdf /tmp/braco.urdf
```

## Rules of the model

- **Joint names** are `joint1..joint6`, the ones `arm_driver` publishes in `/joint_states`. If they differ,
  `robot_state_publisher` ignores the states without any error and the model stays frozen.
- **Limits** (`<limit>`) are `JOINT_LIMITS` of `ros2_ws/src/mycobot_cpp/includes/mycobot.hpp`, converted to radians. Change them
  there first: `test_urdf.py` fails if the two disagree. The URDF limits only inform tools (MoveIt, Gazebo);
  the arm itself is limited by `mycobot::MyCobot280::sendAngles`.
- **Frames**: `world` is the table; `g_base` is the G-shape base plate the arm stands on; `tool0` is the flange (z out of it, REP-103); `tcp` is between the fingertips;
  `camera_optical_frame` has z forward, x right, y down (REP-103).
- **Mesh units**: every `.dae` declares `<unit meter="1">`; the ones drawn in millimetres (`base_link`, `link4`,
  `g_base`, `camera_flange`, the gripper) get `scale="0.001 0.001 0.001"` in the xacro. MoveIt's mesh loader ignores
  the COLLADA unit (Gazebo and RViz apply it): with `<unit meter="0.001">` those meshes looked right but collided
  1000x too big in MoveIt. `test_urdf.py` fails on any other unit.
- Values still **not measured** (velocity, effort, masses/inertia, camera mount, `tcp` length) are marked in the
  xacro and listed in `docs/hardware/urdf-sim-to-real.md`.

## Origin of the model

Adapted from Elephant Robotics' [mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2) (branch `humble`,
BSD-2-Clause): `mycobot_description/urdf/mycobot_280_jn` and `adaptive_gripper` for the links, joints and meshes.
Changes: joint and link names,
the vendor's invalid XML fixed, limits from `JOINT_LIMITS`, inertials, collision, gripper options, TF frames,
the G-base (`mycobot_280_pi/G_base.dae` -> `meshes/arm/g_base.dae`), and `meshes/arm/link4.dae` rewritten from COLLADA `<polygons>` to `<triangles>` (same geometry): Gazebo loads
`<polygons>` as an empty mesh and its physics crashes on the empty collision. Its 8 material bindings also pointed at a
material that does not exist (`#a0000000-...`), so Gazebo and RViz drew it with a plain default material:
they now point at its textured `Material_001-material`.