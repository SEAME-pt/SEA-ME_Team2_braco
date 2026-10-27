# braco_description

Model of the myCobot 280 with the adaptive gripper and a camera mount, for the braco stack: links, joints,
limits, meshes and the TF frames the rest of the system uses. Issue **#25**. The validation against the real
arm (zero pose, joint directions, forward kinematics) and the open measurements are in
[`docs/urdf-sim-to-real.md`](../../../docs/hardware/urdf-sim-to-real.md).

| Folder | Content |
|---|---|
| `urdf/braco.urdf.xacro` | The whole robot: `world` -> G-base -> arm -> `tool0` -> gripper, `tcp`, camera frames |
| `urdf/mycobot_280.urdf.xacro` | The arm (macro): `base_link`, `link1..link6`, `joint1..joint6` |
| `urdf/adaptive_gripper.xacro` | The gripper (macro), fingers fixed open or with the vendor linkage |
| `urdf/braco.gazebo.xacro` | `gz_ros2_control` on `joint1..joint6`, added only with `gazebo:=true` |
| `config/controllers.yaml` | The ros2_control controllers, one set for the simulation and (planned) the real arm: `joint_state_broadcaster` + `arm_controller` (joint trajectory) |
| `meshes/` | Visual and collision meshes of Elephant Robotics (`LICENSE.elephantrobotics`) |
| `launch/gazebo.launch.py` | The simulated arm: Gazebo (gz sim), the model and the controllers. Included by `braco_bringup`'s `sim:=true` |
| `scripts/braco_arm.py` | Runs a list of moves in Gazebo, then (`--real`) the same moves on the real arm |
| `scripts/move_joint.py` | Moves one joint of the simulated arm (`move_joint.py 2 30`: joint2 to 30°) |
| `scripts/move_arm_example.py` | Example: a few poses, written once, run in Gazebo and on the arm |
| `scripts/park.py` | The park pose (`PARK_ANGLES` of `mycobot.hpp`), in Gazebo and on the arm |
| `scripts/sim_to_real.py` | Issue #25 checks against the real arm: `directions`, `fk`, `velocity` |
| `test/test_urdf.py` | Keeps the URDF and the code in sync |

## The simulation (issue #34)

```
 braco_bringup  braco.launch.py sim:=true            (the `sim` container, ROS domain 77)
   └─ braco_description  gazebo.launch.py
        gz sim (empty world) ── spawns braco.urdf.xacro gazebo:=true
          └─ gz_ros2_control (hardware: GazeboSimSystem, joint1..joint6, position)
               ├─ joint_state_broadcaster  ──► /joint_states
               └─ arm_controller           ◄── /arm_controller/follow_joint_trajectory  (action)
                                           ◄── /arm_controller/joint_trajectory         (topic, web page)
        robot_state_publisher ──► /tf        ros_gz_bridge ──► /clock
```

The controllers are `config/controllers.yaml`, the set planned for the real arm too: on the hardware only the
`<hardware>` plugin of the URDF changes (planned: a braco `SystemInterface`), so MoveIt, the mission and the page see the same names.
The validation of the model against the real arm is [`docs/urdf-sim-to-real.md`](../../../docs/hardware/urdf-sim-to-real.md).

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
controllers). `make up SIM=1` waits until it publishes `/joint_states`.
It runs on **ROS domain 77** (`PC_ROS_DOMAIN_ID` in `docker/.env`, also the PC's `dev`), never the real arm's 42:
both publish `/joint_states`. If Gazebo exits, the container stops, like `real` when `arm_driver` exits.

**2. Watch it.** Either:

- the **web page**: `make web`, then http://localhost:8000: the 3D arm follows the simulation, and the Control tab
  moves it; or
- the **Gazebo window**: `make sim-gui` opens a window on that running simulation (Ctrl-C closes the window, the
  simulation keeps running). Or `SIM_GUI=true` in `docker/.env`, then `make restart SIM=1`: the `sim` container
  opens it itself. Do **not** run `gazebo.launch.py` in `dev` for this: that is a second simulation (step 4).

**3. Command a joint.** In a dev shell (already on the simulation's domain, 77):

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
a second Gazebo yourself in `dev`, with a window. `dev` is on the `sim` container's domain (77), so either stop
`sim` first (`make down`, then `make shell`) or give your Gazebo another domain as below. Two simulations on one domain
fight: `Controller already loaded`, `no controller with this name exists`, `Moved backwards in time`.

```bash
make shell
source /ws/install/setup.bash
export ROS_DOMAIN_ID=78     # not 42 (the real arm), not 77 (the sim container); teammates: another number
                            # (each new shell starts on 77 again: export it there too)
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
gives it back when it ends. Check first that the PC sees the arm: `ROS_DOMAIN_ID=42 ros2 node list` shows
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
| `Moved backwards in time`, `Controller already loaded`, `no controller with this name exists` | Two simulations on one `ROS_DOMAIN_ID`: `gazebo.launch.py` in `dev` while the `sim` container runs (both 77). For a window on `sim`: `make sim-gui`. For a second Gazebo: step 4 |
| `arm_controller not found on ROS domain ...` | The shell is not on the simulation's domain: a `dev` shell is on 77 (the `sim` container); for your own Gazebo, `export` its domain |
| `/joint_states` values like `1e-11` on domain 42 | That is a simulation, not the real arm: someone runs Gazebo on 42. Give it its own domain |
| The `sim` container stopped | Gazebo exited: `make logs`, then `make up` |
| `Command of at least one joint is out of limits` | The joint limiter capping a move faster than the measured joint speed; the move still finishes |
| The model looks old after a change to the URDF or meshes | Gazebo reads the model at launch: Ctrl-C and launch again |

## Commands in short

```bash
# The simulation is the sim container (make up SIM=1); its Gazebo window, from the host:
make sim-gui
# a second Gazebo by hand, on its own domain (gui:=false for server only)
ROS_DOMAIN_ID=78 ros2 launch braco_description gazebo.launch.py
# move it there (radians, reached in 2 s)
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.3, -0.3, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# moves in Gazebo (running, same ROS_DOMAIN_ID: the sim container and dev are on 77), then --real: the same on the real arm
ros2 run braco_description move_arm_example.py
ros2 run braco_description park.py --real       # asks before the real arm moves; --yes skips
# Gazebo and the real arm must be on different ROS domains (both publish /joint_states):
# Gazebo on this shell's ROS_DOMAIN_ID, the real arm on --real-domain (default 42)

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
- Values still **not measured** (velocity, effort, masses/inertia, camera mount, `tcp` length) are marked in the
  xacro and listed in `docs/urdf-sim-to-real.md`.

## Origin of the model

Adapted from Elephant Robotics' [mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2) (branch `humble`,
BSD-2-Clause): `mycobot_description/urdf/mycobot_280_jn` and `adaptive_gripper` for the links, joints and meshes.
Changes: joint and link names,
the vendor's invalid XML fixed, limits from `JOINT_LIMITS`, inertials, collision, gripper options, TF frames,
the G-base (`mycobot_280_pi/G_base.dae` -> `meshes/arm/g_base.dae`), and `meshes/arm/link4.dae` rewritten from COLLADA `<polygons>` to `<triangles>` (same geometry): Gazebo loads
`<polygons>` as an empty mesh and its physics crashes on the empty collision. Its 8 material bindings also pointed at a
material that does not exist (`#a0000000-...`), so Gazebo and RViz drew it with a plain default material:
they now point at its textured `Material_001-material`.
