# braco_moveit_config

MoveIt 2 configuration of the braco arm: MoveIt loads the model of `braco_description`, plans for the group `arm`
(joint1..joint6, for the tool tip `tcp`) within the same joint limits as the rest of the stack, and sends the plans
to `arm_controller`.

On the Jetson Nano, `move_group` needs 300 to 600 MB and a full core while planning: run it on the PC; it reaches
the arm on the Jetson through DDS.

## Contents

| File | What it holds |
|---|---|
| `config/braco.srdf` | Planning group `arm` (chain `base_link` -> `tcp`), group `gripper` and its end effector, named states `home` and `park`, disabled collision pairs |
| `config/kinematics.yaml` | IK of `arm`: KDL, timeout and resolution |
| `config/joint_limits.yaml` | Position limits = `JOINT_LIMITS` of `mycobot.hpp`; velocity measured on the real arm (from the URDF); acceleration UNMEASURED (5 rad/s²); default scaling 0.3 |
| `config/moveit_controllers.yaml` | `arm_controller`, `FollowJointTrajectory` on `/arm_controller/follow_joint_trajectory` |
| `config/braco.urdf.xacro` | The demo's model: `braco_description`'s `braco.urdf.xacro` + a `mock_components/GenericSystem` `<ros2_control>` |
| `config/moveit.rviz` | RViz with the MotionPlanning panel on `arm` |
| `launch/demo.launch.py` | RViz + `move_group` + ros2_control with the mock hardware |
| `launch/sim.launch.py` | Gazebo (`braco_description`'s `gazebo.launch.py`), then `move_group` and RViz once `arm_controller` is active |
| `launch/rviz.launch.py` | RViz only, on a `move_group` already running (`make rviz-gui`) |
| `.setup_assistant` | Lets MoveItConfigsBuilder and the Setup Assistant find the model and the SRDF |

The ros2_control controllers are not copied here: the demo starts those of `braco_description/config/controllers.yaml`,
the one set used in Gazebo and (planned) on the real arm.

Named states (radians in the SRDF) are the fixed poses of `mycobot_cpp/includes/mycobot.hpp`:
`home` = `ZERO_ANGLES` (all 0, the arm upright) and `park` = `PARK_ANGLES` {0, 131, -148, 146, 4.5, 0} deg.
Change them there first; `test/test_moveit_config.py` fails if they disagree.

## Run the demo

In the dev container (pc image), after `colcon build`:

```bash
ros2 launch braco_moveit_config demo.launch.py              # RViz window
ros2 launch braco_moveit_config demo.launch.py rviz:=false  # move_group only (headless)
```

In RViz, panel **MotionPlanning** > tab **Planning**: *Goal State* `home` or `park`, then **Plan & Execute**. Or
drag the marker at the tool tip and plan to a Cartesian pose (KDL solves the IK). The mock hardware reports back the
position it is commanded, so `/joint_states` follows the plan.

Ctrl+C on the launch can end with `move_group ... exit code -11`: a crash of `move_group` while it shuts down,
after everything ran; it does not affect planning.

## Run on the Gazebo arm

The `sim` container runs `move_group` by default (`SIM_MOVEIT=true` in `docker/.env`), through
`braco.launch.py sim:=true moveit:=true`. From the host:

```bash
make up SIM=1     # the simulation and move_group (make restart SIM=1 after changing docker/.env)
make sim-gui      # Gazebo and RViz, in the background (only one: make gaze-gui, make rviz-gui)
```

In RViz, panel **MotionPlanning** > tab **Planning**: *Goal State* `home` or `park`, then **Plan & Execute**. The plan
goes to `/arm_controller/follow_joint_trajectory` and the arm moves in Gazebo; `/joint_states` follows it. Everything
runs on simulated time (`use_sim_time`). Closing a window leaves the simulation running.

Without the `sim` container, `sim.launch.py` runs the whole thing in one launch, in a dev shell:

```bash
ros2 launch braco_moveit_config sim.launch.py                         # Gazebo and RViz windows
ros2 launch braco_moveit_config sim.launch.py gui:=false rviz:=false  # headless
```

Run it with `sim` stopped (`make down`, then `make up`) or on another domain (`export ROS_DOMAIN_ID=78`). On one domain,
the second simulation's spawners end with `Controller already loaded` and exit code 1.

`move_group` and RViz start only once the `arm_controller` spawner has succeeded, because nothing can execute without
the controller. If it fails, the launch shuts down, so RViz never opens: look for that spawner's error in the log.

## Rules of the config

- **Joint limits** must never be wider than `JOINT_LIMITS` of `mycobot.hpp`: MoveIt would plan moves the driver
  then clamps. Change `mycobot.hpp` first, then the URDF and `config/joint_limits.yaml`; `test/test_joint_limits.py`
  fails if they disagree.
- **Speed**: the default scaling (0.3 of the measured speed) is applied by the clients (RViz, `MoveGroupInterface`),
  which read it from `robot_description_planning`. A raw `/move_action` request with scaling 0 runs at full speed.
- **Collisions**: the disabled pairs come from `collisions_updater` (below) plus three set by hand, each explained
  in the SRDF. Pairs in contact in the zero pose are never disabled automatically: a collision there is a model bug
  (the millimetre meshes once collided 1000x too big, see `braco_description`'s "Mesh units").

Regenerate the disabled pairs after a change of the meshes or of the joint origins (the hand-set pairs stay):

```bash
cd /ws/src/braco_moveit_config
xacro config/braco.urdf.xacro > /tmp/braco_moveit.urdf
ros2 run moveit_setup_assistant collisions_updater --urdf /tmp/braco_moveit.urdf \
  --srdf config/braco.srdf --output config/braco.srdf --always --keep --trials 100000
```

`collisions_updater` rewrites the file without its comments: restore them from git after it runs.

## Tests

`test/test_moveit_config.py` (`colcon test --packages-select braco_moveit_config`): the SRDF names exist in the
model, `arm` is joint1..joint6, `home`/`park` match `mycobot.hpp` and lie inside `joint_limits.yaml`, the MoveIt
controller matches `braco_description/config/controllers.yaml`, and the mock `<ros2_control>` matches the Gazebo one.

`test/test_joint_limits.py`: the positions of `joint_limits.yaml` equal `JOINT_LIMITS` and the URDF `<limit>`, and its
velocities are not above the measured URDF ones.
