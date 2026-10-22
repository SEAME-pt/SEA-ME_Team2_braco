# braco_description

Model of the myCobot 280 with the adaptive gripper and a camera mount, for the braco stack: links, joints,
limits, meshes and the TF frames the rest of the system uses. Issue **#25**. The validation against the real
arm (zero pose, joint directions, forward kinematics) and the open measurements are in
[`docs/urdf-sim-to-real.md`](../../../docs/urdf-sim-to-real.md).

| Folder | Content |
|---|---|
| `urdf/braco.urdf.xacro` | The whole robot: `world` -> arm -> `tool0` -> gripper, `tcp`, camera frames |
| `urdf/mycobot_280.urdf.xacro` | The arm (macro): `base_link`, `link1..link6`, `joint1..joint6` |
| `urdf/adaptive_gripper.xacro` | The gripper (macro), fingers fixed open or with the vendor linkage |
| `urdf/braco.gazebo.xacro` | `gz_ros2_control` on `joint1..joint6`, added only with `gazebo:=true` |
| `config/gazebo_controllers.yaml` | `joint_state_broadcaster` + `arm_controller` (joint trajectory) for Gazebo |
| `meshes/` | Visual and collision meshes of Elephant Robotics (`LICENSE.elephantrobotics`) |
| `rviz/braco.rviz` | The RViz view |
| `launch/display.launch.py` | `robot_state_publisher` + RViz (+ sliders) |
| `launch/gazebo.launch.py` | Gazebo (gz sim) with the model holding its pose, plus the controllers |
| `test/test_urdf.py` | Keeps the URDF and the code in sync |

`gazebo.launch.py` is a quick viewer: the model in Gazebo, moved through `/arm_controller/joint_trajectory`.
The full simulation bringup (`sim:=true` in `braco_bringup`, the web page and teleop driving the simulated arm)
is issues #34/#37.

## Use

```bash
# simulation only, with sliders (PC, X11)
ros2 launch braco_description display.launch.py

# follow the real arm: the Jetson runs `make up`, /joint_states reaches the PC over DDS
ros2 launch braco_description display.launch.py use_gui:=false

# Gazebo: the model holding its pose (gui:=false for server only)
ros2 launch braco_description gazebo.launch.py
# move it there (radians, reached in 2 s)
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.3, -0.3, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# fingers with the vendor linkage (one slider): gripper_actuated:=true
# check the model by hand
xacro urdf/braco.urdf.xacro > /tmp/braco.urdf && check_urdf /tmp/braco.urdf
```

## Rules of the model

- **Joint names** are `joint1..joint6`, the ones `arm_driver` publishes in `/joint_states`. If they differ,
  `robot_state_publisher` ignores the states without any error and the model stays frozen.
- **Limits** (`<limit>`) are `JOINT_LIMITS` of `libs/cpp/includes/mycobot.hpp`, converted to radians. Change them
  there first: `test_urdf.py` fails if the two disagree. The URDF limits only inform tools (RViz, MoveIt, Gazebo);
  the arm itself is limited by `mycobot::MyCobot280::sendAngles`.
- **Frames**: `world` is the table; `tool0` is the flange (z out of it, REP-103); `tcp` is between the fingertips;
  `camera_optical_frame` has z forward, x right, y down (REP-103).
- Values still **not measured** (velocity, effort, masses/inertia, camera mount, `tcp` length) are marked in the
  xacro and listed in `docs/urdf-sim-to-real.md`.

## Origin of the model

Adapted from Elephant Robotics' [mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2) (branch `humble`,
BSD-2-Clause): `mycobot_description/urdf/mycobot_280_jn` and `adaptive_gripper` for the links, joints and meshes,
and `mycobot_280/mycobot_280jn` for the RViz config and the launch structure. Changes: joint and link names,
the vendor's invalid XML fixed, limits from `JOINT_LIMITS`, inertials, collision, gripper options, TF frames,
and `meshes/arm/link4.dae` rewritten from COLLADA `<polygons>` to `<triangles>` (same geometry): Gazebo loads
`<polygons>` as an empty mesh and its physics crashes on the empty collision.
