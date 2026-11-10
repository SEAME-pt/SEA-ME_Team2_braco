# braco_description

The robot model of the braco arm: the myCobot 280 with the adaptive gripper and a camera mount. It holds the
links, joints, limits, meshes and the TF frames the rest of the system uses (issue **#25**).

- It is only files (URDF/xacro, meshes, `controllers.yaml`): no nodes, no launch
- The simulation, the real arm and MoveIt all use this one model
- It needs no Gazebo, so the Jetson builds it too (it is an `exec_depend` of `braco_bringup`)

The validation against the real arm (zero pose, joint directions, forward kinematics) and the open measurements
are in [`docs/hardware/urdf-sim-to-real.md`](../../../docs/hardware/urdf-sim-to-real.md).

## What is in it

| File | What it is |
|---|---|
| `urdf/braco.urdf.xacro` | The whole robot: `world` -> G-base -> arm -> `tool0` -> gripper, `tcp`, camera frames |
| `urdf/mycobot_280.urdf.xacro` | The arm (macro): `base_link`, `link1..link6`, `joint1..joint6` |
| `urdf/adaptive_gripper.xacro` | The gripper (macro), fingers fixed open or with the vendor linkage |
| `urdf/braco.gazebo.xacro` | The simulation's hardware (`GazeboSimSystem`) on `joint1..joint6`, only with `gazebo:=true` |
| `urdf/braco.real.xacro` | The real arm's hardware (`braco_hardware`) on `joint1..joint6`, only with `real_arm:=true` |
| `config/controllers.yaml` | The controllers: `arm_controller` (sim and real arm), `joint_state_broadcaster` (sim and MoveIt demo only) |
| `meshes/` | Visual and collision meshes of Elephant Robotics (`LICENSE.elephantrobotics`) |
| `test/test_urdf.py` | Keeps the URDF and the code in sync |

## Who uses it

| Who | Builds the model with | Hardware under the controllers |
|---|---|---|
| Simulation ([`braco_simulation`](../braco_simulation/README.md)) | `gazebo:=true` | `GazeboSimSystem` |
| Real arm (`braco_bringup`, `sim:=false`) | `real_arm:=true` | `braco_hardware` ([ADR-0004](../../../docs/decisions/ADR-0004-trajectories-through-arm-driver.md)) |
| MoveIt demo (`braco_moveit_config`) | no option (adds its own mock) | `mock_components/GenericSystem` |

- Turn on at most one of `gazebo` and `real_arm`: each adds a hardware block on the same joints.
- On the real arm, `joint_state_broadcaster` is not used: `arm_driver` is the only `/joint_states` publisher.
- With `gazebo:=true`, `g_base` also shows a grey pedestal for the lab bench scene. It is only its look: the
  kinematics do not change.

To run the simulation, see [`braco_simulation`](../braco_simulation/README.md).

## Check the model by hand

```bash
xacro urdf/braco.urdf.xacro > /tmp/braco.urdf && check_urdf /tmp/braco.urdf
xacro urdf/braco.urdf.xacro real_arm:=true | grep -A2 "<hardware>"     # the real arm's plugin
xacro urdf/braco.urdf.xacro gripper_actuated:=true > /tmp/braco.urdf   # fingers with the vendor linkage
```

## Rules of the model

- **Joint names** are `joint1..joint6`, the ones `arm_driver` publishes in `/joint_states`. If they differ,
  `robot_state_publisher` ignores the states without any error and the model stays frozen.
- **Limits** (`<limit>`) are `JOINT_LIMITS` of `ros2_ws/src/mycobot_cpp/includes/mycobot.hpp`, converted to radians. Change them
  there first: `test_urdf.py` fails if the two disagree (`braco_moveit_config`'s `test_joint_limits.py` checks
  `joint_limits.yaml`). The URDF limits only inform tools (MoveIt, Gazebo);
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