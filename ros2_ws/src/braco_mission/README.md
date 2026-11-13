# braco_mission

Mission layer of the braco arm: tasks such as pick-and-place and drawing with a pen, asking MoveIt 2 for motions.
Its first node, `move_to_pose`, moves the tool centre point (`tcp`) to a Cartesian pose goal (#84) and in straight
lines through waypoints (#85), a first step toward drawing with a pen.

The mission never talks to the serial port: it plans and executes through MoveIt 2 (`braco_moveit_config`), so the
same code runs in simulation and, later, on the real arm. On the real arm it also holds the control as `mission`
around each goal ([ADR-0004](../../../docs/decisions/ADR-0004-trajectories-through-arm-driver.md)). C++, like the
other nodes that move the arm.

## move_to_pose

| | Name | Type |
|---|---|---|
| action | `/move_to_pose` | `braco_interfaces/action/MoveToPose` |
| action | `/move_cartesian` | `braco_interfaces/action/MoveCartesian` ([straight lines](#straight-lines-move_cartesian)) |
| client | `/arm/take_control` | `braco_interfaces/srv/TakeControl` (only with `use_control`) |
| clients | `/controller_manager/list_hardware_components`, `set_hardware_component_state`, `list_controllers`, `switch_controller` | `controller_manager_msgs/srv/*` (only with `use_control`) |

A goal is a `geometry_msgs/PoseStamped` of the `tcp`:
1. A pose that is not finite, or whose orientation is not a rotation, is refused.
2. With `use_control` (real arm): `take_control("mission")` without `force`. Refused → nothing is sent.
   Then `BracoSystem` and `arm_controller` are turned back on if they are off (a STOP or a lost control turns
   them off, ADR-0004). Not possible → nothing is sent.
3. MoveIt 2 plans the full pose (group `arm`). An orientation of all zeros asks for the position only. With
   `position_only_fallback`, a full pose without a plan (no IK) is retried as its position only.
4. `arm_controller` executes the plan. Then the `tcp` is read from TF and compared with the goal.
5. The control is released on every path, also on failure.

The result has `success` (within both tolerances), the position and orientation error, `position_only` and a
`message`. The feedback names the stage: `control`, `activating`, `planning`, `executing`, `measuring`. One goal at a
time, on either action; cancelling stops the arm.

| Parameter | Default | |
|---|---|---|
| `group` / `end_effector` | `arm` / `tcp` | MoveIt group and link |
| `use_control` | `false` | `true` on the real arm (ADR-0004); simulation has no `arm_driver` |
| `velocity_scaling` / `acceleration_scaling` | `0.1` / `0.1` | ADR-0004: ≤ 0.1 on the real arm |
| `planning_time_s` | `5.0` | |
| `position_tolerance_m` | `0.005` | success: tcp within 5 mm |
| `orientation_tolerance_deg` | `2.0` | success: tcp within 2° |
| `settle_s` / `tf_timeout_s` | `0.5` / `1.0` | wait before reading the tcp; TF lookup timeout |
| `eef_step_m` | `0.005` | straight lines: a `tcp` point every 5 mm, when the goal leaves `eef_step_m` at 0 |

The tolerances are a proposal for the team to agree on (#84 "Done when").

## Straight lines: move_cartesian

`/move_to_pose` plans in joint space: the arm reaches the goal, but the `tcp` follows a curve. `/move_cartesian`
moves the `tcp` in straight lines, from where it is through a list of waypoints (one `header` frame for all), with
MoveIt 2 `computeCartesianPath`. It is served by the same node, with the same parameters and tolerances.

1. A request that cannot be planned is refused: no waypoints, a waypoint that is not finite or whose orientation is
   not a rotation, two consecutive waypoints closer than 0.1 mm, `eef_step_m` outside (0, 0.05] m, `min_fraction`
   outside (0, 1].
2. With `use_control` (real arm): the control is taken as for `/move_to_pose`.
3. A waypoint with its orientation all zeros keeps the `tcp` orientation measured at the start (stage
   `measuring start`).
4. `computeCartesianPath` interpolates a `tcp` point every `eef_step_m` and solves IK for each. It returns the
   `fraction` of the path it could plan: -1 is an error, below `min_fraction` (1.0 when the goal leaves it at 0) is
   an incomplete path. In both cases nothing moves.
5. The path is re-timed with TOTG at `velocity_scaling` / `acceleration_scaling`: `computeCartesianPath` times it
   for full speed, and ADR-0004 allows at most 0.1.
6. `arm_controller` executes it; the `tcp` is read from TF and compared with the last waypoint, orientation
   included.

The result adds `fraction` and `max_deviation_m`: the largest distance of the planned `tcp` path (forward
kinematics of every trajectory point) from the ideal polyline, start → waypoints.

`computeCartesianPath` no longer takes a `jump_threshold` in MoveIt 2.15: the overload that has one is deprecated
and ignores it, so IK jumps between consecutive points are not checked yet.

### Draw a line and plot it

`scripts/draw_line.py` sends one `/move_cartesian` goal from where the `tcp` is to `X Y Z` (world, metres, keeping
the orientation), reads the `tcp` from TF every 20 ms while the arm moves, and prints the deviation of the
**executed** path from the line. It writes `bags/draw_line.csv` and `bags/draw_line.png` (top view, side view,
deviation along the line); `ros2_ws/bags` is not tracked by git. A line under 1 mm is refused.

```bash
make run CMD="ros2 action send_goal --feedback /move_to_pose braco_interfaces/action/MoveToPose \
  '{target: {header: {frame_id: world}, pose: {position: {x: 0.2167, y: -0.0646, z: 0.1126}, \
  orientation: {x: 0.9238, y: -0.3826, z: -0.0135, w: 0.0056}}}}'"     # the start: FK of [0, -0.5, -1.0, -0.1, 0, 0]
make run CMD='ros2 run braco_mission draw_line.py 0.2167 0.0654 0.1126'  # 130 mm along y, 11 cm above the table
```

Or the action alone (orientation all zeros: keep the start orientation):

```bash
make run CMD="ros2 action send_goal --feedback /move_cartesian braco_interfaces/action/MoveCartesian \
  '{header: {frame_id: world}, waypoints: [{position: {x: 0.2167, y: 0.0654, z: 0.1126}, \
  orientation: {x: 0, y: 0, z: 0, w: 0}}]}'"
```

Checked in Gazebo (2026-11-11), from the start pose above:

| Line | Result |
|---|---|
| 130 mm along y, `/move_cartesian` | `fraction` 1.0, planned deviation 0.04 mm, reached: 0.02 mm, 0.01° |
| 130 mm along y, `draw_line.py` | `fraction` 1.0; executed path: deviation max 0.12 mm, mean 0.06 mm, end error 0.015 mm |

The executed deviation is about three times the planned one: `arm_controller` lags the reference while the arm
moves (it rises while accelerating, holds ~0.1 mm at cruise speed, and falls back as the arm stops). The real arm's
servos will add far more.

## Run it on the Gazebo arm

```bash
make up SIM=1                                                # Gazebo + move_group (SIM_MOVEIT, the default)
make run CMD='ros2 launch braco_mission move_to_pose.launch.py'
```

A goal the arm can reach comes from forward kinematics of a known joint pose (IK can fail on an arbitrary pose).
`move_group` computes it (`/compute_fk`, joint angles in radians):

```bash
make run CMD="ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK \
  '{header: {frame_id: world}, fk_link_names: [tcp], robot_state: {joint_state: \
  {name: [joint1, joint2, joint3, joint4, joint5, joint6], position: [0.4, -0.3, 0.5, 0.2, 0.0, 0.0]}}}'"
make run CMD="ros2 action send_goal --feedback /move_to_pose braco_interfaces/action/MoveToPose \
  '{target: {header: {frame_id: world}, pose: {position: {x: 0.1604, y: -0.0024, z: 0.5219}, \
  orientation: {x: 0.5423, y: -0.1058, z: 0.6913, w: -0.4656}}}}'"
```

Position only: send the orientation as all zeros. Checked in Gazebo (2026-11-10), each goal from `/compute_fk`:

| Goal | Result |
|---|---|
| Full pose, FK of `[0.4, -0.3, 0.5, 0.2, 0, 0]` | reached: 0.07 mm, 0.06° |
| Position only, FK of `[-0.5, -0.2, 0.6, 0, 0.3, 0]` | reached: 0.06 mm |
| Quaternion `w: 2` (not a rotation) | refused, nothing planned |

## Code

| File | |
|---|---|
| `include/braco_mission/pose_math.hpp` | Goal check and pose error, no ROS |
| `include/braco_mission/mission.hpp`, `src/mission.cpp` | The steps of a goal against `ArmBackend`, no ROS |
| `include/braco_mission/cartesian.hpp`, `src/cartesian.cpp` | The steps of a straight-line goal against `CartesianBackend`, the request check and the line deviation, no ROS |
| `include/braco_mission/control_guard.hpp` | Releases the control on every path of a goal (both actions) |
| `src/move_to_pose_node.cpp` | `CartesianBackend` with MoveIt 2 (`MoveGroupInterface`, `computeCartesianPath`, TOTG), TF, `/arm/take_control` and the controller manager services; both action servers |
| `launch/move_to_pose.launch.py` | The node with the MoveIt config of the Gazebo arm |
| `scripts/draw_line.py` | Draw one straight line and plot the executed `tcp` path |

Tests (`colcon test --packages-select braco_mission`): `test/test_pose_math.cpp`, `test/test_mission.cpp` and
`test/test_cartesian.cpp` (every step against a fake arm: control, fallback, fraction, failures, tolerances), and
`test/test_draw_line.py` (the script's path geometry, CSV and plot). No ROS graph, no MoveIt. The node itself is
checked in Gazebo.
