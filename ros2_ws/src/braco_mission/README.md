# braco_mission

Mission layer of the braco arm: tasks such as pick-and-place and drawing with a pen, asking MoveIt 2 for motions.
Its first node, `move_to_pose`, moves the tool centre point (`tcp`) to a Cartesian pose goal and in straight lines
through waypoints, a first step toward drawing with a pen.

The mission never talks to the serial port: it plans and executes through MoveIt 2 (`braco_moveit_config`), so the
same code runs in simulation and on the real arm (`real:=true`). On the real arm it also holds the control as `mission`
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

The tolerances are a proposal: the team has not agreed on them yet.

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

## The goal set: run_goal_set.py

`scripts/run_goal_set.py` runs a fixed set of goals, the same in Gazebo and on the real arm, and reports the pose
error of each one. The set is [`config/goal_set.yaml`](config/goal_set.yaml): a `start` pose and goals that
are either a `line` (an end point for `/move_cartesian`, keeping the start orientation) or a full `pose` (for
`/move_to_pose`, from `/compute_fk`). Before every goal the arm goes back to `start`, and that return is reported as
a `start` goal too.

```bash
make run CMD='ros2 run braco_mission run_goal_set.py'                          # all goals; asks before moving
make run CMD='ros2 run braco_mission run_goal_set.py --only line_y,pose_turn --out bags/real_1'
```

It prints one line per goal and writes `<out>/summary.csv` (one row per goal: success, position and orientation
error, and for lines `fraction`, planned and executed deviation) and `<out>/<line>.csv/.png` as `draw_line.py` does.
`<out>` is `bags/goal_set` by default. It stops at the first goal that does not run to the end (refused, partial
plan, execution error); a goal outside the tolerance is reported and the set goes on. `--set` takes another YAML.

On the real arm:
- **Start from zero** (`make zero` on the Jetson), not from park: out of the folded park pose MoveIt finds no plan
  (joint4 rests just past its 149° limit there). The script prints this hint when the first `start` has no plan.
- **Temperatures:** before every goal it reads `/arm/servo_temperatures` and stops when a servo is at 55 °C or more
  (`--max-temp`; "warm" in [overheating.md](../../../docs/guides/overheating.md)). The hottest servo before each
  goal is in `summary.csv` (`max_servo_temp_c`). Gazebo publishes no temperatures, so there the check is skipped.
- **Control:** the goals need the control free. If the web page holds it (`/arm/control_owner` is `web`), every
  goal is refused: release it on the page and keep the page open for STOP.

From `start` the tool points down, so the reachable lines go in (toward the base) and down: moving the `tcp` 50 mm
out (+x) or up plans only 55–73 % of the path, and nothing moves. `line_z_up` (30 mm up) is at that edge: it plans
fully in Gazebo but only 87.5 % on the real arm, where the line starts from the measured `tcp`, several mm off `start`.

Checked in Gazebo (2026-11-19), `run_goal_set.py` with the default set:

| Goal | Kind | Length | Reached (position, orientation) | Planned deviation | Executed deviation max |
|---|---|---|---|---|---|
| `start` (8 returns) | pose | | 0.32–0.43 mm, 0.04–0.15° | | |
| `line_y` | line | 130 mm along +y | 0.35 mm, 0.08° | 0.04 mm | 0.33 mm |
| `line_x_in` | line | 50 mm along −x | 0.19 mm, 0.04° | 0.04 mm | 0.25 mm |
| `line_z_down` | line | 40 mm down | 0.33 mm, 0.07° | 0.04 mm | 0.33 mm |
| `line_z_up` | line | 30 mm up | 0.35 mm, 0.09° | 0.06 mm | 0.20 mm |
| `line_xy` | line | 89 mm, x-y diagonal | 0.17 mm, 0.04° | 0.04 mm | 0.31 mm |
| `line_xyz` | line | 78 mm, x-y-z diagonal | 0.20 mm, 0.04° | 0.04 mm | 0.26 mm |
| `pose_turn` | pose | | 0.32 mm, 0.13° | | |
| `pose_tilted` | pose | | 0.42 mm, 0.14° | | |

All within the tolerances (5 mm, 2°). The errors are larger than on 2026-11-11 (0.02 mm at the end of `line_y`):
the `tcp` sits ~0.3 mm below the goal for the whole line, which looks like the simulated arm sagging under gravity.
`line_z_down` takes the `tcp` to z = 73 mm: on the real arm, check where the table is first.

On the real arm the error comes from TF, so from the angles the servos report through the URDF: it shows whether the
joints got there, not errors of the model itself (such as `tcp_length`, still to check with a ruler in
[urdf-sim-to-real.md](../../../docs/hardware/urdf-sim-to-real.md#open-items-need-the-hardware)).

### On the real arm (2026-11-25)

The default set, no load, velocity scaling 0.1, from zero, in three parts so the servos stay cool (hottest 42 °C):

| Goal | Kind | Length | Reached (position, orientation) | Planned deviation | Executed deviation max / mean |
|---|---|---|---|---|---|
| `start` (8 returns) | pose | | 4.2–10.1 mm, 1.05–2.70° | | |
| `line_y` | line | 130 mm along +y | 12.4 mm, 2.73° | 0.04 mm | 8.8 / 5.5 mm |
| `line_x_in` | line | 50 mm along −x | 6.3 mm, 2.13° | 0.03 mm | 6.8 / 4.4 mm |
| `line_z_down` | line | 40 mm down | 8.4 mm, 2.82° | 0.03 mm | 8.4 / 4.2 mm |
| `line_z_up` | line | 30 mm up | not run: `fraction` 0.875 | 0.11 mm | |
| `line_xy` | line | 89 mm, x-y diagonal | 8.2 mm, 2.10° | 0.04 mm | 9.3 / 5.8 mm |
| `line_xyz` | line | 78 mm, x-y-z diagonal | 7.8 mm, 1.98° | 0.04 mm | 9.7 / 6.6 mm |
| `pose_turn` | pose | | 7.5 mm, 2.69° | | |
| `pose_tilted` | pose | | 12.4 mm, 3.79° | | |

- The plans are the same as in Gazebo (`fraction` 1.0, planned deviation ≤ 0.04 mm): the error comes from execution.
- The lines stay straight, diagonals included: the `tcp` runs parallel to the ideal line, 7–10 mm off it. The offset
  builds up in the first ~8 mm and then stays flat; on `line_xyz` the `tcp` goes down in a straight line ~4 mm below
  the ideal one. That is joints stopping short of their targets, as the servos' dead zone (~3°,
  [urdf-sim-to-real.md](../../../docs/hardware/urdf-sim-to-real.md)) does, not a bent path.
- Repeatable: part A (`line_y`, `line_x_in`, `line_xy`, `line_xyz`) run twice differed by at most 0.8 mm.
- `pose_tilted`, the highest pose (z = 149 mm) with the tool tilted, has the largest error.

Earlier, on 2026-11-19, `line_y` alone gave 11.1 mm, 2.51° without load and 16.4 mm, 4.13° with the gripper and
camera (~150 g): the load lowers the `tcp` (z at `start` 106.2 mm vs 110.8 mm, goal 112.6 mm), and joint4 warmed to
56 °C after two runs, hence the temperature check above. A third run aborted on the way to `start` with joint3 moving
the wrong way: that was ros2_control's command limiter starting from the previous goal's command, now off on the real
arm, not joint3.

Only the first `start` is within the 5 mm / 2° tolerance. That tolerance was set from Gazebo; for the real arm it is
for the team to agree on.

## Run it on the real arm

Follow [safety.md §9](../../../docs/guides/safety.md#9-first-moveit-2-motion-on-the-real-arm). The Jetson runs the arm
(`make up`), the PC runs `move_group` (`make moveit`, after the domain check) and this node, on the arm's domain.
`real:=true` gives it the real arm's model, the system clock and `use_control`: it takes the control as `mission`
before each goal and turns `braco_hardware` and `arm_controller` back on if they are off.

```bash
make run CMD='ROS_DOMAIN_ID=22 ros2 launch braco_mission move_to_pose.launch.py real:=true'
```

Goals as below, with `ROS_DOMAIN_ID=22`. Start with a small move near `park`.

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
| `include/braco_mission/move_to_pose_node.hpp`, `src/move_to_pose_node.cpp` | `MoveToPoseNode`: `CartesianBackend` with a `MotionGroup`, TF, `/arm/take_control` and the controller manager services; both action servers. A library, so tests can run it in-process |
| `include/braco_mission/motion_group.hpp` | `MotionGroup`: the MoveIt 2 calls of the node, faked in tests |
| `include/braco_mission/moveit_motion_group.hpp`, `src/moveit_motion_group.cpp` | `MotionGroup` with MoveIt 2 (`MoveGroupInterface`, `computeCartesianPath`, TOTG) |
| `src/move_to_pose.cpp` | `main()` only |
| `launch/move_to_pose.launch.py` | The node with the MoveIt config of the Gazebo arm, or of the real arm with `real:=true` |
| `scripts/draw_line.py` | Draw one straight line and plot the executed `tcp` path |
| `scripts/run_goal_set.py`, `config/goal_set.yaml` | Run the fixed goal set and report the pose error of every goal |

Tests (`colcon test --packages-select braco_mission`): `test/test_pose_math.cpp`, `test/test_mission.cpp` and
`test/test_cartesian.cpp` (every step against a fake arm: control, fallback, fraction, failures, tolerances), and
`test/test_draw_line.py` (the script's path geometry, CSV and plot) and `test/test_run_goal_set.py` (the goal set
check, the package's set, the summary). No ROS graph, no MoveIt.
`test/test_move_to_pose_node.cpp` runs the node in-process with a fake `MotionGroup`: both actions, cancel, one goal
at a time, TF frames and `/arm/take_control`. `MoveItMotionGroup` is checked in Gazebo.
`test/test_launch.py` builds `move_to_pose.launch.py` without starting it: one node for the simulation, one for the
real arm (`real:=true`: real-arm model, `use_control`).
