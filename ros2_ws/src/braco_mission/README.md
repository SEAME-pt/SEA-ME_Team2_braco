# braco_mission

Mission layer of the braco arm: tasks such as pick-and-place and drawing with a pen, asking MoveIt 2 for motions.
Its first node, `move_to_pose`, moves the tool centre point (`tcp`) to a Cartesian pose goal and in straight lines
through waypoints, a first step toward drawing with a pen, and picks and places an object at poses given in the goal.

The mission never talks to the serial port: it plans and executes through MoveIt 2 (`braco_moveit_config`), so the
same code runs in simulation and on the real arm (`real:=true`). On the real arm it also holds the control as `mission`
around each goal ([ADR-0004](../../../docs/decisions/ADR-0004-trajectories-through-arm-driver.md)). C++, like the
other nodes that move the arm.

## move_to_pose

| | Name | Type |
|---|---|---|
| action | `/move_to_pose` | `braco_interfaces/action/MoveToPose` |
| action | `/move_cartesian` | `braco_interfaces/action/MoveCartesian` ([straight lines](#straight-lines-move_cartesian)) |
| action | `/pick_and_place` | `braco_interfaces/action/PickAndPlace` ([pick and place](#pick-and-place)) |
| client | `/arm/take_control` | `braco_interfaces/srv/TakeControl` (only with `use_control`) |
| clients | `/controller_manager/list_hardware_components`, `set_hardware_component_state`, `list_controllers`, `switch_controller` | `controller_manager_msgs/srv/*` (only with `use_control`) |
| sub | `/arm/stopped` | `std_msgs/Bool`, latched (only with `use_control`): a STOP while a goal runs ends it |

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
time, on either action; cancelling stops the arm. On the real arm a STOP latched in `arm_driver` while a goal runs
([ADR-0005](../../../docs/decisions/ADR-0005-stop-latch.md)) stops the trajectory at once, like a cancel: the goal
aborts with a message that starts with `STOP latched in arm_driver`.

| Parameter | Default | |
|---|---|---|
| `group` / `end_effector` | `arm` / `tcp` | MoveIt group and link |
| `use_control` | `false` | `true` on the real arm (ADR-0004); simulation has no `arm_driver` |
| `velocity_scaling` / `acceleration_scaling` | `0.1` / `0.1` | ADR-0004: ≤ 0.1 on the real arm |
| `planning_time_s` | `5.0` | |
| `elbow_max_deg` | `20.0` | point-to-point plans keep `joint3` (the elbow) at or below this along the whole path, so MoveIt never takes the solution folded over the arm (`joint3` near +100°): from it the lines of pick and place cannot go down, and the real arm lags on the long move there. Start with the elbow under it (`make zero`). `180`: no limit |
| `position_tolerance_m` | `0.005` | success: tcp within 5 mm |
| `orientation_tolerance_deg` | `2.0` | success: tcp within 2° |
| `settle_s` / `tf_timeout_s` | `0.5` / `1.0` | wait before reading the tcp; TF lookup timeout |
| `eef_step_m` | `0.005` | straight lines: a `tcp` point every 5 mm, when the goal leaves `eef_step_m` at 0; the lines of pick and place |
| `approach_height_m` | `0.05` | pick and place: pre-grasp and pre-place this far above the pose, when the goal leaves it at 0 |
| `gripper` | `sim` | `sim`: `gripper_action_controller` (Gazebo); `real`: `/arm/set_gripper` on `arm_driver`. Anything else: the node does not start. The launch file's `gripper` argument sets it; empty, it follows `real` |
| `gripper_joint` | `gripper_controller` | sim: the gripper's master joint |
| `gripper_open_position` / `gripper_closed_position` | `0.15` / `-0.74` | sim: rad of the master joint. On an object the gripper stalls before closed, which counts as done |
| `gripper_timeout_s` | `15.0` | longest wait for the gripper to answer and to move: the Gazebo gripper takes ~4 s to close on an object, ~9 s empty |
| `gripper_settle_s` | `1.5` | real: wait after `/arm/set_gripper` answers, the gripper is still moving |

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

## Pick and place

`/pick_and_place` picks an object at one pose and places it at another, on the same node and with the same
tolerances. The poses are `PoseStamped`, transformed into the planning frame like a `/move_to_pose` goal. The stages,
also the feedback:

| Stage | |
|---|---|
| `control` | Real arm: take the control once, for the whole sequence, and turn `arm_controller` on |
| `open` | Open the gripper |
| `pre_grasp` | `move_to_pose` to `approach_height_m` above the pick pose (+z of the planning frame) |
| `approach_pick` | Straight line down to the pick pose: its error is the pick error of the result |
| `close` | Close the gripper; stalling on the object counts as closed |
| `retreat_pick` | Straight line up |
| `pre_place` | `move_to_pose` above the place pose |
| `approach_place` | Straight line down to the place pose: its error is the place error of the result |
| `release` | Open the gripper |
| `retreat_place` | Straight line up |

The first stage that fails (no plan, an aborted execution, a pose outside the tolerances, the gripper) ends the goal:
nothing moves after it, the result names it in `failed_stage` and the control is released. A cancel or a STOP stops
the move in progress, and no other stage starts; the message starts with the stage and the reason, such as
`pre_grasp: cancelled (...)`. A goal that cannot run is refused with `failed_stage: goal`. The sequence is in
`pick_and_place.cpp`, without ROS; the gripper is `gripper_action_controller` in Gazebo and `/arm/set_gripper` on the
real arm (`gripper` parameter).

The web page's **Pick & place** view ([web/README.md](../../../web/README.md)) sends the same goal: the two points
are chosen on a top view of the arm's reach, and the stages and the result are shown to everybody on the page.

### Demo in Gazebo

A 20 x 20 x 40 mm block (`braco_simulation`'s `models/pick_block`) is spawned at the pick pose, then one goal moves it
120 mm along y. Tested five times in a row: every stage succeeded with the `tcp` within 1 mm of both poses, and the
block ended upright 5 to 12 mm from the place pose (it slides a little between the fingers).

```bash
make up SIM=1                                   # Gazebo + move_group, headless
make shell                                      # then, in it: ros2 launch braco_mission move_to_pose.launch.py
# The block at the pick pose: the robot's world frame is Gazebo's (0.07, 0.085, 0.75), the block's centre 20 mm up
make run CMD='GZ_PARTITION=braco-sim ros2 run ros_gz_sim create -world lab_bench -name pick_block -x 0.27 -y 0.045 -z 0.77 \
  -file /ws/install/braco_simulation/share/braco_simulation/models/pick_block/model.sdf'
make run CMD='ros2 action send_goal --feedback /pick_and_place braco_interfaces/action/PickAndPlace "{
  pick:  {header: {frame_id: world}, pose: {position: {x: 0.20, y: -0.04, z: 0.025}, orientation: {x: 1.0, w: 0.0}}},
  place: {header: {frame_id: world}, pose: {position: {x: 0.20, y: 0.08, z: 0.025}, orientation: {x: 1.0, w: 0.0}}}}"'
make run CMD='GZ_PARTITION=braco-sim gz model -m pick_block -p'   # the block now: about (0.27, 0.165, 0.77)
```

`make gaze-gui` shows it. What the poses need:

- The orientation `{x: 1, w: 0}` points the `tcp` down; the fingers then close along x.
- The `tcp` at 25 mm above the table, the block's middle. The open fingertips reach about 15 mm below the `tcp` and
  the table is in MoveIt's planning scene, so a line lower than that stops short (`only 90% of the path planned`).
- Both poses away from the gap between the two tables (robot y = -0.085).
- In Gazebo the gripper closes at about 0.1 rad/s and its controller ends a squeeze when the joint stops (stall), so
  `gripper_timeout_s` is 15 s and `stall_velocity_threshold` is 0.02 in `braco_description`'s `controllers.yaml`.

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
| `include/braco_mission/pick_and_place.hpp`, `src/pick_and_place.cpp` | The stages of a pick-and-place goal on top of `move_to_pose` and `move_cartesian`, the control held for the whole sequence, no ROS |
| `include/braco_mission/control_guard.hpp` | Releases the control on every path of a goal (both actions) |
| `include/braco_mission/move_to_pose_node.hpp`, `src/move_to_pose_node.cpp` | `MoveToPoseNode`: `CartesianBackend` with a `MotionGroup`, TF, `/arm/take_control` and the controller manager services, `/arm/stopped`; the three action servers. A library, so tests can run it in-process |
| `include/braco_mission/gripper.hpp` | `GripperBackend`: open and close the gripper, no ROS, faked in tests |
| `include/braco_mission/ros_gripper.hpp`, `src/ros_gripper.cpp` | `ActionGripper` (`ParallelGripperCommand`, Gazebo) and `ServiceGripper` (`/arm/set_gripper`, real arm), chosen by `make_gripper` from the `gripper` parameter |
| `include/braco_mission/motion_group.hpp` | `MotionGroup`: the MoveIt 2 calls of the node, faked in tests |
| `include/braco_mission/moveit_motion_group.hpp`, `src/moveit_motion_group.cpp` | `MotionGroup` with MoveIt 2 (`MoveGroupInterface`, `computeCartesianPath`, TOTG) |
| `src/move_to_pose.cpp` | `main()` only |
| `launch/move_to_pose.launch.py` | The node with the MoveIt config of the Gazebo arm, or of the real arm with `real:=true` |
| `scripts/draw_line.py` | Draw one straight line and plot the executed `tcp` path |
| `scripts/run_goal_set.py`, `config/goal_set.yaml` | Run the fixed goal set and report the pose error of every goal |

Tests (`colcon test --packages-select braco_mission`): `test/test_pose_math.cpp`, `test/test_mission.cpp` and
`test/test_cartesian.cpp` (every step against a fake arm: control, fallback, fraction, failures, tolerances), and
`test/test_draw_line.py` (the script's path geometry, CSV and plot) and `test/test_run_goal_set.py` (the goal set
check, the package's set, the summary). `test/test_pick_and_place.cpp` runs the sequence against a fake arm that
moves where it was planned and a fake gripper: stage order, straight lines above each pose, the control held once, a
failure at every stage, a pick or place outside the tolerance. No ROS graph, no MoveIt.
`test/test_move_to_pose_node.cpp` runs the node in-process with a fake `MotionGroup`: both actions, cancel, one goal
at a time, TF frames and `/arm/take_control`, and `/pick_and_place` with a fake `gripper_action_controller`: every
stage, default height, frames, cancel and STOP at a stage, the gripper failing. `MoveItMotionGroup` is checked in
Gazebo.
`test/test_ros_gripper.cpp` runs both grippers against a fake `gripper_action_controller` and a fake
`/arm/set_gripper`: commanded positions, abort, reject, timeout and cancel, a refusal, no server.
`test/test_launch.py` builds `move_to_pose.launch.py` without starting it: one node for the simulation, one for the
real arm (`real:=true`: real-arm model, `use_control`), and the `gripper` each one gets.
