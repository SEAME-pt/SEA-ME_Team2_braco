# braco_mission

Mission layer of the braco arm: tasks such as pick-and-place and drawing with a pen, asking MoveIt 2 for motions.
Its first node, `move_to_pose`, moves the tool centre point (`tcp`) to a Cartesian pose goal (#84) and in straight
lines through waypoints (#85), a first step toward drawing with a pen, and picks and places an object at poses given in
the goal (#148).

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
| `position_tolerance_m` | `0.005` | success: tcp within 5 mm |
| `orientation_tolerance_deg` | `2.0` | success: tcp within 2° |
| `settle_s` / `tf_timeout_s` | `0.5` / `1.0` | wait before reading the tcp; TF lookup timeout |
| `eef_step_m` | `0.005` | straight lines: a `tcp` point every 5 mm, when the goal leaves `eef_step_m` at 0; the lines of pick and place |
| `approach_height_m` | `0.05` | pick and place: pre-grasp and pre-place this far above the pose, when the goal leaves it at 0 |
| `gripper` | `sim` | `sim`: `gripper_action_controller` (Gazebo); `real`: `/arm/set_gripper` on `arm_driver`. Anything else: the node does not start. The launch file's `gripper` argument sets it; empty, it follows `real` |
| `gripper_joint` | `gripper_controller` | sim: the gripper's master joint |
| `gripper_open_position` / `gripper_closed_position` | `0.15` / `-0.74` | sim: rad of the master joint. On an object the gripper stalls before closed, which counts as done |
| `gripper_timeout_s` | `5.0` | longest wait for the gripper to answer and to move |
| `gripper_settle_s` | `1.5` | real: wait after `/arm/set_gripper` answers, the gripper is still moving |

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

## Pick and place

`/pick_and_place` (#148) picks an object at one pose and places it at another, on the same node and with the same
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

```bash
ros2 action send_goal --feedback /pick_and_place braco_interfaces/action/PickAndPlace \
  "{pick: {header: {frame_id: world}, pose: {position: {x: 0.18, y: -0.08, z: 0.03},
           orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}},
    place: {header: {frame_id: world}, pose: {position: {x: 0.18, y: 0.08, z: 0.03},
            orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}}"
```

The poses above are an example of the syntax, not yet checked on the bench: the Gazebo and real-arm demos set them.

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

Tests (`colcon test --packages-select braco_mission`): `test/test_pose_math.cpp`, `test/test_mission.cpp` and
`test/test_cartesian.cpp` (every step against a fake arm: control, fallback, fraction, failures, tolerances), and
`test/test_draw_line.py` (the script's path geometry, CSV and plot). `test/test_pick_and_place.cpp` runs the
sequence against a fake arm that moves where it was planned and a fake gripper: stage order, straight lines above
each pose, the control held once, a failure at every stage, a pick or place outside the tolerance. No ROS graph, no
MoveIt.
`test/test_move_to_pose_node.cpp` runs the node in-process with a fake `MotionGroup`: both actions, cancel, one goal
at a time, TF frames and `/arm/take_control`, and `/pick_and_place` with a fake `gripper_action_controller`: every
stage, default height, frames, cancel and STOP at a stage, the gripper failing. `MoveItMotionGroup` is checked in
Gazebo.
`test/test_ros_gripper.cpp` runs both grippers against a fake `gripper_action_controller` and a fake
`/arm/set_gripper`: commanded positions, abort, reject, timeout and cancel, a refusal, no server.
`test/test_launch.py` builds `move_to_pose.launch.py` without starting it: one node for the simulation, one for the
real arm (`real:=true`: real-arm model, `use_control`), and the `gripper` each one gets.
