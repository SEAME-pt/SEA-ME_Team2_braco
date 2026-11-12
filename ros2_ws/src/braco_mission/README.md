# braco_mission

Mission layer of the braco arm: tasks such as pick-and-place and drawing with a pen, asking MoveIt 2 for motions.
Its first node, `move_to_pose`, moves the tool centre point (`tcp`) to a Cartesian pose goal (#84).

The mission never talks to the serial port: it plans and executes through MoveIt 2 (`braco_moveit_config`), so the
same code runs in simulation and, later, on the real arm. On the real arm it also holds the control as `mission`
around each goal ([ADR-0004](../../../docs/decisions/ADR-0004-trajectories-through-arm-driver.md)). C++, like the
other nodes that move the arm.

## move_to_pose

| | Name | Type |
|---|---|---|
| action | `/move_to_pose` | `braco_interfaces/action/MoveToPose` |
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
`message`. The feedback names the stage: `control`, `activating`, `planning`, `executing`, `measuring`. One goal at a time;
cancelling stops the arm.

| Parameter | Default | |
|---|---|---|
| `group` / `end_effector` | `arm` / `tcp` | MoveIt group and link |
| `use_control` | `false` | `true` on the real arm (ADR-0004); simulation has no `arm_driver` |
| `velocity_scaling` / `acceleration_scaling` | `0.1` / `0.1` | ADR-0004: ≤ 0.1 on the real arm |
| `planning_time_s` | `5.0` | |
| `position_tolerance_m` | `0.005` | success: tcp within 5 mm |
| `orientation_tolerance_deg` | `2.0` | success: tcp within 2° |
| `settle_s` / `tf_timeout_s` | `0.5` / `1.0` | wait before reading the tcp; TF lookup timeout |

The tolerances are a proposal for the team to agree on (#84 "Done when").

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
| `src/move_to_pose_node.cpp` | `ArmBackend` with MoveIt 2 (`MoveGroupInterface`), TF, `/arm/take_control` and the controller manager services; the action server |
| `launch/move_to_pose.launch.py` | The node with the MoveIt config of the Gazebo arm |

Tests (`colcon test --packages-select braco_mission`): `test/test_pose_math.cpp` and `test/test_mission.cpp`
(every step against a fake arm: control, fallback, failures, tolerances). No ROS graph, no MoveIt. The node itself
is checked in Gazebo.
