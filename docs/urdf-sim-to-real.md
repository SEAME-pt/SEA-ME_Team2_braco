# Arm URDF: sim-to-real validation (issue #25)

Package: `ros2_ws/src/braco_description/` (model of the myCobot 280 with the adaptive gripper and the camera mount).
This document says where the model came from, what was changed, what is **already validated** and what **still has to be
measured on the hardware** (section "Open items"). Nothing below marked "to measure" was made up: those are placeholder values.

## How to use

```bash
# PC (pc-dev container, with X11): the model in Gazebo, holding its pose (gui:=false: server only)
ros2 launch braco_description gazebo.launch.py

# Put the simulated arm at given joint angles (radians, reached in 2 s)
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

# Check the model without ROS running:
xacro urdf/braco.urdf.xacro > /tmp/braco.urdf && check_urdf /tmp/braco.urdf
```

Tests: `make test` (`test/test_urdf.py`, 16 cases). They fail if a limit changes in `mycobot.hpp` without changing in the
URDF, if a joint name leaves `joint1..joint6`, if a link with geometry has no positive mass/inertia, if a frame (`tool0`,
`tcp`, `camera_optical_frame`) disappears, or if the position of `tool0` in the zero pose changes by accident.

## Origin of the model and what was changed

Base: [elephantrobotics/mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2), branch `humble`: `mycobot_description`,
variant `mycobot_280_jn` + `adaptive_gripper` for the model
(BSD-2-Clause license, copied to `meshes/LICENSE.elephantrobotics`; the `.dae` meshes are the vendor's, ~26 MB).

| Change | Reason |
|---|---|
| Joints renamed from `joint2_to_joint1 ... joint6output_to_joint6` to `joint1..joint6` | `arm_driver` publishes `/joint_states` with `joint1..joint6`. With different names `robot_state_publisher` ignores the states **without any error** and the model never moves. |
| Links renamed to `base_link`, `link1..link6` | The vendor also names the links `joint1..joint6` (easy to confuse with the joints). |
| Vendor XML error fixed (`lower = "-2.932"1`) | The original file does not even parse. |
| Limits taken from `JOINT_LIMITS` in `libs/cpp/includes/mycobot.hpp` | `<limit>` = `JOINT_LIMITS` (the firmware limit minus 1-2°, the same one `sendAngles` applies). There is only one set of limits in the code, so there is no `<safety_controller>`. |
| `inertial` and `collision` added | The vendor only has `visual`. A positive inertia is required by Gazebo/MoveIt. |
| Gripper mounted with `pi/2` instead of `1.579` | The vendor value is ~0.5° off 90°. |
| Fingers **fixed** (open) by default | The driver does not publish the gripper state yet. With `gripper_actuated:=true` the vendor linkage is used (1 master joint + 5 `mimic`). |

The joint origins and the mesh offsets are the vendor's, unchanged.

## Frame tree

```
world -> g_base -> base_link -> link1 -> link2 -> link3 -> link4 -> link5 -> link6 -> tool0
                                                                                        |-- tcp
                                                                                        |-- camera_mount -> camera_link -> camera_optical_frame
                                                                                        `-- gripper_base -> gripper_left2 / gripper_right2
                                                                                                            gripper_left3 -> gripper_left1
                                                                                                            gripper_right3 -> gripper_right1
```

- `world`: the table surface.
- `g_base`: the G-shape base, a 150 x 110 mm plate clamped to the table with its black grip. Mesh from the vendor
  (`mycobot_280_pi/G_base.dae`), turned 90° so its long side runs along y like the Jetson casing; collision is a box
  of the same size.
- `base_link`: the bottom of the Jetson casing, on top of the plate: `g_base_height` = 0.032 m above the table
  (the plate thickness in the vendor mesh). **To measure** with calipers.
- `tool0`: the flange, z axis pointing out of it (REP-103), same as `link6`.
- `tcp`: on the z axis of `tool0`, at `tcp_length` = 0.130 m (from `TOOL_LENGTH_MM`: flange to the tip of the closed
  gripper with camera). **Placeholder**: measure to the middle of the fingertips.
- `camera_optical_frame`: z forward, x right, y down (rotation `-pi/2, 0, -pi/2` from `camera_link`).
  The position of `camera_mount` relative to `tool0` is **zero (to measure)**.

PDF/PNG of the tree: `urdf_to_graphviz /tmp/braco.urdf` (needs `liburdfdom-tools`, added to the `pc-dev` Dockerfile).

## Limits

All in degrees (the URDF uses radians; the test checks the conversion).

| Joint | `JOINT_LIMITS` (`<limit>`) | Measured velocity | Effort |
|---|---|---|---|
| joint1 | -166 .. 166 | to measure | to measure (placeholder 1.0 N·m) |
| joint2 | -138 .. 138 | to measure | to measure |
| joint3 | -149 .. 149 | to measure | to measure |
| joint4 | -149 .. 149 | to measure | to measure |
| joint5 | -153 .. 158 | to measure | to measure |
| joint6 | -178 .. 178 | to measure | to measure |

In the URDF, `velocity` is a placeholder 1.0 rad/s and `effort` 1.0 N·m for every joint (the vendor uses 0 and 1000,
which mean nothing). Nothing in the system depends on these two numbers yet.

## Inertia

| Item | Value in the model | Source |
|---|---|---|
| `base_link` | 0.4 kg | **estimated** (to weigh) |
| `link1..link6` | 0.1 kg each | **estimated** (split the total mass of the arm) |
| `gripper_base` / fingers | 0.1 kg / 0.01 kg each | **estimated** (to weigh on a scale) |
| Inertia tensor | solid cylinder r = 0.03 m, l = 0.08 m | **estimated**; guarantees a positive-definite matrix |

The vendor does not publish masses.

## Zero pose (model prediction)

With every joint at 0, the model's kinematics give (frame `world`, in mm):

| Frame | x | y | z |
|---|---|---|---|
| `tool0` (flange) | 45.6 | -64.6 | 468.1 (436.1 above `base_link`) |

The z axis of `tool0` points to **world +x** (horizontal, forward): the flange faces forward in the zero pose. The model
is the prediction; it still has to be checked against the real arm (below).

## Open items (need the hardware)

Tick them here as they are measured and update the URDF and the tables above.

- [ ] **Velocity** of each joint: command speed 100 across a known angle (e.g. J1 from -90 to +90°) and time it from
  `/joint_states` (`ros2 topic echo` or PlotJuggler). Fill in `joint_velocity` per joint in `mycobot_280.urdf.xacro`.
- [ ] **Effort**: servo stall torque (datasheet), marked as an approximation.
- [ ] **Masses**: weigh the gripper and the camera + flange; arm masses (vendor, or the total split). Update
  `base_mass`, `link_mass`, `gripper_base_mass`.
- [ ] **Offsets** with calipers (photograph the measurements): `g_base_height` (table to the bottom of the casing), `camera_xyz`/`camera_rpy`, `tcp_length`.
- [ ] **Zero pose**: `make zero` on the Jetson; on the PC, Gazebo with all joints at 0 (the default when it starts).
  Side-by-side photos (Gazebo vs. real arm) from the same viewpoint.
  If the model's zero does not look like the arm's (±pi/2 rotations in the origins), fix the origins and update `ZERO_TOOL0`
  in `test/test_urdf.py`.
- [ ] **Sign of each joint**: command +30° (0.5236 rad) on one joint, on the real arm and in Gazebo; does Gazebo move
  the same joint in the same direction?

  | Joint | Same joint? | Same direction? | Note |
  |---|---|---|---|
  | joint1..joint6 | | | |

- [ ] **FK vs. `get_coords()`**: put Gazebo at the real arm's angles, then `ros2 run tf2_ros tf2_echo base_link tool0`
  against `mc.get_coords()` on the real arm (X, Y, Z within a few mm).

  | Pose | Model FK (mm) | `get_coords()` (mm) | Error (mm) |
  |---|---|---|---|
  | zero | 45.6 / -64.6 / 436.1 | | |

- [ ] `ros2 run tf2_tools view_frames` with Gazebo running, and attach the PDF.

## Known gaps

- Gripper not actuated: the driver does not publish the finger state. With `gripper_actuated:=false` they stay fixed open.
- Inertias, velocities and efforts are estimates (tables above) until they are measured.
- Collision = the vendor mesh itself (heavy). Worth simplifying later for MoveIt/Gazebo.
- `world -> g_base -> base_link` assumes the arm stands on the G-base plate on the table; adjust if it is mounted on another base.
- `BASE_HEIGHT_MM` = 140 in `libs/cpp/includes/mycobot.hpp` (used by `minHeightMm()`, the table-clearance check) does not
  match the model: with the G-base, the C++ base frame (26 mm above the bottom of the casing, `JOINT_ORIGINS` J1 =
  0.13156 vs. 0.15756 in the URDF) is about 58 mm above the table. Measure, then fix one or the other.
