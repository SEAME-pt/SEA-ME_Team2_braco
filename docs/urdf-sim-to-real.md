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
| Flange and everything on it turned -45° about the joint 6 axis (`flange_yaw` in `braco.urdf.xacro`) | Compared with the real arm at joint 6 = 0, the flange's LEGO hole grid (and so the gripper pinned to it) was 45° off in the vendor model. Turns the flange mesh and `tool0`, so the gripper, `tcp` and camera frames follow; positions do not change. |
| Fingers **fixed** (open) by default | The driver does not publish the gripper state yet. With `gripper_actuated:=true` the vendor linkage is used (1 master joint + 5 `mimic`). |

The joint origins and the mesh offsets are the vendor's, unchanged.

The table-clearance check (`minHeightMm()` in `libs/cpp/srcs/mycobot.cpp`) was fixed. It used the joint origins of the
**M5** arm (J1 0.13156 m above a short base) with the 140 mm height of the Jetson casing top, so every height it
computed was about 85 mm too high (arm straight out sideways: 96 mm instead of 11 mm, allowed instead of refused).
It now uses the Jetson Nano origins of this URDF from the bottom of the casing (`BASE_HEIGHT_MM - CASING_HEIGHT_MM`);
`test_urdf.py` fails if the C++ joint origins or base height drift from the URDF.

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
- `base_link`: the bottom of the Jetson casing, in the plate's pocket: `g_base_height` = 0.030 m above the table.
  **Measured**: table to the top of the casing = 140 mm (`BASE_HEIGHT_MM`), minus the 109.7 mm casing
  (`CASING_HEIGHT_MM`) = 30.3 mm; the vendor mounts it at 30 mm.
- `tool0`: the flange, z axis pointing out of it (REP-103), same as `link6`.
- `tcp`: on the z axis of `tool0`, at `tcp_length` = 0.130 m (from `TOOL_LENGTH_MM`: flange to the tip of the closed
  gripper with camera). **Placeholder**: measure to the middle of the fingertips.
- `camera_optical_frame`: z forward, x right, y down (rotation `-pi/2, 0, -pi/2` from `camera_link`).
  The position of `camera_mount` relative to `tool0` is **zero (to measure)**.

Frame tree PDFs (in [`urdf-sim-to-real/`](urdf-sim-to-real/)):

- [`urdf_tree.pdf`](urdf-sim-to-real/urdf_tree.pdf): links and joints of the URDF
  (`xacro urdf/braco.urdf.xacro > /tmp/braco.urdf && urdf_to_graphviz /tmp/braco.urdf braco_urdf_tree`)
- [`tf_frames.pdf`](urdf-sim-to-real/tf_frames.pdf): the live TF tree with Gazebo running (`ros2 run tf2_tools view_frames`)

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

## Zero pose

`make zero` on the real arm and Gazebo with every joint at 0, from the same viewpoints:

| | Real arm | Gazebo |
|---|---|---|
| Front | <img src="urdf-sim-to-real/zero_real_front.webp" width="260"> | <img src="urdf-sim-to-real/zero_gazebo_front.webp" width="420"> |
| Back | <img src="urdf-sim-to-real/zero_real_back.webp" width="260"> | <img src="urdf-sim-to-real/zero_gazebo_back.webp" width="420"> |

**Result: the zero poses match.** The arm stands upright with the same zig-zag of the links (left, right, left in the
front view), on the G-base, and the gripper on the flange is turned the same way (after the 45° flange correction).
No joint origin had to change, so `ZERO_TOOL0` in `test/test_urdf.py` stays. At zero the real arm reads within about
1° of 0 on every joint (servo offsets).

With every joint at 0, the model's kinematics give (frame `world`, in mm):

| Frame | x | y | z |
|---|---|---|---|
| `tool0` (flange) | 45.6 | -64.6 | 466.1 (436.1 above `base_link`) |

The z axis of `tool0` points to **world +x** (horizontal, forward): the flange faces forward in the zero pose. The
position is checked against the real arm with `get_coords()` (FK table below).

## Open items (need the hardware)

Tick them here as they are measured and update the URDF and the tables above. `scripts/sim_to_real.py` runs the checks
against the real arm (Gazebo on this shell's `ROS_DOMAIN_ID`, e.g. 77; the real arm on 42):
`ros2 run braco_description sim_to_real.py directions | fk | velocity`.

- [ ] **Velocity** of each joint (`sim_to_real.py velocity`): each joint across a known range at speeds 25/50/75/100,
  timed from `/joint_states`. Fill in `joint_velocity` per joint in `mycobot_280.urdf.xacro`.
- [ ] **Effort**: servo stall torque (datasheet), marked as an approximation.
- [ ] **Masses**: weigh the gripper and the camera + flange; arm masses (vendor, or the total split). Update
  `base_mass`, `link_mass`, `gripper_base_mass`.
- [ ] **Offsets** with calipers (photograph the measurements): `camera_xyz`/`camera_rpy`, `tcp_length`.
- [x] **Zero pose**: `make zero` on the Jetson; on the PC, Gazebo with all joints at 0. Photos in "Zero pose" above:
  they match.
- [x] **Sign of each joint** (`sim_to_real.py directions`): +30° on one joint, in Gazebo and on the real arm; does
  Gazebo move the same joint in the same direction? **All six match.**

  | Joint | Same joint? | Same direction? | Note |
  |---|---|---|---|
  | joint1 | yes | yes |  |
  | joint2 | yes | yes |  |
  | joint3 | yes | yes |  |
  | joint4 | yes | yes |  |
  | joint5 | yes | yes |  |
  | joint6 | yes | yes |  |

- [ ] **FK vs. `get_coords()`** (`sim_to_real.py fk`): puts Gazebo at the real arm's angles and prints the model's
  `tool0` in `base_link` (what `tf2_echo base_link tool0` shows), against `mc.get_coords()` on the real arm (X, Y, Z
  within a few mm).

  | Pose | Model FK (mm) | `get_coords()` (mm) | Error (mm) |
  |---|---|---|---|
  | zero | 45.6 / -64.6 / 436.1 | | |

- [x] `ros2 run tf2_tools view_frames` with Gazebo running, and attach the PDF: see "Frame tree" above.

## Known gaps

- Gripper not actuated: the driver does not publish the finger state. With `gripper_actuated:=false` they stay fixed open.
- Inertias, velocities and efforts are estimates (tables above) until they are measured.
- Collision = the vendor mesh itself (heavy). Worth simplifying later for MoveIt/Gazebo.
- `world -> g_base -> base_link` assumes the arm stands on the G-base plate on the table; adjust if it is mounted on another base.
