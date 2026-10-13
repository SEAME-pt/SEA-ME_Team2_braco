# ROS2 on the myCobot 280 Jetson Nano

Practical guide for our cell: what ROS2 is, how it is wired on the myCobot 280
Jetson Nano (JN), and a phased test ladder to validate the setup from first
boot to the first ROS2-controlled motion.

> **Safety first.** Every test that can move the arm is gated behind the rules
> in [safety.md](safety.md): base clamped, arm near the upright pose, 350 mm
> exclusion zone clear, hand near the power switch, slow speeds only.
> Read-only checks (Phases 0–3) are safe to run at any time.

---

## 1. What ROS2 actually is

ROS2 is not an operating system: it is a **middleware + tooling ecosystem** for
building robot software out of small, independent programs.

| Concept | What it means on our robot |
| :--- | :--- |
| **Node** | One process doing one job, e.g. "read the arm over serial", "show a GUI". Nodes find each other automatically (no master server, unlike ROS1). |
| **Topic** | Named, asynchronous data stream, e.g. `/mycobot/angles_real` (telemetry) or `/joint_states` (joint positions). One publisher, many subscribers. |
| **Message** | Typed data structure sent on a topic, e.g. `MycobotSetAngles` = 6 floats + speed. |
| **Service** | Synchronous request/response call, e.g. `get_joint_angles` (ask, wait, receive). |
| **Action** | Long-running goal with feedback + cancel (used later for trajectories / MoveIt2). Not needed yet. |
| **Package** | Unit of build/release, e.g. `mycobot_communication`. |
| **Workspace** | Folder containing `src/` + `build/` + `install/` (ours: `~/colcon_ws`), built with `colcon`. |
| **Launch file** | Python description of "start these N nodes with these parameters", e.g. `ros2 launch mycobot_280jn ...`. |
| **DDS** | The transport below all of the above. Fully distributed: any two machines on the same network can see each other's topics, provided they share the same `ROS_DOMAIN_ID`. |
| **rviz2** | 3D visualisation tool: shows the robot model, topics, TF frames. |

Why it matters for our project:

* Standard interfaces for everything: sensor drivers, visualisation, motion planning, logging, rosbag recording, simulation.
* **Distributed by design**: the Jetson does the real-time I/O, a laptop can run heavy code (MoveIt2, vision, ML) over the same network.
* Everything is inspectable live: `ros2 topic list/echo/pub`, `ros2 node list`, `ros2 service list`, `ros2 doctor`.

### ROS1 vs ROS2 (why we care)

The arm's product line historically shipped ROS1 Melodic; our JN edition ships
**Ubuntu 20.04 + ROS2** with the workspace already deployed. ROS2 is the
supported path forward (Foxy on this image; Humble on newer hardware), so all
new work in this repo targets ROS2.

---

## 2. How ROS2 is wired on our robot

```
        our laptops / other lab machines                       (optional, over WiFi/LAN)
                    ▲   DDS (same ROS_DOMAIN_ID)
                    │
┌───────────────────┴─────────────────────────────────────────┐
│ Jetson Nano (inside the base) — Ubuntu 20.04 + ROS2 Foxy    │
│                                                             │
│  rviz2 / joint_state_publisher_gui        mycobot_280jn     │
│  mycobot_communication (bridge node)      demos (slider…)   │
│            │                                    │           │
│            │  ROS2 topics/services              │ pymycobot │
│            ▼                                    ▼           │
│        /dev/ttyTHS1 @ 1000000 baud  ◄───────────┘           │
└────────────────────┬────────────────────────────────────────┘
                     │ UART
              ┌──────▼───────┐
              │ Base control │  Atom + servo drivers
              │  (6 servos)  │  J1..J6
              └──────────────┘
```

Two integration styles exist in the official code, and both are useful to know:

1. **Bridge style** — `mycobot_communication` owns the serial port and exposes
   ROS2 topics/services (`/mycobot/angles_goal`, `set_joint_angles`, …).
   Any node on the network can command the arm without knowing anything about
   serial ports. This is the "real" ROS2 architecture.
2. **Direct style** — demo nodes such as `mycobot_280jn/slider_control` import
   `pymycobot` themselves and talk straight to `/dev/ttyTHS1`; ROS2 is used
   only for the GUI plumbing (the sliders publish `/joint_states` in radians,
   the node converts to degrees and calls `send_angles`). This is exactly how
   our `initial_tests/*.py` scripts work already.

### Packages (mycobot_ros2 workspace)

| Package | Role |
| :--- | :--- |
| `mycobot_description` | URDF/xacro models (incl. `mycobot_280_jn`), meshes, rviz configs. |
| `mycobot_interfaces` | All custom messages/services (see below). |
| `mycobot_communication` | The bridge: topic node (`mycobot_topics_jsnn`) and service node (`mycobot_services`). |
| `mycobot_280jn` | Demos and launch files for our exact model (slider GUI, keyboard teleop, camera demos, …). |

> The official repo also contains other models (`mycobot_280pi`, `mycobot_320`,
> `mecharm`, …) we can ignore. Note the foxy branch is inconsistent for the JN:
> the bridge launch defaults to `/dev/ttyUSB0` @ 115200 instead of
> `/dev/ttyTHS1` @ 1000000, and the JN bridge source mixes TCP-socket and
> serial pymycobot classes. **The shipped image may differ from GitHub** — best
> practice: try the launch as shipped, list its arguments with
> `ros2 launch <pkg> <file> --show-args`, and inspect the installed source at
> `~/colcon_ws/src/mycobot_ros2/mycobot_communication/mycobot_communication/`.

### Topics exposed by the bridge

| Topic | Type | Direction | Notes |
| :--- | :--- | :--- | :--- |
| `/mycobot/angles_real` | `MycobotAngles` | out (≈4 Hz) | Live joint angles, degrees. |
| `/mycobot/coords_real` | `MycobotCoords` | out (≈4 Hz) | Live end-effector pose. |
| `/mycobot/angles_goal` | `MycobotSetAngles` | in | Target angles in degrees + speed (1–100). |
| `/mycobot/coords_goal` | `MycobotSetCoords` | in | Target coords + speed + mode. |
| `/mycobot/gripper_status` | `MycobotGripperStatus` | in | `status: true/false` → open/close adaptive gripper. |
| `/mycobot/pump_status` | `MycobotPumpStatus` | in | Suction pump (we don't have one). |

Message fields (degrees, `float32`; speed `int8` 1–100):

```
MycobotSetAngles: joint_1 .. joint_6, speed
MycobotSetCoords: x, y, z, rx, ry, rz, speed, model
MycobotAngles:    joint_1 .. joint_6
```

### Services exposed by the bridge

| Service | Type | Purpose |
| :--- | :--- | :--- |
| `set_joint_angles` / `get_joint_angles` | `SetAngles` / `GetAngles` | Write/read joint angles. |
| `set_joint_coords` / `get_joint_coords` | `SetCoords` / `GetCoords` | Write/read end-effector pose. |
| `switch_gripper_status` | `GripperStatus` | Gripper open/close. |
| `switch_pump_status` | `PumpStatus` | Pump on/off (unused here). |

### JN launch files (package `mycobot_280jn`)

| Launch file | What it starts | Touches hardware? |
| :--- | :--- | :--- |
| `test.launch.py` | rviz2 + robot_state_publisher + joint_state_publisher(_gui) | **No** (pure visualisation) |
| `slider_control.launch.py` | the above + slider node driving the real arm | **Yes** |
| `slider_control_adaptive_gripper.launch.py` | same + gripper widgets | **Yes** |
| `teleop_keyboard.launch.py` | keyboard teleop + rviz2 | **Yes** |
| `test_gripper.launch.py` | gripper open/close test | **Yes** (gripper) |
| `simple_gui.launch.py` | Tk GUI | **Yes** |
| `mycobot_follow.launch.py` | camera + follow demo | **Yes** |

> `slider_control` requires `pymycobot >= 3.6.1` while the
> `mycobot_communication` nodes in the same branch enforce
> `pymycobot <= 3.5.3` — the two official demos were built against different
> library eras. Check the installed version before debugging anything else.

---

## 3. Test ladder

Run everything below **on the Jetson** (robot desktop terminal, or SSH).
The desktop has an "ROS2 Shell" icon that opens a terminal with the ROS2
environment and workspace already sourced — otherwise source manually:

```bash
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash
```

### Phase 0 — Environment baseline (read-only, always safe)

```bash
bash initial_tests/ros2_baseline_check.sh
```

Checks OS/JetPack, ROS2 distro, workspace, packages, `pymycobot` version, serial
device and permissions, and prints a PASS/WARN/FAIL summary. It never talks to
the arm. **Success:** no `[FAIL]` lines. Exit code 1 means something needs
fixing (see §4).

### Phase 1 — ROS2 middleware smoke test (no hardware)

Proves the ROS2 install itself works, node discovery included.

```bash
# Terminal A
ros2 run demo_nodes_cpp talker

# Terminal B
ros2 run demo_nodes_cpp listener     # must print "I heard: [Hello World: N]"
ros2 node list                       # must show /talker and /listener
ros2 topic list                      # must show /chatter
ros2 topic echo /chatter --once
ros2 doctor                          # full environment health report
```

**If `listener` sees nothing:** DDS discovery problem — check both terminals
have the same `ROS_DOMAIN_ID`, and try `export ROS_LOCALHOST_ONLY=1` locally.

### Phase 2 — Packages + model visualisation (no arm motion)

```bash
ros2 pkg list | grep -i mycobot      # mycobot_280jn, mycobot_communication, …
ros2 launch mycobot_280jn test.launch.py
```

`rviz2` opens with the 280 JN model; moving the **joint_state_publisher_gui**
sliders animates the virtual robot only — the real arm does not move.

**Headless (SSH) variant:**

```bash
ros2 launch mycobot_280jn test.launch.py gui:=false
```

…but rviz2 still needs a display. From SSH use `export DISPLAY=:0` (the robot
desktop session must be logged in), or run it from the robot's own desktop.

### Phase 3 — Hardware bridge, read-only

Starts the node that owns `/dev/ttyTHS1` and exposes tables from §2. Note the
documented, image-tested JN path is the direct-style slider demo (end of
Phase 4); the bridge gives the clean topic/service interface but its
foxy-branch code has rough edges (see note in §2).

```bash
# Try the launch exactly as shipped on the robot image first:
ros2 launch mycobot_communication communication_jsnn.launch.py

# See which arguments the installed launch actually accepts:
ros2 launch mycobot_communication communication_jsnn.launch.py --show-args
```

Wait for the startup log, then in another terminal:

```bash
ros2 node list                       # /mycobot_services
ros2 topic list -t                   # /mycobot/... with types
ros2 topic echo /mycobot/angles_real --once    # live joint angles (deg)
ros2 topic echo /mycobot/coords_real --once    # live end-effector pose

# Services (read-only call)
ros2 interface show mycobot_interfaces/srv/GetAngles
ros2 service call /get_joint_angles mycobot_interfaces/srv/GetAngles "{}"
```

**Success:** the echo commands print plausible numbers; with the arm powered
and held in the upright pose they should read near `0`.

> If `/dev/ttyTHS1` is busy or the launch fails: make sure no other program
> (including our `initial_tests/*.py` scripts) is holding the serial port.
> Only one process may own the arm's UART at a time.

### Phase 4 — First ROS2-controlled motion

> **Gate check:** base clamped, arm upright, 350 mm zone clear, hand near the
> power switch. One command at a time. Speeds ≤ 25 for first runs. Remember
> from [safety.md](safety.md): **Ctrl+C does not release servo torque.**

The two topic-based tests below need the Phase 3 bridge running. If the bridge
does not work on your image, skip ahead to the slider demo at the end of this
phase (that is the officially documented path for the JN).

Minimal manual test — read the pose, then re-send it at slow speed (no arm
should move, but it proves the command path end-to-end):

```bash
ros2 topic pub --once /mycobot/angles_goal mycobot_interfaces/msg/MycobotSetAngles \
  "{joint_1: 0.0, joint_2: 0.0, joint_3: 0.0, joint_4: 0.0, joint_5: 0.0, joint_6: 0.0, speed: 10}"
```

Then the scripted smoke test — small, slow, reversible moves (J6 tool roll,
then J1 base yaw) that always end in the pose the arm started from:

```bash
python3 initial_tests/ros2_smoke_move.py
```

It first waits for `/mycobot/angles_real` telemetry, prints the current pose
and asks for an explicit `MOVE` confirmation before sending anything.

Finally, the official slider demo (direct style, bypasses the bridge — stop
the Phase 3 launch first to free the serial port):

```bash
ros2 launch mycobot_280jn slider_control_adaptive_gripper.launch.py
```

Sliders publish `/joint_states` in radians; the node converts and calls the
real arm. Start with tiny slider movements.

### Phase 5 — Remote control from a laptop (optional, no extra hardware)

On both the robot and your laptop:

```bash
export ROS_DOMAIN_ID=42          # any 0–232, but identical on both machines;
                                 # use a different ID per robot in a shared lab
ros2 topic list                  # the laptop must see /mycobot/angles_real, …
```

If topics don't cross machines: same subnet, multicast allowed (WiFi
sometimes blocks it), firewalls off, and both machines' ROS distros sourced.
As a fallback on one machine, `ROS_LOCALHOST_ONLY=1` breaks cross-machine
discovery by design.

### Going further

* Gripper via ROS2: `ros2 topic pub --once /mycobot/gripper_status
  mycobot_interfaces/msg/MycobotGripperStatus "{status: true}"` (keep hands
  clear of the jaws).
* Camera: the USB cell camera can be published with a `usb_cam` / `v4l2_camera`
  node later; check `v4l2-ctl --list-devices` on the Jetson first.
* Motion planning: newer myCobot ROS2 branches ship MoveIt2 configs; our JN
  image (Foxy) is focused on the basic demos. Natural next milestone after
  this ladder passes.

---

## 4. Troubleshooting

| Symptom | Likely cause | Fix |
| :--- | :--- | :--- |
| `ros2: command not found` | Environment not sourced | `source /opt/ros/foxy/setup.bash` (+ workspace). Use the desktop "ROS2 Shell" icon. |
| `Package 'mycobot_280jn' not found` | Workspace not sourced / not built | `source ~/colcon_ws/install/setup.bash`; rebuild with `colcon build --symlink-install` if `install/` is missing. |
| `RuntimeError: … pymycobot library must be less than 3.5.3` (bridge) | Installed pymycobot too new for the bridge code | Check `python3 -c "import pymycobot; print(pymycobot.__version__)"`; use the version pinned on the official image or align with the installed source. |
| `RuntimeError: … must be greater than 3.6.1` (slider demo) | Installed pymycobot too old for the slider node | See per-node version check at the top of the script; the two official demos disagree on purpose — pick the one matching the image. |
| `/dev/ttyTHS1` permission denied | User not in `dialout` | `sudo usermod -aG dialout $USER` then log out/in (official image usually already grants this). |
| Launch exits: port busy | Another process owns the UART (e.g. a leftover test script) | Close it; only one owner of the serial port at a time. |
| `ros2 topic echo` shows nothing | Bridge not running, wrong port/baud, or QoS mismatch | `ros2 topic list -t` to confirm the topic exists; check the launch's `--show-args` and the installed bridge source (JN serial = `/dev/ttyTHS1` @ 1000000). |
| `ros2 topic pub` hangs | It waits for a matching subscriber → bridge not subscribed/running | Start Phase 3 first; `--once` returns as soon as delivery happens. |
| Two robots/teams interfere on the lab network | Same `ROS_DOMAIN_ID` shares the DDS graph | Assign a unique `ROS_DOMAIN_ID` per robot/team. |
| rviz2 fails over SSH | No X display | Run on the robot desktop, or `export DISPLAY=:0` with an active desktop session. |

---

## 5. References

* Elephant Robotics JN docs — ROS2: <https://docs.elephantrobotics.com/docs/mycobot_280_jn_en/3-FunctionsAndApplications/6.developmentGuide/ROS/12.2-ROS2/12.2.1-InstallationOfROS2.html>
* `mycobot_ros2` repository (our branch: `foxy`): <https://github.com/elephantrobotics/mycobot_ros2>
* `pymycobot` serial API: <https://github.com/elephantrobotics/pymycobot>
* ROS2 documentation: <https://docs.ros.org>
