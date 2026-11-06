# braco_hardware

## What it is

A plugin that lets ros2_control see the real arm.

ros2_control runs `arm_controller`, the controller MoveIt uses. The controller does not talk to the arm itself:
a hardware plugin does. Each setup has its own plugin:

| Setup | Hardware plugin |
|---|---|
| Simulation | `gz_ros2_control/GazeboSimSystem` |
| Real arm | `braco_hardware/BracoSystem` (this package) |

## Where it was decided

[ADR-0004](../../../docs/decisions/ADR-0004-trajectories-through-arm-driver.md): the plugin never talks to the arm
directly. It goes through `arm_driver`, over topics.

| Why | Because |
|---|---|
| One serial port owner | Only `arm_driver` opens it |
| Safety in one place | STOP, rest mode, who controls the arm, the watchdog and park on overheat stay in `arm_driver` |
| One `/joint_states` publisher | `arm_driver` keeps publishing it; the plugin only listens |

## The flow

```
real arm ──serial──► arm_driver ──/joint_states──► BracoSystem ──read()──► ros2_control ──► arm_controller
                     (20 per second)               (keeps the latest)      (100 per second)
```

## What it reads and gives

| | Name | Details |
|---|---|---|
| Listens to | `/joint_states` | `sensor_msgs/JointState`, from `arm_driver` |
| Publishes | nothing | |
| Parameters | none | |

For each joint, `joint1` to `joint6`:

| Interface | Kind | Value |
|---|---|---|
| `position` | state | the joint angle from `/joint_states`, in radians |
| `velocity` | state | from `/joint_states`, or 0 when missing. `arm_driver` sends none, so it is always 0 |
| `position` | command | there for `arm_controller`, not sent to the arm yet (#101) |

## How it works

Nobody runs the plugin by hand. `ros2_control_node` (the program that runs ros2_control) starts it:

| # | What happens | Where |
|---|---|---|
| 1 | The robot model (URDF) is built with the real-arm option on | `braco_description/urdf/braco.urdf.xacro real_arm:=true` |
| 2 | The model names the plugin to use: `braco_hardware/BracoSystem` | `braco_description/urdf/braco.real.xacro` |
| 3 | `ros2_control_node` reads the model and finds that name | |
| 4 | It looks the name up and loads the plugin's library | `braco_hardware.xml` → `libbraco_hardware.so` |
| 5 | It calls the plugin's steps, in the order below | `src/braco_system.cpp` |

The plugin's steps:

| Step | When | What it does | Fails if |
|---|---|---|---|
| `on_init` | once | checks the URDF joints | a joint has no `position` or `velocity` state |
| `on_configure` | once | starts listening to `/joint_states`, keeps the latest message | |
| `on_activate` | once | waits up to 2 s for the first message | none arrives: `is arm_driver running?` |
| `read()` | 100 per second | copies the latest message into the joint states | there is no message yet |
| `write()` | 100 per second | nothing yet (#101) | |

Why `on_activate` waits: `read()` starts less than a millisecond after `on_configure`, before any message can arrive.
A failing `read()` switches the plugin off, so without the wait it would switch off at every start.

## Try it on the PC, without the arm

Open four terminals with `make shell` and run one block in each:

| Terminal | Plays the part of |
|---|---|
| 1 | `arm_driver` (a fake `/joint_states`) |
| 2 | the URDF, given to ros2_control |
| 3 | `ros2_control_node`, which loads the plugin |
| 4 | you, checking the plugin |

```bash
# 1
ros2 topic pub -r 20 /joint_states sensor_msgs/msg/JointState \
  "{name: [joint1, joint2, joint3, joint4, joint5, joint6], position: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]}"
```

```bash
# 2 (through a file: the URDF is too long for -p)
xacro src/braco_description/urdf/braco.urdf.xacro real_arm:=true > /tmp/braco_real.urdf
python3 -c "import yaml; print(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': \
  {'robot_description': open('/tmp/braco_real.urdf').read()}}}))" > /tmp/rsp.yaml
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /tmp/rsp.yaml
```

```bash
# 3
ros2 run controller_manager ros2_control_node
```

```bash
# 4
ros2 control list_hardware_components -v
```

| Terminal 1 | Expected in terminal 4 |
|---|---|
| running | `BracoSystem` is `active` |
| stopped | after 2 s, `BracoSystem` is `unconfigured`, and terminal 3 shows `is arm_driver running?` |
