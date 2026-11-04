# ROS 2 in braco

braco is a set of ROS 2 nodes. `arm_driver` is the only one that talks to the arm; the joystick, the web page,
`make zero` / `make park` and the scripts are its clients.

## Where to find what

| I want to know... | Read |
|---|---|
| Which nodes run, and what each one does | [architecture §3: Nodes and devices](../../architecture/architecture.md#3-nodes-and-devices) |
| Every topic and service: who sends, who uses it | [architecture §4: Topics and services](../../architecture/architecture.md#4-topics-and-services) |
| Who may move the arm | [architecture §5: Control of the arm](../../architecture/architecture.md#5-control-of-the-arm) |
| The simulation's nodes, topics and action | [architecture §9](../../architecture/architecture.md#9-simulation-and-planned-layers), [braco_description](../../../ros2_ws/src/braco_description/README.md) |
| Our messages (`ArmCommand`, `TakeControl`...) | [braco_interfaces](../../../ros2_ws/src/braco_interfaces/README.md) |
| One node in detail, and how to run it by hand | Its package README: [braco_driver](../../../ros2_ws/src/braco_driver/README.md), [braco_teleop](../../../ros2_ws/src/braco_teleop/README.md), [braco_camera](../../../ros2_ws/src/braco_camera/README.md), [braco_bringup](../../../ros2_ws/src/braco_bringup/README.md) |

## See it live

| World | ROS domain | Command |
|---|---|---|
| Real arm (Jetson) | 22 | On the Jetson: `make run CMD='ros2 node list'`. From the PC: `ROS_DOMAIN_ID=22 ros2 node list` |
| Simulation (PC, `make up SIM=1`) | yours, 20–29 ([per teammate](../../../docker/README.md#ros-domain-per-teammate); 27 if unlisted) | `make run CMD='ros2 node list'` |

- **Why 20–29:** DDS turns the domain ID into UDP ports (`7400 + 250 × domain + offset`). Above 232 the port
  passes 65535 and nodes cannot start. On Linux, domains 102–214 collide with the ephemeral ports (32768–60999).
  So the real arm (22) and every teammate's PC (20–29) stay inside 0–101. Source:
  [ROS 2 docs, *The ROS_DOMAIN_ID*](https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/nodes/About-Domain-ID.rst).

- Also: `ros2 topic list`, `ros2 service list`, `ros2 action list`, `ros2 topic echo /joint_states`.
- Or without a terminal: the web page's ROS 2 tab shows every node, topic and service.

## More

| Doc | What it is for |
|---|---|
| [INSTRUCTIONS.md](INSTRUCTIONS.md) | Hands-on: write a publisher, subscriber, service, action and lifecycle node |
| [COMMAND_REFERENCES.md](COMMAND_REFERENCES.md) | The commands, on one page |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Common errors and fixes |
