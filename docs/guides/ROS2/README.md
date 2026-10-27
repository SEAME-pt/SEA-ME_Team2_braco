# ROS 2 in braco

braco is a set of ROS 2 nodes. `arm_driver` is the only one that talks to the arm; the joystick, the web page,
`make zero` / `make park` and the scripts are its clients.

## Where to find what

| I want to know... | Read |
|---|---|
| Which nodes run, and what each one does | [architecture §2: Node graph](../../architecture/architecture.md#node-graph) |
| Every topic and service: who sends, who uses it | [architecture §4: ROS 2 graph](../../architecture/architecture.md#4-ros-2-graph-and-control-real-arm-simfalse) |
| Who may move the arm | [architecture §4: Who moves the arm](../../architecture/architecture.md#who-moves-the-arm-adr-0002) |
| The simulation's nodes, topics and action | [architecture §10](../../architecture/architecture.md#10-simulation-and-the-planned-upper-layers), [braco_description](../../../ros2_ws/src/braco_description/README.md) |
| Our messages (`ArmCommand`, `TakeControl`...) | [braco_interfaces](../../../ros2_ws/src/braco_interfaces/README.md) |
| One node in detail, and how to run it by hand | Its package README: [braco_driver](../../../ros2_ws/src/braco_driver/README.md), [braco_teleop](../../../ros2_ws/src/braco_teleop/README.md), [braco_camera](../../../ros2_ws/src/braco_camera/README.md), [braco_bringup](../../../ros2_ws/src/braco_bringup/README.md) |

## See it live

| World | ROS domain | Command |
|---|---|---|
| Real arm (Jetson) | 42 | On the Jetson: `make run CMD='ros2 node list'`. From the PC: `ROS_DOMAIN_ID=42 ros2 node list` |
| Simulation (PC, `make up SIM=1`) | 77 | `make run CMD='ros2 node list'` |

- Also: `ros2 topic list`, `ros2 service list`, `ros2 action list`, `ros2 topic echo /joint_states`.
- Or without a terminal: the web page's ROS 2 tab shows every node, topic and service.

## More

| Doc | What it is for |
|---|---|
| [INSTRUCTIONS.md](INSTRUCTIONS.md) | Hands-on: write a publisher, subscriber, service, action and lifecycle node |
| [CHEATSHEET.md](CHEATSHEET.md) | The commands, on one page |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Common errors and fixes |
