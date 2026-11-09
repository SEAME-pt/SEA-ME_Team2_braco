# Architecture

How the braco software is built today: ROS 2 packages, nodes, topics and services, deployment on the Jetson and the PC.

| Doc | Content |
|---|---|
| [architecture.md](architecture.md) | Layers, nodes, topics, packages and deployment |
| [resources.md](resources.md) | CPU and memory on the Jetson, and the mitigations |
| [timing-budget.md](timing-budget.md) | Measured serial round trip, `/joint_states` rate and command → motion latency on the Jetson |

A PR that changes the architecture updates `architecture.md` in the same PR.
