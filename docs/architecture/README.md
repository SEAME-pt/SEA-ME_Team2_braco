# Architecture

How the braco software is built today: ROS 2 packages, nodes, topics and services, deployment on the Jetson and the PC.

| Doc | Content |
|---|---|
| [architecture.md](architecture.md) | Layers, nodes, topics, packages and deployment |
| [resources.md](resources.md) | CPU and memory on the Jetson, and the mitigations |
| [timing-budget.md](timing-budget.md) | Measured serial round trip, `/joint_states` rate and command → motion latency on the Jetson |
| [timing/sched-fifo/](timing/sched-fifo/) | Plots of the `/joint_states` periods with and without `SCHED_FIFO`, shown in [timing-budget.md](timing-budget.md#4-sched_fifo-under-cpu-load) |

When a PR must update `architecture.md`: [docs-rules](../guides/docs-rules.md#9-architecture-changes).
