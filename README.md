# braco

[![CI](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/workflows/ci.yml)

SEA:ME 2.0 Team 2's robotic arm project: vision-guided pick-and-place on an Elephant Robotics **myCobot 280 Jetson Nano**, built with ROS 2 (Lyrical) in Docker. It runs on the real arm (Jetson) or in Gazebo simulation (PC).

## Install

Requirements: Linux, Docker with Compose, git.

```bash
git clone https://github.com/SEAME-pt/SEA-ME_Team2_braco.git
cd SEA-ME_Team2_braco
make setup      # once per machine: docker/.env, venv, git hooks, checks
make up         # Jetson: the real arm. PC: dev container (SIM=1 adds the simulation)
make            # list every command
```

Read the [safety rules](docs/guides/safety.md) before moving the real arm. Full setup: [dev-setup.md](docs/guides/dev-setup.md).

## Docs

| Folder | Content |
|---|---|
| [architecture/](docs/architecture/README.md) | How the software is built |
| [hardware/](docs/hardware/README.md) | Arm, parts, protocol, URDF model |
| [decisions/](docs/decisions/README.md) | Architecture Decision Records |
| [guides/](docs/guides/README.md) | Setup, safety, operation, ROS 2 |

More: [docker/](docker/README.md), [web/](web/README.md), [ros2_ws/src/braco_description/](ros2_ws/src/braco_description/README.md).
