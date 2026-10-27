---
status: accepted
date: 2026-10-14
deciders: all
---

# Run ROS 2 in Docker on the Jetson Nano

## Context

The myCobot 280 is wired to the Nano's UART (`/dev/ttyTHS1`, 1 Mbaud) and the camera to its USB, so the robot nodes must run on the Nano. The Nano runs JetPack 4.6 on Ubuntu 20.04 (kernel 4.9, 4 GB RAM, SD card). The only native ROS 2 is Galactic, which has been end-of-life since 2022. Newer distros need newer Ubuntu.

**Question:** where and how should ROS 2 run, without depending on the native install or an external computer?

## Drivers

- Standalone: no external host required
- Not locked to an end-of-life distro
- Direct access to the arm (UART) and camera (USB)
- Limited resources (4 GB RAM, 4 cores, SD card)
- Reproducible, versioned environment

## Options

| Option | Pros | Cons |
|---|---|---|
| **Docker on the Nano** | Distro independent of the host; reproducible; standalone; arm and camera verified | No GPU out of the box; disk use and slow builds |
| Native Galactic on the Nano | Already installed; direct GPU access | End-of-life; newer distros impossible; conflicts with ROS 1 Noetic also installed |
| External host, Nano as thin driver | Heavy work on a stronger machine | Depends on a second computer and network; the Nano still needs ROS for the driver |

## Decision

**Docker on the Nano**, because it frees the distro from the host OS and keeps the system standalone.

- Container distro: **ROS 2 Lyrical Luth on Ubuntu 26.04**, on the Nano and the PC.
- Arm and camera reach the container by device passthrough (`--device`), not privileged mode.
- Out of scope: GPU access. The first vision pipeline is CPU-based. GPU needs its own ADR.

## Consequences

- ✅ A current distro runs on a JetPack 4.6 / Ubuntu 20.04 host
- ✅ The environment can be rebuilt from the Dockerfile and compose files; the distro can be changed by swapping the base image
- ✅ No external computer needed (a PC stays useful for builds and visualization)
- ❌ No GPU/TensorRT in a plain ROS container: host Tegra libraries do not match the container's userspace
- ❌ The vendor's `mycobot_ros2` packages target Galactic and may need porting
- ❌ Builds are slow and must be one image at a time (parallel builds froze Docker); `make jetson-image` builds on the PC instead

## Confirmation

| Check | Result |
|---|---|
| Container opens `/dev/ttyTHS1` and reads the arm via `pymycobot` | Pass |
| Container reads camera frames (640x480) | Pass |
| ROS 2 Lyrical Luth on Ubuntu 26.04 runs on the Nano | Pass |
| GPU stack usable from a plain ROS container | Fail (see Consequences) |

## Links

- Setup: [docker/README.md](../../docker/README.md)
- Follow-ups: GPU/vision strategy; arm driver on Lyrical (port `mycobot_ros2` or own node around `pymycobot`)
- Revisit if: the driver cannot run in a Lyrical container, Ubuntu 26.04 is unstable on the 4.9 kernel, SD/RAM limits make containers impractical, or a GPU requirement cannot be met
