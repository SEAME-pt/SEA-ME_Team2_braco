# Jetson resources: CPU and memory

What braco costs on the Jetson Nano, and how the project keeps it within limits. Architecture: [architecture.md](architecture.md).

> **Estimates, not measurements.** Assumes a 4 GB Nano (4 Cortex-A57 cores, RAM shared with the GPU), no desktop, 10 W mode. Replace with real numbers from `make status` ([pending](#measure)).

## Summary

| | Value |
|---|---|
| RAM, system running | ~0.3 GB |
| CPU, in operation | ~0.2 core (~5% of the Nano); ~0.4 with the page's ROS tab open |
| RAM peak, `colcon build` (2 × g++) | ~1.6 GB: the main out-of-memory risk |

```
                         0         1 GB      2 GB      3 GB      4 GB
Operation      ~0.95 GB  ███████▓▓▓·······························   free ~3.1 GB
During a build ~2.55 GB  ███████▓▓▓░░░░░░░░░░░░░░░░···············   free ~1.5 GB
                                             ↑ limit of a 2 GB Nano
█ L4T + Docker   ▓ braco   ░ colcon build   · free   (1 char ≈ 100 MB)
```

Running, braco is light. **The risk is the build:** on a 2 GB Nano or with the desktop on, it triggers the OOM killer or swaps to the SD card.

## Per process

| Process | RAM | CPU (1 core = 100%) |
|---|---|---|
| `arm_driver` | 15–25 MB | 4–8% |
| `temperature_monitor` | 10–20 MB | <1% |
| `joystick_teleop` | 10–20 MB | 1–3% |
| `camera` (Python + OpenCV) | 60–90 MB | 3–8% |
| `ros2 launch` | 50–70 MB | <1% |
| Web page (uvicorn + rclpy) | 100–150 MB | 5–15% (20–40% with the ROS tab open) |
| Docker + containerd | 80–120 MB | 1–2% |
| **Total** | **~330–490 MB** | **~15–38%** |
| `colcon build` | 0.6–1 GB per g++ | 100% per job; 10–25 min from scratch, 1–3 min incremental |

MoveIt 2 (`move_group`, 300–600 MB, a full core while planning) and `braco_mission` run on the PC, not the Jetson.

## Mitigations

| Problem | Mitigation | Where |
|---|---|---|
| Build runs out of RAM | One package at a time, `MAKEFLAGS=-j2`, Release, no tests; `make doctor` warns without swap (suggests 4 GB). Or build on the PC: `make jetson-image` | `mk/config.mk`, `mk/setup.mk` |
| Desktop wastes 0.5–1 GB | `make doctor` warns; disable it: `sudo systemctl set-default multi-user.target` | `mk/setup.mk` |
| 5 W mode halves the CPU | `make doctor` warns; switch: `sudo nvpmodel -m 0` | `mk/setup.mk` |
| SD card fills up | `CCACHE_MAXSIZE=1G`; `make clean` also deletes `~/.ros/log` | `docker/compose.yml`, `mk/dev.mk` |

## Measure

*Pending.* Run `make status` (`docker stats` per container + `tegrastats`) in three situations: arm idle, joystick in use, page's ROS tab open. Replace the estimates above with the results.
