# Jetson resources: CPU and memory

What braco costs on the Jetson Nano, and how the project keeps it within limits. Architecture: [architecture.md](architecture.md).

> **Measured 2026-11-26**: 4 GB Nano, MAXN (10 W), `multi-user.target`, no camera connected. Method: [Measure](#measure).

## Summary

| | Value |
|---|---|
| RAM, braco container | 0.54 GB idle, 0.61 GB during a trajectory, 0.67 GB with the page's ROS tab open |
| RAM, whole system | 2.6–2.7 GB of 3.96 GB; ~2.1 GB is L4T + Docker + other host processes (not itemized) |
| CPU, braco container | ~0.5 core idle, ~0.7 core during a trajectory or with the ROS tab open (13–17% of the Nano) |
| RAM peak, `colcon build` (2 × g++) | ~1.6 GB (estimate): the main out-of-memory risk |

```
                         0         1 GB      2 GB      3 GB      4 GB
Idle           ~2.61 GB  █████████████████████▓▓▓▓▓··············   free ~1.35 GB
ROS tab open   ~2.74 GB  █████████████████████▓▓▓▓▓▓▓············   free ~1.2 GB
During a build ~4.2 GB   █████████████████████▓▓▓▓▓░░░░░░░░░░░░░░░░ → swap
█ L4T + Docker + other   ▓ braco   ░ colcon build (estimate)   · free   (1 char ≈ 100 MB)
```

Running, braco fits. **The risk is the build:** on top of the running system it does not fit in RAM and swaps. Build on the PC (`make jetson-image`) or stop the system first.

## Per process

Second `top` sample over 5 s; CPU as % of one core. RAM is RES, idle → trajectory.

| Process | RAM (MB) | Idle | Joystick | Trajectory | ROS tab |
|---|---|---|---|---|---|
| `ros2_control_node` (`controller_manager` + `arm_controller`, 100 Hz) | 95 → 103 | 38% | 36% | 48% | 28% |
| `arm_driver` | 67 → 75 | 4% | 11% | 7% | 3% |
| `joystick_teleop` | 66 → 71 | 6% | 12% | 5% | 5% |
| `robot_state_publisher` | 63 → 71 | 4% | 3% | 4% | 3% |
| `temperature_monitor` | 65 → 70 | <1% | <1% | <1% | <1% |
| `camera` (no device: not streaming) | 203 → 208 | <1% | <1% | <1% | <1% |
| `ros2 launch` | 133 → 137 | 1% | 1% | 1% | 1% |
| `ros2-daemon` | 100 → 105 | <1% | 1% | 1% | <1% |
| Web page (uvicorn + rclpy) | 167 | – | – | – | 23% |
| **Container** (`docker stats`) | **539 → 605** (666 with the page) | **52%** | **61%** | **67%** | **67%** |
| Docker + containerd (host) | 80–120 (estimate) | 1–2% | | | |
| `colcon build` (estimate) | 0.6–1 GB per g++ | 100% per job; 10–25 min from scratch, 1–3 min incremental | | | |

The RES column sums to more than the container: shared libraries are counted in every process. `arm_driver` timing: [timing-budget.md](timing-budget.md).

MoveIt 2 (`move_group`) and `braco_mission` run on the PC, not the Jetson (checked: no `move_group` in the container). `move_group` on the PC: ~220 MB, ≤6% of one core (1 s samples) while planning the `run_goal_set.py` goals.

## Mitigations

| Problem | Mitigation | Where |
|---|---|---|
| Build runs out of RAM | One package at a time, `MAKEFLAGS=-j2`, Release, no tests; `make doctor` warns without swap (suggests 4 GB). Or build on the PC: `make jetson-image` | `mk/config.mk`, `mk/setup.mk` |
| Desktop wastes 0.5–1 GB | `make doctor` warns; disable it: `sudo systemctl set-default multi-user.target` | `mk/setup.mk` |
| 5 W mode halves the CPU | `make doctor` warns; switch: `sudo nvpmodel -m 0` | `mk/setup.mk` |
| SD card fills up | `CCACHE_MAXSIZE=1G`; `make clean` also deletes `~/.ros/log` | `docker/compose.yml`, `mk/dev.mk` |
| CPU: `ros2_control_node` is the largest user | None needed yet; if CPU gets short, look at its 100 Hz loop first | `braco_description/config/controllers*.yaml` |

## Measure

System up, arm at zero. Four cases, each held for ~5 s: idle, joystick moved continuously, trajectory (`run_goal_set.py --yes` from the PC), page's ROS tab open. For each, at the same time:

- `docker compose exec -T real top -b -c -w 512 -d 5 -n 2 -o %CPU`, keeping the second sample
- `docker stats --no-stream`
- `tegrastats --interval 1000`

On the PC, during the trajectory case: `top -b -d 1 -n 30 -p <move_group pid>`. The cores ran at 204–518 MHz (DVFS), so CPU % is relative to a slowed core.
