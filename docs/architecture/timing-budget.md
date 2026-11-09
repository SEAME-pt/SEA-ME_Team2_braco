# Timing budget of the real arm

Measured timing of the path from ROS 2 to the myCobot 280 on the Jetson Nano: serial round trip, `/joint_states` rate,
and command → motion latency. Input for the ros2_control `update_rate` ([controllers.yaml](../../ros2_ws/src/braco_description/config/controllers.yaml)),
[ADR-0004](../decisions/ADR-0004-trajectories-through-arm-driver.md), #101 and #93. Architecture: [architecture.md](architecture.md).

Measured on **2026-11-09** (#90), real arm, Jetson Nano, no desktop (`multi-user.target`), `arm_driver` alone in a
`real` container (no `temperature_monitor`, joystick, camera or page).

## Summary

| | Value | Bound by |
|---|---|---|
| Serial round trip, one joint read (`getAngles`) | **9.3 ms** mean, 11.8 ms p99, 13.9 ms max, 0 failures / 2000 | Atom firmware (the wire is 0.22 ms) |
| `/joint_states`, arm idle | **18.7 Hz** at `state_rate_hz` 20; ceiling **~45 Hz** from 60 Hz up | `sleep(10ms)` + read in the serial loop |
| `/joint_states`, continuous commands (the #87 plugin) | **~14 Hz** (70 ms) at any `state_rate_hz` from 20 to 100 | `COMMAND_GAP` (50 ms) after every motion command |
| Command → first joint motion (J6, 5°, speed 30) | **~205 ms** p50, 140 ms min, 268 ms max | Atom and servos starting the move |

**Conclusion:** during a trajectory the real arm takes a new command and gives a new position about every **70 ms
(~14 Hz)**, and starts moving **~200 ms** after a command. Neither depends on `state_rate_hz` or on the controller's
`update_rate`. Keep `update_rate: 100` and `state_rate_hz: 20`; the levers are inside `arm_driver` ([below](#what-would-change-it)).

## Serial loop of `arm_driver`

One thread does all serial work ([arm_driver_node.cpp](../../ros2_ws/src/braco_driver/src/arm_driver_node.cpp), `serialLoop()`):

```
every pass:  [motion command pending?] ── yes ──► sendAngles ─► sleep COMMAND_GAP 50 ms ─┐
                      │ no                                                                │
                      ▼                                                                   ▼
             [state due?] ── yes ──► getAngles (~9 ms) ─► publish /joint_states ─► [no command sent?] ─► sleep 10 ms
             [every 2 s]  ──────────► getTemps + getVoltages (~2 × 9 ms)
```

| Case | Period of `/joint_states` | Measured |
|---|---|---|
| Idle, `state_rate_hz` low | T + ~5 ms (the loop checks the clock every ~10 ms; the next period counts from the read) | 20 Hz → 18.7 Hz |
| Idle, ceiling | read ~9.3 ms + sleep 10 ms + overhead ≈ 22 ms | ~45 Hz |
| Continuous commands | send + 50 ms gap + read ≈ 60 ms predicted | **70 ms**: the read after a motion command seems slower (~10 ms more); not confirmed |

## Results

### 1. Serial round trip

`scripts/timing/serial_rtt.py`: 1000 × `getAngles` (request `FE FE 02 20 FA`, 17-byte reply) at 1 000 000 baud on
`/dev/ttyTHS1`, `arm_driver` stopped.

| Pause between reads | Reads | Mean ms | p50 ms | p99 ms | Max ms | Failed |
|---|---|---|---|---|---|---|
| 5 ms | 1000 | 9.34 | 9.27 | 11.83 | 13.87 | 0 |
| 0 ms | 1000 | 8.33 | 8.22 | 11.86 | 13.96 | 0 |

22 bytes at 1 Mbaud take 0.22 ms: ~97 % of the round trip is the Atom answering. A higher baud rate would not help.

### 2. `/joint_states` rate

`scripts/timing/js_rate.py`: 60 s per point, intervals from `header.stamp`. *Load* = `scripts/timing/hold_load.py`
sending `ALL_ANGLES` of the current pose at the same rate as `state_rate_hz`, as the #87 plugin will (ADR-0004); the arm
holds still. CPU: `arm_driver`, % of one core.

| `state_rate_hz` | Load | Achieved Hz | Achieved / requested | Mean ms | p50 ms | p99 ms | Max ms | CPU % |
|---|---|---|---|---|---|---|---|---|
| 20 | idle | 18.7 | 94 % | 53.4 | 52.7 | 60.7 | 76.5 | 5 |
| 20 | commands | 14.4 | 72 % | 69.4 | 71.0 | 91.2 | 98.2 | 11 |
| 30 | idle | 25.5 | 85 % | 39.2 | 41.1 | 52.0 | 60.0 | 7 |
| 30 | commands | 14.1 | 47 % | 70.7 | 71.9 | 95.5 | 100.3 | 12 |
| 40 | idle | 31.4 | 79 % | 31.9 | 32.0 | 52.0 | 56.2 | 5 |
| 40 | commands | 14.0 | 35 % | 71.3 | 71.8 | 94.7 | 99.9 | 12 |
| 50 | idle | 40.3 | 81 % | 24.8 | 23.9 | 39.9 | 56.1 | 5 |
| 50 | commands | 14.2 | 28 % | 70.6 | 71.5 | 95.3 | 99.4 | 14 |
| 60 | idle | 45.9 | 77 % | 21.8 | 20.1 | 39.7 | 47.8 | 6 |
| 60 | commands | 14.0 | 23 % | 71.3 | 71.8 | 95.5 | 98.6 | 16 |
| 60 (repeat) | idle | 44.5 | 74 % | 22.5 | 23.8 | 40.0 | 52.1 | 6 |
| 60 (repeat) | commands | 14.3 | 24 % | 70.0 | 71.2 | 92.4 | 100.3 | 16 |
| 70 | idle | 45.0 | 64 % | 22.2 | 21.4 | 42.4 | 48.2 | 6 |
| 70 | commands | 14.1 | 20 % | 70.8 | 71.4 | 94.8 | 98.6 | 16 |
| 90 | idle | 44.5 | 49 % | 22.5 | 23.8 | 39.8 | 48.0 | 6 |
| 90 | commands | 14.1 | 16 % | 70.8 | 71.0 | 96.3 | 98.3 | 17 |
| 100 | idle | 45.4 | 45 % | 22.0 | 20.5 | 39.7 | 48.0 | 6 |
| 100 | commands | 14.2 | 14 % | 70.5 | 70.6 | 96.0 | 101.2 | 19 |

- Idle: the rate follows the request up to ~50 Hz, then stays at ~45 Hz.
- Commands: ~14 Hz whatever the request. Commands above that rate are overwritten before they are sent (latest wins)
  and only cost CPU (11 → 19 %).
- p99 / max idle (~40 / ~48 ms at the ceiling): the temperature and voltage reads every 2 s. This is the "before"
  jitter for #93.
- Read failures: one `no valid angle feedback` right after each `power on`, none during the runs.

### 3. Command → motion latency

`scripts/timing/latency_step.py`: J6 +5° and back, 20 steps per point, speed 30, the target resent at `state_rate_hz`.
Latency = `header.stamp` of the first `/joint_states` with J6 more than 0.5° from where it was, minus the time of the
step's first command.

| `state_rate_hz` = command rate | Steps | Min ms | p50 ms | Max ms | Not seen |
|---|---|---|---|---|---|
| 20 | 20 | 145 | 209 | 268 | 0 |
| 30 | 20 | 140 | 208 | 232 | 0 |
| 40 | 20 | 144 | 206 | 239 | 0 |
| 50 | 20 | 153 | 207 | 234 | 0 |
| 60 | 20 | 145 | 205 | 241 | 0 |
| 70 | 20 | 141 | 200 | 225 | 0 |
| 80 | 20 | 149 | 204 | 236 | 0 |
| 90 | 20 | 140 | 204 | 229 | 0 |
| 100 | 20 | 146 | 200 | 236 | 0 |

| Part of the latency | Best case (≈ min) | Average (≈ p50) |
|---|---|---|
| Wait for the serial loop (a new command can arrive during the previous one's `COMMAND_GAP`) | 0 | ~35 |
| `sendAngles` + `COMMAND_GAP` + next read | ~60 | ~60 |
| Atom and servos until the joint has moved 0.5° | **X** | **X** |
| Quantization (the motion shows at the next read, every ~70 ms) | 0 | ~35 |
| **Measured** | **140** → X ≈ 80 | **205** → X ≈ 75 |

Both columns give the same servo reaction (~75–80 ms), so the model fits. The rate does not change the latency: the
loop is saturated by commands at every rate tested. With 20 steps p99 equals the max.

## What it means

| For | Consequence |
|---|---|
| `update_rate` (#101) | 100 Hz stays. The controller runs at any rate; the arm gets a new setpoint and gives a new position every ~70 ms, so `read()` sees fresh data about one cycle in seven. Lowering it would not change the arm, only the controller's CPU (not measured here) |
| `JointTrajectoryController` on the real arm (#101) | Tolerances must allow ~200 ms of lag and 70 ms steps: no tight `path_tolerance`, `goal_time` ≥ 0.5 s |
| ADR-0004 | Setpoints are thinned to **~14 Hz**, not 20 Hz |
| Watchdog | `command_timeout_ms` 300 vs 70 ms between commands under load: 4× margin |
| `state_rate_hz` | 20 stays. Higher helps only while idle, where no client needs it |
| #93 (SCHED_FIFO) | Baseline jitter above (idle p99 / max). The loop is bound by sleeps and the firmware, not by scheduling: expect the max to improve, not the rate |

### What would change it

Inside `arm_driver`, for a follow-up issue (measure again with the same scripts):

1. **`COMMAND_GAP` 50 ms**: the main cost under commands. Find the smallest gap the Atom accepts.
2. **`sleep(10ms)`** when no command was sent: half of the idle ceiling.
3. Temperature and voltage reads every 2 s: the idle p99 / max.

## Method

```bash
make down                                                    # host: free the serial port
cd docker && COMPOSE_PROFILES=jetson docker compose run --rm --name timing real bash    # terminal A
docker exec -it timing ros_entrypoint.sh bash                # terminal B, in /opt/braco/scripts/timing
```

1. Serial (A, no `arm_driver`): `python3 serial_rtt.py -n 1000` and `--gap-ms 0`.
2. `arm_driver` (A): `ros2 run braco_driver arm_driver --ros-args -p state_rate_hz:=<F>.0` (a double; read only at
   start, so restart it for each F). B: `ros2 run braco_bringup zero` once.
3. Rate (B), per F: `js_rate.py --label idle`, then `hold_load.py --rate F` with `js_rate.py --label load` (60 s each).
4. Latency (B), per F: `latency_step.py --requested F --rate F --n 20`.
5. B: `ros2 run braco_bringup park`; host: `make up`.

Results are written to `ros2_ws/bags/timing/` (git-ignored).
