# Arm "rest" mode and basic commands

## Why it changed

Running `park.py` directly on the host (old version, direct serial access) while `arm_driver` was running gave
`SerialException: ... multiple access on port?`. The workaround was to bring the containers down, and without
`arm_driver` nobody read the temperature: the arm cooled down unmonitored.

## What changed

| Where | Change |
|---|---|
| `arm_driver.cpp` | New **rest mode** (read only): refuses motion, gripper and `power_on`, but keeps publishing `/joint_states` and `/arm/servo_temperatures`. Service `/arm/set_read_only` (`std_srvs/SetBool`), topic `/arm/read_only` (latched) and parameter `read_only`. `STOP` and `release_servos` keep working. |
| `ros2_ws/src/mycobot_cpp/srcs/mycobot.cpp` | The serial port is opened with `TIOCEXCL`: a second (non-root) process gets "Device or resource busy" right away, instead of corrupting the reads. |
| `park` (C++, `braco_bringup`) | Leaves rest, goes to the park pose, releases the servos and **enters rest**. The driver stays alive measuring temperature. |
| `zero` (C++, `braco_bringup`) | Leaves rest before powering the servos and moving. |
| `arm_moves.cpp` | New `setRest(on)` (client of the service above). |
| `Makefile` (repo root) | Commands `park`, `zero`, `rest`, `resume`, `temps` (run inside the `real` container; they were `arm-*` before #54). |

## How to use (on the Jetson, in `docker/`)

```bash
make park      # parks, releases the servos and enters rest (nothing needs to be stopped)
make temps     # prints the temperatures once (works in rest)
make logs      # follows the nodes (monitor, driver, teleop); the LCD keeps updating
make resume    # leaves rest (motion accepted again)
make zero      # leaves rest, powers the servos and goes to the zero pose
make rest      # enters rest without parking (e.g. just to lock motion)
```

Typical flow with a hot arm: `make park` → wait → `make temps` (or watch the LCD) until it is back to
normal (< 55 °C) → `make zero` (or `make resume` if you will use the joystick; the servos stay released
until `make zero` or `power_on`).

## Rules

- **Never open the serial port outside `arm_driver`** (the standalone `mycobot_cpp` tools, pymycobot on the host). Use the targets above.
- In rest, the joystick and `ArmCommand` are ignored (warning in the driver log every 5 s).
- A servo at ≥ 65 °C (`TEMP_HOT_C`) puts the arm in rest **by itself**: `temperature_monitor` calls
  `/arm/set_read_only true` once per episode (until all joints are below 60 °C: a reading that jitters around 65 °C stays one episode) and the LCD shows
  `REST: SERVO HOT` / `J4 66 C cooling`. Leaving it is always manual (`make resume`); a `make resume` during the
  same episode is not undone (useful to park a hot arm on purpose). Can be disabled with the
  `auto_rest_on_hot` parameter. At that moment the monitor runs `park` (folds the arm, releases the joints to cool down and turns rest on); if `park` cannot run or fails, it only turns rest on. Parameters `park_on_hot` and `park_command`. See [ADR-0003](ADR-0003-rest-per-temp.md).
- `zero` and `park` take control of the arm by force (`take_control("script", force=true)`) and hand it back at the end:
  they work with the joystick connected or the web page in control. See [ADR-0002](ADR-0002-unique-control.md).
- `arm_routine.sh` (entrypoint of the `real` container) does not move the arm when stopping: use `make down PARK=1`, which runs `park` before shutting down and leaves the arm in rest.
- `ros2 topic echo /arm/read_only` shows the current state.

## How to test

1. `make up`
2. `make park`, then `make temps` returns `valid: true`.
3. The joystick does not move the arm; after `make resume` it moves again.
4. With the driver running, a standalone tool (`make -C ros2_ws/src/mycobot_cpp temps_lcd && ros2_ws/src/mycobot_cpp/temps_lcd`)
   fails with "Device or resource busy".
5. Simulated hot servo: with the system running, one fake reading at 66 °C is enough (the next real reading,
   2 s later, ends the episode):
   `make run CMD='ros2 topic pub --once /arm/servo_temperatures braco_interfaces/msg/ServoTemperatures "{celsius: [40, 40, 40, 66, 40, 40], valid: true}"'`.
   The arm enters rest, the monitor log and the LCD warn about it, and `make resume` brings it back.
