# Lab Safety: myCobot 280 Jetson Nano

Safety rules for the arm, its gripper and its camera.

## 1. Induction sign-off

| Date | GitHub Handle | Electrical & Guarded Cell | LiPo Lab Handling | Emergency Protocol Verified | Induction Status |
| :--- | :--- | :---: | :---: | :---: | :--- |
| 2026-10-09 | `@larocqueg` | [x] | [x] | [x] | Completed |
| 2026-10-09 | `@denionline` | [x] | [x] | [x] | Completed |
| 2026-10-09 | `@d-branco` | [x] | [x] | [x] | Completed |
| 2026-10-09 | `@magusk89` | [x] | [x] | [x] | Completed |
| 2026-10-09 | `@carlos-lvt` | [x] | [x] | [x] | Completed |

## 2. Hardware

| Part | Details |
|---|---|
| Arm | 6 joints, reaches 280 mm, carries **250 g at most** |
| Power | 60 W adapter plugged into the wall. No batteries |
| Gripper | Electric |
| Camera | USB, plugged into the Jetson |

## 3. Work area

- **Nothing within 350 mm of the base** while the arm has power: no objects, no loose clothes, no hands.
- **The base is always clamped** to the table.
- **If the arm is not in park position, put the arm straight up (all joints ≈ 0°) before you turn on the motors.**
  - If the arm is bent, its weight pulls hard on J2 and J3: it jumps, can break the gears and can hit the camera or
    gripper.
- **Never move a joint by hand while the motors are on.** Turn them off first (section 6.3).

## 4. Gripper, camera and weight

- **Gripper:** keep your fingers away from it. Do not put objects in it by hand while it moves.
- **Weight:** gripper + camera + object **≤ 250 g**. More than that and the motors get stuck and overheat.
- **Camera cable:** leave it loose on J4-J6, and check it before long moves (J6 turns ±178° and can wrap it).
- **Move slowly** when holding heavy or slippery objects.

## 5. Power

### 5.1 Turning on

1. Clear the 350 mm area and check the camera cable.
2. If the arm is not in the park position, put the arm straight up by hand.
3. Start the Jetson.
4. Press the round switch on the base.
5. Hold the arm lightly while you send `power_on` (the motors turn on).

### 5.2 Electrical rules

- **A joint is blocked:** turn off the motors at once. If not, they draw too much current and overheat.
- **Something wired to the GPIO pins:** connect its ground to the Jetson's ground first.

## 6. Stopping the arm

### 6.1 Emergency: cut the power

- **How:** press the round switch on the back of the base, or pull out the power cable.
- **What happens:** the motors **and** the Jetson turn off at once.
- **When:** short circuit, crash, something caught in the arm, smoke or a burning smell.

### 6.2 Stopping from software

- **Stopping a program does not free the arm.** After Ctrl+C or a crash the motors stay on and hold the arm in place.
- **STOP works from anywhere** (web page, joystick, scripts), no matter who is in control.
- **Only one client moves the arm at a time** ([ADR-0002](../decisions/ADR-0002-unique-control.md)).
  - Who is in control: see the web page, or `ros2 topic echo /arm/control_owner`.
  - The joystick takes control from the page only if you hold **button 8 for about 1 s**.
- **If `arm_driver` crashes,** the whole system stops. Check with `make status` and `make logs`.

### 6.3 Turning off the motors (free the joints)

- **System running:** `make park`. It folds the arm, turns off the motors and puts it in rest mode.
- **Never open the serial port yourself while the system runs:** only `arm_driver` may use it.
- **System off** (`make down`): you can free the joints with pymycobot:

  ```python
  import time
  from pymycobot import MyCobot

  mc = MyCobot('/dev/ttyTHS1', 1000000)
  time.sleep(0.5)          # needed: wait for the port
  mc.release_all_servos()
  ```

## 7. Hot motors

- **The arm is hot?** Follow [overheating.md](overheating.md).

## 8. Starting again after an emergency stop

1. Stop the program that caused the problem.
2. Put the arm straight up by hand.
3. Turn the power back on.
4. Move it to zero slowly (speed ≤ 15%) and check the angles and the camera.

## 9. First MoveIt 2 motion on the real arm

Go/no-go checklist. If any box is not ticked, do not move the arm.

### 9.1 Before

- [ ] MoveIt started with `make moveit`: its domain check passed
  ([#103](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/103)): real arm on domain 22, nothing else on it.
  On FAIL `move_group` does not start.
- [ ] Limits test green ([#99](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/99)).
- [ ] MoveIt velocity and acceleration scaling **≤ 0.1**: `default_velocity_scaling_factor` and
  `default_acceleration_scaling_factor` in `braco_moveit_config/config/joint_limits.yaml`, and `move_to_pose`'s
  `velocity_scaling` / `acceleration_scaling` (ADR-0004).
- [ ] Everyone present has signed the induction (section 1).
- [ ] Work area clear (section 3).
- [ ] Arm at park. Goals go through `move_to_pose` with `use_control: true`: it takes the control as `mission`
  (without force) before each goal and gives it back after ([ADR-0004](../decisions/ADR-0004-trajectories-through-arm-driver.md)).
  `braco_hardware` sends only while `/arm/control_owner` is `mission`, so a goal from RViz alone does not move the arm.
- [ ] Rest mode left (`make resume`), then motors on (`/arm/power_on`).
- [ ] Motor temperatures OK (section 7, [overheating.md](overheating.md)).

### 9.2 During

- [ ] One person on STOP (section 6.2): the web page's STOP button, or
  `ROS_DOMAIN_ID=22 ros2 topic pub --once /arm/command braco_interfaces/msg/ArmCommand '{type: 0}'`.
  - ⚠️ **A STOP does not latch today.** It halts the arm once, but the client in control keeps it and its next
    command moves the arm again. Tested on 2026-11-09 (PR #124): the page's STOP halts a move of the page; a STOP
    from the terminal does **not** stop a move that the page streams. Use STOP **from the client that moves the
    arm**, or the power switch (section 6.1).
  - A MoveIt trajectory should be different (#101, not yet tried on the arm): while `mission` holds the control,
    `braco_hardware` sees a STOP from any client, stops `arm_controller` and the goal is aborted, so nothing moves
    the arm again until a new goal. Check it on the first motion.
- [ ] One person on the power switch (section 6.1).
- [ ] First move small, in a direction with nothing in the way.

### 9.3 After

- [ ] Return the arm to park.
- [ ] Record what happened (date, people, motion, result, problems) in a comment on
  [#101](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/101).

### 9.4 Abort

- **Stop at once if:** the arm leaves the planned path, moves faster than expected, makes unusual noise, gets hot,
  hits something or loses communication, or for any reason in section 6.1.
- **Who decides:** anyone can call STOP. Only the test lead, named before the test, decides whether to continue.
  After an abort, follow section 8.
