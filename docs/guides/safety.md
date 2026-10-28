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
- **Put the arm straight up (all joints ≈ 0°) before you turn on the motors.**
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
2. Put the arm straight up by hand.
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
