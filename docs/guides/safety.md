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
- **STOP works from anywhere** (web page, terminal, scripts), no matter who is in control.
- **A STOP latches** ([ADR-0005](../decisions/ADR-0005-stop-latch.md)): the arm stays stopped, whatever any client
  sends, and nobody can take control.
  - Release it, once the area is safe, with `make resume` or the page's **Release STOP**. Both are refused for 3 s
    after the last STOP.
  - The release does not move the arm. Take control and send a new command to move it.
- **Deactivating `arm_driver` is a STOP** (#95): `ros2 lifecycle set /arm_driver deactivate` stops the arm and
  latches. Motion stays refused until `ros2 lifecycle set /arm_driver activate`, and activating does not release the
  latch. Angles and temperatures are still read.
- **Only one client moves the arm at a time** ([ADR-0002](../decisions/ADR-0002-unique-control.md)).
  - Who is in control: see the web page, or `ros2 topic echo /arm/control_owner`.
  - The joystick takes control from the page only if you hold **button 8 for about 1 s**.
- **If `arm_driver` crashes or cannot open the serial port,** the whole system stops. Check with `make status` and
  `make logs`. A crash sends no stop: the arm finishes the last target it got and holds it, motors on.
- **If the serial link to the arm is lost** ([ADR-0007](../decisions/ADR-0007-serial-loss-latches-stop.md)), a STOP
  latches after 1 s without angle feedback, seen within about 2.5 s. `make resume` is refused until the angles come back. The move in progress
  when the link died still ends on the arm.
- **A client that dies or goes quiet** (page, joystick, script, mission) stops sending: the watchdog stops the arm
  300 ms after the last command. Results of each fault: section 10.

### 6.3 Turning off the motors (free the joints)

- **System running:** `make park`. It folds the arm, turns off the motors and puts it in rest mode.
- **Never open the serial port yourself while the system runs:** only `arm_driver` may use it.
- **Turning the system off:** `make down` (also `restart`, `re`, `fclean` and `systemctl stop braco`) parks the arm
  first. Stopped without a park (`PARK=0`, `docker stop`, a crash of `arm_driver`), the motors stay on, holding the
  pose, and nothing watches their temperature.
- **System off without a park:** you can free the joints with pymycobot (hold the arm: it drops):

  ```python
  import time
  from pymycobot import MyCobot

  mc = MyCobot('/dev/ttyTHS1', 1000000)
  time.sleep(0.5)          # needed: wait for the port
  mc.release_all_servos()
  ```

## 7. Hot motors

- **The arm is hot?** Follow [overheating.md](overheating.md).
- **Parking on overheat needs an active `arm_driver`.** While it is inactive, a hot motor only puts the arm in rest
  mode: the motors stay on.

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
  The STOP latches (section 6.2): the MoveIt client cannot move the arm again until someone releases it.
  `braco_mission`'s next goal is refused (its `take_control` is refused while latched) until `make resume`.
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

## 10. Faults rehearsed

`make fault` on the Jetson injects each fault while joint 1 moves at speed 10, measures the time to stop on
`/joint_states`, and sends a STOP during each fault ([#142](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/142)).
It asks before every move, and its last two sections stop the system (`make up` after it). It writes the measured
table to `ros2_ws/bags/fault/results.md`: copy it into the last column below after each rehearsal on the real arm.

Before running it: section 9.1 (area, induction, arm at zero), one person on the power switch (section 6.1).

| Fault | Expected result | Bound on the time to stop | Covered by | Real arm |
|---|---|---|---|---|
| Client killed mid-move (`web` stream, page server) | Watchdog stops the arm | 400 ms (300 ms watchdog, one serial pass, one sample) | `make fault` §1-2, gtest `WatchdogStopsOnceWhenCommandsStop` | pending |
| Mission client (`move_group`) killed mid-trajectory | The goal runs to its end in `arm_controller` on the Jetson, then the arm holds | end of the goal | `make fault` §5 | pending |
| `ros2_control_node` killed mid-trajectory | `write()` stops, the arm stops at its last setpoint, the watchdog stops it | 400 ms | `make fault` §7 | pending |
| Network PC-Jetson cut mid-move | Page moves and goals run on the Jetson: the move ends there. A STOP from a terminal on the Jetson works | end of the move | `make fault` §6 (by hand) | pending |
| `arm_driver` killed (SIGKILL) | No stop sent: the arm finishes its last target and holds; the launch and the container stop | last target | `make fault` §8 (by eye: no feedback) | pending |
| Serial link lost (cable inside the base) | STOP latched after 1 s without feedback (seen in ~1.3 s, 2.5 s at most), release refused until the link is back, stop sent again then | move in progress ends on the arm | gtests `FeedbackLoss…`, `DeadLink…`, `ClosedPort…`, `SerialLoss…` (ADR-0007) | not rehearsed: the cable is inside the base |
| NaN angles from the owner | Dropped whole: no motion; the lease and the watchdog are not fed | no motion | `make fault` §3, gtests `NonFiniteAngles…` | pending |
| Angle out of the joint limits | Clamped to the limit: the arm goes to the limit, not past it | n/a | gtest `AnglesOutOfTheLimitsAreClamped` (not on the arm: it moves it to the limit) | n/a |
| Pose below the table | Refused: no motion | no motion | `make fault` §3, gtest `PoseIntoTheTableIsNotSent` | pending |
| Motion from a wrong `source` | Ignored: no motion | no motion | `make fault` §3, gtest `MotionFromAWrongSourceIsIgnored` | pending |
| `ros2 lifecycle set /arm_driver deactivate` mid-move | Stops and latches (#95) | 300 ms | `make fault` §4, `make bench` §4, gtest `DeactivateStopsTheArm` | pending |
| STOP during each fault above | Latches; the arm stops | 300 ms | `make fault` (a STOP in each section) | pending |

- **STOP when the network is cut:** a STOP from the PC (page in its browser, MoveIt) cannot reach the arm. Send the
  STOP of section 9.2 from a shell on the Jetson, or use the power switch (section 6.1). The joystick has no STOP.
- **After `arm_driver` dies:** nothing stops the arm in software. The page streams targets at most 15° ahead and the
  mission's targets follow the trajectory, so the last target is close. A single far target from a script is not.
