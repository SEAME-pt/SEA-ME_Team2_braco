# Lab Safety Induction & Workstation Rules: myCobot 280 Jetson Nano

This document records safety induction compliance and defines workstation operating rules for the robotic arm cell, configured with the **Elephant Robotics myCobot 280 Jetson Nano**, **parallel gripper**, and **vision camera**.

---

## 1. Safety Induction Sign-off Record

The following team members have attended all mandatory laboratory training modules, inspected the workstation, and reviewed the emergency shutdown protocols.

| Date | GitHub Handle | Electrical & Guarded Cell | LiPo Lab Handling | Emergency Protocol Verified | Induction Status |
| :--- | :--- | :---: | :---: | :---: | :--- |
| 2026-10-09 | `@larocqueg` | [x] | [x] | [x] | Completed |
| 2026-10-09 | `@denionline` | [x] | [x] | [x] | Completed |
| 2026-10-09 | `@d-branco` | [x] | [x] | [x] | Completed |
| 2026-10-09 | `@magusk89` | [x] | [x] | [x] | Completed |
| 2026-10-09 | `@carlos-lvt` | [x] | [x] | [x] | Completed |

---

## 2. Workstation Hardware Profile

* **Manipulator:** Elephant Robotics myCobot 280 (Jetson Nano edition, 6 DOF, 280 mm working radius, ~250 g rated payload).
* **Power Supply:** Dedicated 60W AC-to-DC desktop power adapter (Mains 100-240V AC input, dedicated DC barrel connector). *Note: The arm operates strictly on mains DC power; no onboard batteries are installed.*
* **End-Effector Tooling:**
  * **Adaptive Parallel Electric Gripper:** Driven via serial/PWM auxiliary port.
  * **Tool-Mounted / Cell Camera:** Connected via USB to the Jetson Nano carrier board.

---

## 3. Workstation Operating Rules & Guarded Cell

### 3.1 Workspace Clearance & Pre-Power Positioning
* **Exclusion Zone:** Maintain a clear operating envelope with a minimum radius of **350 mm** centered on the robot base. No personal items, loose clothing, or non-essential tools may enter this boundary while the arm is powered.
* **Rigid Clamping:** The myCobot base must remain securely clamped or bolted to the workstation surface at all times during operation. Never energize servos on a free-standing base.
* **Pre-Power Upright Pose Protocol (Zero-Gravity Alignment):**
  * Before applying motor power or software torque, manually articulate the arm into the vertical upright configuration (all joint angles positioned close to $0^\circ$).
  * *Rationale:* Powering the arm while slumped or horizontally cantilevered places maximum gravitational torque ($\tau = r \times F_g$) onto Joint 2 and Joint 3, causing violent startup jolts, stripping internal gearbox teeth, and risking surface collisions that can damage the camera lens and gripper tips.
* **Manual Joint Articulation:** Never back-drive or force arm joints manually while servo torques are engaged (`power_on`). Disengage torque via software or cut power before manipulating joints by hand.

### 3.2 Tooling Hazards: Gripper & Camera Integration
* **Pinch Points (Gripper):** Keep hands and fingers clear of the gripper jaws at all times. Do not attempt to seat or extract workpieces by hand while gripper commands are executing.
* **Object Slip & Drop Hazards:** Ensure gripped items conform to size limits and do not exceed remaining payload limits. Never move the arm at high acceleration profiles while holding heavy or slippery objects.
* **Camera Cable Tethering & Clearance:** 
  * Ensure adequate strain relief and slack on the camera USB cable along Joint 4, Joint 5, and Joint 6.
  * Inspect the cable harness before running autonomous paths to prevent cable wrapping, joint binding, or connector damage during full rotations ($J6 \pm 165^\circ$).
* **Payload Budget:** The total combined weight of the gripper, mounting bracket, camera, and payload must not exceed the arm's **250 g** payload rating. Exceeding this limit causes servo stall errors and excessive thermal wear.

---

## 4. Electrical Safety & 60W Power Management

* **Power Delivery:** The workstation is powered by a dedicated 60W regulated DC supply.
* **Standard Power-On & Torque Sequencing:**
  1. **Visual & Harness Check:** Verify camera cable slack and clear foreign objects from the 350 mm zone.
  2. **Manual Pre-Alignment:** Manually support and align the arm into the **vertical upright position** ($J1$–$J6 \approx 0^\circ$).
  3. **Controller Boot:** Boot the Jetson Nano controller and verify baseline serial communication (`/dev/ttyTHS1`).
  4. **DC Bus Energization:** Switch on the main 60W DC power feed via the base circular switch.
  5. **Controlled Torque Engagement:** Lightly support the arm structure with one hand while sending the software torque command (`power_on`) to absorb any residual settling jerk.
* **Stall Current & Thermal Limits:** If a joint encounters an obstacle or binds, cut servo torque immediately. Sustained servo stalls at full load trigger over-current conditions, PSU brownouts, and driver overheating.
* **Common Grounding:** Any auxiliary instrumentation or microcontrollers connected to GPIO pins must share a common ground reference with the Jetson carrier board before applying logic signals.

---

## 5. Emergency Shutdown & Rapid Stop Procedures

### 5.1 Hardware Power Cutoff (Full Workstation Shutdown)
* **Primary Cutoff:** Press the integrated circular power switch on the base back panel, or unplug the 60W DC barrel connector directly from the base.
* **Impact:** Instantly disconnects power from both the servo drive bus and the Jetson Nano carrier board.
* **When to Use:** Use immediately in critical emergencies such as electrical shorts, severe collisions, physical entanglement, or smoke/burning smell.

### 5.2 Software Operational Stop & Holding Torque Behavior
* **Script Interruption Behavior:** Terminating a Python process or interrupting execution (e.g., via `Ctrl+C` or uncaught exceptions) does **not** disengage motor torque. The internal servo microcontrollers and gripper driver retain their active closed-loop holding torque and remain locked at their last commanded angles.
* **Active Torque Release Procedure:** To deliberately disengage holding torque and allow manual positioning of the manipulator or gripper jaws, an explicit serial release command must be executed with a mandatory post-initialization delay for UART buffer stabilization:
  ```python
  import time
  from pymycobot import MyCobot

  # Initialize serial connection
  mc = MyCobot('/dev/ttyTHS1', 1000000)
  time.sleep(0.5)  # Required: wait for UART port and controller handshake stabilization

  # Release all joint servo torques
  mc.release_all_servos()

### 5.3 Post-Shutdown Recovery Routine
1. Disconnect or terminate the faulty control script.
2. Manually guide the arm back into the **vertical upright position** ($0^\circ$).
3. Re-engage power to the base.
4. Execute an initial homing sequence at low speed ($\le 15\%$) to verify joint telemetry and camera alignment.

---

## 6. General Lab LiPo Safety Compliance

Although the myCobot 280 workstation runs exclusively on a 60W mains adapter, all operators have completed the mandatory laboratory LiPo safety module to ensure safe coexistence with mobile robotics in the shared space:
* Batteries must only be charged in designated fireproof bags using certified balance chargers.
* Any battery exhibiting puffing, swelling, or low cell voltage ($<3.0\text{V}$) must be retired to the lab quarantine sand container immediately.
