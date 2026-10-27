---
status: "accepted"
date: 2026-10-14
decision-makers: all
consulted: all
informed: all
---

# Run ROS 2 in Docker on the Jetson Nano instead of natively or on an external host

## Context and Problem Statement

We are building vision-guided pick-and-place (and simple pen drawing) with an
Elephant Robotics myCobot 280 Jetson Nano arm, using ROS 2. The arm is wired to
the Nano's UART (`/dev/ttyTHS1`, 1000000 baud) and an eye-in-hand USB RGB camera
is plugged into the Nano, so the robot-facing nodes must run on the Nano.

Platform facts (measured on the device):

| Item | Value |
|---|---|
| Board / SoC | Jetson Nano, Tegra X1 (T210), aarch64 |
| Kernel | 4.9.253-tegra |
| Ubuntu (host) | 20.04.6 LTS (Focal Fossa) |
| L4T | R32.6.1 (built 2021-07-26) |
| JetPack | 4.6 (CUDA 10.2, cuDNN 8, TensorRT 8, as seen from containers) |
| Docker | 24.0.7 (Ubuntu `docker.io` package), `nvidia` runtime registered, default runtime `runc` |
| Docker Compose | v2.29.7, installed manually as a CLI plugin (the apt method of the official guide did not work) |
| Resources | 4 GB RAM (+ about 2 GB zram swap), 59 GB SD card with 16 GB free at test time |
| Installed ROS (host) | ROS 2 Galactic and ROS 1 Noetic (`/opt/ros`) |
| Arm | Atom firmware 7.3 |
| Arm library | `pymycobot` 4.0.7 (use `MyCobot280`; the generic `MyCobot` class is deprecated) |

The only ROS 2 distro installed natively is Galactic. It matches the host's
Ubuntu 20.04 but reached end-of-life in late 2022 (no updates or security fixes).
Newer distros target newer Ubuntu releases (Humble 22.04, Lyrical Luth 26.04)
and cannot be installed natively on this OS.

Question: where and how should ROS 2 run on this system so that it does not
depend on the limitations of the native install or on an external computer?

## Decision Drivers

* Independence: the system must run standalone, with no external host required
* Avoid being locked to an end-of-life native distro (Galactic)
* Flexibility to choose or change the ROS 2 distro and dependencies per project
* Direct access to the arm (UART) and camera (USB) from the ROS nodes
* Limited Nano resources (4 GB RAM, 4 cores, SD card storage)
* Reproducible, versioned environment
* Compatibility with the vendor's `mycobot_ros2` packages (built for Galactic)

## Considered Options

* Native ROS 2 Galactic on the Nano
* Docker on the Nano
* External host running ROS 2 (Docker or native), with the Nano as a thin driver

## Decision Outcome

Chosen option: "Docker on the Nano", because it removes the dependency on the
native end-of-life distro, keeps the whole system standalone (no external host),
and gives the versatility to choose and change distros and dependencies per
container. The container tests passed on a Galactic (Ubuntu 20.04) image, and a
separate compose setup with its own Dockerfiles for **ROS 2 Lyrical Luth on
Ubuntu 26.04** also ran as intended on the same Nano.

Scope notes:

* **Container distro: ROS 2 Lyrical Luth (Ubuntu 26.04 userspace).** It is the
  distro validated as the target for new work. The Galactic image is kept as a
  reference for the vendor's `mycobot_ros2` packages, which were built for it.
* **GPU access is not part of this decision.** On the plain Ubuntu 20.04 Galactic
  image, the `nvidia` runtime did not give a usable GPU stack (see Consequences).
  The first vision pipeline is CPU-based (background subtraction), so this does
  not block the decision. How to run a learned detector on the GPU needs a
  separate decision.

### Consequences

* Good, because the distro is no longer tied to the host OS: a current distro
  (Lyrical Luth on Ubuntu 26.04) runs on a JetPack 4.6 / Ubuntu 20.04 host with a
  4.9 Tegra kernel, which no native install could offer
* Good, because the environment is reproducible, versioned, and rebuildable from
  the compose files and Dockerfiles
* Good, because the system runs without an external computer
* Good, because arm (UART) and camera access work from a container with plain
  device passthrough (`--device`), without privileged mode
* Good, because the distro can be changed again later by swapping the base image
* Good, because the native Galactic install remains as a fallback and as a
  reference for the vendor packages
* Bad, because **GPU/TensorRT access is not available out of the box in a
  plain Ubuntu 20.04 ROS container.** Observed with the `nvidia` runtime on the
  Galactic image: the CUDA runtime library was not found, and `import cv2` failed
  (`libnvrm.so` missing) because host Tegra libraries built for the 18.04
  userspace are mounted into a newer-userspace container. Without the `nvidia`
  runtime, `cv2` and the camera work normally. The same check has not been
  recorded for the Lyrical image (TODO)
* Bad, because in the L4T base image (Ubuntu 18.04, Python 3.6) CUDA 10.2,
  cuDNN 8 and TensorRT 8 libraries are visible, but TensorRT's Python bindings
  are not, and Python 3.6 cannot run modern `rclpy`
* Bad, because the vendor's `mycobot_ros2` packages target Galactic and may need
  porting to Lyrical Luth
* Bad, because building on the Nano is resource-limited: building several images
  in parallel froze Docker on this device. Images must be built one at a time
  (the Makefile does this), or built on a PC for arm64
* Bad, because image size and build time are significant on a 59 GB SD card

### Confirmation

Tests are in the `task3/` bundle (`make test-all`, results logged in `results/`)
plus the separate Lyrical Luth compose setup.

| Check | Result |
|---|---|
| Docker present, `nvidia` runtime registered | Pass |
| Galactic image builds | Pass (apt key refresh was not a problem) |
| Galactic container opens `/dev/ttyTHS1` at 1000000 baud, reads firmware, power state, 6 joint angles via `pymycobot` | Pass |
| Galactic container reads frames from the USB camera (640x480) | Pass |
| CUDA 10.2 / cuDNN 8 / TensorRT 8 libraries visible in the L4T r32.6.1 image | Pass |
| GPU stack usable from a plain Ubuntu 20.04 ROS image | Fail (see Consequences) |
| ROS 2 Lyrical Luth on Ubuntu 26.04 container (own compose file and Dockerfiles) | Pass: everything worked as intended (reported by the team; TODO list which checks were run) |

## Pros and Cons of the Options

### Native ROS 2 Galactic on the Nano

Already installed at `/opt/ros/galactic`, matching Ubuntu 20.04.

* Good, because it is installed and matches the OS and the vendor packages
* Good, because it has direct access to hardware, CUDA, and TensorRT from JetPack
* Bad, because Galactic is end-of-life and newer packages may not support it
* Bad, because newer distros (Humble, Lyrical Luth) cannot be installed natively
* Bad, because ROS 1 Noetic is installed alongside it, so a mis-sourced shell can
  cause conflicts

### Docker on the Nano

* Good, because the distro is independent of the host OS (Galactic and Lyrical
  Luth containers both run on the same host)
* Good, because it is reproducible and versioned
* Good, because the system is standalone, with no external dependency
* Good, because arm and camera access were verified from containers
* Neutral, because hardware access needs explicit device passthrough
* Bad, because of the GPU/userspace mismatch (Ubuntu 18.04 L4T vs newer ROS images)
* Bad, because of disk usage and build times, and the build-parallelism freeze
  seen on this device

### External host running ROS 2, with the Nano as a thin driver

* Good, because heavy components (MoveIt 2, RViz, builds, model training) run on
  more powerful hardware
* Bad, because it makes the system depend on a second computer and a network link
* Bad, because the driver and camera must still run on the Nano, so it does not
  remove the need for a ROS setup on the Nano
* Note: a PC remains useful as an optional development aid (image builds,
  training, visualization), but not as a production dependency

## More Information

* Task 1 (platform versions): done, see the context table
* Task 2 (native distro): Galactic, already installed
* Task 3 (container test): done on Galactic; repeated on ROS 2 Lyrical Luth
  (Ubuntu 26.04) with a separate compose file and Dockerfiles
* Follow-up decisions needed:
  * Vision/GPU strategy: (a) CPU-only classical vision first, (b) a Jetson-built
    ROS image on the L4T base, or (c) a split with ROS 2 in a plain container and
    GPU vision in an L4T container linked by a non-ROS interface or bridge
  * How to provide the arm driver on Lyrical Luth: port the vendor's
    `mycobot_ros2` packages, or write a small driver node around `pymycobot`
* Revisit this decision if: the arm driver cannot run on Lyrical Luth in a
  container; the Ubuntu 26.04 userspace proves unstable on the L4T 4.9 kernel;
  the SD card or RAM limits make container workflows impractical; or a
  GPU-dependent requirement cannot be met with any container setup
