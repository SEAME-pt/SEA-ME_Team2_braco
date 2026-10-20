# ROS 2: Installation and Usage, Step by Step

This guide takes you from nothing to a working ROS 2 workspace with every core pattern from Sprint 0 ([issue #18](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/18)): publisher/subscriber with explicit QoS in C++ and Python, service, action, lifecycle node, launch file, and inspection with the CLI and `rqt_graph`.

Read [README.md](README.md) first if the concepts (node, topic, QoS, lifecycle...) are new. For a quick list of commands see [CHEATSHEET.md](CHEATSHEET.md); when something breaks see [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

> **Safety:** nothing in this guide moves the arm. Before running any node that talks to the hardware, read [docs/safety.md](../docs/safety.md).

---

## Table of contents

- [0. Which installation path to use](#0-which-installation-path-to-use)
- [1. Install with the project's Docker setup (recommended)](#1-install-with-the-projects-docker-setup-recommended)
  - [1.1 On the PC](#11-on-the-pc)
  - [1.2 On the Jetson Nano](#12-on-the-jetson-nano)
  - [1.3 Daily use of the containers](#13-daily-use-of-the-containers)
- [2. Alternative: native install on Ubuntu 26.04](#2-alternative-native-install-on-ubuntu-2604)
- [3. Verify the installation](#3-verify-the-installation)
- [4. First steps with the ros2 CLI (turtlesim)](#4-first-steps-with-the-ros2-cli-turtlesim)
- [5. Create the workspace and the packages](#5-create-the-workspace-and-the-packages)
- [6. Create a custom interfaces package](#6-create-a-custom-interfaces-package)
- [7. Publisher and subscriber with explicit QoS](#7-publisher-and-subscriber-with-explicit-qos)
- [8. Service server and client](#8-service-server-and-client)
- [9. Action server and client](#9-action-server-and-client)
- [10. Lifecycle node](#10-lifecycle-node)
- [11. Build files (CMakeLists.txt, setup.py, package.xml)](#11-build-files-cmakeliststxt-setuppy-packagexml)
- [12. Build and run everything](#12-build-and-run-everything)
- [13. Launch file](#13-launch-file)
- [14. Parameters](#14-parameters)
- [15. Inspect the system: ros2 CLI and rqt_graph](#15-inspect-the-system-ros2-cli-and-rqt_graph)
- [16. Run across two machines (PC and Jetson)](#16-run-across-two-machines-pc-and-jetson)
- [17. Record and replay with rosbag2](#17-record-and-replay-with-rosbag2)
- [18. Push the examples to the repo](#18-push-the-examples-to-the-repo)

Conventions in this guide:

- `host$` is a terminal on your machine (outside Docker), in the repo root unless noted.
- `container$` is a terminal inside the ROS 2 container (`make shell`).
- `ROS_DISTRO` is `lyrical`. If you ever use another distro, replace `lyrical` in package names.

---

## 0. Which installation path to use

| Your machine | Use |
|---|---|
| Linux PC (any recent distro) | **Docker, section 1.1** |
| Jetson Nano (JetPack 4.6, Ubuntu 20.04 host) | **Docker, section 1.2**. Do not install ROS 2 natively: Lyrical is not available for Ubuntu 20.04 (see [ADR-0001](../docs/ADR-0001-ros2-in-docker-on-jetson-nano.md)) |
| A PC running Ubuntu 26.04 that should have ROS natively | Section 2 (optional; Docker still recommended so everyone has the same environment) |
| Windows | Docker inside WSL2 (Ubuntu), then section 1.1 from the WSL terminal. GUIs need WSLg |
| macOS | Docker Desktop works for the CLI; GUIs (RViz2, rqt) are not supported by our setup |

The Docker setup gives everyone the **same** ROS 2 Lyrical install on Ubuntu 26.04, on both the PC (`pc-dev` image: ROS desktop, RViz2, MoveIt 2, Gazebo, rqt, QA tools) and the Jetson (`jetson-driver` image: headless ROS base, serial tools, `pymycobot`). Full details are in [docker/README.md](../docker/README.md).

---

## 1. Install with the project's Docker setup (recommended)

### 1.1 On the PC

**Step 1. Install Docker Engine and Docker Compose v2.20 or newer.**

On Ubuntu/Debian, follow <https://docs.docker.com/engine/install/ubuntu/>. In short:

```bash
host$ sudo apt-get update
host$ sudo apt-get install -y ca-certificates curl
host$ sudo install -m 0755 -d /etc/apt/keyrings
host$ sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
host$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
host$ sudo apt-get update
host$ sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

**Step 2. Let your user run Docker without sudo**, then log out and back in (or reboot):

```bash
host$ sudo usermod -aG docker $USER
```

Check:

```bash
host$ docker run --rm hello-world
host$ docker compose version          # must be v2.20 or newer
```

**Step 3. Install the X11 helper** (needed so RViz2, rqt and Gazebo can open windows):

```bash
host$ sudo apt-get install -y x11-xserver-utils make git
```

**Step 4. Clone the repo** (skip if you already have it):

```bash
host$ git clone git@github.com:SEAME-pt/SEA-ME_Team2_braco.git
host$ cd SEA-ME_Team2_braco/docker
```

All `make` commands below are run **from the `docker/` folder**.

**Step 5. Create your `.env`** (fills in your user ID, group IDs and the machine role):

```bash
host$ make env ROLE=pc
```

Open `docker/.env` and check:

- `COMPOSE_PROFILES=pc`
- `USER_UID` / `USER_GID` match `id -u` / `id -g`
- `ROS_DOMAIN_ID=42` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` — **these must be identical on every machine that should talk to each other**. If several teams share the lab network, agree on different domain IDs per team.

**Step 6. Check the host:**

```bash
host$ make doctor
```

Every line should be `[ ok ]`. Fix any `[FAIL]` before continuing ([TROUBLESHOOTING.md](TROUBLESHOOTING.md) lists the usual causes).

**Step 7. Build and start the container.** The first build downloads Ubuntu 26.04 and ROS 2 and takes a while (tens of minutes, depending on your connection):

```bash
host$ make up
```

`make up` creates `ros2_ws/src` and `ros2_ws/bags` as your user, builds the `pc-dev` image if missing, and starts it in the background.

**Step 8. Verify:**

```bash
host$ make smoke              # checks packages, pymycobot, RMW loopback
host$ make talker-listener    # C++ talker + Python listener, prints PASS/FAIL
```

**Step 9. Open a shell inside the container:**

```bash
host$ make shell
(pc-dev) dev@host:/ws$ echo $ROS_DISTRO
lyrical
```

You are now in `/ws`, which is the folder `ros2_ws` of the repo, mounted into the container. Anything you put in `/ws/src` is saved in the repo on the host.

### 1.2 On the Jetson Nano

Connect to the Nano (keyboard and screen, or `ssh user@<nano-ip>`). Docker 24 and Compose v2.29 are already installed on our Nano (see ADR-0001).

**Step 1. Free the arm's UART** (on stock JetPack the serial console holds `/dev/ttyTHS1`). Only needed once:

```bash
nano$ sudo systemctl disable --now nvgetty
```

Log out and back in.

**Step 2. Clone the repo and create the `.env`:**

```bash
nano$ git clone git@github.com:SEAME-pt/SEA-ME_Team2_braco.git
nano$ cd SEA-ME_Team2_braco/docker
nano$ make env ROLE=jetson
nano$ make doctor ROLE=jetson
```

Check in `.env`: `COMPOSE_PROFILES=jetson`, `MYCOBOT_PORT=/dev/ttyTHS1`, `MYCOBOT_BAUD=1000000`, and the same `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` as the PC.

**Step 3. Get the image.** Two options:

- **Build on the Nano** (slow, and build only one image at a time: parallel builds froze Docker on this device):

  ```bash
  nano$ make build
  ```

- **Or build on the PC and ship it** (faster). On the PC, from `docker/`:

  ```bash
  host$ make binfmt                              # once: lets the PC build arm64 images
  host$ make jetson-cross                        # slow: emulated arm64 build
  host$ make jetson-ship NANO=user@<nano-ip>     # copies the image over ssh
  ```

  Then on the Nano set `IMAGE_TAG=lyrical-arm64` in `.env`.

**Step 4. Start and verify:**

```bash
nano$ make up
nano$ make smoke
nano$ make talker-listener
nano$ make shell
```

The Jetson image has **no GUI tools**. Inspect the Jetson's nodes from the PC (section 16).

### 1.3 Daily use of the containers

| What | Command (from `docker/`) |
|---|---|
| Start the container | `make up` |
| Open a shell (ROS + workspace sourced) | `make shell` (open as many as you want, one per terminal) |
| Run one command | `make exec CMD='ros2 topic list'` |
| Build the workspace | `make ws-build` (adds `--parallel-workers 2` on the Jetson) |
| Run the workspace tests (linters) | `make ws-test`, then `make exec CMD='colcon test-result --verbose'` (see [12.6](#126-run-the-tests-linters)) |
| Install package dependencies | `make ws-deps` |
| Stop the container | `make down` |
| See all targets | `make` |

Without `make`: `docker compose exec pc-dev bash` (or `jetson-driver`). A bare `docker compose exec pc-dev <command>` does **not** source ROS; prefix it with `ros_entrypoint.sh`.

Packages installed with `sudo apt-get install` inside a running container disappear when the container is recreated. Anything permanent must be added to `docker/Dockerfile`.

---

## 2. Alternative: native install on Ubuntu 26.04

Only on a PC running **Ubuntu 26.04** (Lyrical binaries exist only for it). This follows the official "Installing on Ubuntu (deb packages)" page; check <https://docs.ros.org/en/lyrical/Installation.html> in case it changed.

**Step 1. UTF-8 locale:**

```bash
locale                                         # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**Step 2. Enable the Universe repository:**

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

**Step 3. Add the ROS 2 apt repository** with the `ros2-apt-source` package:

```bash
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

**Step 4. Update and upgrade** (ROS packages are built against current Ubuntu libraries; skipping this can break the install):

```bash
sudo apt update
sudo apt upgrade
```

**Step 5. Install ROS 2 and the development tools:**

```bash
sudo apt install -y ros-lyrical-desktop     # ROS, RViz2, rqt, demos. Use ros-lyrical-ros-base for no GUI
sudo apt install -y ros-dev-tools           # colcon, rosdep, vcstool...
sudo apt install -y ros-lyrical-rclcpp-lifecycle
```

**Step 6. Initialise rosdep:**

```bash
sudo rosdep init
rosdep update
```

**Step 7. Source ROS in every new terminal.** Add it to `~/.bashrc` so you do not forget:

```bash
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc
```

Use the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` as the containers if you want a native PC to talk to them.

**Step 8. Create a workspace folder.** The rest of this guide uses `/ws`; natively use e.g. `~/ros2_ws` and replace `/ws` by that path:

```bash
mkdir -p ~/ros2_ws/src
```

To remove a native install later: `sudo apt remove ~nros-lyrical-* && sudo apt autoremove`, then remove `ros2-apt-source`.

---

## 3. Verify the installation

In **two terminals** (two `make shell`, or two native shells):

```bash
# terminal 1
container$ ros2 run demo_nodes_cpp talker
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'

# terminal 2
container$ ros2 run demo_nodes_py listener
[INFO] [listener]: I heard: [Hello World: 1]
```

If the listener prints messages, C++ (`rclcpp`), Python (`rclpy`) and the middleware work. Stop both with `Ctrl+C`. In the Docker setup, `make talker-listener` does exactly this automatically.

Also useful:

```bash
container$ ros2 doctor --report | less      # environment and network report
container$ printenv | grep -E 'ROS|RMW'     # active ROS variables
```

---

## 4. First steps with the ros2 CLI (turtlesim)

Optional but recommended before writing code: it teaches the CLI with something visible. Turtlesim is installed in the PC image.

```bash
# terminal 1
container$ ros2 run turtlesim turtlesim_node        # a window with a turtle opens

# terminal 2: drive it with the arrow keys
container$ ros2 run turtlesim turtle_teleop_key

# terminal 3: explore
container$ ros2 node list                            # /turtlesim, /teleop_turtle
container$ ros2 topic list -t                        # /turtle1/cmd_vel [geometry_msgs/msg/Twist] ...
container$ ros2 topic echo /turtle1/pose             # live position
container$ ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist \
           "{linear: {x: 2.0}, angular: {z: 1.8}}"   # the turtle drives in circles
container$ ros2 service list
container$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'turtle2'}"
container$ ros2 action list -t                       # /turtle1/rotate_absolute
container$ ros2 action send_goal --feedback /turtle1/rotate_absolute \
           turtlesim/action/RotateAbsolute "{theta: 1.57}"
container$ ros2 param list /turtlesim
container$ ros2 param set /turtlesim background_r 150
container$ ros2 run rqt_graph rqt_graph                # picture of the graph
```

If no window opens, run `xhost +local:` on the host (`make up` does it for you) and check `DISPLAY` (see [TROUBLESHOOTING.md](TROUBLESHOOTING.md)).

---

## 5. Create the workspace and the packages

We create three packages in `ros2_ws/src` (`/ws/src` in the container):

```
ros2_ws/src/
├── seame_interfaces/   ament_cmake   custom service and action definitions
├── ros2_basics_cpp/    ament_cmake   C++ nodes: talker, listener, service server/client, lifecycle node
└── ros2_basics_py/     ament_python  Python nodes: the same, plus action server/client and a launch file
```

Run in the container:

```bash
container$ cd /ws/src

container$ ros2 pkg create seame_interfaces \
    --build-type ament_cmake --license Apache-2.0

container$ ros2 pkg create ros2_basics_cpp \
    --build-type ament_cmake --license Apache-2.0 \
    --dependencies rclcpp rclcpp_lifecycle std_msgs seame_interfaces

container$ ros2 pkg create ros2_basics_py \
    --build-type ament_python --license Apache-2.0 \
    --dependencies rclpy std_msgs action_msgs seame_interfaces
```

`ros2 pkg create` generates the skeleton: `package.xml`, `CMakeLists.txt` or `setup.py`, and empty folders. Open `package.xml` in each package and fill in `<description>` and `<maintainer>` (or pass `--maintainer-name "Your Name" --maintainer-email you@example.com` to `ros2 pkg create`: without `--maintainer-email` it derives an address from the name, and a name with characters like `:` gives an invalid e-mail that makes the build fail with `Invalid email`).

The interfaces package does not need `include/` or `src/`:

```bash
container$ rm -rf /ws/src/seame_interfaces/include /ws/src/seame_interfaces/src
```

> Edit the files from the host with your usual editor (VS Code etc.): `ros2_ws/src` on the host is `/ws/src` in the container. Build and run inside the container.

---

## 6. Create a custom interfaces package

Interface packages must be `ament_cmake`, even if only Python uses them.

**Step 1. Create the definition files:**

```bash
container$ mkdir -p /ws/src/seame_interfaces/srv /ws/src/seame_interfaces/action
```

`seame_interfaces/srv/AddTwoInts.srv`:

```text
# Request
int64 a
int64 b
---
# Response
int64 sum
```

`seame_interfaces/action/Fibonacci.action`:

```text
# Goal: how many numbers of the sequence to compute
int32 order
---
# Result: the full sequence
int32[] sequence
---
# Feedback: the sequence computed so far
int32[] partial_sequence
```

**Step 2. `seame_interfaces/CMakeLists.txt`** (replace the generated one):

```cmake
cmake_minimum_required(VERSION 3.10)
project(seame_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  "action/Fibonacci.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

**Step 3. `seame_interfaces/package.xml`**: add these lines inside `<package>`, after `<buildtool_depend>ament_cmake</buildtool_depend>`:

```xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>action_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

**Step 4. Build and check:**

```bash
container$ cd /ws
container$ colcon build --packages-select seame_interfaces
container$ source install/setup.bash
container$ ros2 interface show seame_interfaces/srv/AddTwoInts
container$ ros2 interface show seame_interfaces/action/Fibonacci
```

---

## 7. Publisher and subscriber with explicit QoS

Both languages use the same topic (`chatter`), type (`std_msgs/msg/String`) and QoS, so a C++ talker works with a Python listener and the other way round.

QoS chosen (see [README.md §8](README.md#8-quality-of-service-qos)):

| Policy | Value |
|---|---|
| History | `KEEP_LAST` |
| Depth | 10 |
| Reliability | `RELIABLE` |
| Durability | `VOLATILE` |

### 7.1 Python

`ros2_basics_py/ros2_basics_py/talker.py`:

```python
"""Publisher with an explicit QoS profile."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

CHATTER_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class Talker(Node):

    def __init__(self):
        super().__init__('talker_py')
        rate_hz = self.declare_parameter('publish_rate_hz', 1.0).value
        self._pub = self.create_publisher(String, 'chatter', CHATTER_QOS)
        self._count = 0
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_timer(self):
        msg = String()
        msg.data = f'Hello from Python: {self._count}'
        self._pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self._count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

`ros2_basics_py/ros2_basics_py/listener.py`:

```python
"""Subscriber with an explicit QoS profile."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros2_basics_py.talker import CHATTER_QOS
from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('listener_py')
        self._sub = self.create_subscription(String, 'chatter', self._on_message, CHATTER_QOS)

    def _on_message(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

### 7.2 C++

`ros2_basics_cpp/src/talker.cpp`:

```cpp
// Publisher with an explicit QoS profile.
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker_cpp")
  {
    const double rate_hz = declare_parameter("publish_rate_hz", 1.0);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();
    pub_ = create_publisher<std_msgs::msg::String>("chatter", qos);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {on_timer();});
  }

private:
  void on_timer()
  {
    std_msgs::msg::String msg;
    msg.data = "Hello from C++: " + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: \"%s\"", msg.data.c_str());
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
```

`ros2_basics_cpp/src/listener.cpp`:

```cpp
// Subscriber with an explicit QoS profile.
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener()
  : Node("listener_cpp")
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();
    sub_ = create_subscription<std_msgs::msg::String>(
      "chatter", qos,
      [this](const std_msgs::msg::String & msg) {
        RCLCPP_INFO(get_logger(), "I heard: \"%s\"", msg.data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
```

### 7.3 QoS experiments (do these, they explain most real bugs)

After building (section 12), with `listener_py` (or `listener_cpp`) running:

**a) Incompatible reliability.** Publish best-effort to a reliable subscriber:

```bash
container$ ros2 topic pub --qos-reliability best_effort /chatter std_msgs/msg/String "{data: 'lost'}"
```

The listener receives **nothing** and logs a warning similar to *"New publisher discovered on topic '/chatter', offering incompatible QoS. No messages will be received from it. Last incompatible policy: RELIABILITY"*. Now publish reliable:

```bash
container$ ros2 topic pub --qos-reliability reliable /chatter std_msgs/msg/String "{data: 'received'}"
```

**b) Durability (late joiners).** Change the talker's durability to `TRANSIENT_LOCAL`, keep the listener `VOLATILE`, start the talker, wait 5 s, then start the listener: it only gets new messages. Now set **both** to `TRANSIENT_LOCAL` and repeat: the listener first receives the last 10 messages (the depth) it missed. Finally, `VOLATILE` talker + `TRANSIENT_LOCAL` listener: incompatible, nothing arrives.

**c) See the QoS of every endpoint:**

```bash
container$ ros2 topic info -v /chatter
```

---

## 8. Service server and client

Service `add_two_ints` of type `seame_interfaces/srv/AddTwoInts`.

### 8.1 Python

`ros2_basics_py/ros2_basics_py/add_two_ints_server.py`:

```python
"""Service server: returns a + b."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from seame_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server_py')
        self._srv = self.create_service(AddTwoInts, 'add_two_ints', self._on_request)

    def _on_request(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

`ros2_basics_py/ros2_basics_py/add_two_ints_client.py`:

```python
"""Service client, usage: ros2 run ros2_basics_py add_two_ints_client 2 3."""
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from seame_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client_py')
        self._client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for service /add_two_ints...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        return self._client.call_async(request)


def main(args=None):
    argv = remove_ros_args(args if args is not None else sys.argv)
    if len(argv) != 3:
        print('usage: add_two_ints_client A B')
        return 1
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    future = node.send_request(int(argv[1]), int(argv[2]))
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info(f'Result: {future.result().sum}')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

Note `call_async` + `spin_until_future_complete`: the client never blocks inside a callback, which is what avoids deadlocks ([README.md §14](README.md#14-executors-callbacks-and-threading)).

### 8.2 C++

`ros2_basics_cpp/src/add_two_ints_server.cpp`:

```cpp
// Service server: returns a + b.
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "seame_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = seame_interfaces::srv::AddTwoInts;

class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer()
  : Node("add_two_ints_server_cpp")
  {
    srv_ = create_service<AddTwoInts>(
      "add_two_ints",
      [this](const std::shared_ptr<AddTwoInts::Request> request,
      std::shared_ptr<AddTwoInts::Response> response) {
        response->sum = request->a + request->b;
        RCLCPP_INFO(
          get_logger(), "%" PRId64 " + %" PRId64 " = %" PRId64,
          request->a, request->b, response->sum);
      });
  }

private:
  rclcpp::Service<AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddTwoIntsServer>());
  rclcpp::shutdown();
  return 0;
}
```

`ros2_basics_cpp/src/add_two_ints_client.cpp`:

```cpp
// Service client: ros2 run ros2_basics_cpp add_two_ints_client 2 3
#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "seame_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = seame_interfaces::srv::AddTwoInts;
using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  const std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  if (args.size() != 3) {
    std::fprintf(stderr, "usage: add_two_ints_client A B\n");
    rclcpp::shutdown();
    return 1;
  }

  auto node = rclcpp::Node::make_shared("add_two_ints_client_cpp");
  auto client = node->create_client<AddTwoInts>("add_two_ints");

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = std::atoll(args[1].c_str());
  request->b = std::atoll(args[2].c_str());

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for the service");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service /add_two_ints...");
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Result: %" PRId64, future.get()->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "service call failed");
  }

  rclcpp::shutdown();
  return 0;
}
```

---

## 9. Action server and client

Action `fibonacci` of type `seame_interfaces/action/Fibonacci`: computes the sequence one number per second, publishes feedback, can be canceled, rejects goals that are too large.

The examples are in Python. For C++ (`rclcpp_action`), follow the official tutorial "Writing an action server and client (C++)" at <https://docs.ros.org/en/lyrical/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html> using the same `seame_interfaces/action/Fibonacci` type; it is good practice for whoever implements the arm's motion action later.

> Use `get_logger().warning(...)`, not `.warn(...)`: older tutorials use `.warn`, which no longer exists in Lyrical (`AttributeError: 'RcutilsLogger' object has no attribute 'warn'`) and crashes the node the first time that line runs.

`ros2_basics_py/ros2_basics_py/fibonacci_server.py`:

```python
"""Action server: computes a Fibonacci sequence with feedback and cancellation."""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from seame_interfaces.action import Fibonacci

MAX_ORDER = 50


class FibonacciServer(Node):

    def __init__(self):
        super().__init__('fibonacci_server_py')
        self._server = ActionServer(
            self, Fibonacci, 'fibonacci',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def _on_goal(self, goal_request):
        if not 1 <= goal_request.order <= MAX_ORDER:
            self.get_logger().warning(f'Rejecting goal: order={goal_request.order}')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepting goal: order={goal_request.order}')
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        feedback = Fibonacci.Feedback()
        feedback.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result(sequence=feedback.partial_sequence)

            feedback.partial_sequence.append(
                feedback.partial_sequence[i] + feedback.partial_sequence[i - 1])
            self.get_logger().info(f'Feedback: {list(feedback.partial_sequence)}')
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)

        goal_handle.succeed()
        return Fibonacci.Result(sequence=feedback.partial_sequence)


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciServer()
    try:
        # Multi-threaded, so cancel requests are processed while a goal executes.
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

`ros2_basics_py/ros2_basics_py/fibonacci_client.py`:

```python
"""Action client, usage: ros2 run ros2_basics_py fibonacci_client 10."""
import sys

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from seame_interfaces.action import Fibonacci


class FibonacciClient(Node):

    def __init__(self):
        super().__init__('fibonacci_client_py')
        self._client = ActionClient(self, Fibonacci, 'fibonacci')
        self.done = False

    def send_goal(self, order):
        self.get_logger().info('waiting for action server /fibonacci...')
        self._client.wait_for_server()
        goal = Fibonacci.Goal()
        goal.order = order
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            self.done = True
            return
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {list(feedback_msg.feedback.partial_sequence)}')

    def _on_result(self, future):
        response = future.result()
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Result: {list(response.result.sequence)}')
        else:
            self.get_logger().warning(f'Goal finished with status {response.status}')
        self.done = True


def main(args=None):
    argv = remove_ros_args(args if args is not None else sys.argv)
    order = int(argv[1]) if len(argv) > 1 else 10
    rclpy.init(args=args)
    node = FibonacciClient()
    node.send_goal(order)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

---

## 10. Lifecycle node

A talker that only publishes while **Active**. It creates its publisher and timer in `configure`, starts publishing on `activate`, stops on `deactivate`, and releases everything on `cleanup`/`shutdown`.

### 10.1 Python

`ros2_basics_py/ros2_basics_py/managed_talker.py`:

```python
"""Lifecycle (managed) talker: publishes on lifecycle_chatter only while Active."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from ros2_basics_py.talker import CHATTER_QOS
from std_msgs.msg import String


class ManagedTalker(Node):

    def __init__(self):
        super().__init__('managed_talker_py')
        self._pub: Publisher | None = None
        self._timer = None
        self._count = 0
        self.get_logger().info('created (Unconfigured)')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._pub = self.create_lifecycle_publisher(String, 'lifecycle_chatter', CHATTER_QOS)
        self._timer = self.create_timer(1.0, self._on_timer)
        self.get_logger().info('on_configure: publisher and timer created')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate: publishing')
        return super().on_activate(state)       # activates the lifecycle publisher

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate: paused')
        return super().on_deactivate(state)     # deactivates the lifecycle publisher

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._release()
        self.get_logger().info('on_cleanup: resources released')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._release()
        self.get_logger().info('on_shutdown')
        return TransitionCallbackReturn.SUCCESS

    def _release(self):
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None

    def _on_timer(self):
        if self._pub is None or not self._pub.is_activated:
            return
        msg = String()
        msg.data = f'Lifecycle Python: {self._count}'
        self._count += 1
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManagedTalker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

### 10.2 C++

`ros2_basics_cpp/src/managed_talker.cpp`:

```cpp
// Lifecycle (managed) talker: publishes on lifecycle_chatter only while Active.
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ManagedTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  ManagedTalker()
  : LifecycleNode("managed_talker_cpp")
  {
    RCLCPP_INFO(get_logger(), "created (Unconfigured)");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    pub_ = create_publisher<std_msgs::msg::String>("lifecycle_chatter", qos);
    timer_ = create_wall_timer(1s, [this]() {on_timer();});
    RCLCPP_INFO(get_logger(), "on_configure: publisher and timer created");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    pub_->on_activate();
    RCLCPP_INFO(get_logger(), "on_activate: publishing");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "on_deactivate: paused");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup: resources released");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on_shutdown");
    return CallbackReturn::SUCCESS;
  }

private:
  void on_timer()
  {
    if (!pub_ || !pub_->is_activated()) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = "Lifecycle C++: " + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: \"%s\"", msg.data.c_str());
    pub_->publish(msg);
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagedTalker>();
  // A lifecycle node is not an rclcpp::Node: spin its base interface.
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
```

---

## 11. Build files (CMakeLists.txt, setup.py, package.xml)

### 11.1 `ros2_basics_cpp/CMakeLists.txt`

Replace the generated file:

```cmake
cmake_minimum_required(VERSION 3.10)
project(ros2_basics_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(seame_interfaces REQUIRED)

add_executable(talker src/talker.cpp)
target_link_libraries(talker rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server rclcpp::rclcpp ${seame_interfaces_TARGETS})

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client rclcpp::rclcpp ${seame_interfaces_TARGETS})

add_executable(managed_talker src/managed_talker.cpp)
target_link_libraries(managed_talker
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle ${std_msgs_TARGETS})

install(TARGETS
  talker listener add_two_ints_server add_two_ints_client managed_talker
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

Every `.cpp` file must start with a license header, or `colcon test` fails the `copyright` and `cpplint` linters. The examples above leave it out to stay short; the files in `ros2_ws/src` have it:

```cpp
// Copyright 2026 SEA:ME Team 2
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ... (the rest of the standard Apache-2.0 header)
```

cpplint also rejects `long long`: print `int64` fields with `PRId64` from `<cinttypes>`, as the service examples do.

Executables must be installed to `lib/<package>` or `ros2 run` will not find them. `ament_target_dependencies` (seen in older tutorials) is deprecated; use `target_link_libraries` with the exported targets as above.

### 11.2 `ros2_basics_cpp/package.xml`

`ros2 pkg create --dependencies ...` already added:

```xml
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>std_msgs</depend>
  <depend>seame_interfaces</depend>
```

### 11.3 `ros2_basics_py/setup.py`

Edit the generated file: add the launch files to `data_files` and every node to `entry_points`:

```python
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ros2_basics_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SEA:ME Team 2',
    maintainer_email='team2@example.com',
    description='ROS 2 basics in Python: topics with QoS, service, action, lifecycle',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = ros2_basics_py.talker:main',
            'listener = ros2_basics_py.listener:main',
            'add_two_ints_server = ros2_basics_py.add_two_ints_server:main',
            'add_two_ints_client = ros2_basics_py.add_two_ints_client:main',
            'fibonacci_server = ros2_basics_py.fibonacci_server:main',
            'fibonacci_client = ros2_basics_py.fibonacci_client:main',
            'managed_talker = ros2_basics_py.managed_talker:main',
        ],
    },
)
```

Replace `maintainer_email` with a real address. Every new Python node needs a line in `console_scripts`, then a rebuild. Keep the generated `extras_require={'test': ['pytest']}`: without it `colcon test` runs no tests for this package (`NO TESTS RAN`), so the flake8/pep257/mypy linters are silently skipped.

### 11.4 `ros2_basics_py/package.xml`

In addition to the generated `<depend>` lines, add the runtime dependencies of the launch file:

```xml
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>ros2_basics_cpp</exec_depend>
```

### 11.5 Final layout

```
ros2_ws/src/
├── seame_interfaces/
│   ├── CMakeLists.txt
│   ├── LICENSE                    (generated by ros2 pkg create --license)
│   ├── package.xml
│   ├── srv/AddTwoInts.srv
│   └── action/Fibonacci.action
├── ros2_basics_cpp/
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── package.xml
│   └── src/                       (every .cpp starts with the Apache-2.0 header, see 11.1)
│       ├── talker.cpp
│       ├── listener.cpp
│       ├── add_two_ints_server.cpp
│       ├── add_two_ints_client.cpp
│       └── managed_talker.cpp
└── ros2_basics_py/
    ├── LICENSE
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/ros2_basics_py
    ├── launch/basics.launch.py
    ├── test/                      (generated lint tests: copyright, flake8, mypy, pep257, xmllint)
    └── ros2_basics_py/
        ├── __init__.py
        ├── py.typed               (generated in Lyrical; marks the package as typed for mypy)
        ├── talker.py
        ├── listener.py
        ├── add_two_ints_server.py
        ├── add_two_ints_client.py
        ├── fibonacci_server.py
        ├── fibonacci_client.py
        └── managed_talker.py
```

The `include/ros2_basics_cpp/` folder that `ros2 pkg create` generates can be deleted: these nodes have no headers. A finished, tested copy of this layout is committed in `ros2_ws/src`; compare yours with it.

---

## 12. Build and run everything

### 12.1 Build

```bash
container$ cd /ws
container$ rosdep install --from-paths src --ignore-src -y   # or, from the host: make ws-deps
container$ colcon build                                      # or, from the host: make ws-build
container$ source install/setup.bash
```

- **Always build from the workspace root** (`/ws`), never from inside `src/`.
- **After every build, source `install/setup.bash`** in each terminal where you run nodes (new `make shell` terminals do it automatically if `install/` exists).
- On the Jetson: `colcon build --parallel-workers 2` (4 GB RAM).
- Build one package and the ones it depends on: `colcon build --packages-up-to ros2_basics_py`.
- Our images use `--symlink-install` by default, so editing a `.py` file does **not** need a rebuild. Adding a new node (new `entry_points` line) does.
- If the build state gets confused: `rm -rf build install log` (or `make ws-clean` from the host) and build again.

Check what got installed:

```bash
container$ ros2 pkg executables ros2_basics_cpp
container$ ros2 pkg executables ros2_basics_py
```

### 12.2 Run the topic examples

Each command in its own terminal (`make shell`):

```bash
container$ ros2 run ros2_basics_cpp talker
container$ ros2 run ros2_basics_py listener        # C++ → Python
```

Try every combination (Python talker → C++ listener, two listeners at once, two talkers). Change the rate without recompiling:

```bash
container$ ros2 run ros2_basics_cpp talker --ros-args -p publish_rate_hz:=5.0
```

### 12.3 Run the service examples

```bash
container$ ros2 run ros2_basics_cpp add_two_ints_server
container$ ros2 run ros2_basics_py add_two_ints_client 2 3        # Result: 5
container$ ros2 run ros2_basics_cpp add_two_ints_client 40 2      # Result: 42
container$ ros2 service call /add_two_ints seame_interfaces/srv/AddTwoInts "{a: 7, b: 8}"
```

### 12.4 Run the action examples

```bash
container$ ros2 run ros2_basics_py fibonacci_server
container$ ros2 run ros2_basics_py fibonacci_client 8             # feedback every second, then the result
container$ ros2 run ros2_basics_py fibonacci_client 100           # rejected (order > 50)
container$ ros2 action send_goal --feedback /fibonacci seame_interfaces/action/Fibonacci "{order: 5}"
```

To test **cancellation**, start a long goal with the CLI (`{order: 30}`) and press `Ctrl+C` in that terminal: `ros2 action send_goal` cancels the goal, and the server logs `Goal canceled`.

### 12.5 Run the lifecycle example

```bash
# terminal 1
container$ ros2 run ros2_basics_cpp managed_talker               # or ros2_basics_py managed_talker

# terminal 2: watch the output topic
container$ ros2 topic echo /lifecycle_chatter

# terminal 3: drive the state machine
container$ ros2 lifecycle nodes                                  # /managed_talker_cpp
container$ ros2 lifecycle get /managed_talker_cpp                # unconfigured [1]
container$ ros2 lifecycle list /managed_talker_cpp               # available transitions
container$ ros2 lifecycle set /managed_talker_cpp configure      # → inactive: nothing published yet
container$ ros2 lifecycle set /managed_talker_cpp activate       # → active: terminal 2 starts printing
container$ ros2 lifecycle set /managed_talker_cpp deactivate     # → inactive: printing stops
container$ ros2 lifecycle set /managed_talker_cpp activate       # → active again
container$ ros2 lifecycle set /managed_talker_cpp deactivate
container$ ros2 lifecycle set /managed_talker_cpp cleanup        # → unconfigured
container$ ros2 lifecycle set /managed_talker_cpp shutdown       # → finalized
```

Things to observe:

- Before `configure`, `ros2 topic list` does not show `/lifecycle_chatter`.
- After `configure` the topic exists but no data flows.
- An invalid transition (e.g. `activate` from `unconfigured`) is refused.
- Watch the transitions being announced: `ros2 topic echo /managed_talker_cpp/transition_event`.

### 12.6 Run the tests (linters)

`ros2 pkg create` adds the standard ament linters to every package, and they are what reviewers and CI check before a merge. Run them after every change:

```bash
host$ make ws-build && make ws-test                          # colcon test for every package
host$ make exec CMD='colcon test-result --all'               # one line per test file
host$ make exec CMD='colcon test-result --verbose'           # only the failures, with the message
```

Inside the container: `colcon test --packages-select ros2_basics_py` tests one package; the full pytest output is in `/ws/log/latest_test/<package>/stdout_stderr.log`.

The expected result for the three example packages:

```
Summary: 33 tests, 0 errors, 0 failures, 6 skipped
```

The 6 skipped are normal: `cppcheck` disables itself with the cppcheck version in the image, and the generated Python `test_copyright.py` is marked as skipped.

What each linter checks:

| Package | Linter | Checks |
|---|---|---|
| C++ | `copyright` | Every source file starts with a license header |
| C++ | `cpplint` | Google C++ style (header, `long` types, includes...) |
| C++ | `uncrustify` | Formatting; `ament_uncrustify --reformat src/` fixes it for you |
| C++ | `lint_cmake`, `xmllint` | `CMakeLists.txt` and `package.xml` |
| Python | `flake8` | PEP 8 and import order: standard library, then third party (incl. `rclpy`, your own packages, `std_msgs`) sorted alphabetically |
| Python | `pep257` | Docstrings: one-line docstrings end with a period |
| Python | `mypy`, `xmllint` | Types; `package.xml` |

If a package reports `NO TESTS RAN`, its `setup.py` lost the `extras_require` test entry (see 11.3). The errors found while testing this guide and their fixes are listed in [TROUBLESHOOTING.md → Tests and linters](TROUBLESHOOTING.md#tests-and-linters-colcon-test).

---

## 13. Launch file

Starts the C++ talker, the Python listener, the service server and the action server together.

`ros2_basics_py/launch/basics.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rate = LaunchConfiguration('publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument('publish_rate_hz', default_value='1.0',
                              description='Talker publish rate in Hz'),

        Node(package='ros2_basics_cpp', executable='talker', name='talker_cpp',
             parameters=[{'publish_rate_hz': rate}], output='screen'),
        Node(package='ros2_basics_py', executable='listener', name='listener_py',
             output='screen'),
        Node(package='ros2_basics_cpp', executable='add_two_ints_server',
             name='add_two_ints_server_cpp', output='screen'),
        Node(package='ros2_basics_py', executable='fibonacci_server',
             name='fibonacci_server_py', output='screen'),
    ])
```

```bash
container$ mkdir -p /ws/src/ros2_basics_py/launch     # then create the file above
container$ cd /ws && colcon build --packages-select ros2_basics_py && source install/setup.bash
container$ ros2 launch ros2_basics_py basics.launch.py
container$ ros2 launch ros2_basics_py basics.launch.py publish_rate_hz:=4.0
container$ ros2 launch ros2_basics_py basics.launch.py --show-args
```

`Ctrl+C` stops all nodes started by the launch file.

---

## 14. Parameters

The talkers declare `publish_rate_hz`. Practice the parameter tools:

```bash
container$ ros2 run ros2_basics_py talker
container$ ros2 param list /talker_py
container$ ros2 param get /talker_py publish_rate_hz
container$ ros2 param describe /talker_py publish_rate_hz
container$ ros2 param dump /talker_py > /ws/talker_params.yaml
container$ ros2 run ros2_basics_py talker --ros-args --params-file /ws/talker_params.yaml
```

`ros2 param set /talker_py publish_rate_hz 5.0` changes the stored value, but our talker reads it only at startup, so the rate does not change. To react at runtime, register a callback with `add_on_set_parameters_callback` (same name in Python and C++) and recreate the timer there. Good exercise.

---

## 15. Inspect the system: ros2 CLI and rqt_graph

Start the launch file from section 13, then explore in another terminal.

### Nodes

```bash
container$ ros2 node list
container$ ros2 node info /listener_py           # its subscriptions, publishers, services, actions
```

### Topics

```bash
container$ ros2 topic list -t                    # names and types
container$ ros2 topic info -v /chatter           # publishers, subscribers and their QoS
container$ ros2 topic echo /chatter              # print messages
container$ ros2 topic echo /chatter --once       # print one and exit
container$ ros2 topic hz /chatter                # publish rate
container$ ros2 topic bw /chatter                # bandwidth
container$ ros2 topic pub --once /chatter std_msgs/msg/String "{data: 'from CLI'}"
```

### Services and actions

```bash
container$ ros2 service list -t
container$ ros2 service type /add_two_ints
container$ ros2 action list -t
container$ ros2 action info /fibonacci           # its clients and servers
```

### Interfaces

```bash
container$ ros2 interface show std_msgs/msg/String
container$ ros2 interface package seame_interfaces
```

### rqt_graph (PC only)

```bash
container$ ros2 run rqt_graph rqt_graph       # or: rqt --standalone rqt_graph
```

- Press the refresh button after starting new nodes.
- Choose **"Nodes/Topics (all)"** in the top-left menu to see every topic.
- Uncheck **"Hide: Dead sinks / Leaf topics"** to see topics nobody subscribes to.
- Hidden by default: the `/rosout` logging topic and the CLI's own nodes.

Save a screenshot of the graph for the issue: **File → Save as** (or a normal screenshot). A good place for it in the repo is `ROS2/images/`.

### rqt (other plugins)

```bash
container$ rqt
```

Useful plugins: *Topics → Topic Monitor*, *Services → Service Caller*, *Configuration → Parameter Reconfigure*, *Logging → Console*, *Visualization → Plot*.

### Logs

Every node writes to the console and to `~/.ros/log/`. Change the verbosity per node:

```bash
container$ ros2 run ros2_basics_cpp talker --ros-args --log-level debug
```

---

## 16. Run across two machines (PC and Jetson)

Requirements: both on the same network (same subnet), same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` in `docker/.env`, both containers up.

**Step 1. Basic connectivity:**

```bash
host$ ping <jetson-ip>
```

**Step 2. Project test** (from `docker/` on each machine):

```bash
nano$ make talker
host$ make listener          # PASS if it hears the Jetson's talker
```

Then the other way round. Or the simplest test: `make ping-echo` on one and `make ping-pub` on the other.

**Step 3. Our nodes across machines** (after building the workspace on both; `git pull` on the Nano so it has the same code):

```bash
nano$ make shell
container$ ros2 run ros2_basics_cpp talker             # on the Jetson

host$ make shell
container$ ros2 run ros2_basics_py listener            # on the PC
container$ ros2 run rqt_graph rqt_graph                 # shows the Jetson's node too
```

If nothing arrives: check the firewall (`sudo ufw status`; allow UDP between the machines), multicast (set `ROS_STATIC_PEERS` with the other machine's IP in `docker/compose.yml`), and the domain/RMW values. More in [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

---

## 17. Record and replay with rosbag2

```bash
container$ ros2 bag record -s mcap -o /ws/bags/chatter_test /chatter     # Ctrl+C to stop
container$ ros2 bag info /ws/bags/chatter_test
container$ ros2 bag play /ws/bags/chatter_test                          # republishes the messages
container$ ros2 bag record -s mcap -a -o /ws/bags/everything            # all topics (careful with images)
```

`ros2_ws/bags` is ignored by git; bags are not committed. On the PC, open bags in PlotJuggler (`plotjuggler4`) to plot numeric fields.

---

## 18. Push the examples to the repo

The packages live in `ros2_ws/src/`, which **is** tracked by git. Generated folders (`ros2_ws/build`, `install`, `log`, `bags`) are ignored by the root `.gitignore`.

**Step 1. Check that everything builds and the linters pass:**

```bash
host$ make ws-build
host$ make ws-test
```

**Step 2. Commit on the issue's branch.** The commit hook (`.githooks/commit-msg`) requires the format `<type>(<scope>): <description>` with type one of `fix feat test style docs build ops chore perf refactor`:

```bash
host$ git checkout 18-ros-2-basics-topics-services-actions-lifecycle
host$ git add ros2_ws/src
host$ git status                           # make sure no build/ install/ log/ folders are staged
host$ git commit -m "feat(ros2): add ROS 2 basics examples (QoS pub/sub, service, action, lifecycle)"
host$ git push
```

**Step 3. Open the pull request** and tick the issue's checklist:

| Issue #18 item | Where it is done |
|---|---|
| Publisher and subscriber with explicit QoS (C++ and Python) | Section 7: `talker`/`listener` in `ros2_basics_cpp` and `ros2_basics_py` |
| Service and action server and client | Sections 8 and 9 |
| Lifecycle node with configure, activate, deactivate transitions | Section 10, exercised in 12.5 |
| Inspect with ros2 topic, node and rqt_graph; push examples to the repo | Sections 15 and 18 |

The issue says **every member** builds these patterns once: each member can follow this guide in their own copy and compare their result with the committed examples.
