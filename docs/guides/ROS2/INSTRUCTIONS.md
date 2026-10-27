# ROS 2: Hands-on Guide

From zero to a working ROS 2 workspace with the core patterns: publisher/subscriber with QoS (C++ and Python), service, action, lifecycle node, launch file, parameters, and inspection tools.

- Concepts: [README.md](README.md) · Commands: [CHEATSHEET.md](CHEATSHEET.md) · Problems: [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **Safety:** nothing here moves the arm. Before running anything on the hardware, read [safety.md](../safety.md).

Conventions: `host$` = your machine, repo root. `nano$` = the Jetson, repo root. `container$` = inside `make shell`. ROS distro: `lyrical`.

---

## Contents

1. [Set up](#1-set-up)
2. [First steps with turtlesim](#2-first-steps-with-turtlesim)
3. [Create the packages](#3-create-the-packages)
4. [Custom interfaces](#4-custom-interfaces)
5. [Publisher and subscriber with QoS](#5-publisher-and-subscriber-with-qos)
6. [Service server and client](#6-service-server-and-client)
7. [Action server and client](#7-action-server-and-client)
8. [Lifecycle node](#8-lifecycle-node)
9. [Build files](#9-build-files)
10. [Build and run](#10-build-and-run)
11. [Launch file](#11-launch-file)
12. [Parameters](#12-parameters)
13. [Inspect the system](#13-inspect-the-system)
14. [PC and Jetson together](#14-pc-and-jetson-together)
15. [Record and replay (rosbag2)](#15-record-and-replay-rosbag2)
16. [Clean up](#16-clean-up)

---

## 1. Set up

ROS 2 runs in Docker on both machines (why: [ADR-0001](../../decisions/ADR-0001-ros2-in-docker-on-jetson-nano.md); details: [docker/README.md](../../../docker/README.md)). Do not install ROS natively on the Jetson: Lyrical does not exist for Ubuntu 20.04.

### 1.1 PC

**1. Install** [Docker Engine](https://docs.docker.com/engine/install/) with Compose v2.20+, then:

```bash
host$ sudo usermod -aG docker $USER          # log out and back in
host$ sudo apt-get install -y make git x11-xserver-utils
host$ docker compose version                  # v2.20 or newer
```

**2. Clone and prepare:**

```bash
host$ git clone git@github.com:SEAME-pt/SEA-ME_Team2_braco.git
host$ cd SEA-ME_Team2_braco
host$ make setup              # answer "dev"; creates docker/.env, venv, git hooks, then runs make doctor
```

Fix any `[FAIL]` from `make doctor` before continuing.

**3. Start and verify** (the first run builds the image: tens of minutes):

```bash
host$ make up                 # starts the `dev` container (SIM=1 also starts the Gazebo simulation)
host$ make net-test           # C++ talker + Python listener: PASS
host$ make shell              # (pc-dev) dev@host:/ws$
```

`/ws` in the container is `ros2_ws/` in the repo: files in `/ws/src` are your files on the host. Edit on the host, build and run in the container.

> The PC's containers use **domain 77** (`PC_ROS_DOMAIN_ID`), the real arm **domain 42** (`ROS_DOMAIN_ID`). To reach the arm from the PC: `ROS_DOMAIN_ID=42 ros2 ...`.

### 1.2 Jetson Nano

```bash
nano$ sudo systemctl disable --now nvgetty      # once: frees the arm's UART /dev/ttyTHS1; log out and in
nano$ git clone git@github.com:SEAME-pt/SEA-ME_Team2_braco.git
nano$ cd SEA-ME_Team2_braco
nano$ make setup                                # answer "jetson"
```

Get the image, either:

```bash
nano$ make image                                # build on the Nano (slow)
host$ make jetson-image NANO=user@<nano-ip>     # or: build on the PC and copy it (faster)
```

Then:

```bash
nano$ make up                                   # builds the workspace and starts `real` (the arm)
nano$ make net-test
```

The Jetson has no GUI tools: inspect its nodes from the PC ([section 14](#14-pc-and-jetson-together)).

### 1.3 Daily commands

| What | Command |
|---|---|
| Start / stop | `make up` / `make down` |
| Shell (ROS sourced; one per terminal) | `make shell` |
| One command | `make run CMD='ros2 topic list'` |
| Build / test the workspace | `make build` / `make test` |
| Install dependencies | `make run CMD='rosdep install --from-paths src --ignore-src -y'` |
| All commands | `make` |

Packages installed with `apt` inside a container are lost when it is recreated. Permanent ones go in `docker/Dockerfile`.

---

## 2. First steps with turtlesim

Learn the CLI with something visible (PC only). Each command in its own `make shell`:

```bash
container$ ros2 run turtlesim turtlesim_node          # a window opens
container$ ros2 run turtlesim turtle_teleop_key       # drive with the arrow keys

container$ ros2 node list
container$ ros2 topic list -t
container$ ros2 topic echo /turtle1/pose
container$ ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist \
           "{linear: {x: 2.0}, angular: {z: 1.8}}"     # drives in circles
container$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'turtle2'}"
container$ ros2 action send_goal --feedback /turtle1/rotate_absolute \
           turtlesim/action/RotateAbsolute "{theta: 1.57}"
container$ ros2 param set /turtlesim background_r 150
container$ ros2 run rqt_graph rqt_graph
```

No window? See [TROUBLESHOOTING.md → GUI tools](TROUBLESHOOTING.md#gui-tools-rqt-rqt_graph-rviz2-gazebo).

---

## 3. Create the packages

Three learning packages. **Do not commit them** ([section 16](#16-clean-up)).

```
ros2_ws/src/
├── ros2_basics_interfaces/   ament_cmake    service and action definitions
├── ros2_basics_cpp/          ament_cmake    C++: talker, listener, service, lifecycle
└── ros2_basics_py/           ament_python   Python: the same, plus action and launch file
```

```bash
container$ cd /ws/src
container$ M="--maintainer-name Team2 --maintainer-email team2@example.com --license Apache-2.0"

container$ ros2 pkg create ros2_basics_interfaces --build-type ament_cmake $M
container$ ros2 pkg create ros2_basics_cpp --build-type ament_cmake $M \
    --dependencies rclcpp rclcpp_lifecycle std_msgs ros2_basics_interfaces
container$ ros2 pkg create ros2_basics_py --build-type ament_python $M \
    --dependencies rclpy std_msgs action_msgs ros2_basics_interfaces

container$ rm -rf ros2_basics_interfaces/include ros2_basics_interfaces/src ros2_basics_cpp/include
```

Always pass `--maintainer-email`: without it, a derived invalid address fails the build (`Invalid email`).

---

## 4. Custom interfaces

```bash
container$ mkdir -p /ws/src/ros2_basics_interfaces/srv /ws/src/ros2_basics_interfaces/action
```

`ros2_basics_interfaces/srv/AddTwoInts.srv`:

```text
int64 a
int64 b
---
int64 sum
```

`ros2_basics_interfaces/action/Fibonacci.action`:

```text
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

`ros2_basics_interfaces/CMakeLists.txt` (replace the generated one):

```cmake
cmake_minimum_required(VERSION 3.10)
project(ros2_basics_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  "action/Fibonacci.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

`ros2_basics_interfaces/package.xml`: add after `<buildtool_depend>ament_cmake</buildtool_depend>`:

```xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>action_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

Build and check:

```bash
container$ cd /ws && colcon build --packages-select ros2_basics_interfaces && source install/setup.bash
container$ ros2 interface show ros2_basics_interfaces/srv/AddTwoInts
container$ ros2 interface show ros2_basics_interfaces/action/Fibonacci
```

---

## 5. Publisher and subscriber with QoS

Topic `chatter`, type `std_msgs/msg/String`, QoS `KEEP_LAST` 10, `RELIABLE`, `VOLATILE` ([README §8](README.md#8-quality-of-service-qos)). Same in both languages, so they interoperate.

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

---

## 6. Service server and client

Service `add_two_ints`, type `ros2_basics_interfaces/srv/AddTwoInts`.

`ros2_basics_py/ros2_basics_py/add_two_ints_server.py`:

```python
"""Service server: returns a + b."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ros2_basics_interfaces.srv import AddTwoInts


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

from ros2_basics_interfaces.srv import AddTwoInts


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

`call_async` + `spin_until_future_complete` never blocks inside a callback, which avoids deadlocks ([README §14](README.md#14-executors-callbacks-and-threading)).

`ros2_basics_cpp/src/add_two_ints_server.cpp`:

```cpp
// Service server: returns a + b.
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_basics_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = ros2_basics_interfaces::srv::AddTwoInts;

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
#include "ros2_basics_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = ros2_basics_interfaces::srv::AddTwoInts;
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

## 7. Action server and client

Action `fibonacci`: one number per second, with feedback and cancellation; rejects `order` outside 1–50. Python only; for C++ follow the [official tutorial](https://docs.ros.org/en/lyrical/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) with the same type.

> Use `get_logger().warning(...)`. `.warn(...)` no longer exists in Lyrical and crashes the node.

`ros2_basics_py/ros2_basics_py/fibonacci_server.py`:

```python
"""Action server: computes a Fibonacci sequence with feedback and cancellation."""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from ros2_basics_interfaces.action import Fibonacci

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

from ros2_basics_interfaces.action import Fibonacci


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

## 8. Lifecycle node

A talker that publishes on `lifecycle_chatter` only while **Active**: resources created on `configure`, publishing on `activate`, paused on `deactivate`, released on `cleanup`/`shutdown`.

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

## 9. Build files

### `ros2_basics_cpp/CMakeLists.txt` (replace the generated one)

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
find_package(ros2_basics_interfaces REQUIRED)

add_executable(talker src/talker.cpp)
target_link_libraries(talker rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server rclcpp::rclcpp ${ros2_basics_interfaces_TARGETS})

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client rclcpp::rclcpp ${ros2_basics_interfaces_TARGETS})

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

- Executables must install to `lib/<package>`, or `ros2 run` cannot find them.
- Use `target_link_libraries`; `ament_target_dependencies` (older tutorials) is deprecated.
- For `make test` to pass: start every `.cpp` with an Apache-2.0 license header (`// Copyright 2026 SEA:ME Team 2` ...), and print `int64` with `PRId64`, never `long long`.

### `ros2_basics_py/setup.py` (edit the generated one)

Add the launch files to `data_files` and every node to `console_scripts`:

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
    maintainer='Team2',
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

Keep `extras_require`: without it `colcon test` silently skips the Python linters (`NO TESTS RAN`).

### `ros2_basics_py/package.xml`

Add the launch file's runtime dependencies:

```xml
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>ros2_basics_cpp</exec_depend>
```

(`ros2_basics_cpp/package.xml` already has its dependencies from `ros2 pkg create`.)

---

## 10. Build and run

### Build

```bash
container$ cd /ws                     # always build from the workspace root
container$ colcon build               # or from the host: make build
container$ source install/setup.bash  # in every terminal, after every build
container$ ros2 pkg executables ros2_basics_cpp
container$ ros2 pkg executables ros2_basics_py
```

- `--symlink-install` is on by default: editing a `.py` needs no rebuild, adding a node does.
- One package and its dependencies: `colcon build --packages-up-to ros2_basics_py`.
- Confused build: `make clean`, then build again.

Each `ros2 run` below in its own `make shell`.

### Topics

```bash
container$ ros2 run ros2_basics_cpp talker
container$ ros2 run ros2_basics_py listener       # C++ → Python works
container$ ros2 run ros2_basics_cpp talker --ros-args -p publish_rate_hz:=5.0
```

Try other combinations (Python → C++, two listeners).

### QoS experiments

With a listener running:

```bash
container$ ros2 topic pub --qos-reliability best_effort /chatter std_msgs/msg/String "{data: 'lost'}"
container$ ros2 topic pub --qos-reliability reliable    /chatter std_msgs/msg/String "{data: 'received'}"
container$ ros2 topic info -v /chatter            # QoS of every endpoint
```

The `best_effort` message never arrives: the listener warns about incompatible QoS (`RELIABILITY`).

Durability: set the talker to `TRANSIENT_LOCAL` and start the listener 5 s later:

| Talker | Listener | Result |
|---|---|---|
| `TRANSIENT_LOCAL` | `VOLATILE` | Only new messages |
| `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL` | The last 10 missed messages first |
| `VOLATILE` | `TRANSIENT_LOCAL` | Incompatible: nothing |

### Services

```bash
container$ ros2 run ros2_basics_cpp add_two_ints_server
container$ ros2 run ros2_basics_py add_two_ints_client 2 3        # Result: 5
container$ ros2 run ros2_basics_cpp add_two_ints_client 40 2      # Result: 42
container$ ros2 service call /add_two_ints ros2_basics_interfaces/srv/AddTwoInts "{a: 7, b: 8}"
```

### Actions

```bash
container$ ros2 run ros2_basics_py fibonacci_server
container$ ros2 run ros2_basics_py fibonacci_client 8             # feedback every second, then result
container$ ros2 run ros2_basics_py fibonacci_client 100           # rejected
container$ ros2 action send_goal --feedback /fibonacci ros2_basics_interfaces/action/Fibonacci "{order: 30}"
```

Press `Ctrl+C` during the last one: the goal is canceled (`Goal canceled` on the server).

### Lifecycle

```bash
container$ ros2 run ros2_basics_cpp managed_talker
container$ ros2 topic echo /lifecycle_chatter

container$ ros2 lifecycle get /managed_talker_cpp                 # unconfigured
container$ ros2 lifecycle set /managed_talker_cpp configure       # topic exists, nothing published
container$ ros2 lifecycle set /managed_talker_cpp activate        # messages flow
container$ ros2 lifecycle set /managed_talker_cpp deactivate      # messages stop
container$ ros2 lifecycle set /managed_talker_cpp cleanup         # unconfigured
container$ ros2 lifecycle set /managed_talker_cpp shutdown        # finalized
```

`ros2 lifecycle list /managed_talker_cpp` shows the valid transitions; invalid ones are refused. `ros2 topic echo /managed_talker_cpp/transition_event` shows each change.

### Tests (linters)

```bash
host$ make build && make test                                   # all packages, with a summary
host$ make run CMD='colcon test-result --verbose'               # only the failures
```

A few skipped tests are normal. Failures and their fixes: [TROUBLESHOOTING.md → Tests and linters](TROUBLESHOOTING.md#tests-and-linters-colcon-test).

---

## 11. Launch file

Starts the C++ talker, Python listener, service server and action server together.

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
container$ cd /ws && colcon build --packages-select ros2_basics_py && source install/setup.bash
container$ ros2 launch ros2_basics_py basics.launch.py
container$ ros2 launch ros2_basics_py basics.launch.py publish_rate_hz:=4.0
container$ ros2 launch ros2_basics_py basics.launch.py --show-args
```

---

## 12. Parameters

```bash
container$ ros2 run ros2_basics_py talker
container$ ros2 param list /talker_py
container$ ros2 param get /talker_py publish_rate_hz
container$ ros2 param dump /talker_py > /ws/talker_params.yaml
container$ ros2 run ros2_basics_py talker --ros-args --params-file /ws/talker_params.yaml
```

`ros2 param set` changes the value, but the talker only reads it at startup. Exercise: react at runtime with `add_on_set_parameters_callback` and recreate the timer.

---

## 13. Inspect the system

Run the launch file (section 11), then:

```bash
container$ ros2 node list
container$ ros2 node info /listener_py
container$ ros2 topic list -t
container$ ros2 topic echo /chatter --once
container$ ros2 topic hz /chatter
container$ ros2 topic pub --once /chatter std_msgs/msg/String "{data: 'from CLI'}"
container$ ros2 service list -t
container$ ros2 action info /fibonacci
container$ ros2 interface package ros2_basics_interfaces
container$ ros2 run ros2_basics_cpp talker --ros-args --log-level debug
```

GUIs (PC only):

```bash
container$ ros2 run rqt_graph rqt_graph   # refresh after new nodes; "Nodes/Topics (all)" shows every topic
container$ rqt                            # Topic Monitor, Service Caller, Console, Plot...
```

---

## 14. PC and Jetson together

Requirements: same subnet, same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` in both `docker/.env`, both machines `make up`.

**1. Connectivity test:**

```bash
host$ ping <jetson-ip>
nano$ make net-talk
host$ make net-listen          # PASS
```

Then swap the roles.

**2. Your nodes across machines** (same code on both: `git pull`). The Jetson's `make build` only builds the arm packages, so build the example there by hand:

```bash
nano$ make shell
container$ colcon build --packages-up-to ros2_basics_cpp --parallel-workers 1 && source install/setup.bash
container$ ros2 run ros2_basics_cpp talker

host$ make shell
container$ ROS_DOMAIN_ID=42 ros2 run ros2_basics_py listener     # the Jetson's domain
container$ ROS_DOMAIN_ID=42 ros2 run rqt_graph rqt_graph
```

Nothing arrives? Check the firewall (UDP between the machines), the domain/RMW values, or set `ROS_STATIC_PEERS` in `docker/compose.yml`. See [TROUBLESHOOTING.md → Between machines](TROUBLESHOOTING.md#communication-between-machines-pc--jetson).

---

## 15. Record and replay (rosbag2)

```bash
container$ ros2 bag record -s mcap -o /ws/bags/chatter_test /chatter    # Ctrl+C to stop
container$ ros2 bag info /ws/bags/chatter_test
container$ ros2 bag play /ws/bags/chatter_test
```

`ros2_ws/bags` is not committed. On the PC, plot numeric fields with `plotjuggler4`.

---

## 16. Clean up

The tutorial packages are for learning only. When done:

```bash
host$ rm -rf ros2_ws/src/ros2_basics_interfaces ros2_ws/src/ros2_basics_cpp ros2_ws/src/ros2_basics_py
host$ make clean
```
