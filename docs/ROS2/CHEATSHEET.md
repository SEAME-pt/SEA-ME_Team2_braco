# ROS 2 Cheat Sheet

Quick reference for daily work. Explanations are in [README.md](README.md), step-by-step usage in [INSTRUCTIONS.md](INSTRUCTIONS.md).

## Project containers (run from `docker/`)

| Command | What it does |
|---|---|
| `make env ROLE=pc` / `make env ROLE=jetson` | Create `.env` for this machine (once) |
| `make doctor` | Check Docker, Compose, `.env`, folders, display/serial |
| `make up` / `make down` | Start / stop the container |
| `make shell` | Shell inside the container (ROS + `/ws` sourced) |
| `make exec CMD='ros2 node list'` | Run one command in the running container |
| `make smoke` | Check the install |
| `make talker-listener` | C++ talker + Python listener self-test (PASS/FAIL) |
| `make talker` / `make listener` | Two-machine test (one on each machine) |
| `make ping-pub` / `make ping-echo` | Simplest cross-machine test |
| `make ws-deps` / `make ws-build` / `make ws-test` / `make ws-clean` | rosdep / colcon build / colcon test / delete build output |
| `make` | List all targets |

## Environment

```bash
source /opt/ros/lyrical/setup.bash      # underlay (automatic in the containers)
source /ws/install/setup.bash           # your workspace, after every build
printenv | grep -E 'ROS|RMW'            # ROS_DISTRO, ROS_DOMAIN_ID, RMW_IMPLEMENTATION...
ros2 doctor --report                    # environment + network report
```

## Workspace and packages

```bash
ros2 pkg create <name> --build-type ament_cmake  --license Apache-2.0 --dependencies rclcpp std_msgs
ros2 pkg create <name> --build-type ament_python --license Apache-2.0 --dependencies rclpy std_msgs
ros2 pkg create <name> --build-type ament_cmake  --node-name my_node     # with a hello-world node

rosdep install --from-paths src --ignore-src -y      # install dependencies (from /ws)
colcon build                                         # build all (from /ws, never from src/)
colcon build --packages-select <pkg>                 # only this package
colcon build --packages-up-to <pkg>                  # this package and its dependencies
colcon build --parallel-workers 2                    # on the Jetson
colcon test && colcon test-result --verbose          # tests + linters
rm -rf build install log                             # clean

ros2 pkg list | grep <name>
ros2 pkg executables <pkg>
ros2 pkg prefix <pkg>
```

## Running

```bash
ros2 run <pkg> <executable>
ros2 run <pkg> <exe> --ros-args -p name:=value                    # parameter
ros2 run <pkg> <exe> --ros-args --params-file params.yaml
ros2 run <pkg> <exe> --ros-args -r old_topic:=new_topic           # remap
ros2 run <pkg> <exe> --ros-args -r __node:=new_name -r __ns:=/ns  # rename / namespace
ros2 run <pkg> <exe> --ros-args --log-level debug
ros2 launch <pkg> <file>.launch.py [arg:=value]
ros2 launch <pkg> <file>.launch.py --show-args
```

## Nodes

```bash
ros2 node list
ros2 node info /<node>
```

## Topics

```bash
ros2 topic list -t
ros2 topic info -v /<topic>                       # endpoints + QoS
ros2 topic echo /<topic> [--once] [--qos-reliability best_effort]
ros2 topic hz /<topic>
ros2 topic bw /<topic>
ros2 topic pub /<topic> <type> "<yaml>"           # repeats at 1 Hz
ros2 topic pub --once /<topic> <type> "<yaml>"
ros2 topic pub -r 10 /<topic> <type> "<yaml>"     # 10 Hz
ros2 topic pub --qos-reliability best_effort --qos-durability transient_local ...
```

Example: `ros2 topic pub --once /chatter std_msgs/msg/String "{data: 'hi'}"`

## Services

```bash
ros2 service list -t
ros2 service type /<service>
ros2 service find <type>
ros2 service call /<service> <type> "<yaml>"
```

Example: `ros2 service call /add_two_ints seame_interfaces/srv/AddTwoInts "{a: 2, b: 3}"`

## Actions

```bash
ros2 action list -t
ros2 action info /<action>
ros2 action send_goal [--feedback] /<action> <type> "<yaml>"     # Ctrl+C cancels the goal
```

Example: `ros2 action send_goal --feedback /fibonacci seame_interfaces/action/Fibonacci "{order: 5}"`

## Lifecycle

```bash
ros2 lifecycle nodes
ros2 lifecycle get /<node>
ros2 lifecycle list /<node>                       # transitions available now
ros2 lifecycle set /<node> configure | activate | deactivate | cleanup | shutdown
ros2 topic echo /<node>/transition_event
```

```
Unconfigured ─configure→ Inactive ─activate→ Active
Unconfigured ←cleanup─── Inactive ←deactivate─ Active
any primary state ─shutdown→ Finalized
```

## Parameters

```bash
ros2 param list /<node>
ros2 param get /<node> <name>
ros2 param set /<node> <name> <value>
ros2 param describe /<node> <name>
ros2 param dump /<node> > params.yaml
ros2 param load /<node> params.yaml
```

## Interfaces

```bash
ros2 interface list
ros2 interface package <pkg>
ros2 interface show <pkg>/msg/<Type>
ros2 interface proto <pkg>/msg/<Type>             # YAML template to fill in for pub/call
```

## Bags

```bash
ros2 bag record -s mcap -o /ws/bags/<name> /topic1 /topic2
ros2 bag record -s mcap -a -o /ws/bags/<name>     # all topics
ros2 bag info /ws/bags/<name>
ros2 bag play /ws/bags/<name> [--rate 0.5] [--loop]
```

## GUI tools (PC image only)

```bash
ros2 run rqt_graph rqt_graph   # node/topic graph (no bare `rqt_graph` command in Lyrical)
rqt                # plugin GUI (topic monitor, service caller, console, plot...)
rviz2              # 3D visualisation
plotjuggler4       # plots, live or from bags
ros2 run tf2_tools view_frames          # PDF of the tf tree
ros2 run tf2_ros tf2_echo <parent> <child>
```

## Daemon and discovery

```bash
ros2 daemon stop; ros2 daemon start       # stale `ros2 node list` / `topic list`
ros2 multicast receive                    # on machine A
ros2 multicast send                       # on machine B: A must print the packet
```

## QoS at a glance

| Want | Python | C++ |
|---|---|---|
| Reliable, depth 10 | `QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)` | `rclcpp::QoS(10).reliable()` |
| Best effort (sensors) | `qos_profile_sensor_data` | `rclcpp::SensorDataQoS()` |
| Latched (late joiners get the last value) | `QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)` | `rclcpp::QoS(1).transient_local()` |

Publisher `BEST_EFFORT` + subscriber `RELIABLE` → **no connection**. Publisher `VOLATILE` + subscriber `TRANSIENT_LOCAL` → **no connection**.

## Minimal node templates

**Python**

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.create_timer(1.0, lambda: self.get_logger().info('tick'))

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

**C++**

```cpp
#include <chrono>
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    timer_ = create_wall_timer(1s, [this]() { RCLCPP_INFO(get_logger(), "tick"); });
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
}
```
