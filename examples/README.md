# examples

ROS 2 basics, written while learning (issue #18): topics with QoS, a service, an action and lifecycle nodes, in
C++ and Python. They are **not** part of the braco product, but they are built with it (`make build` adds this
folder to colcon's `--base-paths`), because the web page's *ROS 2 > Demos* tab drives them.

| Package | Content |
|---|---|
| `ros2_basics_interfaces` | `srv/AddTwoInts`, `action/Fibonacci` |
| `ros2_basics_cpp` | talker, listener, add_two_ints server/client, managed (lifecycle) talker |
| `ros2_basics_py` | the same in Python, plus the Fibonacci action server/client and `basics.launch.py` |

```bash
make demos             # the demos the Demos tab expects, inside the system container
ros2 launch ros2_basics_py basics.launch.py
ros2 service call /add_two_ints ros2_basics_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

The step-by-step guide that builds them is `docs/ROS2/INSTRUCTIONS.md`.
