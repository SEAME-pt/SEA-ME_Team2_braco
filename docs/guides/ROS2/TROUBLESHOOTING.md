# ROS 2 Troubleshooting

Common ROS 2 problems in this project, and how to fix them. Some problems have their own table next to the thing
that causes them:

| Your problem is about... | Look in |
|---|---|
| Building the Docker image | [docker/README.md → If the build fails](../../../docker/README.md#10-if-the-build-fails) |
| Setting up your machine (Docker, NVIDIA, `make doctor`) | [dev-setup.md → Problems](../dev-setup.md#problems) |
| The simulation / Gazebo | [braco_description → When something goes wrong](../../../ros2_ws/src/braco_description/README.md#when-something-goes-wrong) |
| A hot arm, or the arm does nothing | [hot-arm guide](../hot-arm.md) |
| ROS 2: build, tests, topics, two machines, GUI tools | Below, in this file |

When something fails, first collect the basics:

```bash
printenv | grep -E 'ROS|RMW'        # distro, domain, RMW
ros2 doctor --report                # environment and network
ros2 node list; ros2 topic list     # what this machine can see
```

---

## Environment and commands

| Symptom | Cause | Fix |
|---|---|---|
| `ros2: command not found` | ROS is not sourced in this shell | In the container use `make shell` (sources automatically). Natively: `source /opt/ros/lyrical/setup.bash`. With `docker compose exec <svc> <cmd>`, prefix with `ros_entrypoint.sh` |
| `Package 'my_pkg' not found` | Workspace not built or not sourced in this terminal | `cd /ws && colcon build && source install/setup.bash` |
| `No executable found` | The executable is not installed | C++: add it to `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})`. Python: add it to `entry_points` in `setup.py`. Rebuild and re-source |
| A Python change has no effect | Built without `--symlink-install`, or a new file/entry point was added | Rebuild. Our images use symlink install by default; new entry points always need a rebuild |
| Strange errors mixing ROS 1 and ROS 2, or two distros | Several `setup.bash` files sourced in one shell (the Nano host has Noetic and Galactic) | Open a new shell and source only one distro. Never source ROS from `~/.bashrc` on the Nano host |
| `bash: /opt/seame/tests/talker-listener.sh: No such file or directory` (from `make net-test`) | `docker/tests` was deleted and recreated on the host (e.g. by a `git checkout`) after the container started; the bind mount still points to the old, deleted folder | Recreate the containers: `make down && make up`. `ws/` and the named volumes are kept |
| `AttributeError: 'RcutilsLogger' object has no attribute 'warn'` | Older tutorials use `get_logger().warn()`, which does not exist in Lyrical | Use `get_logger().warning()` |
| `ModuleNotFoundError: No module named 'ros2_basics_interfaces'` | Interface package not built, or built after the shell was sourced | `colcon build --packages-select ros2_basics_interfaces && source install/setup.bash` |

## Building (colcon / CMake)

| Symptom | Cause | Fix |
|---|---|---|
| `Could not find a package configuration file provided by "X"` | Missing dependency | Add `<depend>X</depend>` to `package.xml`, then `rosdep install --from-paths src --ignore-src -y` (or `make run CMD='rosdep install --from-paths src --ignore-src -y'`). If it is your own package, build it first (colcon orders by `package.xml`) |
| `colcon build` creates `build/` inside `src/` | Ran from the wrong folder | Always build from the workspace root `/ws`. Delete the stray folders |
| Build of the interface package fails with action errors | Missing `action_msgs` dependency or `member_of_group` | See [INSTRUCTIONS.md §4](INSTRUCTIONS.md#4-custom-interfaces): `<depend>action_msgs</depend>` and `<member_of_group>rosidl_interface_packages</member_of_group>` |
| `InvalidPackage: Invalid email "...@todo.todo" for person "..."` | `ros2 pkg create --maintainer-name` without `--maintainer-email` derives an address from the name; characters like `:` make it invalid | Put a real address in `<maintainer email="...">` in `package.xml` (and `maintainer_email` in `setup.py`), or pass `--maintainer-email` to `ros2 pkg create` |
| `CMake Deprecation Warning ... Compatibility with CMake < 3.10 will be removed` | `cmake_minimum_required(VERSION 3.8)` in old tutorials | `cmake_minimum_required(VERSION 3.10)` |
| `ament_target_dependencies` deprecation warning | Old tutorial style | Use `target_link_libraries(target rclcpp::rclcpp ${std_msgs_TARGETS})` |
| Jetson freezes or the compiler is killed (`c++: fatal error: Killed signal`) | Out of memory (4 GB) | `make build` already builds one package at a time with `MAKEFLAGS=-j2`; add swap (`make doctor` checks it), or `MAKEFLAGS=-j1 colcon build --executor sequential` |
| `Permission denied` writing `build/`, `install/`, `log/` | `ros2_ws` created by Docker as root, or wrong `USER_UID` in `.env` | `sudo chown -R $(id -u):$(id -g) ros2_ws` on the host; check `USER_UID`/`USER_GID` in `.env` and rebuild the image |
| Weird stale build errors after moving/renaming packages | Old build cache | `rm -rf build install log` (or `make clean`) and build again |

## Tests and linters (colcon test)

How to run them: [INSTRUCTIONS.md §10](INSTRUCTIONS.md#tests-linters). See the failures with `colcon test-result --verbose`.

| Symptom | Cause | Fix |
|---|---|---|
| Python package: `Ran 0 tests ... NO TESTS RAN` | `setup.py` has no `extras_require={'test': ['pytest']}`, so colcon falls back to `unittest` and skips all the linters | Keep the `extras_require` block that `ros2 pkg create` generates |
| `copyright`: `could not find copyright notice` / cpplint `legal/copyright` | A `.cpp` file has no license header | Start the file with the Apache-2.0 header (`// Copyright 2026 SEA:ME Team 2` + the license text), see [INSTRUCTIONS.md §9](INSTRUCTIONS.md#ros2_basics_cppcmakeliststxt-replace-the-generated-one) |
| cpplint `runtime/int`: `Use int16/int64/etc, rather than the C type long` | `long long` (e.g. `static_cast<long long>(x)` for `printf`) | `#include <cinttypes>` and print with `"%" PRId64`, passing the `int64_t` directly |
| uncrustify fails | Formatting differs from the ROS 2 style | `ament_uncrustify --reformat src/` inside the package |
| flake8 `I100 Import statements are in the wrong order` | Imports not alphabetical inside a group. `rclpy`, `std_msgs` and your own packages (`ros2_basics_py`) all count as third party | Sort them alphabetically in one group: `from action_msgs... / import rclpy / from rclpy... / from ros2_basics_py... / from std_msgs...`. Standard library first (`from glob import glob` before `import os`) |
| pep257 `D400`/`D415`: first line should end with a period | One-line docstring without a final `.` | End it with a period |
| pep257 `D213`: multi-line docstring summary should start at the second line | ROS 2 uses the D213 convention | Keep module docstrings on one line, or start the summary on the line after `"""` |
| `git status` shows `__pycache__/*.pyc` under `ros2_ws/src` | `--symlink-install` runs the Python files straight from `src/` | Already ignored in the root `.gitignore`; never `git add` them |

## Communication: "my subscriber receives nothing"

Go through this list in order.

1. **Is the publisher really publishing?** `ros2 topic hz /topic` on the publisher's machine.
2. **Same topic name?** `ros2 topic list`. Watch for relative vs absolute names and namespaces (`/chatter` vs `/robot/chatter`), and for typos.
3. **Same type?** `ros2 topic info /topic` shows the type; if publisher and subscriber disagree, the tools show two types and nothing connects.
4. **Compatible QoS?** `ros2 topic info -v /topic`. A `BEST_EFFORT` publisher with a `RELIABLE` subscriber, or a `VOLATILE` publisher with a `TRANSIENT_LOCAL` subscriber, never connect. The node logs an "incompatible QoS" warning. For `ros2 topic echo`, pass matching `--qos-*` options.
5. **Does the subscriber node actually spin?** A node that is created but never passed to `spin()` (or whose executor is blocked in a long callback) never runs callbacks.
6. **Lifecycle node?** A lifecycle publisher only publishes while the node is **Active**: `ros2 lifecycle get /node`.
7. **Different machines?** See the next section.

## Communication between machines (PC ↔ Jetson)

| Symptom | Cause | Fix |
|---|---|---|
| Nodes on the other machine do not appear at all | Different `ROS_DOMAIN_ID` or `RMW_IMPLEMENTATION` | Compare `printenv | grep -E 'ROS_DOMAIN_ID|RMW'` on both. They come from `docker/.env`; make them identical and `make down && make up` |
| Still nothing, IDs match | Multicast blocked (typical on Wi-Fi / campus networks) | Test with `ros2 multicast receive` (A) and `ros2 multicast send` (B). If it fails, set `ROS_STATIC_PEERS` to the other machine's IP in `docker/compose.yml` (`x-ros-env`), or use a wired connection / own router |
| Still nothing | Firewall | `sudo ufw status`. Allow UDP from the other machine: `sudo ufw allow from <other-ip>` |
| Still nothing | Discovery range | `ROS_AUTOMATIC_DISCOVERY_RANGE` must be `SUBNET` (our default), not `LOCALHOST`, and both machines in the same subnet |
| Nodes appear but no data, or only small messages arrive | Network MTU / fragmentation, or very large messages over Wi-Fi | Test with small messages first; for images use `SensorDataQoS`, lower resolution or compressed images |
| Two teams see each other's nodes | Same domain ID on a shared network | Agree on a different `ROS_DOMAIN_ID` per team (0 to 101) |
| Timestamps / tf look wrong across machines | Clocks out of sync | Enable NTP on both hosts (`timedatectl status`; install `chrony`) |
| `ros2 node list` shows nodes that were stopped, or misses new ones | CLI daemon cache | `ros2 daemon stop; ros2 daemon start` |
| Duplicate node names warning | Two nodes with the same name (e.g. same program on both machines) | Rename one: `--ros-args -r __node:=talker_jetson` |

## Services and actions

| Symptom | Cause | Fix |
|---|---|---|
| Client waits forever "waiting for service" | Server not running, wrong name, different domain | `ros2 service list`; check namespaces and remapping |
| Program hangs when calling a service from inside a callback | Deadlock: synchronous call on a single-threaded executor | Use `call_async` / `async_send_request` and handle the result in a callback, or a `MultiThreadedExecutor` with the client in a separate callback group |
| Action cancel requests are ignored | Server's executor is busy in `execute` | Python: spin with `MultiThreadedExecutor` and give the server a `ReentrantCallbackGroup` (as in the example); check `goal_handle.is_cancel_requested` inside the loop |
| `ros2 service call` / `action send_goal`: "The passed service/action type is invalid" | Type not found in this shell | Build and `source install/setup.bash`; type format is `pkg/srv/Name` or `pkg/action/Name` |
| YAML errors in `ros2 ... call/pub` | Wrong quoting | Wrap the YAML in double quotes and use `field: value` with a space after the colon: `"{a: 2, b: 3}"`. `ros2 interface proto <type>` prints a template |

## Lifecycle

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 lifecycle set` says the transition is not available | Not valid from the current state | `ros2 lifecycle get /node` and `ros2 lifecycle list /node` |
| Node is Active but publishes nothing | Lifecycle publisher not activated | Python: call `super().on_activate(state)`. C++: call `pub_->on_activate()` in `on_activate` |
| "Trying to publish message on the topic ..., but the publisher is not activated" | Publishing while Inactive | Expected outside Active; check `is_activated` before publishing |
| C++ compile error spinning a lifecycle node | `rclcpp::spin` expects a node base | `rclcpp::spin(node->get_node_base_interface())` |
| Transition goes to `ErrorProcessing` | Callback returned `ERROR` or threw an exception | Check the node's log; implement `on_error` to recover |

## GUI tools (rqt, rqt_graph, RViz2, Gazebo)

| Symptom | Cause | Fix |
|---|---|---|
| `qt.qpa.xcb: could not connect to display`, or Gazebo: `Authorization required ... could not connect to display :0` | Container cannot reach your X server (the `xhost +local:` grant ends with each login) | On the host: `xhost +local:` (done by `make up` on the PC) and check `echo $DISPLAY` in the container. Wayland desktops need XWayland |
| `rqt_graph: command not found` on the PC | Lyrical no longer installs a `rqt_graph` shortcut in `/opt/ros/lyrical/bin` (only `rqt` and `rqt_bag`) | `ros2 run rqt_graph rqt_graph` or `rqt --standalone rqt_graph`, inside the container (`make shell`), not on the host |
| `rqt_graph: command not found` on the Jetson | The Jetson image is headless on purpose | Run `ros2 run rqt_graph rqt_graph` on the PC; it shows the Jetson's nodes over the network |
| rqt_graph is empty or outdated | Not refreshed, or filters | Press refresh; select "Nodes/Topics (all)"; uncheck the "Hide" options |
| RViz2/Gazebo very slow | Software rendering (no GPU) | See [Gazebo slow](../../../ros2_ws/src/braco_description/README.md#when-something-goes-wrong) |

## Jetson / hardware

| Symptom | Cause | Fix |
|---|---|---|
| Container does not start: `error gathering device information ... /dev/ttyTHS1` | `real` needs the serial port and the LCD on the host | `make doctor` checks both. On a PC use `ROLE=pc`. On the Jetson: `ls -l /dev/ttyTHS1` |
| `Permission denied` / port busy on `/dev/ttyTHS1` | Serial console (`nvgetty`) holds the UART, or wrong `DIALOUT_GID` | `nvgetty`: [dev-setup → Jetson](../dev-setup.md#jetson). `DIALOUT_GID`: `getent group dialout`, set it in `.env` |
| Docker on the Nano freezes during builds | Parallel image builds | Build one image at a time, or build on the PC (`make jetson-image NANO=user@host`) |
| SD card full | Images and build caches | `docker system df`; `docker image prune`; `make clean` (asks first) |

## Still stuck

1. Reproduce with the smallest case (CLI `ros2 topic pub` + `ros2 topic echo`).
2. Run `make run CMD=seame-smoke-test` and `make net-test` to rule out the installation.
3. Search the error message in <https://robotics.stackexchange.com/> (tag `ros2`) and the GitHub issues of the package involved.
4. Write down the fix in this file so the next person does not lose the same time.
