#!/usr/bin/env bash
# seame-smoke-test
#
# Verifies that this image has the expected pieces and that the selected RMW can move a message.
# Exit status is non-zero when a hard requirement is missing (warnings do not fail the run).
#   make run CMD=seame-smoke-test          (in the dev container of this machine)
#   docker compose exec dev seame-smoke-test   (from docker/)

# Source the ROS environment first (ROS setup scripts are not safe under `set -u`).
# shellcheck disable=SC1091
. /usr/local/share/seame/env.sh
set -u

ROLE="${SEAME_ROLE:-unknown}"
FAIL=0
WARN=0

ok()      { printf '  [ ok ] %s\n' "$*"; }
warn()    { printf '  [warn] %s\n' "$*"; WARN=$((WARN + 1)); }
bad()     { printf '  [FAIL] %s\n' "$*"; FAIL=$((FAIL + 1)); }
section() { printf '\n== %s ==\n' "$*"; }

need_cmd() { if command -v "$1" >/dev/null 2>&1; then ok "$1"; else bad "command not found: $1"; fi; }
want_cmd() { if command -v "$1" >/dev/null 2>&1; then ok "$1"; else warn "command not found: $1"; fi; }

if ! command -v ros2 >/dev/null 2>&1; then
    bad "ros2 not on PATH - ROS environment was not sourced"
    exit 1
fi

PKGS="$(ros2 pkg list 2>/dev/null)"
has_pkg() { grep -qx "$1" <<<"${PKGS}"; }
need_pkg() { if has_pkg "$1"; then ok "ROS package: $1"; else bad "missing ROS package: $1"; fi; }
want_pkg() { if has_pkg "$1"; then ok "ROS package: $1"; else warn "missing optional ROS package: $1 ($2)"; fi; }

section "System (${ROLE})"
# shellcheck disable=SC1091
. /etc/os-release
if [ "${VERSION_ID:-}" = "26.04" ]; then ok "Ubuntu ${VERSION_ID} on $(uname -m)"; else bad "expected Ubuntu 26.04, found ${PRETTY_NAME:-unknown}"; fi
if [ "${ROS_DISTRO:-}" = "lyrical" ]; then ok "ROS_DISTRO=lyrical"; else bad "ROS_DISTRO=${ROS_DISTRO:-unset} (expected lyrical)"; fi
ok "python $(python3 --version 2>&1 | cut -d' ' -f2)"
ok "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<distro default>}"
if [ -s /etc/seame/missing-packages.txt ]; then
    warn "optional packages were skipped at build time:"
    sed 's/^/        /' /etc/seame/missing-packages.txt
fi

section "Build and workflow tools"
for c in colcon rosdep vcs cmake ninja ccache git; do need_cmd "$c"; done

section "ROS 2 core, lifecycle, ros2_control"
for p in rclcpp rclpy rclcpp_lifecycle ros2lifecycle tf2_ros xacro robot_state_publisher \
         controller_manager joint_state_broadcaster joint_trajectory_controller \
         ament_cmake_gtest ament_cmake_pytest launch_testing_ament_cmake \
         rmw_fastrtps_cpp rmw_cyclonedds_cpp; do
    need_pkg "$p"
done
want_pkg rmw_zenoh_cpp "Zenoh transport for Module 3"
want_pkg rosbag2_storage_mcap "MCAP bag storage"
if [ -n "${RMW_IMPLEMENTATION:-}" ]; then
    need_pkg "${RMW_IMPLEMENTATION}"
fi

section "Python"
if python3 -c 'import rclpy, numpy, yaml, serial' 2>/dev/null; then ok "rclpy, numpy, yaml, pyserial import"; else bad "python import of rclpy/numpy/yaml/serial failed"; fi
if pip3 show pymycobot >/dev/null 2>&1; then
    ok "pymycobot $(pip3 show pymycobot | awk '/^Version:/ {print $2}') installed"
    if python3 -c 'import pymycobot' 2>/tmp/pymycobot_import.err; then ok "import pymycobot"; else bad "import pymycobot failed: $(tail -n1 /tmp/pymycobot_import.err)"; fi
else
    bad "pymycobot is not installed"
fi

if [ "${ROLE}" = "pc-dev" ]; then
    section "PC: visualisation, MoveIt 2, Gazebo"
    for p in rviz2 moveit_ros_move_group moveit_setup_assistant moveit_ros_planning_interface \
             ros_gz_sim ros_gz_bridge; do
        need_pkg "$p"
    done
    want_pkg moveit_servo "visual servoing; install with: sudo apt-get update && sudo apt-get install ros-${ROS_DISTRO}-moveit-servo"
    want_pkg gz_ros2_control "needed to run ros2_control inside Gazebo; see README"
    # PlotJuggler 4: the app, plus a ROS 2 subscriber build for this distro whose ROS libraries all resolve.
    if command -v plotjuggler4 >/dev/null 2>&1; then
        ok "plotjuggler4 (PlotJuggler 4)"
        PJ_PLUGIN="${HOME}/.local/share/PlotJuggler/PlotJuggler4/extensions/ros2-topic-subscriber/dist/${ROS_DISTRO}/libros2_stream_plugin-${ROS_DISTRO}.pjros2"
        if [ ! -f "${PJ_PLUGIN}" ]; then
            warn "PlotJuggler's ROS 2 subscriber has no ${ROS_DISTRO} build installed: live topics unavailable (bag files still open)"
        elif PJ_MISSING="$(ldd "${PJ_PLUGIN}" 2>/dev/null | awk '/not found/ {print $1}' | tr '\n' ' ')" && [ -n "${PJ_MISSING}" ]; then
            warn "PlotJuggler ROS 2 subscriber (${ROS_DISTRO}) cannot resolve: ${PJ_MISSING}"
        else
            ok "PlotJuggler ROS 2 subscriber (${ROS_DISTRO}) resolves all its ROS libraries"
        fi
    else
        warn "plotjuggler4 not installed (see the skipped list above); use Foxglove Studio via foxglove_bridge, or rqt_plot"
    fi
    want_pkg foxglove_bridge "Foxglove Studio connection"
    if command -v gz >/dev/null 2>&1; then
        ok "gz (Gazebo) version: $(timeout 20 gz sim --versions 2>/dev/null | head -n1)"
    else
        bad "gz command not found"
    fi
    for c in clang-tidy clang-format cppcheck lcov gcovr gdb ruff pre-commit pytest; do need_cmd "$c"; done
fi

section "RMW loopback (publish + subscribe in one process)"
if timeout 40 python3 - <<'PY'
import sys, time
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("seame_smoke")
got = []
node.create_subscription(String, "seame_smoke", lambda m: got.append(m.data), 10)
pub = node.create_publisher(String, "seame_smoke", 10)
deadline = time.time() + 20
while not got and time.time() < deadline:
    pub.publish(String(data="ping"))
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
PY
then ok "message delivered through the selected RMW"; else bad "loopback message was not delivered"; fi

printf '\nResult: %d failure(s), %d warning(s)\n' "${FAIL}" "${WARN}"
[ "${FAIL}" -eq 0 ]
