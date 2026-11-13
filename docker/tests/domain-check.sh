#!/usr/bin/env bash
# Domain check (#103) - before MoveIt: is this shell on the right ROS domain, with exactly one arm on it?
#
#   domain-check.sh [--target real]   MoveIt on the real arm: this shell must be on the arm's domain
#   domain-check.sh --target sim      MoveIt on Gazebo: this shell must NOT be on the arm's domain
#
# Run it in the shell that will start MoveIt (a dev shell), or with make domain-check [TARGET=sim]. The MoveIt
# launches run it first: real.launch.py (make moveit) with --target real, sim.launch.py with --target sim.
# The real arm: one /joint_states publisher (arm_driver), no /clock, one controller_manager (braco_hardware). A
# simulation (Gazebo + ros2_control) adds a /joint_states publisher (joint_state_broadcaster), /clock and a
# controller_manager.
# Fails when:
#   the domain       real: not ARM_ROS_DOMAIN_ID.  sim: ARM_ROS_DOMAIN_ID (checked first, no discovery needed)
#   /joint_states    publishers: 0 (nothing seen) or more than 1 (a second arm or simulation)
#   /clock           publishers: real: any (a simulation on the arm's domain).  sim: not exactly 1
#   controller_manager nodes: more than 1
# The counting is in domain_check.py, next to this script.
#
# Environment: ARM_ROS_DOMAIN_ID  the real arm's domain (docker/compose.yml sets it; default 22)
#              WAIT               seconds of DDS discovery before counting (default 3)
# Exit status: 0 PASS, 1 FAIL, 2 cannot run (ROS not sourced, bad arguments).
set -uo pipefail

usage() { echo "usage: $0 [--target real|sim]   (WAIT=<seconds>)" >&2; exit 2; }

TARGET=real
while [ $# -gt 0 ]; do
    case "$1" in
        --target) TARGET="${2:-}"; shift 2 || shift ;;
        --target=*) TARGET="${1#*=}"; shift ;;
        *) usage ;;
    esac
done
case "${TARGET}" in real|sim) ;; *) usage ;; esac
WAIT="${WAIT:-3}"
[[ "${WAIT}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "WAIT must be a number of seconds, not '${WAIT}'" >&2; exit 2; }

if ! python3 -c 'import rclpy' 2>/dev/null; then
    echo "  [FAIL] rclpy not found: run this in the dev container (make domain-check)" >&2
    exit 2
fi

exec python3 "$(dirname "$0")/domain_check.py" "${TARGET}" "${ARM_ROS_DOMAIN_ID:-22}" "${WAIT}"
