#!/usr/bin/env bash
# Domain check (#103) - before MoveIt: is this shell on the right ROS domain, with exactly one arm on it?
#
#   domain-check.sh [--target real]   MoveIt on the real arm: this shell must be on the arm's domain
#   domain-check.sh --target sim      MoveIt on Gazebo: this shell must NOT be on the arm's domain
#
# Run it in the shell that will start MoveIt (a dev shell), or with make domain-check [TARGET=sim].
# The real arm is arm_driver alone: one /joint_states publisher, no /clock, no controller_manager. A simulation
# (Gazebo + ros2_control) adds a /joint_states publisher (joint_state_broadcaster), /clock and a controller_manager.
# Fails when:
#   the domain       real: not ARM_ROS_DOMAIN_ID.  sim: ARM_ROS_DOMAIN_ID (checked first, no discovery needed)
#   /joint_states    publishers: 0 (nothing seen) or more than 1 (a second arm or simulation)
#   /clock           publishers: real: any (a simulation on the arm's domain).  sim: not exactly 1
#   controller_manager nodes: more than 1
#
# Environment: ARM_ROS_DOMAIN_ID  the real arm's domain (docker/compose.yml sets it; default 22)
#              WAIT               seconds of DDS discovery before counting (default 3)
# Exit status: 0 PASS, 1 FAIL, 2 cannot run (ROS not sourced, bad arguments).
set -uo pipefail

TARGET=real
while [ $# -gt 0 ]; do
    case "$1" in
        --target) TARGET="${2:-}"; shift 2 || shift ;;
        --target=*) TARGET="${1#*=}"; shift ;;
        *) echo "usage: $0 [--target real|sim]" >&2; exit 2 ;;
    esac
done
case "${TARGET}" in real|sim) ;; *) echo "usage: $0 [--target real|sim]" >&2; exit 2 ;; esac

if ! python3 -c 'import rclpy' 2>/dev/null; then
    echo "  [FAIL] rclpy not found: run this in the dev container (make domain-check)" >&2
    exit 2
fi

exec python3 - "${TARGET}" "${ARM_ROS_DOMAIN_ID:-22}" "${WAIT:-3}" <<'PY'
import os
import sys
import time

target, arm, wait = sys.argv[1], sys.argv[2], float(sys.argv[3])
domain = os.environ.get('ROS_DOMAIN_ID', '0')
print(f'== Domain check ({target}): ROS_DOMAIN_ID={domain}, real arm on {arm}, '
      f'RMW={os.environ.get("RMW_IMPLEMENTATION", "default")} (discovery {wait:g} s) ==')

# The guard: refuse the wrong domain before looking at anything on it
if target == 'real' and domain != arm:
    print(f'  [FAIL] this shell is on domain {domain}, the real arm on {arm}')
    print(f'FAIL: not the real arm\'s domain. MoveIt here would drive whatever runs on {domain} '
          f'(your simulation?). For the real arm: export ROS_DOMAIN_ID={arm}')
    sys.exit(1)
if target == 'sim' and domain == arm:
    print(f'  [FAIL] this shell is on domain {domain}: the real arm\'s')
    print('FAIL: MoveIt for the simulation would drive the real arm. Use your PC_ROS_DOMAIN_ID '
          '(a new make shell starts on it; docker/README.md, ROS domain per teammate)')
    sys.exit(1)
print(f'  [ ok ] domain {domain}: ' + ('the real arm\'s' if target == 'real' else 'not the real arm\'s'))

import rclpy  # noqa: E402  (after the guard: a wrong domain fails without DDS discovery)

rclpy.init()
node = rclpy.create_node('braco_domain_check')
end = time.monotonic() + wait
while time.monotonic() < end:        # spin so discovery runs while we wait
    rclpy.spin_once(node, timeout_sec=0.1)


def publishers(topic):
    return sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                  for i in node.get_publishers_info_by_topic(topic))


joint_states = publishers('/joint_states')
clocks = publishers('/clock')
managers = sorted(f'{ns.rstrip("/")}/{name}' for name, ns in node.get_node_names_and_namespaces()
                  if name == 'controller_manager')
node.destroy_node()
rclpy.shutdown()

fail = 0


def report(ok, text):
    global fail
    print(f'  [{" ok " if ok else "FAIL"}] {text}')
    fail |= not ok


def names(found):
    return ', '.join(found) if found else 'none'


clock_ok = not clocks if target == 'real' else len(clocks) == 1
report(len(joint_states) == 1, f'/joint_states publishers: {len(joint_states)} ({names(joint_states)})')
report(clock_ok, f'/clock publishers: {len(clocks)} ({names(clocks)})')
report(len(managers) <= 1, f'controller_manager nodes: {len(managers)} ({names(managers)})')

what = 'one arm' if target == 'real' else 'one simulation'
if not fail:
    print(f'PASS: {what} on domain {domain}, nothing else publishes /joint_states')
    sys.exit(0)

print(f'FAIL: domain {domain} is not safe for MoveIt ({target}).')
if not joint_states:
    print('  - No /joint_states: ' + (
        'the real arm is not running or not reachable (Jetson: make up; PC firewall: '
        'docs/guides/ROS2/TROUBLESHOOTING.md).' if target == 'real' else
        'no simulation running on this domain (make up SIM=1).'))
elif len(joint_states) > 1:
    print('  - More than one /joint_states publisher: a second arm or a simulation shares this domain. '
          'Stop it, or move it to its own domain (docker/README.md, ROS domain per teammate).')
if target == 'real' and clocks:
    print('  - /clock is published: a simulation (Gazebo) runs on the real arm\'s domain. Stop it or move it.')
if target == 'sim' and len(clocks) > 1:
    print('  - Several /clock publishers: more than one simulation on this domain. Stop one (one per domain).')
if target == 'sim' and not clocks and joint_states:
    print('  - /joint_states without /clock: that is a real arm, not a simulation. Wrong domain?')
if len(managers) > 1:
    print('  - Several controller_manager nodes: more than one ros2_control system on this domain.')
sys.exit(1)
PY
