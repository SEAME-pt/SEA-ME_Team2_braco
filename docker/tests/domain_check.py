"""Domain check (#103): the counting half of domain-check.sh, which parses the arguments.

Usage: domain_check.py TARGET ARM_DOMAIN WAIT   (TARGET real|sim; see domain-check.sh)
"""

import os
import sys
import time


def guard(target, domain, arm):
    """Refuse the wrong domain before looking at anything on it. True when it may go on."""
    if target == 'real' and domain != arm:
        print(f'  [FAIL] this shell is on domain {domain}, the real arm on {arm}')
        print(
            f"FAIL: not the real arm's domain. MoveIt here would drive whatever runs on "
            f'{domain} (your simulation?). For the real arm: export ROS_DOMAIN_ID={arm}'
        )
        return False
    if target == 'sim' and domain == arm:
        print(f"  [FAIL] this shell is on domain {domain}: the real arm's")
        print(
            'FAIL: MoveIt for the simulation would drive the real arm. Use your '
            'PC_ROS_DOMAIN_ID (a new make shell starts on it; docker/README.md, ROS domain '
            'per teammate)'
        )
        return False
    kind = "the real arm's" if target == 'real' else "not the real arm's"
    print(f'  [ ok ] domain {domain}: {kind}')
    return True


def discover(wait):
    """Spin one node for `wait` s; return the /joint_states and /clock publishers, the managers."""
    import rclpy  # here, so a wrong domain fails without DDS discovery

    rclpy.init()
    node = rclpy.create_node('braco_domain_check')
    end = time.monotonic() + wait
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    def publishers(topic):
        return sorted(
            f'{i.node_namespace.rstrip("/")}/{i.node_name}'
            for i in node.get_publishers_info_by_topic(topic)
        )

    found = (
        publishers('/joint_states'),
        publishers('/clock'),
        sorted(
            f'{ns.rstrip("/")}/{name}'
            for name, ns in node.get_node_names_and_namespaces()
            if name == 'controller_manager'
        ),
    )
    node.destroy_node()
    rclpy.shutdown()
    return found


def names(found):
    return ', '.join(found) if found else 'none'


def report(target, domain, joint_states, clocks, managers):
    """Print the counts and the reason for each failure. True on PASS."""
    checks = [
        (
            len(joint_states) == 1,
            f'/joint_states publishers: {len(joint_states)} ({names(joint_states)})',
        ),
        (
            not clocks if target == 'real' else len(clocks) == 1,
            f'/clock publishers: {len(clocks)} ({names(clocks)})',
        ),
        (len(managers) <= 1, f'controller_manager nodes: {len(managers)} ({names(managers)})'),
    ]
    for ok, text in checks:
        print(f'  [{" ok " if ok else "FAIL"}] {text}')
    if all(ok for ok, _ in checks):
        what = 'one arm' if target == 'real' else 'one simulation'
        print(f'PASS: {what} on domain {domain}, nothing else publishes /joint_states')
        return True

    print(f'FAIL: domain {domain} is not safe for MoveIt ({target}).')
    if not joint_states:
        why = (
            'the real arm is not running or not reachable (Jetson: make up; PC firewall: '
            'docs/guides/ROS2/TROUBLESHOOTING.md).'
            if target == 'real'
            else 'no simulation running on this domain (make up SIM=1).'
        )
        print(f'  - No /joint_states: {why}')
    elif len(joint_states) > 1:
        print(
            '  - More than one /joint_states publisher: a second arm or a simulation shares '
            'this domain. Stop it, or move it to its own domain (docker/README.md, ROS '
            'domain per teammate).'
        )
    if target == 'real' and clocks:
        print(
            "  - /clock is published: a simulation (Gazebo) runs on the real arm's domain. "
            'Stop it or move it.'
        )
    if target == 'sim' and len(clocks) > 1:
        print(
            '  - Several /clock publishers: more than one simulation on this domain. '
            'Stop one (one per domain).'
        )
    if target == 'sim' and not clocks and joint_states:
        print(
            '  - /joint_states without /clock: that is a real arm, not a simulation. Wrong domain?'
        )
    if len(managers) > 1:
        print(
            '  - Several controller_manager nodes: more than one ros2_control system on this '
            'domain.'
        )
    return False


def main(argv):
    target, arm, wait = argv[1], argv[2], float(argv[3])
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    print(
        f'== Domain check ({target}): ROS_DOMAIN_ID={domain}, real arm on {arm}, '
        f'RMW={os.environ.get("RMW_IMPLEMENTATION", "default")} (discovery {wait:g} s) =='
    )
    if not guard(target, domain, arm):
        return 1
    return 0 if report(target, domain, *discover(wait)) else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
