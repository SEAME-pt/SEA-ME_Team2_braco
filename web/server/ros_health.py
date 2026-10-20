"""Health of the arm's ROS 2 graph: are the arm nodes up, what does /diagnostics say.

Expected nodes and services come from ros2_ws/src/seame_arm/launch/arm.launch.py
and arm_driver.cpp; /diagnostics is published by temperature_monitor.
"""

import threading
import time

EXPECTED_NODES = [
    ("arm_driver", "owns the serial port: angles, temperatures, commands"),
    ("temperature_monitor", "temperatures -> /diagnostics and the LCD"),
    ("joystick_teleop", "joystick -> /arm/command"),
]
EXPECTED_SERVICES = ["/arm/power_on", "/arm/release_servos", "/arm/set_gripper", "/arm/set_read_only"]
LEVELS = {0: "ok", 1: "warn", 2: "error", 3: "stale"}


def level_name(level):
    """DiagnosticStatus.level is a `byte`: rclpy gives bytes, tests give ints."""
    if isinstance(level, (bytes, bytearray)):
        level = level[0] if level else 0
    return LEVELS.get(int(level), "unknown")


class Health:
    def __init__(self, bridge):
        self._bridge = bridge
        self._lock = threading.Lock()
        self._diagnostics = {}

    def start(self):
        self._bridge.start()
        self._bridge.when_ready(self._setup)

    def _setup(self, node):
        if node is None:
            return
        from diagnostic_msgs.msg import DiagnosticArray
        node.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)

    def _on_diagnostics(self, msg):
        now = time.time()
        with self._lock:
            for status in msg.status:
                self._diagnostics[status.name] = {
                    "name": status.name,
                    "level": level_name(status.level),
                    "message": status.message,
                    "hardware_id": status.hardware_id,
                    "values": [[kv.key, kv.value] for kv in status.values],
                    "t": now,
                }

    def snapshot(self):
        node = self._bridge.node
        if node is None or not self._bridge.available:
            return {"available": False, "error": self._bridge.error}
        try:
            names = {name for name, _ns in node.get_node_names_and_namespaces()}
            services = {name for name, _types in node.get_service_names_and_types()}
        except Exception as e:   # the context is shutting down
            return {"available": False, "error": str(e)}
        with self._lock:
            diagnostics = sorted(self._diagnostics.values(), key=lambda d: d["name"])
        return {
            "available": True,
            "error": None,
            "nodes": [{"name": n, "role": role, "present": n in names} for n, role in EXPECTED_NODES],
            "services": [{"name": s, "present": s in services} for s in EXPECTED_SERVICES],
            "diagnostics": diagnostics,
        }
