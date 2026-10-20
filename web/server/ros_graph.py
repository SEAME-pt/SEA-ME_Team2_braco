"""Read-only explorer of the ROS 2 graph: nodes, topics, services, actions, live echo.

Nothing here publishes or calls a service: the explorer only looks. A topic is
subscribed while at least one browser watches it, and dropped ECHO_LINGER
seconds after the last one leaves.
"""

import collections
import json
import threading
import time

ECHO_LINGER = 30.0     # seconds a subscription survives without viewers
ECHO_MAX_CHARS = 4000  # longer messages are cut for the page
ECHO_ARRAY_MAX = 32    # array/string elements kept per field
RATE_WINDOW = 5.0      # seconds of message stamps used for the rate
INTERNAL_SUFFIXES = (
    "/describe_parameters", "/get_parameter_types", "/get_parameters", "/list_parameters",
    "/set_parameters", "/set_parameters_atomically", "/get_type_description",
)


def is_internal_service(name):
    """Parameter and type-description services every node has; hidden by default."""
    return name.endswith(INTERNAL_SUFFIXES) or "/_action/" in name


def jsonable(value):
    """Turn what message_to_ordereddict returns into plain JSON types."""
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return list(value[:ECHO_ARRAY_MAX])
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, "tolist"):   # array.array, numpy
        return jsonable(value.tolist())
    return str(value)


def cut(data):
    """The message as JSON-able data, or a truncated string if it is too long."""
    text = json.dumps(data)
    if len(text) <= ECHO_MAX_CHARS:
        return data
    return text[:ECHO_MAX_CHARS] + " … (cut)"


class _Echo:
    def __init__(self, topic, type_name):
        self.topic = topic
        self.type = type_name
        self.subscription = None
        self.viewers = set()
        self.idle_since = None
        self.count = 0
        self.last = None
        self.last_t = None
        self.stamps = collections.deque(maxlen=200)

    def on_message(self, msg):
        now = time.time()
        self.count += 1
        self.last = msg
        self.last_t = now
        self.stamps.append(now)

    def rate(self, now):
        recent = [t for t in self.stamps if now - t <= RATE_WINDOW]
        if len(recent) < 2:
            return None
        return round((len(recent) - 1) / (recent[-1] - recent[0]), 2) if recent[-1] > recent[0] else None


class Graph:
    def __init__(self, bridge):
        self._bridge = bridge
        self._lock = threading.Lock()
        self._echoes = {}

    def snapshot(self):
        node = self._bridge.node
        if node is None or not self._bridge.available:
            return {"available": False, "error": self._bridge.error}
        try:
            nodes = sorted(
                (ns.rstrip("/") + "/" + name) for name, ns in node.get_node_names_and_namespaces())
            topics = [{
                "name": name,
                "types": types,
                "publishers": node.count_publishers(name),
                "subscribers": node.count_subscribers(name),
            } for name, types in sorted(node.get_topic_names_and_types())]
            services = [{"name": name, "types": types, "internal": is_internal_service(name)}
                        for name, types in sorted(node.get_service_names_and_types())]
            actions = []
            try:
                from rclpy.action import get_action_names_and_types
                actions = [{"name": name, "types": types}
                           for name, types in sorted(get_action_names_and_types(node))]
            except ImportError:
                pass
        except Exception as e:   # the context is shutting down
            return {"available": False, "error": str(e)}
        return {"available": True, "error": None, "nodes": nodes, "topics": topics,
                "services": services, "actions": actions}

    # ---- echo -----------------------------------------------------------------

    def watch(self, topic, viewer):
        """Start showing `topic` to `viewer`; returns an error string or None."""
        node = self._bridge.node
        if node is None:
            return self._bridge.error or "ROS 2 not available"
        with self._lock:
            echo = self._echoes.get(topic)
            if echo is not None:
                echo.viewers.add(viewer)
                echo.idle_since = None
                return None
        types = dict(node.get_topic_names_and_types()).get(topic)
        if not types:
            return "topic {} not found".format(topic)
        try:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from rosidl_runtime_py.utilities import get_message
            cls = get_message(types[0])
        except Exception as e:
            return "cannot load message type {}: {}".format(types[0], e)
        echo = _Echo(topic, types[0])
        # Best effort subscribes to both reliable and best-effort publishers
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        with self._lock:
            if topic in self._echoes:   # another viewer won the race
                self._echoes[topic].viewers.add(viewer)
                return None
            echo.subscription = node.create_subscription(cls, topic, echo.on_message, qos)
            echo.viewers.add(viewer)
            self._echoes[topic] = echo
        return None

    def unwatch(self, topic, viewer):
        with self._lock:
            echo = self._echoes.get(topic)
            if echo is not None:
                echo.viewers.discard(viewer)
                if not echo.viewers:
                    echo.idle_since = time.time()

    def drop_viewer(self, viewer):
        with self._lock:
            topics = [t for t, e in self._echoes.items() if viewer in e.viewers]
        for topic in topics:
            self.unwatch(topic, viewer)

    def echoes(self):
        """{topic: (viewers, payload)} for every watched topic; also drops idle ones."""
        now = time.time()
        out = {}
        with self._lock:
            for topic, echo in list(self._echoes.items()):
                if not echo.viewers:
                    if echo.idle_since and now - echo.idle_since > ECHO_LINGER:
                        node = self._bridge.node
                        if node is not None and echo.subscription is not None:
                            node.destroy_subscription(echo.subscription)
                        del self._echoes[topic]
                    continue
                out[topic] = (set(echo.viewers), self._payload(echo, now))
        return out

    @staticmethod
    def _payload(echo, now):
        data = None
        if echo.last is not None:
            try:
                from rosidl_runtime_py.convert import message_to_ordereddict
                data = cut(jsonable(message_to_ordereddict(echo.last, truncate_length=ECHO_ARRAY_MAX)))
            except Exception as e:
                data = "cannot show this message: {}".format(e)
        return {"topic": echo.topic, "type": echo.type, "count": echo.count,
                "hz": echo.rate(now), "last_t": echo.last_t, "message": data}
