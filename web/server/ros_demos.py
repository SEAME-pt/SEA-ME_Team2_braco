"""The ros2_basics demos of issue 18, usable from the page.

    topics      /chatter, /lifecycle_chatter      (talker_cpp / managed talkers)
    service     /add_two_ints  seame_interfaces/srv/AddTwoInts
    action      /fibonacci     seame_interfaces/action/Fibonacci (feedback, cancel)
    lifecycle   managed_talker_py, managed_talker_cpp (get_state / change_state)

Start them in the arm container with `make arm-basics` (docker/Makefile).
"""

import collections
import threading
import time

FEED_LENGTH = 20
FIB_MAX_ORDER = 50     # same as fibonacci_server.MAX_ORDER
LIFECYCLE_NODES = ["managed_talker_py", "managed_talker_cpp"]
LIFECYCLE_POLL = 1.5
STATUS = {4: "succeeded", 5: "canceled", 6: "aborted"}   # action_msgs/GoalStatus


class Demos:
    def __init__(self, bridge):
        self._bridge = bridge
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._types = None
        self._feeds = {"chatter": collections.deque(maxlen=FEED_LENGTH),
                       "lifecycle_chatter": collections.deque(maxlen=FEED_LENGTH)}
        self._sum = None
        self._fib = {"state": "idle"}
        self._fib_handle = None
        self._lifecycle = {n: {"node": n, "present": False, "state": None, "transitions": []}
                           for n in LIFECYCLE_NODES}
        self._clients = {}

    def start(self):
        self._bridge.start()
        self._bridge.when_ready(self._setup)

    def close(self):
        self._closed.set()

    def _setup(self, node):
        if node is None:
            return
        try:
            from seame_interfaces.action import Fibonacci
            from seame_interfaces.srv import AddTwoInts
        except ImportError:
            Fibonacci = AddTwoInts = None   # workspace not built: only the topic feeds work
        from lifecycle_msgs.srv import ChangeState, GetAvailableTransitions, GetState
        from rclpy.action import ActionClient
        from std_msgs.msg import String

        self._types = {"AddTwoInts": AddTwoInts, "Fibonacci": Fibonacci, "GetState": GetState,
                       "ChangeState": ChangeState, "GetAvailableTransitions": GetAvailableTransitions}
        for topic in self._feeds:
            node.create_subscription(String, "/" + topic, self._feed(topic), 10)
        if AddTwoInts is not None:
            self._clients["add_two_ints"] = node.create_client(AddTwoInts, "/add_two_ints")
            self._clients["fibonacci"] = ActionClient(node, Fibonacci, "/fibonacci")
        for name in LIFECYCLE_NODES:
            self._clients[name] = {
                "get_state": node.create_client(GetState, "/{}/get_state".format(name)),
                "change_state": node.create_client(ChangeState, "/{}/change_state".format(name)),
                "transitions": node.create_client(
                    GetAvailableTransitions, "/{}/get_available_transitions".format(name)),
            }
        threading.Thread(target=self._poll_lifecycle, daemon=True).start()

    def _feed(self, topic):
        def on_message(msg):
            with self._lock:
                self._feeds[topic].append({"t": time.time(), "data": msg.data})
        return on_message

    def _ready(self, what):
        if self._types is None:
            raise RuntimeError(self._bridge.error or "ROS 2 not available")
        if what not in self._clients:
            raise RuntimeError("seame_interfaces not found: build the workspace (make ws-build)")
        return self._clients[what]

    # ---- service -----------------------------------------------------------------

    def add_two_ints(self, a, b, who):
        client = self._ready("add_two_ints")
        request = self._types["AddTwoInts"].Request(a=int(a), b=int(b))
        result = self._bridge.call(client, request, timeout=5.0)
        with self._lock:
            self._sum = {"a": int(a), "b": int(b), "sum": int(result.sum), "by": who, "t": time.time()}
        return int(result.sum)

    # ---- action ------------------------------------------------------------------

    def fibonacci_start(self, order, who):
        client = self._ready("fibonacci")
        if not 1 <= order <= FIB_MAX_ORDER:
            raise ValueError("order must be 1-{}".format(FIB_MAX_ORDER))
        with self._lock:
            if self._fib["state"] in ("sending", "running"):
                raise RuntimeError("a Fibonacci goal is already running")
        if not client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError("action /fibonacci is not available (make arm-basics)")
        with self._lock:
            self._fib = {"state": "sending", "order": order, "partial": [], "result": None,
                         "by": who, "t": time.time()}
        goal = self._types["Fibonacci"].Goal(order=order)
        future = client.send_goal_async(goal, feedback_callback=self._on_fib_feedback)
        future.add_done_callback(self._on_fib_accepted)

    def _on_fib_accepted(self, future):
        handle = future.result()
        with self._lock:
            if handle is None or not handle.accepted:
                self._fib["state"] = "rejected"
                return
            self._fib["state"] = "running"
            self._fib_handle = handle
        handle.get_result_async().add_done_callback(self._on_fib_result)

    def _on_fib_feedback(self, feedback):
        with self._lock:
            self._fib["partial"] = list(feedback.feedback.partial_sequence)

    def _on_fib_result(self, future):
        response = future.result()
        with self._lock:
            self._fib_handle = None
            self._fib["state"] = STATUS.get(response.status, "status {}".format(response.status))
            self._fib["result"] = list(response.result.sequence)
            self._fib["t"] = time.time()

    def fibonacci_cancel(self):
        with self._lock:
            handle = self._fib_handle
        if handle is None:
            raise RuntimeError("no Fibonacci goal is running")
        handle.cancel_goal_async()

    # ---- lifecycle ------------------------------------------------------------------

    def _poll_lifecycle(self):
        while not self._closed.wait(LIFECYCLE_POLL):
            node = self._bridge.node
            if node is None:
                continue
            try:
                names = {name for name, _ns in node.get_node_names_and_namespaces()}
            except Exception:
                continue
            for name in LIFECYCLE_NODES:
                info = {"node": name, "present": name in names, "state": None, "transitions": []}
                if info["present"]:
                    try:
                        clients = self._clients[name]
                        state = self._bridge.call(clients["get_state"], self._types["GetState"].Request(), 2.0)
                        info["state"] = state.current_state.label
                        available = self._bridge.call(
                            clients["transitions"], self._types["GetAvailableTransitions"].Request(), 2.0)
                        info["transitions"] = sorted({t.transition.label for t in available.available_transitions})
                    except Exception as e:
                        info["error"] = str(e)
                with self._lock:
                    self._lifecycle[name] = info

    def lifecycle_transition(self, name, label):
        if name not in LIFECYCLE_NODES:
            raise ValueError("unknown lifecycle node " + name)
        clients = self._ready(name)
        available = self._bridge.call(
            clients["transitions"], self._types["GetAvailableTransitions"].Request(), 2.0)
        match = [t.transition for t in available.available_transitions if t.transition.label == label]
        if not match:
            raise RuntimeError("transition '{}' is not available from the current state".format(label))
        request = self._types["ChangeState"].Request()
        request.transition.id = match[0].id
        request.transition.label = label
        result = self._bridge.call(clients["change_state"], request, 5.0)
        if not result.success:
            raise RuntimeError("{} refused '{}'".format(name, label))
        state = self._bridge.call(clients["get_state"], self._types["GetState"].Request(), 2.0)
        with self._lock:
            self._lifecycle[name]["state"] = state.current_state.label
        return state.current_state.label

    # ---- page -------------------------------------------------------------------------

    def snapshot(self):
        node = self._bridge.node
        if node is None or not self._bridge.available:
            return {"available": False, "error": self._bridge.error}
        try:
            services = {name for name, _types in node.get_service_names_and_types()}
        except Exception as e:
            return {"available": False, "error": str(e)}
        with self._lock:
            return {
                "available": True,
                "error": None if self._types and self._types["AddTwoInts"] else
                "seame_interfaces not found: build the workspace (make ws-build)",
                "chatter": list(self._feeds["chatter"]),
                "lifecycle_chatter": list(self._feeds["lifecycle_chatter"]),
                "add_two_ints": {"present": "/add_two_ints" in services, "last": self._sum},
                "fibonacci": dict(self._fib, present="/fibonacci/_action/send_goal" in services,
                                  max_order=FIB_MAX_ORDER),
                "lifecycle": [dict(self._lifecycle[n]) for n in LIFECYCLE_NODES],
            }
