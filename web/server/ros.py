"""One shared ROS 2 node for the whole web server.

The arm source, the health panel and the graph explorer all hang
off the same rclpy node, spun by a MultiThreadedExecutor in a daemon thread.
rclpy is imported in that thread, never at module load, so the server still
runs where ROS is not installed (a PC, make web, the tests): every ROS
feature then reports `error` instead of data.

    bridge.start()
    bridge.when_ready(setup)   # setup(node), or setup(None) if ROS is unavailable
    bridge.call(client, request, timeout)
"""

import threading

NODE_NAME = 'braco_web'
SPIN_PERIOD = 0.1


class RosBridge:
    def __init__(self, name=NODE_NAME):
        self.name = name
        self.node = None
        self.available = False
        self.error = 'ROS 2 not started'
        self._lock = threading.Lock()
        self._started = False
        self._finished = False  # setup ended, with or without a node
        self._pending = []
        self._closed = threading.Event()
        self._thread = None

    def start(self):
        """Idempotent: the first caller starts the node."""
        with self._lock:
            if self._started:
                return
            self._started = True
            self.error = 'ROS 2 starting'
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def when_ready(self, fn):
        """Run fn(node) once the node exists, or fn(None) if ROS failed to start."""
        with self._lock:
            if not self._finished:
                self._pending.append(fn)
                return
        fn(self.node)

    def _finish(self):
        with self._lock:
            self._finished = True
            pending, self._pending = self._pending, []
        for fn in pending:
            try:
                fn(self.node)
            except Exception as e:  # one broken feature must not stop the others
                print(
                    'braco: ROS setup of {} failed: {}'.format(getattr(fn, '__qualname__', fn), e)
                )

    def _spin(self):
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
        except ImportError as e:
            self.error = 'rclpy not available, ROS 2 is not sourced here ({})'.format(e)
            self._finish()
            return
        try:
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node(self.name)
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
        except Exception as e:
            self.error = 'ROS 2 failed to start: {}'.format(e)
            self._finish()
            return
        self.node = node
        self.available = True
        self.error = None
        self._finish()
        try:
            while not self._closed.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=SPIN_PERIOD)
        finally:
            self.available = False
            self.error = 'ROS 2 stopped'
            executor.shutdown()
            node.destroy_node()
            rclpy.try_shutdown()

    def call(self, client, request, timeout=5.0):
        """Call a service from any thread except the executor's; return the response."""
        if not client.wait_for_service(timeout_sec=min(1.0, timeout)):
            raise RuntimeError('service {} is not available'.format(client.srv_name))
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            future.cancel()
            raise RuntimeError(
                'service {} did not answer in {:.0f} s'.format(client.srv_name, timeout)
            )
        result = future.result()
        if result is None:
            raise RuntimeError('service {} failed: {}'.format(client.srv_name, future.exception()))
        return result

    def close(self):
        self._closed.set()
        if self._thread is not None:
            self._thread.join(timeout=5)


bridge = RosBridge()
