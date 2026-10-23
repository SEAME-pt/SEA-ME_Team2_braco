"""The arm's camera on the page: the camera node's JPEG frames as an MJPEG stream.

The page never opens the camera: braco_camera's `camera` node owns it and publishes
/camera/image_raw/compressed. This keeps the newest frame and hands it to every
browser watching (GET /api/camera/stream, an <img> can show it as it is).
Without ROS (a PC, make web, the tests) there are no frames and status() says why.
"""

import asyncio
import threading
import time

TOPIC = '/camera/image_raw/compressed'
STALE_S = 2.0  # no frame for this long: the camera is offline
STREAM_MAX_FPS = 15.0  # per browser, whatever the camera sends
POLL_S = 0.02
BOUNDARY = 'frame'


class Camera:
    def __init__(self, bridge):
        self._bridge = bridge
        self._lock = threading.Lock()
        self._jpeg = None
        self._seq = 0
        self._t = 0.0
        self._times = []  # arrival of the last frames, for the rate

    def start(self):
        self._bridge.start()
        self._bridge.when_ready(self._setup)

    def _setup(self, node):
        if node is None:
            return
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        node.create_subscription(CompressedImage, TOPIC, self._on_frame, qos_profile_sensor_data)

    def _on_frame(self, msg):
        now = time.time()
        data = bytes(msg.data)
        with self._lock:
            self._jpeg = data
            self._seq += 1
            self._t = now
            self._times = [t for t in self._times if now - t < 2.0] + [now]

    def latest(self):
        """(seq, jpeg) of the newest frame, or (seq, None) when it is stale."""
        with self._lock:
            if self._jpeg is None or time.time() - self._t > STALE_S:
                return self._seq, None
            return self._seq, self._jpeg

    def status(self):
        with self._lock:
            now = time.time()
            age = now - self._t if self._jpeg is not None else None
            times = [t for t in self._times if now - t < 2.0]
            size = len(self._jpeg) if self._jpeg is not None else None
        online = age is not None and age <= STALE_S
        if online:
            error = None
        elif not self._bridge.available:
            error = self._bridge.error
        elif age is None:
            error = 'no frames on {} (is the camera node running?)'.format(TOPIC)
        else:
            error = 'no frame for {:.0f} s (camera unplugged?)'.format(age)
        return {
            'online': online,
            'topic': TOPIC,
            'fps': round(len(times) / 2.0, 1) if online else 0.0,
            'age': None if age is None else round(age, 2),
            'bytes': size,
            'error': error,
        }

    async def stream(self):
        """multipart/x-mixed-replace parts, one per new frame; ends when the camera goes stale."""
        last = None
        sent = 0.0
        while True:
            seq, jpeg = self.latest()
            if jpeg is None:
                return
            now = time.monotonic()
            if seq != last and now - sent >= 1.0 / STREAM_MAX_FPS:
                last, sent = seq, now
                yield (
                    b'--'
                    + BOUNDARY.encode()
                    + b'\r\nContent-Type: image/jpeg\r\nContent-Length: '
                    + str(len(jpeg)).encode()
                    + b'\r\n\r\n'
                    + jpeg
                    + b'\r\n'
                )
            await asyncio.sleep(POLL_S)
