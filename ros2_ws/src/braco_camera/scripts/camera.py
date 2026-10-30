#!/usr/bin/env python3
# Copyright 2026 SEA:ME Team 2
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
camera - the eye-in-hand USB camera of the arm as JPEG frames on ROS 2.

The only process that opens the camera device: everybody else (the web page,
later the vision nodes) subscribes to the topic.

    pub  /camera/image_raw/compressed   sensor_msgs/CompressedImage (format "jpeg")

The camera is asked for MJPG, so its frames are published as they come, without
decoding them; a camera that only gives raw frames is encoded here. Without the
camera (unplugged, wrong device) the node keeps running and tries again every
reopen_s seconds, so the rest of the system never depends on it.
"""

import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

TOPIC = '/camera/image_raw/compressed'
JPEG_START = b'\xff\xd8'


def to_jpeg(frame, quality=80):
    """Return the JPEG bytes of a frame: the camera's own, or encoded here."""
    if frame.ndim == 1 or frame.shape[0] == 1:   # a raw MJPG buffer
        data = frame.tobytes()
        return data if data.startswith(JPEG_START) else None
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return buf.tobytes() if ok else None


def frame_period(fps):
    """Return the seconds between frames, at least 1 fps."""
    return 1.0 / max(1.0, float(fps))


class Camera(Node):
    """Read the camera in a thread and publish every frame."""

    def __init__(self):
        super().__init__('camera')
        self.device = self.declare_parameter('device', '/dev/video0').value
        self.width = self.declare_parameter('width', 640).value
        self.height = self.declare_parameter('height', 480).value
        self.fps = self.declare_parameter('fps', 15.0).value
        self.quality = self.declare_parameter('jpeg_quality', 80).value
        self.frame_id = self.declare_parameter('frame_id', 'camera_link').value
        self.reopen_s = self.declare_parameter('reopen_s', 2.0).value
        self.pub = self.create_publisher(CompressedImage, TOPIC, qos_profile_sensor_data)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # the newest frame, not a queue of old ones
        if int(cap.get(cv2.CAP_PROP_FOURCC)) == cv2.VideoWriter_fourcc(*'MJPG'):
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # hand over the camera's own JPEG
        return cap

    def _run(self):
        cap = None
        warned = False
        period = frame_period(self.fps)
        while not self._stop.is_set() and rclpy.ok():
            if cap is None:
                cap = self._open()
                if cap is None:
                    if not warned:
                        self.get_logger().warn(
                            'no camera at {}, trying again every {:g} s'.format(
                                self.device, self.reopen_s))
                        warned = True
                    self._stop.wait(self.reopen_s)
                    continue
                warned = False
                self.get_logger().info('camera {} open: {}x{} at {:g} fps'.format(
                    self.device, int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), self.fps))
            started = time.monotonic()
            ok, frame = cap.read()
            data = to_jpeg(frame, self.quality) if ok and frame is not None else None
            if data is None:
                self.get_logger().warn('camera {} stopped giving frames'.format(self.device))
                cap.release()
                cap = None
                self._stop.wait(self.reopen_s)
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = data
            self.pub.publish(msg)
            # cap.read() already waits for the camera; this only caps a faster one
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))
        if cap is not None:
            cap.release()

    def close(self):
        """Stop the reading thread and release the camera."""
        self._stop.set()
        self._thread.join(timeout=2.0)


def main():
    """Run the camera node until Ctrl-C."""
    rclpy.init()
    node = Camera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
