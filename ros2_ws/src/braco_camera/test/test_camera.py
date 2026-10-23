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
The camera node's frame handling, without a camera or a ROS graph.

scripts/camera.py is installed as a program, not a Python package, so it is
loaded from its path. Importing it does not start ROS: only main() does.
"""

import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

_SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'camera.py'
_spec = importlib.util.spec_from_file_location('camera', _SCRIPT)
camera = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(camera)

MJPG = b'\xff\xd8' + bytes(range(64)) + b'\xff\xd9'


def test_mjpg_buffer_passes_through():
    """A 1-D MJPG buffer from the camera is published as it is."""
    frame = np.frombuffer(MJPG, dtype=np.uint8)
    assert camera.to_jpeg(frame) == MJPG


def test_mjpg_single_row_passes_through():
    """Some V4L2 backends give the MJPG buffer as a 1xN array."""
    frame = np.frombuffer(MJPG, dtype=np.uint8).reshape(1, -1)
    assert camera.to_jpeg(frame) == MJPG


def test_buffer_that_is_not_jpeg_is_rejected():
    """A corrupt buffer is dropped, so the node reopens the camera."""
    frame = np.zeros(64, dtype=np.uint8)
    assert camera.to_jpeg(frame) is None


def test_raw_frame_is_encoded():
    """A camera without MJPG gives BGR frames: they are encoded here."""
    frame = np.full((48, 64, 3), (0, 128, 255), dtype=np.uint8)
    data = camera.to_jpeg(frame, quality=90)
    assert data.startswith(camera.JPEG_START)
    decoded = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (48, 64, 3)


@pytest.mark.parametrize('fps', [0, -5, 0.5, 1])
def test_frame_period_is_at_most_one_second(fps):
    """A missing or nonsense fps never stalls the loop for more than 1 s."""
    assert camera.frame_period(fps) == 1.0


def test_frame_period():
    """At 10 fps a frame every 0.1 s."""
    assert camera.frame_period(10) == pytest.approx(0.1)
