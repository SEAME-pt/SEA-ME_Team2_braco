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
Keep the simulation scripts and the real arm's code in sync.

Checks that

  * scripts/park.py parks the Gazebo arm at PARK_ANGLES of mycobot.hpp
  * scripts/move_joint.py refuses the angles JOINT_LIMITS of mycobot.hpp would clamp
"""

import os
import re

import pytest

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MYCOBOT_HPP_CANDIDATES = [
    os.environ.get('MYCOBOT_HPP', ''),
    # the mycobot_cpp package next to this one (ros2_ws/src/mycobot_cpp; /ws/src in the container)
    os.path.join(PACKAGE_DIR, '..', 'mycobot_cpp', 'includes', 'mycobot.hpp'),
]


def mycobot_hpp():
    """Return the text of mycobot_cpp includes/mycobot.hpp."""
    for candidate in MYCOBOT_HPP_CANDIDATES:
        if candidate and os.path.isfile(candidate):
            with open(candidate, encoding='utf-8') as f:
                return f.read()
    pytest.skip('mycobot.hpp not found (set MYCOBOT_HPP)')


def script(name):
    """Return the text of scripts/<name>."""
    with open(os.path.join(PACKAGE_DIR, 'scripts', name), encoding='utf-8') as f:
        return f.read()


def test_park_script_matches_mycobot_hpp():
    match = re.search(r'PARK_DEG = \[(.*?)\]', script('park.py'))
    assert match, 'PARK_DEG not found in scripts/park.py'
    hpp = re.search(r'PARK_ANGLES = \{(.*?)\};', mycobot_hpp())
    assert hpp, 'PARK_ANGLES not found in mycobot.hpp'
    assert [float(v) for v in match.group(1).split(',')] == \
        [float(v) for v in hpp.group(1).split(',')]


def test_move_joint_limits_match_mycobot_hpp():
    match = re.search(r'LIMITS_DEG = \[(.*?)\]\n', script('move_joint.py'))
    assert match, 'LIMITS_DEG not found in scripts/move_joint.py'
    pairs = re.findall(r'\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)', match.group(1))
    block = re.search(r'JOINT_LIMITS\s*=\s*\{\{(.*?)\}\};', mycobot_hpp(), re.S)
    assert block, 'JOINT_LIMITS not found in mycobot.hpp'
    limits = re.findall(r'\{\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\}', block.group(1))
    assert len(limits) == 6
    assert [(float(lo), float(hi)) for lo, hi in pairs] == \
        [(float(lo), float(hi)) for lo, hi in limits]
