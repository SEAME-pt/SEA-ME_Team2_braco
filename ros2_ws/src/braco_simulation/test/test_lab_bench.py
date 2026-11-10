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
Keep the lab bench scene and the arm's spawn pose in sync.

Checks that the default robot_x, robot_y, robot_z of launch/gazebo.launch.py put the arm's
pedestal (braco_description's URDF) on one table of worlds/lab_bench.sdf, flush with its rear edge.
"""

import math
import os
import re
import xml.etree.ElementTree as ET

import pytest
import xacro

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# braco_description next to this package (ros2_ws/src/braco_description; /ws/src in the container)
DESCRIPTION_DIR = os.path.join(PACKAGE_DIR, '..', 'braco_description')


def floats(text):
    return [float(v) for v in text.split()]


def load_urdf():
    """Process braco_description's braco.urdf.xacro (gazebo:=true) from the source folder."""
    path = os.path.join(DESCRIPTION_DIR, 'urdf', 'braco.urdf.xacro')
    if not os.path.isfile(path):
        pytest.skip('braco_description not found next to braco_simulation')
    with open(path, encoding='utf-8') as f:
        text = f.read().replace('$(find braco_description)', DESCRIPTION_DIR)
    doc = xacro.parse(text)
    xacro.process_doc(doc, mappings={'gazebo': 'true'})
    return ET.fromstring(doc.toxml())


def test_arm_spawns_on_the_lab_bench_table():
    # The model's world frame is the table surface: robot_z must be the height of the table top,
    # and the default robot_x, robot_y must put the mount on one table, flush with its rear edge.
    # The tables are turned 90 deg in the world: their long side runs along x, along the gap.
    with open(os.path.join(PACKAGE_DIR, 'launch', 'gazebo.launch.py'), encoding='utf-8') as f:
        launch = f.read()

    def default(name):
        match = re.search(r"'%s', default_value='([^']+)'" % name, launch)
        assert match, name
        return float(match.group(1))

    table = ET.parse(os.path.join(PACKAGE_DIR, 'models', 'lab_table', 'model.sdf')).getroot()
    top = table.find(".//collision[@name='top_collision']")
    short_side, long_side, thickness = floats(top.find('geometry/box/size').text)
    assert floats(top.find('pose').text)[2] + thickness / 2 == pytest.approx(default('robot_z'))

    world = ET.parse(os.path.join(PACKAGE_DIR, 'worlds', 'lab_bench.sdf')).getroot()
    poses = {i.find('name').text: floats(i.find('pose').text) for i in world.iter('include')}
    centre = poses['lab_table_left']
    assert centre[5] == pytest.approx(math.pi / 2, abs=1e-3)
    assert poses['lab_table_right'][5] == pytest.approx(math.pi / 2, abs=1e-3)
    pedestal = load_urdf().find("link[@name='g_base']/visual[@name='pedestal']/geometry/box")
    pedestal_depth, pedestal_width, _ = floats(pedestal.get('size'))
    assert default('robot_x') - pedestal_depth / 2 == pytest.approx(centre[0] - long_side / 2)
    assert default('robot_y') - pedestal_width / 2 >= centre[1] - short_side / 2
    assert default('robot_y') + pedestal_width / 2 <= centre[1] + short_side / 2


def test_world_runs_the_systems_the_model_needs():
    # #123: the gripper's five mimic joints need a physics engine that enforces <mimic> (Bullet
    # Featherstone; DART does not), and the wrist camera renders only with the Sensors system.
    world = ET.parse(os.path.join(PACKAGE_DIR, 'worlds', 'lab_bench.sdf')).getroot()
    engine = world.find(".//plugin[@name='gz::sim::systems::Physics']/engine/filename")
    assert engine is not None and 'bullet-featherstone' in engine.text
    assert world.find(".//plugin[@name='gz::sim::systems::Sensors']") is not None
