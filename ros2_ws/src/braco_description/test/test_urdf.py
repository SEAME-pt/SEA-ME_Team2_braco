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
Keep the URDF and the code in sync.

Processes braco.urdf.xacro and checks that

  * the arm joints are named joint1..joint6, as arm_driver publishes them in /joint_states
  * the <limit> of every joint is JOINT_LIMITS of libs/cpp/includes/mycobot.hpp
  * every link with geometry has a positive mass and a positive-definite inertia
  * the frames the system uses exist (world, base_link, tool0, tcp, camera_optical_frame)
  * every mesh file exists
  * the model's zero pose has not moved by accident (see ZERO_TOOL0)
"""

import math
import os
import re
import xml.etree.ElementTree as ET

import pytest
import xacro

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MYCOBOT_HPP_CANDIDATES = [
    os.environ.get('MYCOBOT_HPP', ''),
    # this repository (ros2_ws/src/braco_description/test -> libs/cpp/includes)
    os.path.join(PACKAGE_DIR, '..', '..', '..', 'libs', 'cpp', 'includes', 'mycobot.hpp'),
    # inside the containers (docker-compose.yml mounts ../libs here)
    '/opt/seame/libs/cpp/includes/mycobot.hpp',
]
ARM_JOINTS = ['joint%d' % n for n in range(1, 7)]

# tool0 position at all joints = 0 (metres, in world), from the vendor joint origins and the
# base height. NOT checked against the real arm yet: that is the zero-pose comparison of
# docs/urdf-sim-to-real.md. This only catches an accidental edit of the origins.
ZERO_TOOL0 = (0.0456, -0.06462, 0.140 + 0.43614)


def load_urdf(gripper_actuated='false'):
    """Process braco.urdf.xacro without needing the workspace to be sourced."""
    path = os.path.join(PACKAGE_DIR, 'urdf', 'braco.urdf.xacro')
    with open(path, encoding='utf-8') as f:
        text = f.read().replace('$(find braco_description)', PACKAGE_DIR)
    doc = xacro.parse(text)
    xacro.process_doc(doc, mappings={'gripper_actuated': gripper_actuated})
    return ET.fromstring(doc.toxml())


def floats(text):
    return [float(v) for v in text.split()]


def hpp_limits(name):
    """Return the {min, max} pairs of `constexpr JointLimits <name>` in mycobot.hpp, in degrees."""
    for candidate in MYCOBOT_HPP_CANDIDATES:
        if candidate and os.path.isfile(candidate):
            with open(candidate, encoding='utf-8') as f:
                text = f.read()
            break
    else:
        pytest.skip('mycobot.hpp not found (set MYCOBOT_HPP)')
    block = re.search(name + r'\s*=\s*\{\{(.*?)\}\};', text, re.S)
    assert block, '%s not found in mycobot.hpp' % name
    pairs = re.findall(r'\{\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\}', block.group(1))
    assert len(pairs) == 6
    return [(float(lo), float(hi)) for lo, hi in pairs]


def test_arm_joints_are_joint1_to_joint6():
    robot = load_urdf()
    revolute = [j.get('name') for j in robot.findall('joint')
                if j.get('type') == 'revolute' and not j.get('name').startswith('gripper')]
    assert revolute == ARM_JOINTS


def test_limits_match_mycobot_hpp():
    robot = load_urdf()
    expected = hpp_limits('JOINT_LIMITS')
    for name, (lo, hi) in zip(ARM_JOINTS, expected):
        limit = robot.find("joint[@name='%s']/limit" % name)
        assert float(limit.get('lower')) == pytest.approx(math.radians(lo), abs=1e-9), name
        assert float(limit.get('upper')) == pytest.approx(math.radians(hi), abs=1e-9), name


def test_every_link_with_geometry_has_positive_mass_and_inertia():
    robot = load_urdf()
    for link in robot.findall('link'):
        name = link.get('name')
        if link.find('visual') is None and link.find('collision') is None:
            continue
        inertial = link.find('inertial')
        assert inertial is not None, '%s has geometry but no <inertial>' % name
        assert float(inertial.find('mass').get('value')) > 0.0, name
        i = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
        # positive definite: every leading minor > 0 (matrix is symmetric)
        m = [[i['ixx'], i['ixy'], i['ixz']],
             [i['ixy'], i['iyy'], i['iyz']],
             [i['ixz'], i['iyz'], i['izz']]]
        det2 = m[0][0] * m[1][1] - m[0][1] ** 2
        det3 = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] ** 2)
                - m[0][1] * (m[0][1] * m[2][2] - m[1][2] * m[0][2])
                + m[0][2] * (m[0][1] * m[1][2] - m[1][1] * m[0][2]))
        assert m[0][0] > 0 and det2 > 0 and det3 > 0, '%s inertia is not positive definite' % name


@pytest.mark.parametrize('frame', ['world', 'base_link', 'link6', 'tool0', 'tcp',
                                   'camera_mount', 'camera_link', 'camera_optical_frame'])
def test_frame_exists(frame):
    robot = load_urdf()
    assert robot.find("link[@name='%s']" % frame) is not None


def test_tree_is_connected():
    """Every link except `world` is the child of exactly one joint (a single tree)."""
    robot = load_urdf()
    links = {link.get('name') for link in robot.findall('link')}
    children = [j.find('child').get('link') for j in robot.findall('joint')]
    assert len(children) == len(set(children))
    assert links - set(children) == {'world'}


def test_meshes_exist():
    robot = load_urdf()
    prefix = 'package://braco_description/'
    meshes = {m.get('filename') for m in robot.iter('mesh')}
    assert meshes
    for filename in meshes:
        assert filename.startswith(prefix), filename
        assert os.path.isfile(os.path.join(PACKAGE_DIR, filename[len(prefix):])), filename


def test_actuated_gripper_has_one_master_and_five_mimics():
    robot = load_urdf(gripper_actuated='true')
    mimics = robot.findall('.//mimic')
    assert len(mimics) == 5
    assert {m.get('joint') for m in mimics} == {'gripper_controller'}
    assert robot.find("joint[@name='gripper_controller']").get('type') == 'revolute'


def test_fixed_gripper_has_no_movable_finger_joints():
    robot = load_urdf(gripper_actuated='false')
    assert robot.findall('.//mimic') == []
    movable = [j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed']
    assert movable == ARM_JOINTS


def rotation(r, p, y):
    """URDF rpy (fixed axes x, y, z) as a 3x3 matrix: Rz(y) * Ry(p) * Rx(r)."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def test_zero_pose_tool0_position_is_unchanged():
    """Check the forward kinematics at all joints = 0, down to tool0.

    All joint angles are zero, so only the fixed origins of the chain count.
    """
    robot = load_urdf()
    parent_of = {j.find('child').get('link'): j for j in robot.findall('joint')}
    chain, link = [], 'tool0'
    while link in parent_of:
        chain.append(parent_of[link])
        link = parent_of[link].find('parent').get('link')
    rot = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    pos = [0.0, 0.0, 0.0]
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = floats(origin.get('xyz'))
        rpy = floats(origin.get('rpy'))
        pos = [pos[i] + sum(rot[i][k] * xyz[k] for k in range(3)) for i in range(3)]
        local = rotation(*rpy)
        rot = [[sum(rot[i][k] * local[k][j] for k in range(3)) for j in range(3)]
               for i in range(3)]
    for got, want in zip(pos, ZERO_TOOL0):
        assert got == pytest.approx(want, abs=1e-3)
