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
Keep the MoveIt config in sync with the model and the code.

Checks that

  * every link and joint config/braco.srdf names exists in the model (config/braco.urdf.xacro)
  * the group `arm` is the chain base_link -> tcp, so it moves exactly joint1..joint6
  * the named states home and park are ZERO_ANGLES and PARK_ANGLES of mycobot.hpp
  * the named states are inside config/joint_limits.yaml
  * config/moveit_controllers.yaml sends to arm_controller, with the joints of
    braco_description/config/controllers.yaml
  * the mock <ros2_control> of the demo has the joints and interfaces of the Gazebo one

joint_limits.yaml against JOINT_LIMITS and the URDF: test/test_joint_limits.py.
"""

import math
import os
import re
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.dirname(PACKAGE_DIR)   # ros2_ws/src (/ws/src in the container)
DESCRIPTION_DIR = os.path.join(SRC_DIR, 'braco_description')
MYCOBOT_HPP = os.environ.get('MYCOBOT_HPP') or os.path.join(
    SRC_DIR, 'mycobot_cpp', 'includes', 'mycobot.hpp')
ARM_JOINTS = ['joint%d' % n for n in range(1, 7)]


def config(name):
    return os.path.join(PACKAGE_DIR, 'config', name)


def load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def process_xacro(path, mappings=None):
    """
    Process a xacro file of this package or braco_description.

    Only the top file's $(find braco_description) is replaced with the source folder: the files
    it includes resolve theirs through the ament index (colcon test sources braco_description).
    """
    with open(path, encoding='utf-8') as f:
        text = f.read().replace('$(find braco_description)', DESCRIPTION_DIR)
    doc = xacro.parse(text)
    xacro.process_doc(doc, mappings=mappings or {})
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope='module')
def model():
    return process_xacro(config('braco.urdf.xacro'))


@pytest.fixture(scope='module')
def srdf():
    return ET.parse(config('braco.srdf')).getroot()


def chain_joints(model, base, tip):
    """Movable joints from base to tip, base first."""
    parent_of = {j.find('child').get('link'): j for j in model.findall('joint')}
    joints, link = [], tip
    while link != base:
        assert link in parent_of, '%s is not below %s' % (tip, base)
        joint = parent_of[link]
        if joint.get('type') != 'fixed':
            joints.append(joint.get('name'))
        link = joint.find('parent').get('link')
    return joints[::-1]


def hpp_angles(name):
    """Return `constexpr Angles <name> = {...}` of mycobot.hpp, in radians."""
    if not os.path.isfile(MYCOBOT_HPP):
        pytest.skip('mycobot.hpp not found (set MYCOBOT_HPP)')
    with open(MYCOBOT_HPP, encoding='utf-8') as f:
        match = re.search(r'constexpr Angles ' + name + r'\s*=\s*\{(.*?)\};', f.read())
    assert match, '%s not found in mycobot.hpp' % name
    return [math.radians(float(v)) for v in match.group(1).split(',')]


def named_state(srdf, name):
    state = srdf.find("group_state[@name='%s']" % name)
    assert state is not None, 'named state %s missing' % name
    assert state.get('group') == 'arm'
    return {j.get('name'): float(j.get('value')) for j in state.findall('joint')}


def test_srdf_names_exist_in_the_model(model, srdf):
    links = {link.get('name') for link in model.findall('link')}
    for pair in srdf.findall('disable_collisions'):
        assert pair.get('link1') in links and pair.get('link2') in links, pair.attrib
    for link in srdf.iter('link'):
        assert link.get('name') in links
    for chain in srdf.iter('chain'):
        assert chain.get('base_link') in links and chain.get('tip_link') in links
    assert srdf.find('end_effector').get('parent_link') in links


def test_arm_group_is_joint1_to_joint6(model, srdf):
    chain = srdf.find("group[@name='arm']/chain")
    assert (chain.get('base_link'), chain.get('tip_link')) == ('base_link', 'tcp')
    assert chain_joints(model, 'base_link', 'tcp') == ARM_JOINTS


@pytest.mark.parametrize('state, constant', [('home', 'ZERO_ANGLES'), ('park', 'PARK_ANGLES')])
def test_named_states_match_mycobot_hpp(srdf, state, constant):
    values = named_state(srdf, state)
    assert list(values) == ARM_JOINTS
    assert list(values.values()) == pytest.approx(hpp_angles(constant), abs=1e-5)


@pytest.mark.parametrize('state', ['home', 'park'])
def test_named_states_are_inside_the_joint_limits(srdf, state):
    limits = load_yaml(config('joint_limits.yaml'))['joint_limits']
    for joint, value in named_state(srdf, state).items():
        assert limits[joint]['min_position'] <= value <= limits[joint]['max_position'], joint


def test_moveit_controllers_match_braco_description():
    moveit = load_yaml(config('moveit_controllers.yaml'))['moveit_simple_controller_manager']
    ros2_control = load_yaml(os.path.join(DESCRIPTION_DIR, 'config', 'controllers.yaml'))
    assert moveit['controller_names'] == ['arm_controller']
    arm = moveit['arm_controller']
    assert arm['type'] == 'FollowJointTrajectory'
    assert arm['action_ns'] == 'follow_joint_trajectory'
    assert ros2_control['controller_manager']['ros__parameters']['arm_controller']['type'] == \
        'joint_trajectory_controller/JointTrajectoryController'
    assert arm['joints'] == ros2_control['arm_controller']['ros__parameters']['joints']


def interfaces(system):
    return {j.get('name'): ([c.get('name') for c in j.findall('command_interface')],
                            [s.get('name') for s in j.findall('state_interface')])
            for j in system.findall('joint')}


def test_mock_system_matches_the_gazebo_one(model):
    systems = model.findall('ros2_control')
    assert len(systems) == 1
    assert systems[0].find('hardware/plugin').text == 'mock_components/GenericSystem'
    gazebo = process_xacro(os.path.join(DESCRIPTION_DIR, 'urdf', 'braco.urdf.xacro'),
                           {'gazebo': 'true'}).find('ros2_control')
    assert list(interfaces(systems[0])) == ARM_JOINTS
    assert interfaces(systems[0]) == interfaces(gazebo)
