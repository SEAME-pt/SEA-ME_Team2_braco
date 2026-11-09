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
Keep config/joint_limits.yaml in sync with JOINT_LIMITS of mycobot.hpp and the URDF.

Checks per arm joint that

  * the position limits equal JOINT_LIMITS and the URDF <limit>, and are inside the URDF
  * the velocity limit is not above the measured URDF velocity

hpp <-> URDF is checked in braco_description/test/test_urdf.py.
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
JOINT_LIMITS_YAML = os.path.join(PACKAGE_DIR, 'config', 'joint_limits.yaml')
ARM_JOINTS = ['joint%d' % n for n in range(1, 7)]
TOL = 1e-5   # joint_limits.yaml is rounded to 6 decimals


def hpp_limits(name):
    """Return the {min, max} pairs of `constexpr JointLimits <name>` in mycobot.hpp, in degrees."""
    if not os.path.isfile(MYCOBOT_HPP):
        pytest.skip('mycobot.hpp not found (set MYCOBOT_HPP)')
    with open(MYCOBOT_HPP, encoding='utf-8') as f:
        text = f.read()
    block = re.search(name + r'\s*=\s*\{\{(.*?)\}\};', text, re.S)
    assert block, '%s not found in mycobot.hpp' % name
    pairs = re.findall(r'\{\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\}', block.group(1))
    assert len(pairs) == 6
    return [(float(lo), float(hi)) for lo, hi in pairs]


@pytest.fixture(scope='module')
def yaml_limits():
    with open(JOINT_LIMITS_YAML, encoding='utf-8') as f:
        return yaml.safe_load(f)['joint_limits']


@pytest.fixture(scope='module')
def urdf_limits():
    """{joint: <limit> attributes as floats} of the model MoveIt loads."""
    with open(os.path.join(PACKAGE_DIR, 'config', 'braco.urdf.xacro'), encoding='utf-8') as f:
        text = f.read().replace('$(find braco_description)', DESCRIPTION_DIR)
    doc = xacro.parse(text)
    xacro.process_doc(doc)
    robot = ET.fromstring(doc.toxml())
    return {name: {k: float(v) for k, v in robot.find("joint[@name='%s']/limit" % name).items()}
            for name in ARM_JOINTS}


def test_yaml_limits_the_arm_joints(yaml_limits):
    assert sorted(yaml_limits) == ARM_JOINTS
    for joint, limit in yaml_limits.items():
        for flag in ('has_position_limits', 'has_velocity_limits'):
            assert limit.get(flag) is True, (
                '%s %s is not true in joint_limits.yaml' % (joint, flag))


@pytest.mark.parametrize('index, joint', enumerate(ARM_JOINTS))
def test_yaml_positions_match_mycobot_hpp(yaml_limits, index, joint):
    lo, hi = hpp_limits('JOINT_LIMITS')[index]
    limit = yaml_limits[joint]
    for key, deg in (('min_position', lo), ('max_position', hi)):
        assert limit[key] == pytest.approx(math.radians(deg), abs=TOL), (
            '%s %s %.6f in joint_limits.yaml != JOINT_LIMITS %.1f deg (%.6f rad) in mycobot.hpp'
            % (joint, key, limit[key], deg, math.radians(deg)))


@pytest.mark.parametrize('joint', ARM_JOINTS)
def test_yaml_positions_match_urdf(yaml_limits, urdf_limits, joint):
    limit, urdf = yaml_limits[joint], urdf_limits[joint]
    for key, attr in (('min_position', 'lower'), ('max_position', 'upper')):
        assert limit[key] == pytest.approx(urdf[attr], abs=TOL), (
            '%s %s %.6f in joint_limits.yaml != URDF %s %.6f'
            % (joint, key, limit[key], attr, urdf[attr]))
    # MoveIt must never plan outside the URDF limits
    assert urdf['lower'] - TOL <= limit['min_position'] <= limit['max_position'] \
        <= urdf['upper'] + TOL, '%s joint_limits.yaml is outside the URDF limits' % joint


@pytest.mark.parametrize('joint', ARM_JOINTS)
def test_yaml_velocity_not_above_urdf(yaml_limits, urdf_limits, joint):
    yaml_vel, urdf_vel = yaml_limits[joint]['max_velocity'], urdf_limits[joint]['velocity']
    assert yaml_vel <= urdf_vel + TOL, (
        '%s max_velocity %.3f in joint_limits.yaml > measured URDF velocity %.3f'
        % (joint, yaml_vel, urdf_vel))
