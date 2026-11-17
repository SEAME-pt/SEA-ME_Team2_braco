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
The launch description of launch/move_to_pose.launch.py, built without starting anything.

It builds one move_to_pose node for the simulation and one for the real arm (real:=true).

The file is loaded from the source tree (the installed copy is not measured by coverage) and
its generate_launch_description() called: MoveItConfigsBuilder runs xacro and reads the share
dirs of braco_description and braco_moveit_config. The Node keeps its parameters only in a
private, normalized form: the module's Node is wrapped to record the arguments it is given.
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'move_to_pose.launch.py'


@pytest.fixture(scope='module')
def launch():
    """Return the launch description and the keyword arguments of each Node it built."""
    spec = importlib.util.spec_from_file_location('move_to_pose_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []

    def recording_node(**kwargs):
        calls.append(kwargs)
        return Node(**kwargs)

    module.Node = recording_node
    return module.generate_launch_description(), calls


def own_parameters(kwargs):
    """Return the node's own parameters: the last dict, after the MoveIt config."""
    return kwargs['parameters'][-1]


def all_parameters(kwargs):
    """Return every parameter given to the node, in one dict."""
    return {k: v for p in kwargs['parameters'] for k, v in p.items()}


def test_arguments(launch):
    """Declare real (simulation by default), the velocity scaling and the tolerances."""
    ld, _ = launch
    args = {a.name: perform_substitutions(LaunchContext(), a.default_value)
            for a in ld.entities if isinstance(a, DeclareLaunchArgument)}
    assert args == {'real': 'false', 'velocity_scaling': '0.1', 'position_tolerance_m': '0.005',
                    'orientation_tolerance_deg': '2.0'}


def test_two_move_to_pose_nodes(launch):
    """Build move_to_pose twice, after the arguments: real:=true starts one, false the other."""
    ld, calls = launch
    nodes = [e for e in ld.entities if isinstance(e, Node)]
    assert ld.entities[-2:] == nodes and len(calls) == 2
    assert all(n.node_package == 'braco_mission' for n in nodes)
    assert all(n.node_executable == 'move_to_pose' for n in nodes)
    for real, started in (('false', [True, False]), ('true', [False, True])):
        context = LaunchContext()
        context.launch_configurations['real'] = real
        assert [n.condition.evaluate(context) for n in nodes] == started


def test_simulation_model(launch):
    """Give the simulation node the Gazebo arm's model (actuated gripper) and its MoveIt config."""
    _, (sim, _) = launch
    params = all_parameters(sim)
    # gazebo:=true adds the Gazebo ros2_control system, gripper_actuated:=true a moving gripper
    assert 'gz_ros2_control/GazeboSimSystem' in params['robot_description']
    assert '<joint name="gripper_controller" type="revolute">' in params['robot_description']
    assert '<group name="arm">' in params['robot_description_semantic']
    assert 'arm' in params['robot_description_kinematics']
    assert 'robot_description_planning' in params


def test_real_arm_model(launch):
    """Give the real-arm node the Jetson's model: braco_hardware, fingers fixed."""
    _, (_, real) = launch
    params = all_parameters(real)
    assert 'braco_hardware/BracoSystem' in params['robot_description']
    assert 'GazeboSimSystem' not in params['robot_description']
    assert '<joint name="gripper_controller" type="revolute">' not in params['robot_description']
    assert '<group name="arm">' in params['robot_description_semantic']


def test_simulation_parameters(launch):
    """Run on simulated time without arm_driver; the tolerances come from the arguments."""
    _, (sim, _) = launch
    own = own_parameters(sim)
    assert own['use_sim_time'] is True
    assert own['use_control'] is False
    context = LaunchContext()
    context.launch_configurations.update(velocity_scaling='0.05', position_tolerance_m='0.01',
                                         orientation_tolerance_deg='5')
    assert {k: own[k].evaluate(context) for k in
            ('velocity_scaling', 'position_tolerance_m', 'orientation_tolerance_deg')} == {
        'velocity_scaling': 0.05, 'position_tolerance_m': 0.01, 'orientation_tolerance_deg': 5.0}


def test_real_arm_parameters(launch):
    """Take the control as mission (ADR-0004) on the system clock."""
    _, (_, real) = launch
    own = own_parameters(real)
    assert own['use_sim_time'] is False
    assert own['use_control'] is True
