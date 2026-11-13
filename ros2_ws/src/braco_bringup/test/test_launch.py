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
The launch description of launch/braco.launch.py, built without starting anything.

The file is loaded from the source tree (the installed copy is not measured by coverage) and
its generate_launch_description() called. The conditions are evaluated for each sim/moveit
combination. The included launch files are only named, not loaded: braco_simulation and
braco_moveit_config exist in the PC image only. The arm_driver exit handler is driven with a
ProcessExited event.
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown)
from launch.events.process import ProcessExited
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'braco.launch.py'


def of_type(entities, cls):
    return [e for e in entities if isinstance(e, cls)]


def resolve(context, value):
    """Return a string, a substitution or a list of them as text."""
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def configured(sim, moveit):
    context = LaunchContext()
    context.launch_configurations.update(sim=sim, moveit=moveit)
    return context


def exited(action):
    return ProcessExited(action=action, name='arm_driver', cmd=[], cwd=None, env=None, pid=1,
                         returncode=1)


@pytest.fixture(scope='module')
def ld():
    spec = importlib.util.spec_from_file_location('braco_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


@pytest.fixture(scope='module')
def includes(ld):
    """Return the Gazebo and the MoveIt includes."""
    return of_type(ld.entities, IncludeLaunchDescription)


def test_arguments(ld):
    """Default to the real arm, with the Jetson's wiring and no window."""
    args = {a.name: resolve(LaunchContext(), a.default_value)
            for a in of_type(ld.entities, DeclareLaunchArgument)}
    assert args == {'sim': 'false', 'port': '/dev/ttyTHS1', 'baud': '1000000',
                    'lcd_bus': '/dev/i2c-0', 'device': '/dev/input/js0', 'camera': '/dev/video0',
                    'gui': 'false', 'moveit': 'false', 'rviz': 'false'}


def test_real_nodes(ld):
    """Start the driver, the monitor, ros2_control, teleop and the camera on the real arm only."""
    nodes = of_type(ld.entities, Node)
    assert [n.node_executable for n in nodes] == [
        'arm_driver', 'temperature_monitor', 'robot_state_publisher', 'ros2_control_node',
        'joystick_teleop', 'camera']
    for node in nodes:
        assert node.condition.evaluate(configured(sim='false', moveit='false'))
        assert not node.condition.evaluate(configured(sim='true', moveit='false'))


@pytest.mark.parametrize('sim, moveit, started', [
    ('false', 'false', []),
    ('false', 'true', []),
    ('true', 'false', ['gazebo']),
    ('true', 'true', ['moveit_sim']),
])
def test_simulation_includes(includes, sim, moveit, started):
    """Include Gazebo alone for sim:=true, the MoveIt simulation (with Gazebo) for moveit:=true."""
    context = configured(sim=sim, moveit=moveit)
    names = dict(zip(['gazebo', 'moveit_sim'], includes))
    assert [n for n, i in names.items() if i.condition.evaluate(context)] == started


def test_simulation_include_arguments(includes):
    """Pass gui to Gazebo; gui and rviz to the MoveIt simulation."""
    context = LaunchContext()
    context.launch_configurations.update(gui='true', rviz='false')
    gazebo, moveit_sim = includes
    # location is the PathJoinSubstitution's description until the include is loaded
    assert "'braco_simulation'), 'launch', 'gazebo.launch.py'" in (
        gazebo.launch_description_source.location)
    assert "'braco_moveit_config'), 'launch', 'sim.launch.py'" in (
        moveit_sim.launch_description_source.location)
    assert {k: resolve(context, v) for k, v in gazebo.launch_arguments} == {'gui': 'true'}
    assert {k: resolve(context, v) for k, v in moveit_sim.launch_arguments} == {
        'gui': 'true', 'rviz': 'false'}


def test_arm_driver_exit_shuts_down(ld):
    """End the whole launch when arm_driver exits, on the real arm only."""
    arm_driver = of_type(ld.entities, Node)[0]
    (register,) = of_type(ld.entities, RegisterEventHandler)
    assert not register.condition.evaluate(configured(sim='true', moveit='false'))
    handler = register.event_handler
    assert handler.matches(exited(arm_driver))
    assert not handler.matches(exited(of_type(ld.entities, Node)[1]))
    (shutdown,) = handler.handle(exited(arm_driver), LaunchContext())
    assert isinstance(shutdown, Shutdown)
