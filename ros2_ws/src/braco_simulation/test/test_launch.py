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
The launch descriptions of launch/, built without starting Gazebo.

Each file is loaded from the source tree (the installed copy is not measured by coverage) and
its generate_launch_description() called. gazebo.launch.py builds the simulation in an
OpaqueFunction, after checking for another /controller_manager: the check is replaced to get
either branch, and run for real on its own ROS domain (ament_add_ros_isolated_pytest_test).
The controllers' start-up chain is driven with ProcessExited events.
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.events.process import ProcessExited
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest
import rclpy

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'


def load(name):
    path = LAUNCH_DIR / name
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def of_type(entities, cls):
    return [e for e in entities if isinstance(e, cls)]


def resolve(context, value):
    """Return a string, a substitution or a list of them as text."""
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def node_args(node):
    """Return the node's command line after the executable (before the parameters)."""
    return [perform_substitutions(LaunchContext(), part) for part in node.cmd[1:]]


def executables(entities):
    return [n.node_executable for n in of_type(entities, Node)]


def configured(**values):
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def exited(action, returncode=0):
    return ProcessExited(action=action, name='process', cmd=[], cwd=None, env=None, pid=1,
                         returncode=returncode)


def gz_args(include, context):
    return resolve(context, dict(include.launch_arguments)['gz_args'])


@pytest.fixture
def gazebo():
    return load('gazebo.launch.py')


@pytest.fixture
def gazebo_ld(gazebo):
    return gazebo.generate_launch_description()


@pytest.fixture
def simulation(gazebo, gazebo_ld, monkeypatch):
    """Return what the OpaqueFunction starts when no other controller_manager runs."""
    monkeypatch.setattr(gazebo, 'controller_manager_running', lambda: False)
    (start,) = of_type(gazebo_ld.entities, OpaqueFunction)
    return start.execute(LaunchContext())


def test_gazebo_arguments(gazebo_ld):
    """Declare the window, the world and the spawn pose (values: test_lab_bench.py)."""
    args = {a.name: resolve(LaunchContext(), a.default_value)
            for a in of_type(gazebo_ld.entities, DeclareLaunchArgument)}
    assert list(args) == ['gui', 'world', 'robot_x', 'robot_y', 'robot_z', 'robot_yaw']
    assert args['gui'] == 'true'
    assert args['world'].endswith('braco_simulation/worlds/lab_bench.sdf')


def test_gazebo_refuses_a_second_controller_manager(gazebo, gazebo_ld, monkeypatch):
    """Another controller_manager on the domain: say so and shut down, start nothing."""
    monkeypatch.setattr(gazebo, 'controller_manager_running', lambda: True)
    (start,) = of_type(gazebo_ld.entities, OpaqueFunction)
    log, shutdown = start.execute(LaunchContext())
    assert isinstance(log, LogInfo)
    assert isinstance(shutdown, Shutdown)


def test_gazebo_simulation_nodes(simulation):
    """Start the model, the bridges, the spawner and the window tracking (gui only)."""
    assert len(of_type(simulation, AppendEnvironmentVariable)) == 2
    assert executables(simulation) == [
        'robot_state_publisher', 'parameter_bridge', 'image_bridge', 'create', 'gz_track_arm.sh']
    track = of_type(simulation, Node)[-1]
    assert track.condition.evaluate(configured(gui='true'))
    assert not track.condition.evaluate(configured(gui='false'))


def test_gazebo_server_only_without_gui(simulation):
    """Run gz sim with its window for gui:=true, as a server (-s) for gui:=false."""
    (include,) = of_type(simulation, IncludeLaunchDescription)
    with_gui = gz_args(include, configured(gui='true', world='w.sdf'))
    assert with_gui.startswith('-r -v 1 --gui-config ')
    assert with_gui.endswith('braco_simulation/config/gazebo_gui.config w.sdf')
    assert gz_args(include, configured(gui='false', world='w.sdf')) == '-s ' + with_gui
    assert resolve(LaunchContext(), dict(include.launch_arguments)['on_exit_shutdown']) == 'true'


def test_gazebo_controller_chain(simulation):
    """Spawn, then joint_state_broadcaster, then arm_controller, then the gripper controller."""
    handlers = [r.event_handler for r in of_type(simulation, RegisterEventHandler)]
    (action,) = [n for n in of_type(simulation, Node) if n.node_executable == 'create']
    started = []
    for handler in handlers:
        assert handler.matches(exited(action))
        (action,) = handler.handle(exited(action), LaunchContext())
        started.append(node_args(action)[0])
    assert started == ['joint_state_broadcaster', 'arm_controller', 'gripper_action_controller']


def test_controller_manager_running(gazebo, monkeypatch):
    """Find a /controller_manager on this (isolated) ROS domain, and only then."""
    monkeypatch.setattr(gazebo, 'CHECK_S', 0.3)
    assert not gazebo.controller_manager_running()
    context = rclpy.Context()
    rclpy.init(args=[], context=context)
    try:
        rclpy.create_node('controller_manager', context=context)
        monkeypatch.setattr(gazebo, 'CHECK_S', 10.0)
        assert gazebo.controller_manager_running()
    finally:
        rclpy.shutdown(context=context)


def test_gazebo_gui():
    """Open the Gazebo window (-g) on the running simulation and track the arm."""
    ld = load('gazebo_gui.launch.py').generate_launch_description()
    (include,) = of_type(ld.entities, IncludeLaunchDescription)
    assert gz_args(include, LaunchContext()).startswith('-g --gui-config ')
    assert executables(ld.entities) == ['gz_track_arm.sh']
