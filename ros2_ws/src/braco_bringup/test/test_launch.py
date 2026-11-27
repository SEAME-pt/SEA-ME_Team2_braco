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
braco_moveit_config exist in the PC image only. The arm_driver handlers are driven with
ProcessExited and StateTransition events.
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited, ShutdownProcess
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import StateTransition
from lifecycle_msgs.msg import TransitionEvent
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


def transition(action, start_state, goal_state):
    msg = TransitionEvent()
    msg.start_state.label = start_state
    msg.goal_state.label = goal_state
    return StateTransition(action=action, msg=msg)


def handlers(ld, cls):
    return [r for r in of_type(ld.entities, RegisterEventHandler)
            if isinstance(r.event_handler, cls)]


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
        'spawner', 'joystick_teleop', 'camera']
    # arm_controller only: arm_driver publishes /joint_states, no joint_state_broadcaster (#97)
    (spawner,) = [n for n in nodes if n.node_executable == 'spawner']
    spawner_args = [perform_substitutions(LaunchContext(), part) for part in spawner.cmd[1:]]
    assert spawner_args[0] == 'arm_controller'
    assert 'joint_state_broadcaster' not in spawner_args
    # The real-arm override last: the last param file wins
    files = [spawner_args[i + 1] for i, a in enumerate(spawner_args) if a == '--param-file']
    assert [Path(f).name for f in files] == ['controllers.yaml', 'controllers_real.yaml']
    # The controller manager reads enforce_command_limits at start, from controllers_real.yaml
    # (#157): the spawner cannot set it
    (manager,) = [n for n in nodes if n.node_executable == 'ros2_control_node']
    manager_files = [p.evaluate(LaunchContext()) for p in manager._Node__parameters]
    assert [Path(f).name for f in manager_files] == ['controllers.yaml', 'controllers_real.yaml']
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
    (register,) = handlers(ld, OnProcessExit)
    assert not register.condition.evaluate(configured(sim='true', moveit='false'))
    handler = register.event_handler
    assert handler.matches(exited(arm_driver))
    assert not handler.matches(exited(of_type(ld.entities, Node)[1]))
    (shutdown,) = handler.handle(exited(arm_driver), LaunchContext())
    assert isinstance(shutdown, Shutdown)


def test_arm_driver_is_started_as_a_lifecycle_node(ld):
    """Configure and activate arm_driver at launch (#95)."""
    arm_driver = of_type(ld.entities, Node)[0]
    assert isinstance(arm_driver, LifecycleNode)
    assert perform_typed_substitution(LaunchContext(), arm_driver.node_autostart, bool)


@pytest.mark.parametrize('start_state, goal_state, shuts_down', [
    ('configuring', 'unconfigured', True),     # configure failed: no port
    ('errorprocessing', 'unconfigured', True),
    ('shuttingdown', 'finalized', True),
    ('errorprocessing', 'finalized', True),
    ('deactivating', 'inactive', False),       # a STOP: monitoring goes on
    ('cleaningup', 'unconfigured', False),
    ('configuring', 'inactive', False),
])
def test_arm_driver_unusable_ends_its_process(ld, start_state, goal_state, shuts_down):
    """End arm_driver (so the launch) when left unusable, not on deactivate or cleanup."""
    arm_driver = of_type(ld.entities, Node)[0]
    registers = handlers(ld, OnStateTransition)
    for register in registers:
        assert not register.condition.evaluate(configured(sim='true', moveit='false'))
    event = transition(arm_driver, start_state, goal_state)
    matched = [r.event_handler for r in registers if r.event_handler.matches(event)]
    assert len(matched) == shuts_down
    for handler in matched:
        _, emit = handler.handle(event, LaunchContext())
        # Not a Shutdown: the exit handler sends the only one
        assert isinstance(emit, EmitEvent)
        assert isinstance(emit.event, ShutdownProcess)
