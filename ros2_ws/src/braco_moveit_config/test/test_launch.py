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
The launch descriptions of launch/, built without starting anything.

Each file is loaded from the source tree (the installed copy is not measured by coverage) and
its generate_launch_description() called: MoveItConfigsBuilder runs xacro and reads the share
dirs of braco_description and this package. The exit handlers are driven with a ProcessExited
event: a spawner that succeeds starts the domain check (#103), one that fails shuts the launch
down; a check that passes starts move_group and RViz, one that fails shuts the launch down.
"""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown)
from launch.events.process import ProcessExited
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest

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


def arguments(ld):
    """Return the declared launch arguments and their defaults."""
    return {a.name: perform_substitutions(LaunchContext(), a.default_value)
            for a in of_type(ld.entities, DeclareLaunchArgument)}


def node_args(node):
    """Return the node's command line after the executable (before the parameters)."""
    return [perform_substitutions(LaunchContext(), part) for part in node.cmd[1:]]


def handlers(ld):
    return [r.event_handler for r in of_type(ld.entities, RegisterEventHandler)]


def exited(action, returncode):
    return ProcessExited(action=action, name='spawner', cmd=[], cwd=None, env=None, pid=1,
                         returncode=returncode)


def executables(entities):
    return [n.node_executable for n in of_type(entities, Node)]


def rviz_enabled(node, value):
    context = LaunchContext()
    context.launch_configurations['rviz'] = value
    return node.condition.evaluate(context)


@pytest.fixture(scope='module')
def sim():
    return load('sim.launch.py')


@pytest.fixture(scope='module')
def sim_ld(sim):
    return sim.generate_launch_description()


@pytest.fixture(scope='module')
def demo_ld():
    return load('demo.launch.py').generate_launch_description()


def spawner(command):
    """Stand in for the spawner gazebo.launch.py starts, as the handler sees it."""
    return SimpleNamespace(process_description=SimpleNamespace(final_cmd=command))


def test_sim_arguments(sim, sim_ld):
    """Default both gui and rviz to true, and the domain check to the repository's script."""
    assert arguments(sim_ld) == {'gui': 'true', 'rviz': 'true', 'domain_check': sim.DOMAIN_CHECK}


def test_sim_includes_gazebo_with_gui(sim_ld):
    """The Gazebo launch of braco_simulation is included and gets the gui argument."""
    (include,) = of_type(sim_ld.entities, IncludeLaunchDescription)
    context = LaunchContext()
    context.launch_configurations['gui'] = 'false'
    # The location is only resolved (to the installed file) when the description is loaded
    source = include.launch_description_source
    assert source.get_launch_description(context).entities
    assert source.location.endswith('braco_simulation/launch/gazebo.launch.py')
    launch_arguments = {resolve(context, k): resolve(context, v)
                        for k, v in include.launch_arguments}
    assert launch_arguments == {'gui': 'false'}


def test_sim_spawner_matcher(sim):
    """Only the arm_controller spawner is matched, by its command line."""
    assert sim.is_arm_controller_spawner(spawner(['spawner', 'arm_controller']))
    assert not sim.is_arm_controller_spawner(spawner(['spawner', 'joint_state_broadcaster']))


def test_sim_handler_matches_arm_controller_exit(sim_ld):
    """The exit handler reacts to the arm_controller spawner only."""
    (handler,) = handlers(sim_ld)
    assert handler.matches(exited(spawner(['spawner', 'arm_controller']), 0))
    assert not handler.matches(exited(spawner(['spawner', 'joint_state_broadcaster']), 0))


def check_command(check, script='/check.sh'):
    """Return the domain check's command line, with the domain_check argument set to `script`."""
    context = LaunchContext()
    context.launch_configurations['domain_check'] = script
    return [perform_substitutions(context, part) for part in check.cmd]


def sim_check(sim_ld):
    """arm_controller active: return the domain check it starts and the handler of its exit."""
    (handler,) = handlers(sim_ld)
    started = handler.handle(exited(spawner(['spawner', 'arm_controller']), 0), LaunchContext())
    assert executables(started) == []   # nothing of MoveIt before the check (#103)
    (check,) = of_type(started, ExecuteProcess)
    (check_handler,) = [r.event_handler for r in of_type(started, RegisterEventHandler)]
    return check, check_handler


def test_sim_controller_up_runs_the_domain_check(sim_ld):
    """arm_controller active: the domain check runs first, with --target sim."""
    check, check_handler = sim_check(sim_ld)
    assert check_command(check) == ['bash', '/check.sh', '--target', 'sim']
    assert check_handler.matches(exited(check, 0))


def test_sim_check_pass_starts_moveit(sim_ld):
    """Domain check PASS: move_group, RViz (rviz arg) and the lab bench scene start."""
    check, check_handler = sim_check(sim_ld)
    started = check_handler.handle(exited(check, 0), LaunchContext())
    assert executables(started) == ['move_group', 'rviz2', 'lab_bench_scene.py']
    rviz = of_type(started, Node)[1]
    assert rviz_enabled(rviz, 'true')
    assert not rviz_enabled(rviz, 'false')


@pytest.mark.parametrize('returncode', [1, 2])
def test_sim_check_fail_shuts_down(sim_ld, returncode):
    """Domain check FAIL (1) or could not run (2): no move_group, the launch shuts down."""
    check, check_handler = sim_check(sim_ld)
    started = check_handler.handle(exited(check, returncode), LaunchContext())
    assert len(started) == 1
    assert isinstance(started[0], Shutdown)


def test_sim_controller_failed_shuts_down(sim_ld):
    """arm_controller failed: nothing can execute, so the launch shuts down."""
    (handler,) = handlers(sim_ld)
    started = handler.handle(exited(spawner(['spawner', 'arm_controller']), 1), LaunchContext())
    assert len(started) == 1
    assert isinstance(started[0], Shutdown)


@pytest.fixture(scope='module')
def real():
    return load('real.launch.py')


@pytest.fixture(scope='module')
def real_ld(real):
    return real.generate_launch_description()


def real_check(real_ld):
    """Return the domain check real.launch.py starts at once and the handler of its exit."""
    (check,) = of_type(real_ld.entities, ExecuteProcess)
    (check_handler,) = handlers(real_ld)
    return check, check_handler


def test_real_arguments(real, real_ld):
    """Default rviz to true and the domain check to the repository's script."""
    assert arguments(real_ld) == {'rviz': 'true', 'domain_check': real.DOMAIN_CHECK}


def test_real_runs_only_the_domain_check_first(real_ld):
    """Nothing of MoveIt at start: only the domain check, with --target real (#103)."""
    assert executables(real_ld.entities) == []
    check, check_handler = real_check(real_ld)
    assert check_command(check) == ['bash', '/check.sh', '--target', 'real']
    assert check_handler.matches(exited(check, 0))


def test_real_check_pass_starts_moveit(real_ld):
    """Domain check PASS: move_group and RViz (rviz arg) start, no lab bench scene."""
    check, check_handler = real_check(real_ld)
    started = check_handler.handle(exited(check, 0), LaunchContext())
    assert executables(started) == ['move_group', 'rviz2']
    rviz = of_type(started, Node)[1]
    assert rviz_enabled(rviz, 'true')
    assert not rviz_enabled(rviz, 'false')


@pytest.mark.parametrize('returncode', [1, 2])
def test_real_check_fail_shuts_down(real_ld, returncode):
    """Domain check FAIL (1) or could not run (2): no move_group, the launch shuts down."""
    check, check_handler = real_check(real_ld)
    started = check_handler.handle(exited(check, returncode), LaunchContext())
    assert len(started) == 1
    assert isinstance(started[0], Shutdown)


def test_demo_arguments_and_nodes(demo_ld):
    """Default rviz to true; start the model, ros2_control and the broadcaster spawner at once."""
    assert arguments(demo_ld) == {'rviz': 'true'}
    assert executables(demo_ld.entities) == [
        'robot_state_publisher', 'ros2_control_node', 'spawner']
    assert node_args(of_type(demo_ld.entities, Node)[2])[0] == 'joint_state_broadcaster'


def test_demo_startup_chain(demo_ld):
    """joint_state_broadcaster, then arm_controller, then move_group and RViz."""
    broadcaster = of_type(demo_ld.entities, Node)[2]
    first, second = handlers(demo_ld)
    assert first.matches(exited(broadcaster, 0))
    (arm_controller,) = first.handle(exited(broadcaster, 0), LaunchContext())
    assert node_args(arm_controller)[0] == 'arm_controller'
    assert not second.matches(exited(broadcaster, 0))
    assert second.matches(exited(arm_controller, 0))
    started = second.handle(exited(arm_controller, 0), LaunchContext())
    assert executables(started) == ['move_group', 'rviz2']
    rviz = started[1]
    assert rviz_enabled(rviz, 'true')
    assert not rviz_enabled(rviz, 'false')


def test_demo_controller_failed_shuts_down(demo_ld):
    """arm_controller failed: the launch shuts down."""
    broadcaster = of_type(demo_ld.entities, Node)[2]
    first, second = handlers(demo_ld)
    (arm_controller,) = first.handle(exited(broadcaster, 0), LaunchContext())
    started = second.handle(exited(arm_controller, 1), LaunchContext())
    assert len(started) == 1
    assert isinstance(started[0], Shutdown)


def test_rviz_node():
    """One RViz node with the MotionPlanning config of this package."""
    ld = load('rviz.launch.py').generate_launch_description()
    (rviz,) = of_type(ld.entities, Node)
    assert len(ld.entities) == 1
    assert rviz.node_executable == 'rviz2'
    args = node_args(rviz)
    assert args[0] == '-d'
    assert args[1].endswith('braco_moveit_config/config/moveit.rviz')
