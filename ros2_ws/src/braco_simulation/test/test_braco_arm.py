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
Unit tests of scripts/braco_arm.py.

ROS is faked (node, executor, clients, action client, clock): no DDS, no ROS domain, no wait.
Each executor spin advances the fake clock and delivers /joint_states and /arm/stopped.
"""

import importlib
import math
import os
import sys
import types

from braco_interfaces.msg import ArmCommand
import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'scripts'))
braco_arm = importlib.import_module('braco_arm')

ZERO = [0.0] * 6


class Future:

    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._result


class Logger:

    def __init__(self):
        self.records = []

    def error(self, msg):
        self.records.append(('error', msg))

    def warning(self, msg):
        self.records.append(('warning', msg))

    def info(self, msg):
        self.records.append(('info', msg))

    def has(self, level, text):
        return any(lv == level and text in msg for lv, msg in self.records)


class Client:
    """A service client; world.services[name] sets ready / done / success / message."""

    def __init__(self, world, name):
        self.world = world
        self.srv_name = name
        self.requests = []

    def _cfg(self):
        return {'ready': True, 'done': True, 'success': True, 'message': '',
                **self.world.services.get(self.srv_name, {})}

    def service_is_ready(self):
        return self._cfg()['ready']

    def wait_for_service(self, timeout_sec=None):
        return self._cfg()['ready']

    def call_async(self, request):
        self.requests.append(request)
        cfg = self._cfg()
        return Future(types.SimpleNamespace(success=cfg['success'], message=cfg['message']),
                      cfg['done'])


class Publisher:

    def __init__(self, world):
        self.world = world

    def publish(self, msg):
        self.world.commands.append(msg)
        if self.world.follow and msg.type == ArmCommand.ALL_ANGLES:
            self.world.joints = list(msg.angles_deg)


class Node:

    def __init__(self, world, name):
        self.world = world
        self.name = name
        self.subs = {}
        self.destroyed = False
        world.nodes.append(self)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs[topic] = callback

    def create_publisher(self, msg_type, topic, qos):
        return Publisher(self.world)

    def create_client(self, srv_type, name):
        client = Client(self.world, name)
        self.world.clients[name] = client
        return client

    def get_logger(self):
        return self.world.log

    def count_publishers(self, topic):
        return self.world.clocks

    def destroy_node(self):
        self.destroyed = True


class Executor:

    def __init__(self, world):
        self.world = world
        self.node = None
        self.shut = False

    def add_node(self, node):
        self.node = node

    def spin_once(self, timeout_sec=0.0):
        self.world.tick(self.node, timeout_sec)

    def spin_until_future_complete(self, future, timeout_sec=0.0):
        self.world.tick(self.node, 0.0 if future.done() else timeout_sec)

    def shutdown(self):
        self.shut = True


class ActionClient:
    """FollowJointTrajectory; world.action sets server / accepted / goal_done / ..."""

    def __init__(self, world):
        self.world = world
        self.goals = []

    def wait_for_server(self, timeout_sec=None):
        return self.world.action['server']

    def send_goal_async(self, goal):
        self.goals.append(goal)
        cfg = self.world.action
        wrapped = types.SimpleNamespace(error_code=cfg['error_code'])
        result = Future(types.SimpleNamespace(result=wrapped), cfg['result_done'])
        handle = types.SimpleNamespace(accepted=cfg['accepted'], get_result_async=lambda: result)
        return Future(handle, cfg['goal_done'])


class World:
    """The fake clock, the arm's joints and every fake ROS object the arms create."""

    def __init__(self):
        self.t = 1000.0
        self.joints = list(ZERO)  # degrees published on /joint_states; None = nothing published
        self.follow = True        # the arm goes to every ALL_ANGLES target at once
        self.stopped = None       # Bool on arm/stopped; None = nothing published
        self.ok = True
        self.clocks = 1
        self.services = {}
        self.action = {'server': True, 'accepted': True, 'goal_done': True,
                       'result_done': True, 'error_code': 0}
        self.hooks = []           # f(world), called on every spin
        self.commands = []
        self.clients = {}
        self.nodes = []
        self.executors = []
        self.action_clients = []
        self.log = Logger()
        self.shutdowns = 0

    def tick(self, node, seconds):
        self.t += seconds
        for hook in self.hooks:
            hook(self)
        if self.joints is not None and 'joint_states' in node.subs:
            node.subs['joint_states'](JointState(
                name=list(braco_arm.JOINTS), position=[math.radians(d) for d in self.joints]))
        if self.stopped is not None and 'arm/stopped' in node.subs:
            node.subs['arm/stopped'](Bool(data=self.stopped))

    def halts(self):
        return [c for c in self.commands if c.type == ArmCommand.HALT]

    def targets(self):
        return [list(c.angles_deg) for c in self.commands if c.type == ArmCommand.ALL_ANGLES]


@pytest.fixture
def world(monkeypatch):
    w = World()

    def shutdown(context=None):
        w.shutdowns += 1

    def executor(context=None):
        e = Executor(w)
        w.executors.append(e)
        return e

    def action_client(node, action_type, name):
        a = ActionClient(w)
        w.action_clients.append(a)
        return a

    monkeypatch.setattr(braco_arm, 'rclpy', types.SimpleNamespace(
        Context=object, init=lambda context=None, domain_id=None: None, shutdown=shutdown,
        ok=lambda context=None: w.ok))
    monkeypatch.setattr(braco_arm, 'Node', lambda name, context=None: Node(w, name))
    monkeypatch.setattr(braco_arm, 'SingleThreadedExecutor', executor)
    monkeypatch.setattr(braco_arm, 'ActionClient', action_client)
    monkeypatch.setattr(braco_arm, 'time', types.SimpleNamespace(monotonic=lambda: w.t))
    return w


def at(seconds, action):
    """Run action(world) once, `seconds` after the hook is added."""
    state = {}

    def hook(w):
        state.setdefault('start', w.t)
        if not state.get('done') and w.t - state['start'] >= seconds:
            state['done'] = True
            action(w)
    return hook


# _Arm

def test_joint_states_are_converted_to_degrees(world):
    arm = braco_arm.SimArm('t', 27)
    world.joints = [10.0, -20.0, 30.0, 0.0, 45.0, -90.0]
    arm.spin(0.01)
    assert arm.fresh_angles() == pytest.approx(world.joints)


def test_joint_states_missing_a_joint_are_ignored(world):
    arm = braco_arm.SimArm('t', 27)
    arm._on_joints(JointState(name=['joint1'], position=[0.1]))
    assert arm.angles is None
    assert arm.fresh_angles() is None


def test_stale_joint_states_are_not_fresh(world):
    arm = braco_arm.SimArm('t', 27)
    arm.spin(0.01)
    world.joints = None
    world.t += braco_arm.STATE_TIMEOUT_S + 0.1
    assert arm.angles is not None
    assert arm.fresh_angles() is None


def test_pause_waits_while_spinning(world):
    arm = braco_arm.SimArm('t', 27)
    start = world.t
    arm.pause(0.3)
    assert world.t - start == pytest.approx(0.3)
    assert arm.fresh_angles() is not None


def test_close_shuts_everything_down(world):
    arm = braco_arm.SimArm('t', 27)
    arm.close()
    assert world.executors[0].shut
    assert world.nodes[0].destroyed
    assert world.shutdowns == 1


# SimArm

def test_sim_prepare_without_controller_fails(world):
    world.action['server'] = False
    assert not braco_arm.SimArm('t', 27).prepare()
    assert world.log.has('error', 'arm_controller not found on ROS domain 27')


def test_sim_prepare_refuses_two_simulations(world):
    world.clocks = 2
    assert not braco_arm.SimArm('t', 27).prepare()
    assert world.log.has('error', '2 Gazebo simulations on ROS domain 27')


def test_sim_prepare(world):
    assert braco_arm.SimArm('t', 27).prepare()


def test_sim_move_sends_one_goal_in_radians(world):
    arm = braco_arm.SimArm('t', 27)
    target = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    world.joints = list(target)
    assert arm.move(target, 2.5)
    goal = world.action_clients[0].goals[0]
    assert goal.trajectory.joint_names == braco_arm.JOINTS
    point = goal.trajectory.points[0]
    assert list(point.positions) == pytest.approx([math.radians(d) for d in target])
    assert (point.time_from_start.sec, point.time_from_start.nanosec) == (2, 500000000)


@pytest.mark.parametrize('failure', [
    {'goal_done': False}, {'accepted': False}, {'result_done': False}, {'error_code': -4}])
def test_sim_move_fails_when_the_controller_does(world, failure):
    world.action.update(failure)
    assert not braco_arm.SimArm('t', 27).move(ZERO, 1.0)


def test_sim_move_fails_when_the_arm_lags_behind(world):
    arm = braco_arm.SimArm('t', 27)
    assert not arm.move([30.0] * 6, 1.0)
    assert world.log.has('error', 'did not reach the pose')


def test_sim_move_without_joint_states_fails(world):
    world.joints = None
    assert not braco_arm.SimArm('t', 27).move(ZERO, 1.0)
    assert world.log.has('error', 'at None')


def test_sim_stop_does_nothing(world):
    braco_arm.SimArm('t', 27).stop()
    assert world.commands == []


# RealArm: services

def real(world, speed=30):
    return braco_arm.RealArm('t', 22, speed)


def test_call_without_the_service_fails(world):
    world.services['arm/power_on'] = {'ready': False}
    arm = real(world)
    assert not arm._call(arm.power_client, None, 'power on')
    assert world.log.has('error', 'power on failed: arm_driver has no arm/power_on service')


def test_call_timeout_fails(world):
    world.services['arm/power_on'] = {'done': False}
    arm = real(world)
    assert not arm._call(arm.power_client, None, 'power on')
    assert world.log.has('error', 'power on failed: timeout')


def test_call_refused_logs_the_reason(world):
    world.services['arm/power_on'] = {'success': False, 'message': 'rest mode'}
    arm = real(world)
    assert not arm._call(arm.power_client, None, 'power on')
    assert world.log.has('error', 'power on failed: rest mode')


def test_set_rest_and_release_servos(world):
    arm = real(world)
    assert arm.set_rest(True)
    assert world.clients['arm/set_read_only'].requests[-1].data is True
    assert arm.set_rest(False)
    assert world.clients['arm/set_read_only'].requests[-1].data is False
    assert arm.release_servos()
    assert len(world.clients['arm/release_servos'].requests) == 1


def test_take_control_is_forced_as_script(world):
    arm = real(world)
    assert arm.take_control()
    request = world.clients['arm/take_control'].requests[0]
    assert (request.source, request.force, request.release) == ('script', True, False)
    assert arm.has_control


def test_release_control_only_when_held(world):
    arm = real(world)
    arm.release_control()
    assert world.clients['arm/take_control'].requests == []
    arm.take_control()
    arm.release_control()
    request = world.clients['arm/take_control'].requests[-1]
    assert (request.source, request.release) == ('script', True)
    assert not arm.has_control


def test_real_prepare_without_driver_fails(world):
    world.joints = None
    assert not real(world).prepare()
    assert world.log.has('error', 'arm_driver not found on ROS domain 22')


def test_real_prepare_leaves_rest_takes_control_and_powers_on(world):
    arm = real(world)
    assert arm.prepare()
    assert world.clients['arm/set_read_only'].requests[0].data is False
    assert arm.has_control
    assert len(world.clients['arm/power_on'].requests) == 1


def test_real_prepare_fails_if_power_on_is_refused(world):
    world.services['arm/power_on'] = {'success': False, 'message': 'no'}
    assert not real(world).prepare()


# RealArm: move

def test_real_move_steps_toward_the_target(world):
    target = [30.0, -45.0, 60.0, 0.0, 10.0, 0.0]
    assert real(world, speed=25).move(target)
    previous = list(ZERO)
    for step in world.targets():
        assert max(abs(s - p) for s, p in zip(step, previous)) <= braco_arm.STEP_DEG + 0.1
        previous = step
    assert previous == pytest.approx(target, abs=braco_arm.TOLERANCE_DEG)
    assert all(c.speed == 25 and c.source == 'script' for c in world.commands)
    assert world.halts() == []


def test_real_move_at_the_target_sends_nothing(world):
    assert real(world).move(ZERO)
    assert world.commands == []


def test_real_move_refused_control_fails(world):
    world.services['arm/take_control'] = {'success': False, 'message': 'STOP latched'}
    assert not real(world).move([30.0] * 6)
    assert world.commands == []


def test_real_move_gives_up_on_a_stop_latch_without_halt(world):
    world.follow = False

    def latch(w):
        w.stopped = True
    world.hooks.append(at(0.5, latch))
    assert not real(world).move([30.0] * 6)
    assert world.log.has('error', 'STOP latched (make resume)')
    assert world.halts() == []


def test_real_move_halts_without_joint_states(world):
    world.joints = None
    assert not real(world).move([30.0] * 6)
    assert len(world.halts()) == 1
    assert world.log.has('error', 'no /joint_states from arm_driver')
    assert world.log.has('error', 'arm at None')


def test_real_move_halts_when_the_arm_stalls(world):
    world.follow = False
    assert not real(world).move([30.0] * 6)
    assert len(world.halts()) == 1
    assert world.halts()[0].source == 'script'
    assert world.log.has('error', 'no progress for 5 s')


def test_real_move_stalled_in_the_dead_zone_is_close_enough(world):
    world.follow = False
    world.joints = [2.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert real(world).move(ZERO)
    assert world.log.has('warning', '2.5 deg from the target')
    assert world.halts() == []


def test_real_move_halts_after_the_move_timeout(world, monkeypatch):
    monkeypatch.setattr(braco_arm, 'MOVE_TIMEOUT_S', 1.0)
    world.follow = False
    assert not real(world).move([30.0] * 6)
    assert len(world.halts()) == 1
    assert world.log.has('error', 'took longer than 1 s')


def test_real_move_halts_on_shutdown(world):
    world.ok = False
    assert not real(world).move([30.0] * 6)
    assert len(world.halts()) == 1


# _run_on

class StubArm:

    def __init__(self, prepare=True, moves_ok=True):
        self.log = Logger()
        self.angles = None
        self._prepare = prepare
        self._moves_ok = moves_ok
        self.moved = []

    def prepare(self):
        return self._prepare

    def move(self, degrees, seconds):
        self.moved.append(degrees)
        self.angles = degrees
        return self._moves_ok

    def pause(self, seconds):
        pass


MOVES = [([0.0] * 6, 1.0), ([10.0] * 6, 1.0)]


def test_run_on_stops_if_prepare_fails():
    arm = StubArm(prepare=False)
    assert not braco_arm._run_on(arm, 'gazebo', MOVES)
    assert arm.moved == []


def test_run_on_stops_at_the_first_failed_move():
    arm = StubArm(moves_ok=False)
    assert not braco_arm._run_on(arm, 'gazebo', MOVES)
    assert arm.moved == [MOVES[0][0]]
    assert arm.log.has('error', 'gazebo: move to')


def test_run_on_runs_every_move():
    arm = StubArm()
    assert braco_arm._run_on(arm, 'gazebo', MOVES)
    assert arm.moved == [m[0] for m in MOVES]
    assert arm.log.has('info', 'gazebo: reached')


# run

class Record:
    """Fake SimArm / RealArm for run(): records what run() did with them."""

    def __init__(self, ok=True, interrupt=False):
        self.ok = ok
        self.interrupt = interrupt
        self.made = []
        self.events = []

    def __call__(self, name, domain, speed=None):
        record = self
        self.made.append((name, domain, speed))

        class Arm(StubArm):

            def prepare(self):
                if record.interrupt:
                    raise KeyboardInterrupt
                return record.ok

            def stop(self):
                record.events.append('stop')

            def release_control(self):
                record.events.append('release_control')

            def close(self):
                record.events.append('close')
        return Arm()


@pytest.fixture
def arms(monkeypatch):
    sim, real_arm = Record(), Record()
    monkeypatch.setattr(braco_arm, 'SimArm', sim)
    monkeypatch.setattr(braco_arm, 'RealArm', real_arm)
    monkeypatch.setenv('ROS_DOMAIN_ID', '27')
    return sim, real_arm


def run(monkeypatch, *args, answer=None, **kwargs):
    monkeypatch.setattr(sys, 'argv', ['park.py', *args])
    if answer is not None:
        monkeypatch.setattr('builtins.input', lambda prompt: answer)
    return braco_arm.run('park', MOVES, **kwargs)


@pytest.mark.parametrize('args', [
    ['--skip-sim'], ['--real', '--sim-domain', '22'], ['--speed', '0'], ['--speed', '101']])
def test_run_refuses_bad_arguments(monkeypatch, arms, args):
    with pytest.raises(SystemExit) as error:
        run(monkeypatch, *args)
    assert error.value.code == 2


def test_run_gazebo_only(monkeypatch, arms):
    sim, real_arm = arms
    assert run(monkeypatch) == 0
    assert sim.made == [('park_sim', 27, None)]
    assert sim.events == ['close']
    assert real_arm.made == []


def test_run_gazebo_failure_stops_before_the_real_arm(monkeypatch, arms):
    sim, real_arm = arms
    sim.ok = False
    assert run(monkeypatch, '--real', '--yes') == 1
    assert sim.events == ['close']
    assert real_arm.made == []


def test_run_real_asks_first(monkeypatch, arms, capsys):
    sim, real_arm = arms
    assert run(monkeypatch, '--real', answer='n') == 0
    assert 'real arm not moved' in capsys.readouterr().out
    assert real_arm.made == []


def test_run_real_after_yes(monkeypatch, arms):
    sim, real_arm = arms
    done = []
    assert run(monkeypatch, '--real', '--speed', '20', answer='y',
               after_real=lambda arm: done.append(arm) or True, after_real_note='release') == 0
    assert real_arm.made == [('park_real', 22, 20)]
    assert len(done) == 1
    assert real_arm.events == ['release_control', 'close']


def test_run_real_skip_sim(monkeypatch, arms):
    sim, real_arm = arms
    assert run(monkeypatch, '--real', '--yes', '--skip-sim', '--real-domain', '27') == 0
    assert sim.made == []
    assert real_arm.made == [('park_real', 27, 30)]


def test_run_real_after_real_failure(monkeypatch, arms):
    assert run(monkeypatch, '--real', '--yes', after_real=lambda arm: False) == 1


def test_run_real_interrupted_halts_and_releases(monkeypatch, arms):
    sim, real_arm = arms
    real_arm.interrupt = True
    assert run(monkeypatch, '--real', '--yes') == 1
    assert real_arm.events == ['stop', 'release_control', 'close']
