"""Pick and place from the page (web/server/mission.py): the goal check, the mock, and the ROS 2
action client against a fake /pick_and_place. No ROS, no braco_mission.
"""

import math
import threading
import time

import pytest

from web.server import mission as mission_module
from web.server.mission import STAGES, MissionBusyError, MockPickPlace, PickPlace, check_goal

PICK = (0.20, -0.04, 0.025)
PLACE = (0.20, 0.08, 0.025)


def wait_for(predicate, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


# ---- the goal check -------------------------------------------------------------------


def test_check_goal_accepts_the_gazebo_demo():
    check_goal(PICK, PLACE, 0.05)


@pytest.mark.parametrize(
    'pick, place, approach, match',
    [
        ((0.2, 0.0), PLACE, 0.05, 'three'),
        ((math.nan, 0.0, 0.03), PLACE, 0.05, 'finite'),
        ((0.5, 0.0, 0.03), PLACE, 0.05, 'out of reach'),
        ((0.2, 0.0, 0.005), PLACE, 0.05, 'above the table'),
        (PICK, (0.2, 0.08, 0.30), 0.05, 'above the table'),
        (PICK, PLACE, 0.005, 'approach'),
        (PICK, PLACE, math.inf, 'approach'),
        (PICK, (0.201, -0.04, 0.025), 0.05, 'same point'),
    ],
)
def test_check_goal_refuses(pick, place, approach, match):
    with pytest.raises(ValueError, match=match):
        check_goal(pick, place, approach)


# ---- the mock ---------------------------------------------------------------------------


def test_mock_walks_every_stage_and_reports_once(monkeypatch):
    monkeypatch.setattr(mission_module, 'MOCK_STAGE_S', 0.005)
    m = MockPickPlace()
    seen, done = [], []
    m.on_change = lambda s: seen.append(s['stage'])
    m.on_done = lambda result, by, by_id: done.append((result, by, by_id))
    m.run(PICK, PLACE, 0.05, by='Ana', by_id='t-ana')
    assert wait_for(lambda: done)
    result, by, by_id = done[0]
    assert result['success'] and by == 'Ana' and by_id == 't-ana'
    assert [s for s in seen if s in STAGES] == list(STAGES)
    s = m.snapshot()
    assert not s['running'] and s['goal']['pick'] == list(PICK) and s['result'] == result
    assert len(done) == 1


def test_mock_is_one_goal_at_a_time_and_cancels(monkeypatch):
    monkeypatch.setattr(mission_module, 'MOCK_STAGE_S', 0.05)
    m = MockPickPlace()
    done = []
    m.on_done = lambda result, by, by_id: done.append(result)
    m.run(PICK, PLACE, 0.05)
    with pytest.raises(MissionBusyError):
        m.run(PICK, PLACE, 0.05)
    assert m.cancel() is True
    assert done[0]['cancelled'] and not done[0]['success'] and done[0]['failed_stage'] in STAGES
    time.sleep(0.15)
    assert len(done) == 1 and m.cancel() is False  # the walk stopped, nothing to cancel


# ---- the ROS 2 action client against a fake /pick_and_place ------------------------------


class Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class FakePose:
    def __init__(self):
        self.header = Obj(frame_id='')
        self.pose = Obj(position=Obj(x=0, y=0, z=0), orientation=Obj(x=0, y=0, z=0, w=0))


class FakeGoal:
    pass


class FakeFuture:
    def __init__(self):
        self._result = None
        self._done = False
        self._callbacks = []

    def set_result(self, value):
        self._result, self._done = value, True
        for cb in self._callbacks:
            cb(self)

    def add_done_callback(self, cb):
        if self._done:
            cb(self)
        else:
            self._callbacks.append(cb)

    def result(self):
        return self._result

    def cancel(self):
        pass


class FakeActionClient:
    """send_goal_async answers at once (accepted or not); the result comes when the test says."""

    def __init__(self, ready=True, accept=True):
        self.ready = ready
        self.accept = accept
        self.goals = []
        self.feedback = None
        self.result_future = FakeFuture()
        self.cancelled = False

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        self.feedback = feedback_callback
        future = FakeFuture()
        handle = Obj(
            accepted=self.accept,
            get_result_async=lambda: self.result_future,
            cancel_goal_async=self._cancel,
        )
        future.set_result(handle)
        return future

    def _cancel(self):
        self.cancelled = True

    def finish(self, **fields):
        values = dict(
            success=True,
            failed_stage='',
            pick_position_error_m=0.0008,
            pick_orientation_error_rad=0.002,
            place_position_error_m=0.0009,
            place_orientation_error_rad=float('nan'),
            message='done',
        )
        values.update(fields)
        self.result_future.set_result(Obj(status=4, result=Obj(**values)))


def ros_client(**kw):
    m = PickPlace(bridge=Obj())
    m._types = {'PickAndPlace': Obj(Goal=FakeGoal), 'PoseStamped': FakePose}
    m._client = FakeActionClient(**kw)
    m.poll()
    return m


def test_ros_goal_has_world_poses_with_the_tool_down():
    m = ros_client()
    m.run(PICK, PLACE, 0.04, by='Ana')
    goal = m._client.goals[0]
    assert goal.pick.header.frame_id == 'world' and goal.place.header.frame_id == 'world'
    p = goal.pick.pose.position
    assert (p.x, p.y, p.z) == PICK
    o = goal.place.pose.orientation
    assert (o.x, o.y, o.z, o.w) == (1.0, 0.0, 0.0, 0.0)
    assert goal.approach_height_m == 0.04
    assert m.snapshot()['running'] and m.snapshot()['by'] == 'Ana'


def test_ros_feedback_and_result():
    m = ros_client()
    done = threading.Event()
    results = []
    m.on_done = lambda result, by, by_id: (results.append(result), done.set())
    m.run(PICK, PLACE, 0.05, by='Ana', by_id='t-ana')
    m._client.feedback(Obj(feedback=Obj(stage='approach_pick')))
    assert m.snapshot()['stage'] == 'approach_pick'
    m._client.finish()
    assert done.wait(2)
    r = results[0]
    assert r['success'] and r['failed_stage'] is None and r['pick_position_error_m'] == 0.0008
    assert r['place_orientation_error_rad'] is None  # NaN: JSON has no NaN
    assert not m.snapshot()['running'] and m.snapshot()['result'] == r


def test_ros_failure_names_the_stage_and_cancel_is_reported():
    m = ros_client()
    results = []
    done = threading.Event()
    m.on_done = lambda result, by, by_id: (results.append(result), done.set())
    m.run(PICK, PLACE, 0.05)
    assert m.cancel() is True and m._client.cancelled
    m._client.finish(success=False, failed_stage='pre_grasp', message='pre_grasp: cancelled')
    assert done.wait(2)
    assert results[0]['failed_stage'] == 'pre_grasp' and results[0]['cancelled'] is True


def test_ros_without_server_or_refused_goal():
    m = ros_client(ready=False)
    assert m.snapshot()['available'] is False and 'move_to_pose.launch.py' in m.snapshot()['error']
    with pytest.raises(RuntimeError, match='no /pick_and_place'):
        m.run(PICK, PLACE, 0.05)

    m = ros_client(accept=False)
    done = []
    m.on_done = lambda *a: done.append(a)
    with pytest.raises(RuntimeError, match='refused'):
        m.run(PICK, PLACE, 0.05)
    s = m.snapshot()
    assert not s['running'] and s['result']['failed_stage'] == 'goal'
    assert done == []  # never started: the caller cleans up, not on_done
    m._client.accept = True
    m.run(PICK, PLACE, 0.05)  # not left busy


def test_ros_without_rclpy_reports_the_bridge_error():
    m = PickPlace(bridge=Obj(error='rclpy not available'))
    m._setup(None)
    assert m.snapshot()['available'] is False and m.snapshot()['error'] == 'rclpy not available'
    with pytest.raises(RuntimeError, match='rclpy'):
        m.run(PICK, PLACE, 0.05)
