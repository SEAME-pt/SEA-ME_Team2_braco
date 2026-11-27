"""Pick and place from the page: a client of braco_mission's /pick_and_place action.

    PickPlace      ROS 2 action client (real arm and Gazebo). The goal goes to braco_mission's
                   move_to_pose node, which plans with MoveIt 2 and moves the arm
    MockPickPlace  walks through the stages on a timer and moves nothing (ARM_MOCK=1)

The page never plans: it sends two tcp positions in the planning frame (`world`, the table
surface under the arm, metres) with the tool pointing down, and shows the stage (the action's
feedback) and the result. Planning, the straight lines, the gripper and the tolerances are
braco_mission's (ros2_ws/src/braco_mission/README.md#pick-and-place).

On the real arm braco_mission takes arm_driver's control as `mission` for the whole sequence
(docs/decisions/ADR-0004-trajectories-through-arm-driver.md), so the page lends it the control
for one goal (Hub.lend) and takes it back when the goal ends (Hub.give_back).

    mission.start()
    mission.run(pick, place, approach_m, by='Ana', by_id='tab id')   # (x, y, z) in metres
    mission.cancel()
    mission.snapshot()   # what the page shows; on_change(snapshot) on every change
    mission.on_done = fn(result, by, by_id)                         # once per accepted goal
"""

import math
import threading
import time

from . import ros

ACTION = '/pick_and_place'
FRAME = 'world'
# tcp pointing down, the fingers closing along world x: the orientation of the Gazebo demo
# (braco_mission/README.md, "Demo in Gazebo"). Quaternion x, y, z, w.
TOOL_DOWN = (1.0, 0.0, 0.0, 0.0)

# The stages of PickAndPlace.action, in order (its Feedback STAGE_* constants)
STAGES = (
    'control',
    'open',
    'pre_grasp',
    'approach_pick',
    'close',
    'retreat_pick',
    'pre_place',
    'approach_place',
    'release',
    'retreat_place',
)

# What the page accepts before asking braco_mission. Not the reach (MoveIt decides that): only
# what is surely wrong. The open fingertips reach ~15 mm below the tcp and the table is z = 0.
Z_MIN_M = 0.020
Z_MAX_M = 0.250
APPROACH_MIN_M = 0.020
APPROACH_MAX_M = 0.100
APPROACH_DEFAULT_M = 0.050  # braco_mission's approach_height_m
XY_MAX_M = 0.400  # the myCobot 280 reaches 280 mm
MIN_DISTANCE_M = 0.010  # pick and place closer than this: nothing to move

ACCEPT_TIMEOUT_S = 5.0  # braco_mission answers the goal at once; longer: it is not there
MOCK_STAGE_S = 0.8


class MissionBusyError(RuntimeError):
    """A pick and place is already running."""


def check_goal(pick, place, approach_m):
    """Refuse a goal that is surely wrong (ValueError); MoveIt decides the reach."""
    for name, point in (('pick', pick), ('place', place)):
        if len(point) != 3 or not all(math.isfinite(v) for v in point):
            raise ValueError('{} must be three finite numbers (x, y, z in metres)'.format(name))
        x, y, z = point
        if math.hypot(x, y) > XY_MAX_M:
            raise ValueError(
                '{} is {:.0f} mm from the base: out of reach'.format(name, 1000 * math.hypot(x, y))
            )
        if not Z_MIN_M <= z <= Z_MAX_M:
            raise ValueError(
                '{} z must be between {:.0f} and {:.0f} mm above the table'.format(
                    name, 1000 * Z_MIN_M, 1000 * Z_MAX_M
                )
            )
    if not math.isfinite(approach_m) or not APPROACH_MIN_M <= approach_m <= APPROACH_MAX_M:
        raise ValueError(
            'the approach height must be between {:.0f} and {:.0f} mm'.format(
                1000 * APPROACH_MIN_M, 1000 * APPROACH_MAX_M
            )
        )
    if math.dist(pick, place) < MIN_DISTANCE_M:
        raise ValueError('pick and place are the same point')


def _finite(value):
    """NaN (not reached) becomes None, so the result is valid JSON."""
    value = float(value)
    return value if math.isfinite(value) else None


class _Base:
    def __init__(self):
        self._lock = threading.Lock()
        self.on_change = None
        self.on_done = None
        self._by_id = None
        self._state = {
            'available': False,  # the /pick_and_place server is there
            'error': None,  # why it is not
            'running': False,
            'stage': None,  # the goal's stage (feedback), 'sending' before it is accepted
            'stages': list(STAGES),
            'goal': None,  # {'pick': [x, y, z], 'place': [...], 'approach_m'}
            'by': None,  # name of who sent it
            'started': None,
            'finished': None,
            'result': None,
            'mock': False,
        }

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def _set(self, **fields):
        with self._lock:
            changed = any(self._state.get(k) != v for k, v in fields.items())
            self._state.update(fields)
            state = dict(self._state)
        if changed and self.on_change is not None:
            self.on_change(state)

    def _begin(self, pick, place, approach_m, by, by_id):
        with self._lock:
            if self._state['running']:
                raise MissionBusyError('a pick and place is already running')
            self._state['running'] = True  # claimed: a second run() is refused from here on
        self._by_id = by_id
        self._set(
            stage='sending',
            goal={'pick': list(pick), 'place': list(place), 'approach_m': approach_m},
            by=by,
            started=time.time(),
            finished=None,
            result=None,
        )

    def _finish(self, result, notify=True):
        """The goal ended (any way). on_done runs once per accepted goal, in the caller's thread.

        notify=False: the goal never started (run() raises, its caller cleans up).
        """
        state = self.snapshot()
        if not state['running']:
            return
        self._set(running=False, stage=None, finished=time.time(), result=result)
        if notify and self.on_done is not None:
            self.on_done(result, state['by'], self._by_id)

    def start(self):
        pass

    def close(self):
        pass

    def poll(self):
        """Refresh `available` (the pump calls it every second)."""


class PickPlace(_Base):
    """braco_mission's /pick_and_place through the web server's one ROS 2 node."""

    def __init__(self, bridge=None):
        super().__init__()
        self._bridge = bridge or ros.bridge
        self._client = None
        self._types = None
        self._handle = None
        self._cancelled = False
        self._state['error'] = 'waiting for ROS 2'

    def start(self):
        self._bridge.start()
        self._bridge.when_ready(self._setup)

    def _setup(self, node):
        if node is None:
            self._set(available=False, error=self._bridge.error)
            return
        try:
            from braco_interfaces.action import PickAndPlace
        except ImportError as e:
            self._set(
                available=False,
                error='braco_interfaces has no PickAndPlace, build and source the workspace '
                '({})'.format(e),
            )
            return
        from geometry_msgs.msg import PoseStamped
        from rclpy.action import ActionClient

        self._types = {'PickAndPlace': PickAndPlace, 'PoseStamped': PoseStamped}
        self._client = ActionClient(node, PickAndPlace, ACTION)
        self.poll()

    def poll(self):
        if self._client is None:
            return
        if self._client.server_is_ready():
            self._set(available=True, error=None)
        else:
            self._set(
                available=False,
                error='no {} server: start braco_mission '
                '(ros2 launch braco_mission move_to_pose.launch.py)'.format(ACTION),
            )

    def _pose(self, point):
        pose = self._types['PoseStamped']()
        pose.header.frame_id = FRAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = TOOL_DOWN
        return pose

    def run(self, pick, place, approach_m=APPROACH_DEFAULT_M, by=None, by_id=None):
        """Send the goal and return once braco_mission accepted it; the rest runs in ROS."""
        check_goal(pick, place, approach_m)
        if self._client is None:
            raise RuntimeError(self.snapshot()['error'] or 'ROS 2 not ready')
        if not self._client.server_is_ready():
            self.poll()
            raise RuntimeError(self.snapshot()['error'])
        self._begin(pick, place, approach_m, by, by_id)
        self._cancelled = False
        goal = self._types['PickAndPlace'].Goal()
        goal.pick = self._pose(pick)
        goal.place = self._pose(place)
        goal.approach_height_m = float(approach_m)

        accepted = threading.Event()
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda _f: accepted.set())
        if not accepted.wait(ACCEPT_TIMEOUT_S):
            future.cancel()
            message = 'braco_mission did not answer the goal in {:.0f} s'.format(ACCEPT_TIMEOUT_S)
            self._finish(self._failure('goal', message), notify=False)
            raise RuntimeError(message)
        handle = future.result()
        if handle is None or not handle.accepted:
            message = 'braco_mission refused the goal (another goal running?)'
            self._finish(self._failure('goal', message), notify=False)
            raise RuntimeError(message)
        self._handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def cancel(self):
        """Stop the goal: the move in progress stops and no other stage starts."""
        handle = self._handle
        if handle is None or not self.snapshot()['running']:
            return False
        self._cancelled = True
        handle.cancel_goal_async()
        return True

    def _on_feedback(self, message):
        self._set(stage=str(message.feedback.stage))

    def _on_result(self, future):
        response = future.result()
        if response is None:
            result = self._failure(self.snapshot()['stage'] or 'goal', 'no result')
        else:
            r = response.result
            result = {
                'success': bool(r.success),
                'failed_stage': str(r.failed_stage) or None,
                'cancelled': self._cancelled,
                'pick_position_error_m': _finite(r.pick_position_error_m),
                'pick_orientation_error_rad': _finite(r.pick_orientation_error_rad),
                'place_position_error_m': _finite(r.place_position_error_m),
                'place_orientation_error_rad': _finite(r.place_orientation_error_rad),
                'message': str(r.message),
            }
        self._handle = None
        # Not in the executor's thread: on_done calls /arm/take_control and waits for it
        threading.Thread(target=self._finish, args=(result,), daemon=True).start()

    @staticmethod
    def _failure(stage, message):
        return {
            'success': False,
            'failed_stage': stage,
            'cancelled': False,
            'pick_position_error_m': None,
            'pick_orientation_error_rad': None,
            'place_position_error_m': None,
            'place_orientation_error_rad': None,
            'message': message,
        }


class MockPickPlace(_Base):
    """The stages one after the other, every MOCK_STAGE_S; nothing moves."""

    def __init__(self):
        super().__init__()
        self._state.update(available=True, error=None, mock=True)
        self._token = 0

    def run(self, pick, place, approach_m=APPROACH_DEFAULT_M, by=None, by_id=None):
        check_goal(pick, place, approach_m)
        self._begin(pick, place, approach_m, by, by_id)
        with self._lock:
            self._token += 1
            token = self._token
        threading.Thread(target=self._walk, args=(token,), daemon=True).start()

    def cancel(self):
        if not self.snapshot()['running']:
            return False
        with self._lock:
            self._token += 1
        stage = self.snapshot()['stage']
        self._finish(
            {
                'success': False,
                'failed_stage': stage,
                'cancelled': True,
                'pick_position_error_m': None,
                'pick_orientation_error_rad': None,
                'place_position_error_m': None,
                'place_orientation_error_rad': None,
                'message': '{}: cancelled (mock)'.format(stage),
            }
        )
        return True

    def _walk(self, token):
        for stage in STAGES:
            if token != self._token:
                return
            self._set(stage=stage)
            time.sleep(MOCK_STAGE_S)
        if token != self._token:
            return
        self._finish(
            {
                'success': True,
                'failed_stage': None,
                'cancelled': False,
                'pick_position_error_m': 0.0004,
                'pick_orientation_error_rad': 0.001,
                'place_position_error_m': 0.0005,
                'place_orientation_error_rad': 0.001,
                'message': 'mock: every stage ran, nothing moved',
            }
        )
