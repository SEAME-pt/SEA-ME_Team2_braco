"""Tests for the web dashboard server. No arm and no network needed.

make ci
"""

import math
import os
import sys
import time

import pytest

os.environ['ARM_MOCK'] = '1'  # before importing the app: the mock, never ROS or the arm
os.environ['BRACO_ROS'] = '0'  # and never start ROS 2, even where it is installed

from fastapi.testclient import TestClient  # noqa: E402

from web.server import app as app_module  # noqa: E402
from web.server import arm, camera, docs, github, role, ros  # noqa: E402
from web.server.app import app  # noqa: E402
from web.server.session import ControlError  # noqa: E402

STATIC_FILES = [
    'index.html',
    'css/style.css',
    'js/app.js',
    'js/util.js',
    'js/specs.js',
    'js/arm3d.js',
    'js/activity.js',
    'js/camera.js',
    'js/views/home.js',
    'js/views/arm.js',
    'js/views/ros.js',
    'js/views/docs.js',
    'js/views/sprints.js',
    'js/views/sprint-detail.js',
]


@pytest.fixture(autouse=True)
def close_sources(monkeypatch):
    """Stop the polling thread of every source a test starts (not the app's own)."""
    created = []
    for cls in (arm.MockArm, arm.GazeboArm, arm.RosArm):

        def start(self, _original=cls.start):
            if self is not app_module.arm:
                created.append(self)
            _original(self)

        monkeypatch.setattr(cls, 'start', start)
    yield
    for source in created:
        source.close()


def wait_for(predicate, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


# ---- arm.py ---------------------------------------------------------------------


def test_temp_level_matches_mycobot_hpp():
    assert arm.temp_level(None) is None
    assert arm.temp_level(54) == 'normal'
    assert arm.temp_level(55) == 'warm'
    assert arm.temp_level(64) == 'warm'
    assert arm.temp_level(65) == 'hot'


def test_mock_arm_stays_inside_joint_limits(monkeypatch):
    monkeypatch.setattr(arm, 'ANGLE_PERIOD', 0.01)
    mock = arm.MockArm()
    mock.start()
    assert wait_for(lambda: mock.snapshot()['temps'] is not None)
    for _ in range(20):
        angles = mock.snapshot()['angles']
        for a, (lo, hi) in zip(angles, arm.JOINT_LIMITS):
            assert lo <= a <= hi
        time.sleep(0.01)


def test_mock_coords_use_dh_home_pose():
    # all joints at 0: upright, flange at the 280's known home position
    assert arm.MockArm._fake_coords([0] * 6)[:3] == [45.6, -63.4, 412.7]


def test_mock_arm_stop_freezes_angles(monkeypatch):
    monkeypatch.setattr(arm, 'ANGLE_PERIOD', 0.01)
    mock = arm.MockArm()
    mock.start()
    assert wait_for(lambda: mock.snapshot()['angles'] is not None)
    mock.stop_motion()
    time.sleep(0.05)
    first = mock.snapshot()['angles']
    time.sleep(0.1)
    s = mock.snapshot()
    assert s['angles'] == first
    assert s['moving'] is False


# ---- docs.py --------------------------------------------------------------------


def test_list_docs_includes_repo_docs():
    paths = {d['path'] for d in docs.list_docs()}
    assert {
        'docs/guides/safety.md',
        'docs/guides/dev-setup.md',
        'docs/hardware/protocol_commands.md',
    } <= paths
    assert 'docker/README.md' in paths and 'web/README.md' in paths
    safety = next(d for d in docs.list_docs() if d['path'] == 'docs/guides/safety.md')
    assert safety['title'].startswith('Lab Safety')
    assert safety['group'] == 'Safety'
    assert '.github/PULL_REQUEST_TEMPLATE.md' in paths


def test_list_docs_includes_architecture():
    listed = docs.list_docs()
    arch = next(d for d in listed if d['path'] == 'docs/architecture/architecture.md')
    assert arch['title'] == 'braco architecture'
    assert arch['group'] == 'Architecture and decisions'
    assert not any(d['path'].startswith('web/') and d['path'].endswith('.html') for d in listed)
    assert docs.read_doc('docs/architecture/architecture.md').startswith('# braco architecture')


def test_list_docs_skips_caches_and_groups_in_order():
    listed = docs.list_docs()
    assert not any('.pytest_cache' in d['path'] or '.venv' in d['path'] for d in listed)
    groups = list(dict.fromkeys(d['group'] for d in listed))
    assert groups == sorted(groups, key=docs.ORDER.index)


@pytest.mark.parametrize(
    'path',
    [
        'docs/../Makefile',
        'docs/../README.md',
        'docs/../../../../etc/passwd.md',
        'docs/hardware/../../web/server/app.py',
        'Makefile',
        '/etc/passwd',
        'docs/missing.md',
    ],
)
def test_read_doc_rejects_paths_outside_docs(path):
    assert docs.read_doc(path) is None


def test_read_doc_reads_nested_doc():
    assert docs.read_doc('docs/hardware/inventory.md')


# ---- github.py ------------------------------------------------------------------


def gh_issue(number, title, state='open', milestone=2, assignees=()):
    return {
        'number': number,
        'title': title,
        'state': state,
        'html_url': 'https://github.com/x/y/issues/{}'.format(number),
        'assignees': [{'login': a} for a in assignees],
        'labels': [],
        'milestone': {'number': milestone},
        'closed_at': None,
        'body': '',
    }


@pytest.fixture
def fake_github(monkeypatch):
    responses = {
        '/milestones': [
            {
                'number': 1,
                'title': 'Milestone 0',
                'description': '',
                'state': 'open',
                'due_on': '2026-10-09T00:00:00Z',
                'open_issues': 1,
                'closed_issues': 2,
                'html_url': 'https://github.com/x/y/milestone/1',
            }
        ],
        '/issues?': [
            gh_issue(2, 'SPRINT 0 | Onboarding & Baseline', milestone=1),
            gh_issue(24, 'Sprint 0 demo and retrospective', milestone=1),
            gh_issue(1, 'SPRINT 1 | Foundations'),
            dict(gh_issue(50, 'a pull request'), pull_request={}),
        ],
        '/issues/2/sub_issues': [
            gh_issue(4, 'Complete safety induction', 'closed', 1, ['denionline']),
            gh_issue(3, 'Repository, Git flow and PR rules', 'open', 1),
        ],
        '/issues/1/sub_issues': [],
        '/pulls?': [
            {
                'number': 7,
                'title': 'feat: kinematics',
                'html_url': 'https://github.com/x/y/pull/7',
                'user': {'login': 'larocqueg'},
                'labels': [],
                'milestone': None,
                'merged_at': '2026-10-05T10:00:00Z',
            },
            {
                'number': 8,
                'title': 'closed without merge',
                'html_url': 'u',
                'user': {'login': 'a'},
                'labels': [],
                'milestone': None,
                'merged_at': None,
            },
        ],
    }
    calls = []

    def fake_get_all(url):
        calls.append(url)
        for key, value in responses.items():
            if key in url:
                return value
        raise AssertionError('unexpected URL ' + url)

    monkeypatch.setattr(github, '_get_all', fake_get_all)
    monkeypatch.setitem(github._cache, 'time', 0.0)
    monkeypatch.setitem(github._cache, 'data', None)
    return calls


def test_sprints_groups_sprint_issues_and_sub_issues(fake_github):
    data = github.sprints()
    assert [s['sprint'] for s in data['sprints']] == [0, 1]
    s0 = data['sprints'][0]
    assert s0['name'] == 'Onboarding & Baseline'
    assert [i['number'] for i in s0['sub_issues']] == [4, 3]
    assert s0['sub_issues'][0]['assignees'] == ['denionline']
    # the demo issue is not a sprint, pull requests are not issues
    assert 24 in {i['number'] for i in data['issues']}
    assert 50 not in {i['number'] for i in data['issues']}
    assert data['milestones'][0]['title'] == 'Milestone 0'
    # only merged pull requests are kept
    assert [p['number'] for p in data['pulls']] == [7]
    assert data['pulls'][0]['author'] == 'larocqueg'


def test_sprints_are_cached(fake_github):
    github.sprints()
    count = len(fake_github)
    github.sprints()
    assert len(fake_github) == count
    github.sprints(force=True)
    assert len(fake_github) == 2 * count


def test_sprints_keep_last_copy_on_error(fake_github, monkeypatch):
    good = github.sprints()

    def offline(url):
        raise OSError('network down')

    monkeypatch.setattr(github, '_get_all', offline)
    data = github.sprints(force=True)
    assert data['error'] == 'network down'
    assert data['sprints'] == good['sprints']


def test_sprints_error_without_cache(monkeypatch):
    monkeypatch.setitem(github._cache, 'data', None)
    monkeypatch.setattr(
        github, '_get_all', lambda url: (_ for _ in ()).throw(OSError('rate limited'))
    )
    data = github.sprints(force=True)
    assert data['error'] == 'rate limited'
    assert data['sprints'] == [] and data['milestones'] == []


# ---- GazeboArm (no ROS needed: messages and publisher are faked) -------------------


class FakeJointState:
    def __init__(self, names, degrees):
        self.name = list(names)
        self.position = [math.radians(d) for d in degrees]


class FakePublisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeMsg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


@pytest.fixture
def gz():
    source = arm.GazeboArm()
    source._publisher = FakePublisher()
    source._msgs = (lambda sec, nanosec: (sec, nanosec), FakeMsg, FakeMsg)
    return source


def test_gazebo_joint_state_becomes_snapshot(gz):
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [10, -20, 30, 0, 5, 0]))
    s = gz.snapshot()
    assert s['source'] == 'gazebo' and s['mock'] is False and s['connected'] is True
    assert s['angles'] == [10, -20, 30, 0, 5, 0]
    assert len(s['coords']) == 6 and s['temps'] is None and s['power_on'] is True


def test_gazebo_joint_state_order_follows_names(gz):
    names = list(reversed(arm.GZ_JOINT_NAMES))
    gz._on_joint_state(FakeJointState(names, [6, 5, 4, 3, 2, 1]))
    assert gz.snapshot()['angles'] == [1, 2, 3, 4, 5, 6]


def test_gazebo_missing_joint_is_an_error(gz):
    gz._on_joint_state(FakeJointState(['other'], [0]))
    s = gz.snapshot()
    assert s['connected'] is False and arm.GZ_JOINT_NAMES[0] in s['error']


def test_gazebo_moving_flag(gz):
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [0] * 6))
    gz._last_move -= 10
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [0] * 6))
    assert gz.snapshot()['moving'] is False
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [5, 0, 0, 0, 0, 0]))
    assert gz.snapshot()['moving'] is True


def test_gazebo_stale_states_disconnect(gz):
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [0] * 6))
    gz._last_msg -= arm.GZ_STALE_AFTER + 1
    gz._check_stale()
    assert gz.snapshot()['connected'] is False


def test_gazebo_move_joint_publishes_trajectory(gz):
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [0, 10, 0, 0, 0, 0]))
    gz.move_joint(2, 60, speed=30)
    traj = gz._publisher.sent[-1]
    assert traj.joint_names == arm.GZ_JOINT_NAMES
    point = traj.points[0]
    assert [round(math.degrees(p)) for p in point.positions] == [0, 10, 60, 0, 0, 0]
    assert point.time_from_start == (2, 0)  # 60 deg at 30 deg/s


def test_gazebo_pose_stop_and_wake(gz):
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [1, 2, 3, 4, 5, 6]))
    gz.go_to_pose('park')
    assert [
        round(math.degrees(p), 1) for p in gz._publisher.sent[-1].points[0].positions
    ] == arm.POSES['park']
    gz.stop_motion()
    assert [round(math.degrees(p)) for p in gz._publisher.sent[-1].points[0].positions] == [
        1,
        2,
        3,
        4,
        5,
        6,
    ]
    sent = len(gz._publisher.sent)
    gz.wake_up()
    assert len(gz._publisher.sent) == sent  # wake up does not move the arm
    with pytest.raises(ValueError):
        gz.go_to_pose('nope')


def test_gazebo_commands_need_joint_states(gz):
    with pytest.raises(RuntimeError):
        gz.move_joint(0, 10)


def test_gazebo_without_rclpy_reports_error(monkeypatch):
    monkeypatch.setitem(sys.modules, 'rclpy', None)  # makes `import rclpy` fail
    source = arm.GazeboArm(bridge=ros.RosBridge())
    source.start()
    assert wait_for(lambda: 'rclpy' in (source.snapshot()['error'] or ''))
    assert source.snapshot()['connected'] is False


# ---- role.py and the choice of source ----------------------------------------------


@pytest.mark.parametrize(
    'seame_role, profiles, expected',
    [
        ('pc-dev', None, 'pc'),
        ('jetson-driver', 'pc', 'jetson'),
        (None, 'jetson', 'jetson'),
        (None, 'pc', 'pc'),
    ],
)
def test_machine_role_from_environment(monkeypatch, seame_role, profiles, expected):
    for name, value in (('SEAME_ROLE', seame_role), ('COMPOSE_PROFILES', profiles)):
        if value is None:
            monkeypatch.delenv(name, raising=False)
        else:
            monkeypatch.setenv(name, value)
    assert role.machine_role() == expected


def test_machine_role_reads_docker_env_then_cpu(monkeypatch, tmp_path):
    monkeypatch.delenv('SEAME_ROLE', raising=False)
    monkeypatch.delenv('COMPOSE_PROFILES', raising=False)
    env = tmp_path / '.env'
    env.write_text('# comment\nCOMPOSE_PROFILES=jetson   # the Nano\n')
    monkeypatch.setattr(role, 'DOCKER_ENV', env)
    assert role.machine_role() == 'jetson'
    monkeypatch.setattr(role, 'DOCKER_ENV', tmp_path / 'missing')
    monkeypatch.setattr(role.platform, 'machine', lambda: 'aarch64')
    assert role.machine_role() == 'jetson'
    monkeypatch.setattr(role.platform, 'machine', lambda: 'x86_64')
    assert role.machine_role() == 'pc'


def test_gazebo_source_is_pc_only(monkeypatch):
    monkeypatch.delenv('ARM_MOCK', raising=False)
    monkeypatch.setenv('ARM_SOURCE', 'gazebo')
    monkeypatch.setattr(app_module, 'machine_role', lambda: 'jetson')
    with pytest.raises(RuntimeError, match='only runs on the PC'):
        app_module.build_arm()
    monkeypatch.setattr(app_module, 'machine_role', lambda: 'pc')
    monkeypatch.setenv('GZ_JOINT_NAMES', 'a, b,c,d,e,f')
    source = app_module.build_arm()
    assert isinstance(source, arm.GazeboArm) and source.joint_names == list('abcdef')


# ---- app.py (HTTP + WebSocket) ----------------------------------------------------


@pytest.fixture(scope='module')
def client():
    with TestClient(app) as c:  # runs the startup hook: starts the mock arm
        assert wait_for(lambda: c.get('/api/status').json()['angles'] is not None)
        yield c


def test_status(client):
    s = client.get('/api/status').json()
    assert s['mock'] is True and s['connected'] is True
    assert len(s['angles']) == 6 and len(s['coords']) == 6


def test_stop(client):
    assert client.post('/api/stop').json() == {'ok': True}
    assert client.get('/api/stop').status_code != 200  # only POST stops the arm


def test_websocket_streams_snapshots(client):
    with client.websocket_connect('/ws/telemetry') as ws:
        first = ws.receive_json()
        second = ws.receive_json()
    assert len(first['angles']) == 6
    assert second['updated'] >= first['updated']


def test_docs_api(client):
    listed = client.get('/api/docs').json()
    assert any(d['path'] == 'docs/guides/safety.md' for d in listed)
    r = client.get('/api/docs/docs/guides/safety.md')
    assert r.status_code == 200 and r.text.startswith('# Lab Safety')


@pytest.mark.parametrize(
    'url',
    [
        '/api/docs/docs/../Makefile',
        '/api/docs/docs/..%2F..%2FMakefile',
        '/api/docs/docs%2F..%2FREADME.md',
        '/api/docs/web/server/app.py',
    ],
)
def test_docs_api_blocks_traversal(client, url):
    r = client.get(url)
    assert r.status_code == 404
    assert 'MyCobot' not in r.text and 'uvicorn' not in r.text


def test_sprints_api(client, fake_github):
    data = client.get('/api/sprints?refresh=1').json()
    assert [s['sprint'] for s in data['sprints']] == [0, 1]


@pytest.mark.parametrize('path', STATIC_FILES)
def test_static_files_served(client, path):
    assert client.get('/' + path).status_code == 200


def test_index_is_root(client):
    r = client.get('/')
    assert r.status_code == 200 and 'js/app.js' in r.text
    assert '<title>braco</title>' in r.text and 'lang="en"' in r.text


def test_ros_source_is_chosen(monkeypatch):
    monkeypatch.delenv('ARM_MOCK', raising=False)
    monkeypatch.setenv('ARM_SOURCE', 'ros')
    assert isinstance(app_module.build_arm(), arm.RosArm)


def test_no_source_is_an_error_not_the_serial_port(monkeypatch):
    """The page never opens the serial port itself: without a source it refuses to start."""
    monkeypatch.delenv('ARM_MOCK', raising=False)
    monkeypatch.delenv('ARM_SOURCE', raising=False)
    with pytest.raises(RuntimeError, match='ARM_SOURCE=ros .* or ARM_MOCK=1'):
        app_module.build_arm()
    monkeypatch.setenv('ARM_SOURCE', 'serial')
    with pytest.raises(RuntimeError, match='unknown ARM_SOURCE'):
        app_module.build_arm()


def test_ros_panels_without_ros(client):
    for path in ('/api/ros/health', '/api/ros/graph'):
        data = client.get(path).json()
        assert data['available'] is False and 'BRACO_ROS=0' in data['error']


# ---- camera (fake frames: no ROS, no camera) --------------------------------------------


class FakeFrame:
    def __init__(self, data):
        self.data = data


def jpeg(tag):
    return b'\xff\xd8' + tag + b'\xff\xd9'


def test_camera_without_ros_is_offline():
    cam = camera.Camera(ros.RosBridge())
    s = cam.status()
    assert s['online'] is False and s['fps'] == 0.0 and s['error'] == 'ROS 2 not started'
    assert cam.latest()[1] is None


def test_camera_frames_and_staleness(monkeypatch):
    clock = [1000.0]
    monkeypatch.setattr(camera.time, 'time', lambda: clock[0])
    bridge = ros.RosBridge()
    bridge.available = True
    cam = camera.Camera(bridge)
    assert 'is the camera node running' in cam.status()['error']
    for i in range(10):
        clock[0] += 0.1
        cam._on_frame(FakeFrame(jpeg(bytes([i]))))
    seq, data = cam.latest()
    assert seq == 10 and data == jpeg(b'\x09')
    s = cam.status()
    assert s['online'] is True and s['error'] is None and s['fps'] == 5.0
    clock[0] += camera.STALE_S + 1
    assert cam.latest()[1] is None
    assert cam.status()['online'] is False and 'unplugged' in cam.status()['error']


def test_camera_api_offline(client):
    s = client.get('/api/camera/status').json()
    assert s['online'] is False and s['topic'] == '/camera/image_raw/compressed'
    assert client.get('/api/camera/snapshot').status_code == 503
    assert client.get('/api/camera/stream').status_code == 503


def test_camera_api_snapshot_and_stream(client, monkeypatch):
    monkeypatch.setattr(camera, 'STALE_S', 0.3)  # the stream ends once the frame is stale
    app_module.camera._on_frame(FakeFrame(jpeg(b'frame')))
    r = client.get('/api/camera/snapshot')
    assert (
        r.status_code == 200
        and r.headers['content-type'] == 'image/jpeg'
        and r.content == jpeg(b'frame')
    )
    r = client.get('/api/camera/stream')
    assert r.status_code == 200 and r.headers['content-type'].startswith(
        'multipart/x-mixed-replace'
    )
    assert r.content.count(b'--frame\r\n') == 1 and jpeg(b'frame') in r.content


# ---- shared session over /ws -----------------------------------------------------------


def next_of(ws, kind, predicate=lambda m: True, limit=60):
    """Read messages until one of `kind` matches (telemetry keeps the stream moving)."""
    for _ in range(limit):
        message = ws.receive_json()
        if message.get('type') == kind and predicate(message):
            return message
    raise AssertionError('no {} message'.format(kind))


def hello(ws, cid, name, page='arm'):
    ws.send_json({'type': 'hello', 'id': cid, 'name': name, 'page': page})
    return next_of(ws, 'welcome')


def test_ws_needs_hello(client):
    with client.websocket_connect('/ws') as ws:
        ws.send_json({'type': 'nope'})
        with pytest.raises(Exception):
            ws.receive_json()


def test_ws_welcome_presence_and_telemetry(client):
    with client.websocket_connect('/ws') as ws:
        welcome = hello(ws, 't-presence', 'Carla')
        assert welcome['you'] == 't-presence' and welcome['name'] == 'Carla'
        assert 'entries' in next_of(ws, 'activity')
        presence = next_of(
            ws, 'presence', lambda m: any(c['id'] == 't-presence' for c in m['clients'])
        )
        me = next(c for c in presence['clients'] if c['id'] == 't-presence')
        assert me['name'] == 'Carla' and me['online'] and me['page'] in ('arm', '')
        assert len(next_of(ws, 'telemetry')['data']['angles']) == 6


def test_lease_and_live_sync_between_two_pages(client):
    A, B = {'X-Client-Id': 't-ana'}, {'X-Client-Id': 't-bruno'}
    with client.websocket_connect('/ws') as wa, client.websocket_connect('/ws') as wb:
        hello(wa, 't-ana', 'Ana')
        hello(wb, 't-bruno', 'Bruno')
        try:
            # nobody in control: commands are locked
            assert client.post('/api/pose/zero', headers=A).status_code == 423
            lease = client.post('/api/control/take', headers=A).json()
            assert lease['holder']['name'] == 'Ana'
            next_of(wb, 'lease', lambda m: (m['holder'] or {}).get('id') == 't-ana')

            # Bruno is a viewer: locked out, and cannot take over
            r = client.post('/api/pose/park', headers=B)
            assert r.status_code == 423 and 'Ana' in r.json()['detail']
            assert client.post('/api/control/take', headers=B).status_code == 423
            assert (
                client.post('/api/joint/1', json={'angle': 10}).status_code == 423
            )  # no id at all

            # Ana commands, Bruno sees it happen
            assert (
                client.post('/api/joint/1', json={'angle': 10, 'speed': 20}, headers=A).status_code
                == 200
            )
            entry = next_of(
                wb, 'activity', lambda m: 'entry' in m and m['entry']['action'].startswith('J1')
            )['entry']
            assert entry['who'] == 'Ana' and entry['ok'] and entry['by'] == 't-ana'

            # Ana's control panel is mirrored live to Bruno, not echoed back to her
            wa.send_json({'type': 'ui', 'state': {'speed': 55, 'drag': {'j': 2, 'value': 42}}})
            ui = next_of(wb, 'ui')
            assert ui['name'] == 'Ana' and ui['state']['drag'] == {'j': 2, 'value': 42}
            # a viewer's ui is ignored
            wb.send_json({'type': 'ui', 'state': {'speed': 1}})

            # STOP works for the viewer, and Ana is told who did it
            assert client.post('/api/stop', headers=B).json() == {'ok': True}
            stop = next_of(
                wa, 'activity', lambda m: 'entry' in m and m['entry']['action'] == 'STOP'
            )['entry']
            assert stop['who'] == 'Bruno'

            assert client.get('/api/session').json()['lease']['holder']['id'] == 't-ana'
        finally:
            client.post('/api/control/release', headers=A)
        assert client.get('/api/session').json()['lease']['holder'] is None
        next_of(wb, 'lease', lambda m: m['holder'] is None)


def test_take_control_is_409_while_the_joystick_has_the_arm(client, monkeypatch):
    class Joystick:
        """arm_driver's answer while the joystick has the control."""

        def take_control(self):
            raise ControlError('the arm is controlled by joystick')

        def release_control(self):
            pass

    monkeypatch.setattr(app_module.hub, 'control', Joystick())
    with client.websocket_connect('/ws') as ws:
        hello(ws, 't-ana', 'Ana')
        r = client.post('/api/control/take', headers={'X-Client-Id': 't-ana'})
        assert r.status_code == 409 and 'joystick' in r.json()['detail']
        assert client.get('/api/session').json()['lease']['holder'] is None


def test_speed_is_shared_capped_and_operator_only(client):
    A, B = {'X-Client-Id': 't-eva'}, {'X-Client-Id': 't-fabio'}
    with client.websocket_connect('/ws') as wa, client.websocket_connect('/ws') as wb:
        hello(wa, 't-eva', 'Eva')
        welcome_b = hello(wb, 't-fabio', 'Fabio')
        assert next_of(wb, 'speed')['limit'] == 80
        try:
            # nobody in control, or a viewer: locked
            assert client.post('/api/speed', json={'value': 40}, headers=A).status_code == 423
            client.post('/api/control/take', headers=A)
            assert client.post('/api/speed', json={'value': 40}, headers=B).status_code == 423

            # the operator sets it and everybody sees it
            assert client.post('/api/speed', json={'value': 55}, headers=A).json()['value'] == 55
            assert next_of(wb, 'speed', lambda m: m['value'] == 55)['fast'] is False

            # above 80 needs the warning accepted
            r = client.post('/api/speed', json={'value': 90}, headers=A)
            assert r.status_code == 422 and '80' in r.json()['detail']
            assert (
                client.post(
                    '/api/speed', json={'value': 101, 'unlocked': True}, headers=A
                ).status_code
                == 422
            )
            assert client.post('/api/speed', json={'value': 0}, headers=A).status_code == 422
            assert (
                client.post('/api/speed', json={'value': 90, 'unlocked': True}, headers=A).json()[
                    'fast'
                ]
                is True
            )
            next_of(wb, 'speed', lambda m: m['value'] == 90 and m['fast'])

            # commands use the shared speed
            assert client.post('/api/joint/1', json={'angle': 5}, headers=A).json()['speed'] == 90
            assert client.post('/api/pose/zero', headers=A).json()['speed'] == 90
        finally:
            client.post('/api/control/release', headers=A)
        # the fast range does not survive the lease
        assert next_of(wb, 'speed', lambda m: m['value'] == 80)['fast'] is False
        with client.websocket_connect('/ws') as wc:
            hello(wc, 't-gil', 'Gil')
            assert next_of(wc, 'speed')['value'] == 80


def test_operator_only_extras(client):
    A = {'X-Client-Id': 't-dani'}
    with client.websocket_connect('/ws') as ws:
        hello(ws, 't-dani', 'Dani')
        client.post('/api/control/take', headers=A)
        try:
            assert (
                client.post('/api/led', json={'r': 255, 'g': 0, 'b': 0}, headers=A).status_code
                == 200
            )
            assert (
                client.post('/api/led', json={'r': 300, 'g': 0, 'b': 0}, headers=A).status_code
                == 422
            )
            assert client.post('/api/gripper', json={'closed': True}, headers=A).status_code == 200
            assert client.get('/api/status').json()['gripper'] == 'closed'
            assert client.post('/api/rest', json={'on': True}, headers=A).status_code == 200
            r = client.post('/api/pose/zero', headers=A)
            assert r.status_code == 409  # resting: wake up first
            assert client.post('/api/wake', headers=A).status_code == 200
            assert client.get('/api/status').json()['resting'] is False
        finally:
            client.post('/api/control/release', headers=A)
