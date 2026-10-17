"""Tests for the web dashboard server. No arm and no network needed.

    make web-test
"""

import math
import os
import sys
import time
import types

import pytest

os.environ["ARM_MOCK"] = "1"   # before importing the app: never open a serial port

from fastapi.testclient import TestClient  # noqa: E402

from web.server import app as app_module  # noqa: E402
from web.server import arm, docs, github, role  # noqa: E402
from web.server.app import app  # noqa: E402

STATIC_FILES = [
    "index.html", "css/style.css", "js/app.js", "js/util.js", "js/specs.js", "js/arm3d.js",
    "js/views/home.js", "js/views/arm.js", "js/views/docs.js", "js/views/sprints.js", "js/views/sprint-detail.js",
]


@pytest.fixture(autouse=True)
def close_sources(monkeypatch):
    """Stop the polling thread of every source a test starts (not the app's own)."""
    created = []
    for cls in (arm.ArmSource, arm.MockArm, arm.GazeboArm):
        def start(self, _original=cls.start):
            if self is not app_module.arm:
                created.append(self)
            _original(self)
        monkeypatch.setattr(cls, "start", start)
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
    assert arm.temp_level(54) == "normal"
    assert arm.temp_level(55) == "warm"
    assert arm.temp_level(64) == "warm"
    assert arm.temp_level(65) == "hot"


def test_read_value_retries_until_six_numbers():
    replies = iter([-1, None, [1, 2, 3], [1, 2, 3, 4, 5, 6]])
    assert arm.read_value(lambda: next(replies), retries=5) == [1, 2, 3, 4, 5, 6]
    assert arm.read_value(lambda: -1, retries=2) is None


def test_as_bool():
    assert arm.as_bool(1) is True
    assert arm.as_bool(0) is False
    assert arm.as_bool(-1) is None
    assert arm.as_bool(None) is None


class FakeMyCobot:
    """Stands in for pymycobot.MyCobot280 and records every call."""

    instances = []
    angles = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    temps = [30, 56, 66, 40, 41, 42]

    def __init__(self, port, baud):
        self.port, self.baud = port, baud
        self.calls = []
        FakeMyCobot.instances.append(self)

    def get_angles(self):
        self.calls.append("get_angles")
        return list(self.angles)

    def get_coords(self):
        self.calls.append("get_coords")
        return [100.0, 0.0, 200.0, 0.0, 0.0, 0.0]

    def is_power_on(self):
        self.calls.append("is_power_on")
        return 1

    def is_moving(self):
        self.calls.append("is_moving")
        return 0

    def get_servo_temps(self):
        self.calls.append("get_servo_temps")
        return list(self.temps)

    def stop(self):
        self.calls.append("stop")


@pytest.fixture
def fake_pymycobot(monkeypatch):
    FakeMyCobot.instances = []
    module = types.ModuleType("pymycobot")
    module.MyCobot280 = FakeMyCobot
    monkeypatch.setitem(sys.modules, "pymycobot", module)
    monkeypatch.setattr(arm, "ANGLE_PERIOD", 0.01)
    return FakeMyCobot


def test_arm_source_reads_telemetry(fake_pymycobot):
    source = arm.ArmSource("/dev/fake", 115200)
    source.start()
    assert wait_for(lambda: source.snapshot()["temps"] is not None)

    s = source.snapshot()
    mc = fake_pymycobot.instances[0]
    assert (mc.port, mc.baud) == ("/dev/fake", 115200)
    assert s["connected"] is True and s["error"] is None and s["mock"] is False
    assert s["angles"] == FakeMyCobot.angles
    assert s["coords"][2] == 200.0
    assert s["power_on"] is True and s["moving"] is False
    assert s["temps"] == [30, 56, 66, 40, 41, 42]
    assert s["temp_levels"] == ["normal", "warm", "hot", "normal", "normal", "normal"]


def test_arm_source_is_read_only(fake_pymycobot):
    source = arm.ArmSource()
    source.start()
    assert wait_for(lambda: source.snapshot()["temps"] is not None)
    time.sleep(0.1)

    reads = {"get_angles", "get_coords", "is_power_on", "is_moving", "get_servo_temps"}
    assert set(fake_pymycobot.instances[0].calls) <= reads

    source.stop_motion()
    assert fake_pymycobot.instances[0].calls[-1] == "stop"


def test_arm_source_reports_missing_replies(fake_pymycobot, monkeypatch):
    monkeypatch.setattr(FakeMyCobot, "get_angles", lambda self: -1)
    source = arm.ArmSource()
    source.start()
    assert wait_for(lambda: source.snapshot()["error"] is not None)
    s = source.snapshot()
    assert s["connected"] is False
    assert s["angles"] is None
    assert "no reply" in s["error"]


def test_arm_source_survives_port_error(monkeypatch):
    def broken(port, baud):
        raise OSError("could not open port /dev/ttyTHS1: Device or resource busy")

    module = types.ModuleType("pymycobot")
    module.MyCobot280 = broken
    monkeypatch.setitem(sys.modules, "pymycobot", module)

    source = arm.ArmSource()
    source.start()
    assert wait_for(lambda: source.snapshot()["error"] is not None)
    s = source.snapshot()
    assert s["connected"] is False
    assert "busy" in s["error"]
    with pytest.raises(RuntimeError):
        source.stop_motion()


def test_arm_source_handles_missing_temps_method(fake_pymycobot, monkeypatch):
    monkeypatch.delattr(FakeMyCobot, "get_servo_temps")
    source = arm.ArmSource()
    source.start()
    assert wait_for(lambda: source.snapshot()["power_on"] is not None)
    time.sleep(0.05)
    s = source.snapshot()
    assert s["connected"] is True
    assert s["temps"] is None and s["temp_levels"] is None


def test_mock_arm_stays_inside_soft_limits(monkeypatch):
    monkeypatch.setattr(arm, "ANGLE_PERIOD", 0.01)
    mock = arm.MockArm()
    mock.start()
    assert wait_for(lambda: mock.snapshot()["temps"] is not None)
    for _ in range(20):
        angles = mock.snapshot()["angles"]
        for a, (lo, hi) in zip(angles, arm.SOFT_LIMITS):
            assert lo <= a <= hi
        time.sleep(0.01)


def test_mock_coords_use_dh_home_pose():
    # all joints at 0: upright, flange at the 280's known home position
    assert arm.MockArm._fake_coords([0] * 6)[:3] == [45.6, -63.4, 412.7]


def test_mock_arm_stop_freezes_angles(monkeypatch):
    monkeypatch.setattr(arm, "ANGLE_PERIOD", 0.01)
    mock = arm.MockArm()
    mock.start()
    assert wait_for(lambda: mock.snapshot()["angles"] is not None)
    mock.stop_motion()
    time.sleep(0.05)
    first = mock.snapshot()["angles"]
    time.sleep(0.1)
    s = mock.snapshot()
    assert s["angles"] == first
    assert s["moving"] is False


# ---- docs.py --------------------------------------------------------------------

def test_list_docs_includes_repo_docs():
    paths = {d["path"] for d in docs.list_docs()}
    assert {"docs/safety.md", "docs/dev-setup.md", "docs/protocol_commands.md"} <= paths
    assert "docker/README.md" in paths and "web/README.md" in paths
    safety = next(d for d in docs.list_docs() if d["path"] == "docs/safety.md")
    assert safety["title"].startswith("Lab Safety")
    assert safety["group"] == "Segurança"
    assert ".github/PULL_REQUEST_TEMPLATE.md" in paths


def test_list_docs_skips_caches_and_groups_in_order():
    listed = docs.list_docs()
    assert not any(".pytest_cache" in d["path"] or ".venv" in d["path"] for d in listed)
    groups = list(dict.fromkeys(d["group"] for d in listed))
    assert groups == sorted(groups, key=docs.ORDER.index)


@pytest.mark.parametrize("path", [
    "docs/../Makefile",
    "docs/../README.md",
    "docs/../../../../etc/passwd.md",
    "docs/inventory/../../web/server/app.py",
    "Makefile",
    "/etc/passwd",
    "docs/missing.md",
])
def test_read_doc_rejects_paths_outside_docs(path):
    assert docs.read_doc(path) is None


def test_read_doc_reads_nested_doc():
    assert docs.read_doc("docs/inventory/inventory.md")


# ---- github.py ------------------------------------------------------------------

def gh_issue(number, title, state="open", milestone=2, assignees=()):
    return {
        "number": number, "title": title, "state": state,
        "html_url": "https://github.com/x/y/issues/{}".format(number),
        "assignees": [{"login": a} for a in assignees], "labels": [],
        "milestone": {"number": milestone}, "closed_at": None, "body": "",
    }


@pytest.fixture
def fake_github(monkeypatch):
    responses = {
        "/milestones": [{
            "number": 1, "title": "Milestone 0", "description": "", "state": "open",
            "due_on": "2026-10-09T00:00:00Z", "open_issues": 1, "closed_issues": 2,
            "html_url": "https://github.com/x/y/milestone/1",
        }],
        "/issues?": [
            gh_issue(2, "SPRINT 0 | Onboarding & Baseline", milestone=1),
            gh_issue(24, "Sprint 0 demo and retrospective", milestone=1),
            gh_issue(1, "SPRINT 1 | Foundations"),
            dict(gh_issue(50, "a pull request"), pull_request={}),
        ],
        "/issues/2/sub_issues": [
            gh_issue(4, "Complete safety induction", "closed", 1, ["denionline"]),
            gh_issue(3, "Repository, Git flow and PR rules", "open", 1),
        ],
        "/issues/1/sub_issues": [],
        "/pulls?": [
            {"number": 7, "title": "feat: kinematics", "html_url": "https://github.com/x/y/pull/7",
             "user": {"login": "larocqueg"}, "labels": [], "milestone": None,
             "merged_at": "2026-10-05T10:00:00Z"},
            {"number": 8, "title": "closed without merge", "html_url": "u",
             "user": {"login": "a"}, "labels": [], "milestone": None, "merged_at": None},
        ],
    }
    calls = []

    def fake_get_all(url):
        calls.append(url)
        for key, value in responses.items():
            if key in url:
                return value
        raise AssertionError("unexpected URL " + url)

    monkeypatch.setattr(github, "_get_all", fake_get_all)
    monkeypatch.setitem(github._cache, "time", 0.0)
    monkeypatch.setitem(github._cache, "data", None)
    return calls


def test_sprints_groups_sprint_issues_and_sub_issues(fake_github):
    data = github.sprints()
    assert [s["sprint"] for s in data["sprints"]] == [0, 1]
    s0 = data["sprints"][0]
    assert s0["name"] == "Onboarding & Baseline"
    assert [i["number"] for i in s0["sub_issues"]] == [4, 3]
    assert s0["sub_issues"][0]["assignees"] == ["denionline"]
    # the demo issue is not a sprint, pull requests are not issues
    assert 24 in {i["number"] for i in data["issues"]}
    assert 50 not in {i["number"] for i in data["issues"]}
    assert data["milestones"][0]["title"] == "Milestone 0"
    # only merged pull requests are kept
    assert [p["number"] for p in data["pulls"]] == [7]
    assert data["pulls"][0]["author"] == "larocqueg"


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
        raise OSError("network down")

    monkeypatch.setattr(github, "_get_all", offline)
    data = github.sprints(force=True)
    assert data["error"] == "network down"
    assert data["sprints"] == good["sprints"]


def test_sprints_error_without_cache(monkeypatch):
    monkeypatch.setitem(github._cache, "data", None)
    monkeypatch.setattr(github, "_get_all", lambda url: (_ for _ in ()).throw(OSError("rate limited")))
    data = github.sprints(force=True)
    assert data["error"] == "rate limited"
    assert data["sprints"] == [] and data["milestones"] == []


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
    assert s["source"] == "gazebo" and s["mock"] is False and s["connected"] is True
    assert s["angles"] == [10, -20, 30, 0, 5, 0]
    assert len(s["coords"]) == 6 and s["temps"] is None and s["power_on"] is True


def test_gazebo_joint_state_order_follows_names(gz):
    names = list(reversed(arm.GZ_JOINT_NAMES))
    gz._on_joint_state(FakeJointState(names, [6, 5, 4, 3, 2, 1]))
    assert gz.snapshot()["angles"] == [1, 2, 3, 4, 5, 6]


def test_gazebo_missing_joint_is_an_error(gz):
    gz._on_joint_state(FakeJointState(["other"], [0]))
    s = gz.snapshot()
    assert s["connected"] is False and "joint2_to_joint1" in s["error"]


def test_gazebo_moving_flag(gz):
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [0] * 6))
    gz._last_move -= 10
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [0] * 6))
    assert gz.snapshot()["moving"] is False
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [5, 0, 0, 0, 0, 0]))
    assert gz.snapshot()["moving"] is True


def test_gazebo_stale_states_disconnect(gz):
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [0] * 6))
    gz._last_msg -= arm.GZ_STALE_AFTER + 1
    gz._check_stale()
    assert gz.snapshot()["connected"] is False


def test_gazebo_move_joint_publishes_trajectory(gz):
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [0, 10, 0, 0, 0, 0]))
    gz.move_joint(2, 60, speed=30)
    traj = gz._publisher.sent[-1]
    assert traj.joint_names == arm.GZ_JOINT_NAMES
    point = traj.points[0]
    assert [round(math.degrees(p)) for p in point.positions] == [0, 10, 60, 0, 0, 0]
    assert point.time_from_start == (2, 0)   # 60 deg at 30 deg/s


def test_gazebo_pose_stop_and_wake(gz):
    gz._on_joint_state(FakeJointState(arm.GZ_JOINT_NAMES, [1, 2, 3, 4, 5, 6]))
    gz.go_to_pose("park")
    assert [round(math.degrees(p), 1) for p in gz._publisher.sent[-1].points[0].positions] == arm.POSES["park"]
    gz.stop_motion()
    assert [round(math.degrees(p)) for p in gz._publisher.sent[-1].points[0].positions] == [1, 2, 3, 4, 5, 6]
    gz.wake_up()
    assert [round(math.degrees(p)) for p in gz._publisher.sent[-1].points[0].positions] == [0] * 6
    with pytest.raises(ValueError):
        gz.go_to_pose("nope")


def test_gazebo_commands_need_joint_states(gz):
    with pytest.raises(RuntimeError):
        gz.move_joint(0, 10)


def test_gazebo_without_rclpy_reports_error(monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)   # makes `import rclpy` fail
    source = arm.GazeboArm()
    source.start()
    assert wait_for(lambda: "rclpy" in (source.snapshot()["error"] or ""))
    assert source.snapshot()["connected"] is False


# ---- role.py and the choice of source ----------------------------------------------

@pytest.mark.parametrize("seame_role, profiles, expected", [
    ("pc-dev", None, "pc"),
    ("jetson-driver", "pc", "jetson"),
    (None, "jetson", "jetson"),
    (None, "pc", "pc"),
])
def test_machine_role_from_environment(monkeypatch, seame_role, profiles, expected):
    for name, value in (("SEAME_ROLE", seame_role), ("COMPOSE_PROFILES", profiles)):
        if value is None:
            monkeypatch.delenv(name, raising=False)
        else:
            monkeypatch.setenv(name, value)
    assert role.machine_role() == expected


def test_machine_role_reads_docker_env_then_cpu(monkeypatch, tmp_path):
    monkeypatch.delenv("SEAME_ROLE", raising=False)
    monkeypatch.delenv("COMPOSE_PROFILES", raising=False)
    env = tmp_path / ".env"
    env.write_text("# comment\nCOMPOSE_PROFILES=jetson   # the Nano\n")
    monkeypatch.setattr(role, "DOCKER_ENV", env)
    assert role.machine_role() == "jetson"
    monkeypatch.setattr(role, "DOCKER_ENV", tmp_path / "missing")
    monkeypatch.setattr(role.platform, "machine", lambda: "aarch64")
    assert role.machine_role() == "jetson"
    monkeypatch.setattr(role.platform, "machine", lambda: "x86_64")
    assert role.machine_role() == "pc"


def test_gazebo_source_is_pc_only(monkeypatch):
    monkeypatch.delenv("ARM_MOCK", raising=False)
    monkeypatch.setenv("ARM_SOURCE", "gazebo")
    monkeypatch.setattr(app_module, "machine_role", lambda: "jetson")
    with pytest.raises(RuntimeError, match="only runs on the PC"):
        app_module.build_arm()
    monkeypatch.setattr(app_module, "machine_role", lambda: "pc")
    monkeypatch.setenv("GZ_JOINT_NAMES", "a, b,c,d,e,f")
    source = app_module.build_arm()
    assert isinstance(source, arm.GazeboArm) and source.joint_names == list("abcdef")


# ---- app.py (HTTP + WebSocket) ----------------------------------------------------

@pytest.fixture(scope="module")
def client():
    with TestClient(app) as c:   # runs the startup hook: starts the mock arm
        assert wait_for(lambda: c.get("/api/status").json()["angles"] is not None)
        yield c


def test_status(client):
    s = client.get("/api/status").json()
    assert s["mock"] is True and s["connected"] is True
    assert len(s["angles"]) == 6 and len(s["coords"]) == 6


def test_stop(client):
    assert client.post("/api/stop").json() == {"ok": True}
    assert client.get("/api/stop").status_code != 200   # only POST stops the arm


def test_websocket_streams_snapshots(client):
    with client.websocket_connect("/ws/telemetry") as ws:
        first = ws.receive_json()
        second = ws.receive_json()
    assert len(first["angles"]) == 6
    assert second["updated"] >= first["updated"]


def test_docs_api(client):
    listed = client.get("/api/docs").json()
    assert any(d["path"] == "docs/safety.md" for d in listed)
    r = client.get("/api/docs/docs/safety.md")
    assert r.status_code == 200 and r.text.startswith("# Lab Safety")


@pytest.mark.parametrize("url", [
    "/api/docs/docs/../Makefile",
    "/api/docs/docs/..%2F..%2FMakefile",
    "/api/docs/docs%2F..%2FREADME.md",
    "/api/docs/web/server/app.py",
])
def test_docs_api_blocks_traversal(client, url):
    r = client.get(url)
    assert r.status_code == 404
    assert "MyCobot" not in r.text and "uvicorn" not in r.text


def test_sprints_api(client, fake_github):
    data = client.get("/api/sprints?refresh=1").json()
    assert [s["sprint"] for s in data["sprints"]] == [0, 1]


@pytest.mark.parametrize("path", STATIC_FILES)
def test_static_files_served(client, path):
    assert client.get("/" + path).status_code == 200


def test_index_is_root(client):
    r = client.get("/")
    assert r.status_code == 200 and "js/app.js" in r.text
