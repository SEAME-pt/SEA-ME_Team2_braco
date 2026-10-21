"""The shared session: presence, control lease, one command at a time, activity log.

Unit tests on a Hub with a fake clock and no event loop (broadcasts are dropped);
the websocket side is tested through the app in test_server.py.
"""

import threading

import pytest

from web.server.session import BusyError, ControlError, Hub, LockedError, clean_name


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def hub(clock):
    return Hub(idle_timeout=60, grace=5, clock=clock)


class FakeControl:
    """arm_driver's control as the hub sees it: `owner` outside the page refuses the take."""

    def __init__(self):
        self.owner = ""
        self.calls = []

    def take_control(self):
        self.calls.append("take")
        if self.owner not in ("", "web"):
            raise ControlError("the arm is controlled by " + self.owner)
        self.owner = "web"

    def release_control(self):
        self.calls.append("release")
        if self.owner == "web":
            self.owner = ""


@pytest.fixture
def control():
    return FakeControl()


@pytest.fixture
def ruled(clock, control):
    """A hub tied to arm_driver's control, as app.py builds it."""
    return Hub(idle_timeout=60, grace=5, clock=clock, control=control)


def join(hub, cid, name):
    queue = object()
    hub.connect(cid, name, "10.0.0.{}".format(len(cid)), queue)
    return queue


def test_clean_name():
    assert clean_name("  Ana   Maria ") == "Ana Maria"
    assert clean_name("") == "Guest" and clean_name(None) == "Guest"
    assert len(clean_name("x" * 100)) == 32


def test_presence_and_welcome(hub):
    welcome = hub.connect("a", "Ana", "10.0.0.1", object())
    assert welcome[0] == {"type": "welcome", "you": "a", "name": "Ana", "idle_timeout": 60}
    join(hub, "b", "Bruno")
    names = [c["name"] for c in hub.presence()]
    assert names == ["Ana", "Bruno"]
    assert all(c["online"] for c in hub.presence())
    assert [e["action"] for e in hub.activity()] == ["joined", "joined"]


def test_only_one_operator(hub):
    join(hub, "a", "Ana")
    join(hub, "b", "Bruno")
    hub.take("a")
    assert hub.lease()["holder"] == {"id": "a", "name": "Ana"}
    with pytest.raises(LockedError, match="controlled by Ana"):
        hub.take("b")
    assert hub.release("b") is False   # only the holder can release
    assert hub.release("a") is True
    hub.take("b")
    assert hub.is_operator("b") and not hub.is_operator("a")


def test_take_needs_a_live_connection(hub):
    with pytest.raises(LockedError):
        hub.take("ghost")


def test_commands_need_the_lease(hub):
    join(hub, "a", "Ana")
    join(hub, "b", "Bruno")
    with pytest.raises(LockedError, match="take control first"):
        hub.run("a", "pose zero", lambda: None)
    hub.take("a")
    assert hub.run("a", "pose zero", lambda: 42) == 42
    with pytest.raises(LockedError, match="controlled by Ana"):
        hub.run("b", "pose park", lambda: None)
    with pytest.raises(LockedError):
        hub.run(None, "pose park", lambda: None)   # no X-Client-Id at all


def test_stop_is_for_everybody(hub):
    join(hub, "a", "Ana")
    join(hub, "b", "Bruno")
    hub.take("a")
    calls = []
    hub.run("b", "STOP", lambda: calls.append("stop"), operator=False)
    hub.run(None, "STOP", lambda: calls.append("stop"), operator=False, ip="10.1.1.1")
    assert calls == ["stop", "stop"]
    who = [(e["who"], e["action"]) for e in hub.activity() if e["action"] == "STOP"]
    assert who == [("Bruno", "STOP"), ("someone at 10.1.1.1", "STOP")]


def test_one_command_at_a_time(hub):
    join(hub, "a", "Ana")
    hub.take("a")
    started, finish = threading.Event(), threading.Event()

    def slow():
        started.set()
        finish.wait(2)

    worker = threading.Thread(target=hub.run, args=("a", "wake up", slow))
    worker.start()
    assert started.wait(2)
    with pytest.raises(BusyError):
        hub.run("a", "pose zero", lambda: None)
    # STOP is never blocked by a running command
    hub.run("a", "STOP", lambda: None, operator=False)
    finish.set()
    worker.join()
    hub.run("a", "pose zero", lambda: None)


def test_failed_command_is_logged_and_raised(hub):
    join(hub, "a", "Ana")
    hub.take("a")

    def broken():
        raise RuntimeError("arm not connected")

    with pytest.raises(RuntimeError):
        hub.run("a", "pose zero", broken)
    last = hub.activity()[-1]
    assert last["action"] == "pose zero" and last["ok"] is False and last["detail"] == "arm not connected"
    assert last["by"] == "a"
    hub.run("a", "pose zero", lambda: None)   # the command lock was released


def test_idle_lease_expires(hub, clock):
    join(hub, "a", "Ana")
    hub.take("a")
    clock.now += 30
    hub.run("a", "J1 -> 10", lambda: None)   # a command refreshes the lease
    clock.now += 50
    hub.tick()
    assert hub.lease()["holder"] is not None
    assert hub.lease()["idle_left"] == 10
    clock.now += 11
    hub.tick()
    assert hub.lease()["holder"] is None
    assert "idle" in hub.activity()[-1]["action"]


def test_ui_from_the_operator_refreshes_the_lease(hub, clock):
    join(hub, "a", "Ana")
    join(hub, "b", "Bruno")
    hub.take("a")
    clock.now += 50
    hub.ui("a", {"speed": 40})
    clock.now += 50
    hub.tick()
    assert hub.is_operator("a")
    hub.ui("b", {"speed": 99})   # ignored: not the operator
    assert hub._ui["state"] == {"speed": 40}


def test_reload_keeps_lease_closing_frees_it(hub, clock):
    q = join(hub, "a", "Ana")
    hub.take("a")
    hub.disconnect("a", q)
    clock.now += 3
    q = join(hub, "a", "Ana")   # page reload within the grace period
    hub.tick()
    assert hub.is_operator("a")
    assert [e["action"] for e in hub.activity()].count("joined") == 1

    hub.disconnect("a", q)
    clock.now += 6
    hub.tick()
    assert hub.lease()["holder"] is None
    assert hub.presence() == []
    actions = [e["action"] for e in hub.activity()]
    assert "released control (left the page)" in actions and actions[-1] == "left"


def test_two_tabs_same_client(hub, clock):
    q1 = join(hub, "a", "Ana")
    join(hub, "a", "Ana")
    hub.disconnect("a", q1)
    clock.now += 10
    hub.tick()
    assert hub.presence()[0]["online"] is True


def test_rename(hub):
    join(hub, "a", "Ana")
    hub.take("a")
    hub.rename("a", "Ana Paula")
    assert hub.lease()["holder"]["name"] == "Ana Paula"
    assert hub.activity()[-1]["action"] == "renamed (was Ana)"


def test_page_and_clients_on(hub):
    join(hub, "a", "Ana")
    join(hub, "b", "Bruno")
    hub.set_page("a", "ros")
    hub.set_page("b", "arm")
    assert hub.clients_on("ros") == {"a"}


def test_activity_is_bounded(hub):
    for i in range(300):
        hub.log("arm", "event {}".format(i))
    entries = hub.activity()
    assert len(entries) == 200 and entries[-1]["action"] == "event 299"


def test_speed_needs_the_lease_and_the_fast_range_is_unlocked_on_purpose(hub):
    join(hub, "a", "Ana")
    join(hub, "b", "Bruno")
    with pytest.raises(LockedError):
        hub.set_speed("a", 40)
    hub.take("a")
    with pytest.raises(LockedError):
        hub.set_speed("b", 40)
    assert hub.set_speed("a", 80)["value"] == 80
    for bad in (0, 101, 50.5, True, "50"):
        with pytest.raises(ValueError):
            hub.set_speed("a", bad, fast=True)
    with pytest.raises(ValueError):
        hub.set_speed("a", 81)   # above the limit without the warning accepted
    assert hub.speed == 80
    assert hub.set_speed("a", 100, fast=True) == {"type": "speed", "value": 100, "fast": True, "limit": 80, "max": 100}
    assert hub.activity()[-1]["action"] == "speed 100"


def test_released_lease_caps_the_speed(hub):
    join(hub, "a", "Ana")
    hub.take("a")
    hub.set_speed("a", 95, fast=True)
    hub.release("a")
    assert hub.speed == 80
    hub.take("a")
    hub.set_speed("a", 20)
    hub.release("a")
    assert hub.speed == 20   # a safe speed is kept for the next operator


# ---- the lease follows arm_driver's control owner ------------------------------------

def test_take_asks_the_driver(ruled, control):
    join(ruled, "a", "Ana")
    ruled.take("a")
    assert control.owner == "web" and control.calls == ["take"]
    ruled.release("a")
    assert control.owner == "" and control.calls == ["take", "release"]


def test_take_refused_while_the_joystick_has_the_arm(ruled, control):
    join(ruled, "a", "Ana")
    control.owner = "joystick"
    with pytest.raises(ControlError, match="joystick"):
        ruled.take("a")
    assert ruled.lease()["holder"] is None


def test_tick_keeps_the_control_while_the_lease_lasts(ruled, control):
    join(ruled, "a", "Ana")
    ruled.take("a")
    ruled.tick()
    ruled.tick()
    assert control.calls == ["take"] * 3
    ruled.release("a")
    ruled.tick()   # no lease: nothing to keep
    assert control.calls == ["take"] * 3 + ["release"]


def test_forced_control_ends_the_lease(ruled, control):
    join(ruled, "a", "Ana")
    ruled.take("a")
    control.owner = "joystick"   # held button 8
    ruled.control_lost("joystick")
    assert ruled.lease()["holder"] is None
    assert "release" not in control.calls   # nothing to give back: it is the joystick's
    assert ruled.activity()[-1]["action"] == "released control (joystick took control)"
    with pytest.raises(ControlError):
        ruled.take("a")


def test_tick_notices_a_lost_control(ruled, control):
    join(ruled, "a", "Ana")
    ruled.take("a")
    control.owner = "script"   # make zero, before the owner topic arrived
    ruled.tick()
    assert ruled.lease()["holder"] is None
    assert "script" in ruled.activity()[-1]["action"]


def test_tick_survives_an_unreachable_driver(ruled, control):
    join(ruled, "a", "Ana")
    ruled.take("a")

    def gone():
        raise RuntimeError("service /arm/take_control is not available")
    control.take_control = gone
    ruled.tick()
    assert ruled.lease()["holder"]["id"] == "a"   # a restart is not a lost control
