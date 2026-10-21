"""braco: the team's web interface to the myCobot 280.

Live telemetry and control of the arm, the ROS 2 graph, docs and sprints,
shared in real time by everybody who has the page open (see session.py).

    ARM_MOCK=1 uvicorn web.server.app:app --port 8000      # PC, no arm
    ARM_SOURCE=ros uvicorn web.server.app:app --host 0.0.0.0 --port 8000
                                                           # Jetson, inside the arm container

Run ONE process (no --workers > 1): presence, the control lease and the
activity log live in memory.

Environment:
    ARM_MOCK=1         synthetic data instead of the arm
    ARM_SOURCE=ros     the arm through arm_driver (ROS 2); what the arm container uses
    ARM_SOURCE=gazebo  arm simulated in Gazebo over ROS 2 (PC only: make web-sim
                       in docker/ runs it inside the pc-dev container)
    (neither)          the arm straight through pymycobot on ARM_PORT / ARM_BAUD
    BRACO_ROS=0        do not start ROS 2 at all (health, graph and demos stay empty)
    GZ_JOINT_NAMES     joint names in /joint_states, comma separated
    ARM_PORT           serial port (default /dev/ttyTHS1)
    ARM_BAUD           baud rate (default 1000000)
    GITHUB_TOKEN       optional, raises the GitHub API rate limit
"""

import asyncio
import json
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from . import docs, github, ros
from .arm import (
    DEFAULT_BAUD,
    DEFAULT_PORT,
    POSES,
    SOFT_LIMITS,
    ArmSource,
    GazeboArm,
    MockArm,
    RestingError,
    RosArm,
)
from .role import machine_role
from .ros_demos import FIB_MAX_ORDER, Demos
from .ros_graph import Graph
from .ros_health import Health
from .session import BusyError, Hub, LockedError, drain, new_queue

STATIC = Path(__file__).resolve().parents[1] / 'static'
PUSH_PERIOD = 0.2  # telemetry
HEALTH_EVERY = 5  # x PUSH_PERIOD: health, demos, lease tick
GRAPH_EVERY = 10  # x PUSH_PERIOD: graph, to the clients on the ROS page
ECHO_EVERY = 3  # x PUSH_PERIOD: watched topics
ROS_ENABLED = os.environ.get('BRACO_ROS', '1') != '0'


def build_arm():
    if os.environ.get('ARM_MOCK') == '1':
        return MockArm()
    source = os.environ.get('ARM_SOURCE', '')
    if source == 'ros':
        return RosArm()
    if source == 'gazebo':
        role = machine_role()
        if role != 'pc':
            raise RuntimeError(
                'ARM_SOURCE=gazebo only runs on the PC (this machine is the {})'.format(role)
            )
        names = os.environ.get('GZ_JOINT_NAMES')
        return GazeboArm([n.strip() for n in names.split(',')] if names else None)
    return ArmSource(
        os.environ.get('ARM_PORT', DEFAULT_PORT),
        int(os.environ.get('ARM_BAUD', DEFAULT_BAUD)),
    )


arm = build_arm()
hub = Hub()
health = Health(ros.bridge)
graph = Graph(ros.bridge)
demos = Demos(ros.bridge)
_latest = {}  # last health / demos sent, for new connections and change detection


async def _pump():
    """Telemetry for everybody, plus the slower ROS panels and the lease clock."""
    loop = asyncio.get_running_loop()
    last_notice = None
    tick = 0
    while True:
        await asyncio.sleep(PUSH_PERIOD)
        tick += 1
        try:
            snapshot = arm.snapshot()
            hub.broadcast({'type': 'telemetry', 'data': snapshot})
            notice = snapshot.get('notice')
            if notice and notice != last_notice:
                last_notice = notice
                hub.log('arm', notice['text'], ok=False)

            if tick % HEALTH_EVERY == 0:
                hub.tick()
                for name, source in (('health', health), ('demos', demos)):
                    data = await loop.run_in_executor(None, source.snapshot)
                    text = json.dumps(data, sort_keys=True, default=str)
                    if _latest.get(name, (None,))[0] != text:
                        _latest[name] = (text, data)
                        hub.broadcast({'type': name, 'data': data})

            if tick % GRAPH_EVERY == 0:
                viewers = hub.clients_on('ros')
                if viewers:
                    data = await loop.run_in_executor(None, graph.snapshot)
                    hub.broadcast({'type': 'graph', 'data': data}, only=viewers)

            if tick % ECHO_EVERY == 0:
                for _topic, (viewers, payload) in graph.echoes().items():
                    hub.broadcast(dict(payload, type='echo'), only=viewers)
        except Exception as e:  # one bad snapshot must not stop the stream
            print('braco: pump error: {}'.format(e))


@asynccontextmanager
async def lifespan(_app):
    hub.bind(asyncio.get_running_loop())
    if ROS_ENABLED:
        for part in (health, demos):
            part.start()
    else:
        ros.bridge.error = 'ROS 2 disabled (BRACO_ROS=0)'
    arm.start()
    pump = asyncio.create_task(_pump())
    yield
    pump.cancel()
    demos.close()
    arm.close()
    ros.bridge.close()


app = FastAPI(title='braco', lifespan=lifespan)


# ---- commands: every one goes through the hub (lease, one at a time, activity log) ----


def _client(request):
    return request.headers.get('x-client-id') or None, (
        request.client.host if request.client else None
    )


def command(request, action, fn, operator=True):
    cid, ip = _client(request)
    try:
        return hub.run(cid, action, fn, operator=operator, ip=ip)
    except LockedError as e:
        raise HTTPException(status_code=423, detail=str(e))
    except (BusyError, RestingError) as e:
        raise HTTPException(status_code=409, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except NotImplementedError as e:
        raise HTTPException(status_code=501, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=503, detail=str(e))


def capable(name):
    if name not in arm.snapshot().get('capabilities', []):
        raise NotImplementedError("'{}' is not available with this arm source".format(name))


@app.get('/api/status')
def status():
    return arm.snapshot()


@app.post('/api/stop')
def stop(request: Request):
    """Anyone may stop the arm, operator or not."""
    command(request, 'STOP', arm.stop_motion, operator=False)
    return {'ok': True}


@app.post('/api/pose/{name}')
def pose(name: str, request: Request):
    """Move to a fixed pose at the shared speed."""
    if name not in POSES:
        raise HTTPException(status_code=404, detail='unknown pose')
    speed = hub.speed
    command(
        request,
        'pose {} (speed {})'.format(name, speed),
        lambda: arm.go_to_pose(name, speed),
    )
    return {'ok': True, 'pose': name, 'angles': POSES[name], 'speed': speed}


@app.post('/api/wake')
def wake(request: Request):
    """Leave rest mode and power the servos on. The arm does not move."""
    command(request, 'wake up', arm.wake_up)
    return {'ok': True}


class JointMove(BaseModel):
    angle: float


@app.post('/api/joint/{index}')
def joint(index: int, move: JointMove, request: Request):
    """Move one joint (1-6) inside its soft limits."""
    if not 1 <= index <= 6:
        raise HTTPException(status_code=404, detail='joint must be 1-6')
    lo, hi = SOFT_LIMITS[index - 1]
    if not lo <= move.angle <= hi:
        raise HTTPException(
            status_code=422,
            detail='J{} must be between {} and {} degrees'.format(index, lo, hi),
        )
    speed = hub.speed
    command(
        request,
        'J{} -> {:g} deg (speed {})'.format(index, move.angle, speed),
        lambda: arm.move_joint(index - 1, move.angle, speed),
    )
    return {'ok': True, 'joint': index, 'angle': move.angle, 'speed': speed}


class Speed(BaseModel):
    value: int
    unlocked: bool = False  # the operator accepted the warning for speeds above the limit


@app.post('/api/speed')
def set_speed(body: Speed, request: Request):
    """The one speed everybody shares. Operator only; above 80 needs unlocked=true."""
    cid, _ip = _client(request)
    try:
        return hub.set_speed(cid, body.value, body.unlocked)
    except LockedError as e:
        raise HTTPException(status_code=423, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))


class Switch(BaseModel):
    on: bool


@app.post('/api/rest')
def rest(body: Switch, request: Request):
    """Rest mode on/off without moving (arm_driver refuses motion while on)."""

    def run():
        capable('rest')
        arm.set_rest(body.on)

    command(request, 'rest mode ' + ('on' if body.on else 'off'), run)
    return {'ok': True, 'on': body.on}


class Gripper(BaseModel):
    closed: bool


@app.post('/api/gripper')
def gripper(body: Gripper, request: Request):
    def run():
        capable('gripper')
        arm.set_gripper(body.closed)

    command(request, 'gripper ' + ('close' if body.closed else 'open'), run)
    return {'ok': True, 'closed': body.closed}


class Led(BaseModel):
    r: int = Field(ge=0, le=255)
    g: int = Field(ge=0, le=255)
    b: int = Field(ge=0, le=255)


@app.post('/api/led')
def led(body: Led, request: Request):
    def run():
        capable('led')
        arm.set_led(body.r, body.g, body.b)

    command(request, 'LED #{:02x}{:02x}{:02x}'.format(body.r, body.g, body.b), run)
    return {'ok': True}


# ---- control lease ------------------------------------------------------------------


@app.post('/api/control/take')
def take_control(request: Request):
    cid, _ip = _client(request)
    try:
        hub.take(cid)
    except LockedError as e:
        raise HTTPException(status_code=423, detail=str(e))
    return hub.lease()


@app.post('/api/control/release')
def release_control(request: Request):
    cid, _ip = _client(request)
    hub.release(cid)
    return hub.lease()


@app.get('/api/session')
def session():
    return {'clients': hub.presence(), 'lease': hub.lease(), 'activity': hub.activity()}


# ---- ROS 2 ------------------------------------------------------------------------------


@app.get('/api/ros/health')
def ros_health():
    return health.snapshot()


@app.get('/api/ros/graph')
def ros_graph():
    return graph.snapshot()


@app.get('/api/ros/demos')
def ros_demos():
    return demos.snapshot()


class AddTwoInts(BaseModel):
    a: int
    b: int


@app.post('/api/ros/demos/add_two_ints')
def demo_add_two_ints(body: AddTwoInts, request: Request):
    cid, ip = _client(request)
    who = hub.name_of(cid, ip)
    total = command(
        request,
        'add_two_ints({}, {})'.format(body.a, body.b),
        lambda: demos.add_two_ints(body.a, body.b, who),
    )
    return {'ok': True, 'sum': total}


class FibonacciGoal(BaseModel):
    order: int = Field(ge=1, le=FIB_MAX_ORDER)


@app.post('/api/ros/demos/fibonacci')
def demo_fibonacci(body: FibonacciGoal, request: Request):
    cid, ip = _client(request)
    who = hub.name_of(cid, ip)
    command(
        request,
        'fibonacci goal (order {})'.format(body.order),
        lambda: demos.fibonacci_start(body.order, who),
    )
    return {'ok': True}


@app.post('/api/ros/demos/fibonacci/cancel')
def demo_fibonacci_cancel(request: Request):
    command(request, 'fibonacci cancel', demos.fibonacci_cancel)
    return {'ok': True}


class Transition(BaseModel):
    transition: str


@app.post('/api/ros/demos/lifecycle/{node}')
def demo_lifecycle(node: str, body: Transition, request: Request):
    state = command(
        request,
        '{}: {}'.format(node, body.transition),
        lambda: demos.lifecycle_transition(node, body.transition),
    )
    return {'ok': True, 'state': state}


# ---- websockets ---------------------------------------------------------------------


@app.websocket('/ws')
async def live(ws: WebSocket):
    """The page's one connection: telemetry, presence, lease, activity, ROS panels.

    The browser's first message must be {"type": "hello", "id", "name", "page"}.
    """
    await ws.accept()
    loop = asyncio.get_running_loop()
    queue = new_queue()
    cid = None
    sender = None
    try:
        hello = await ws.receive_json()
        if not isinstance(hello, dict) or hello.get('type') != 'hello' or not hello.get('id'):
            await ws.close(code=4400)
            return
        cid = str(hello['id'])[:64]
        ip = ws.client.host if ws.client else None
        for message in hub.connect(cid, hello.get('name'), ip, queue):
            await ws.send_json(message)
        await ws.send_json({'type': 'telemetry', 'data': arm.snapshot()})
        for name in ('health', 'demos'):
            if name in _latest:
                await ws.send_json({'type': name, 'data': _latest[name][1]})
        hub.set_page(cid, hello.get('page'))
        sender = asyncio.create_task(drain(queue, ws.send_json))
        while True:
            message = await ws.receive_json()
            if not isinstance(message, dict):
                continue
            kind = message.get('type')
            if kind == 'ui':
                hub.ui(cid, message.get('state'))
            elif kind == 'page':
                hub.set_page(cid, message.get('page'))
                if message.get('page') == 'ros':
                    data = await loop.run_in_executor(None, graph.snapshot)
                    queue.put_nowait({'type': 'graph', 'data': data})
            elif kind == 'rename':
                hub.rename(cid, message.get('name'))
            elif kind == 'watch' and message.get('topic'):
                topic = str(message['topic'])
                error = await loop.run_in_executor(None, graph.watch, topic, cid)
                if error:
                    queue.put_nowait({'type': 'echo', 'topic': topic, 'error': error})
            elif kind == 'unwatch' and message.get('topic'):
                graph.unwatch(str(message['topic']), cid)
    except (WebSocketDisconnect, RuntimeError, ValueError):
        pass
    finally:
        if sender is not None:
            sender.cancel()
        if cid is not None:
            hub.disconnect(cid, queue)
            graph.drop_viewer(cid)


@app.websocket('/ws/telemetry')
async def telemetry(ws: WebSocket):
    """Bare snapshot stream (older pages and scripts)."""
    await ws.accept()
    try:
        while True:
            await ws.send_json(arm.snapshot())
            await asyncio.sleep(PUSH_PERIOD)
    except (WebSocketDisconnect, RuntimeError):
        pass


# ---- docs and sprints ---------------------------------------------------------------


@app.get('/api/docs')
def list_docs():
    return docs.list_docs()


@app.get('/api/docs/{path:path}', response_class=PlainTextResponse)
def read_doc(path: str):
    text = docs.read_doc(path)
    if text is None:
        raise HTTPException(status_code=404, detail='document not found')
    return text


@app.get('/api/sprints')
def sprints(refresh: bool = False):
    return github.sprints(force=refresh)


app.mount('/', StaticFiles(directory=str(STATIC), html=True), name='static')
