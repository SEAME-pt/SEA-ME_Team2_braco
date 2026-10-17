"""Developer portal for the myCobot 280: live telemetry, docs and sprints.

    ARM_MOCK=1 uvicorn web.server.app:app --port 8000      # PC, no arm
    uvicorn web.server.app:app --host 0.0.0.0 --port 8000  # Jetson

Environment:
    ARM_MOCK=1       synthetic data instead of the serial port
    ARM_PORT         serial port (default /dev/ttyTHS1)
    ARM_BAUD         baud rate (default 1000000)
    GITHUB_TOKEN     optional, raises the GitHub API rate limit
"""

import asyncio
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from . import docs, github
from .arm import (DEFAULT_BAUD, DEFAULT_PORT, POSE_SPEED, POSES, SOFT_LIMITS,
                  ArmSource, MockArm, RestingError)

STATIC = Path(__file__).resolve().parents[1] / "static"
PUSH_PERIOD = 0.2

if os.environ.get("ARM_MOCK") == "1":
    arm = MockArm()
else:
    arm = ArmSource(os.environ.get("ARM_PORT", DEFAULT_PORT),
                    int(os.environ.get("ARM_BAUD", DEFAULT_BAUD)))


@asynccontextmanager
async def lifespan(_app):
    arm.start()
    yield
    arm.close()


app = FastAPI(title="braco", lifespan=lifespan)


@app.get("/api/status")
def status():
    return arm.snapshot()


@app.post("/api/stop")
def stop():
    try:
        arm.stop_motion()
    except Exception as e:
        raise HTTPException(status_code=503, detail=str(e))
    return {"ok": True}


@app.post("/api/pose/{name}")
def pose(name: str, speed: int = Query(POSE_SPEED, ge=1, le=100)):
    if name not in POSES:
        raise HTTPException(status_code=404, detail="unknown pose")
    try:
        arm.go_to_pose(name, speed)
    except RestingError as e:
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=503, detail=str(e))
    return {"ok": True, "pose": name, "angles": POSES[name], "speed": speed}


@app.post("/api/wake")
def wake(speed: int = Query(POSE_SPEED, ge=1, le=100)):
    """Leave rest: power the servos on and move to the zero pose."""
    try:
        arm.wake_up(speed)
    except Exception as e:
        raise HTTPException(status_code=503, detail=str(e))
    return {"ok": True}


class JointMove(BaseModel):
    angle: float
    speed: int = Field(POSE_SPEED, ge=1, le=100)


@app.post("/api/joint/{index}")
def joint(index: int, move: JointMove):
    """Move one joint (1-6) inside its soft limits."""
    if not 1 <= index <= 6:
        raise HTTPException(status_code=404, detail="joint must be 1-6")
    lo, hi = SOFT_LIMITS[index - 1]
    if not lo <= move.angle <= hi:
        raise HTTPException(status_code=422,
                            detail="J{} must be between {} and {} degrees".format(index, lo, hi))
    try:
        arm.move_joint(index - 1, move.angle, move.speed)
    except RestingError as e:
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=503, detail=str(e))
    return {"ok": True, "joint": index, "angle": move.angle, "speed": move.speed}


@app.websocket("/ws/telemetry")
async def telemetry(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(arm.snapshot())
            await asyncio.sleep(PUSH_PERIOD)
    except (WebSocketDisconnect, RuntimeError):
        pass


@app.get("/api/docs")
def list_docs():
    return docs.list_docs()


@app.get("/api/docs/{path:path}", response_class=PlainTextResponse)
def read_doc(path: str):
    text = docs.read_doc(path)
    if text is None:
        raise HTTPException(status_code=404, detail="document not found")
    return text


@app.get("/api/sprints")
def sprints(refresh: bool = False):
    return github.sprints(force=refresh)


app.mount("/", StaticFiles(directory=str(STATIC), html=True), name="static")
