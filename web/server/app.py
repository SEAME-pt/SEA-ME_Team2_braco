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

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles

from . import docs, github
from .arm import DEFAULT_BAUD, DEFAULT_PORT, ArmSource, MockArm

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
