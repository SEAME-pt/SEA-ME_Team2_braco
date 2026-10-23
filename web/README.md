# braco (web page)

The team's interface to the myCobot 280: live telemetry and control of the arm, its camera, its ROS 2 graph, the repository
docs and the sprints. It is served by the Jetson and opened from everybody's PC at the same time, and **every page
is in sync**: who is online, who controls the arm, what each person did, and the operator's control panel as it moves.

| Page | What it shows | Where it comes from |
|---|---|---|
| **Home** | arm summary, who is on the page, ROS 2 node health, activity log, current sprint, shortcuts | the other pages |
| **Arm** | 3D model, angles with soft/firmware limits, temperatures, the camera, cartesian pose, 60 s history, activity, protocol table, limits and factory PID | `/ws` + `js/specs.js` + `/api/camera/stream` |
| **Control** | the only page that moves the arm: STOP, Park, Zero, wake up, shared speed, per-joint sliders, gripper, LED, rest mode, with the camera beside them; activity | `/ws` + `/api/camera/stream` |
| **ROS 2** | *Health*: arm_driver / temperature_monitor / joystick_teleop up or missing, driver services, `/diagnostics`. *Graph*: every node, topic (with live echo and rate), service and action | the web server's ROS 2 node |
| **Docs** | every `.md` in the repository, grouped by topic, rendered | `GET /api/docs` |
| **Sprints** | milestone timeline, countdown, one card per `SPRINT N` issue with its sub-issues, and a page per sprint (`#/sprints/N`) | GitHub API, 5 min cache |

## Working together on the page

- **Your name** is asked once and remembered in the browser (change it with the 👤 button in the sidebar).
- **One operator at a time.** Click *Take control* in the sidebar. Everybody else sees "View only: <name> is in
  control", with your panel mirrored live (speed, step, the slider you are dragging). Every command except STOP
  needs control, and runs one at a time (a second one while the first is running is refused as *busy*).
- **STOP works for everybody**, operator or not, and stops the arm whoever moves it (the joystick too).
- **The page is one client among others.** On the real arm, taking control also asks `arm_driver` for the arm
  ([ADR-0002](../docs/ADR-0002-unique-control.md)): while the **joystick** or `make zero` / `make park` has it,
  *Take control* is refused (409) and the page shows "the joystick is in control". While the page has it, the
  joystick is ignored unless someone holds its button 8 for about 1 s; then the page's control ends at once.
- **Speed is shared**: one value held by the server, shown to everybody, changed only by the operator (viewers see
  it locked). It is capped at **80**; going above asks the operator to accept a warning, and the fast range is
  dropped again when control is released.
- Control is released by the operator, when the operator's page is closed (5 s later, so a reload keeps it), or
  after **60 s** without a command or a touch of the control panel. The sidebar shows the countdown.
- **The camera** (eye-in-hand) is shown live on the Arm and Control pages to everybody, operator or not. It comes
  from the `camera` node ([braco_camera](../ros2_ws/src/braco_camera/README.md)) through
  `/camera/image_raw/compressed`: the page never opens the camera. While no frames arrive (camera unplugged, node
  down, `make web` without ROS) it shows why and opens the stream again once they come back. *⤓ Frame* saves the
  current frame as a JPEG.
- **Activity log**: commands, STOPs, failures, control changes, joins and leaves, for everybody (Home and Arm pages).
  Other people's actions also pop up as toasts.

Presence and the lease live in the server's memory: run **one** uvicorn process (`--workers 1`, the default).

## Run

The page is part of the braco system: `make up` starts it with the rest, on port 8000 (`WEB_PORT` in `docker/.env`).

**On the Jetson** it runs in the `real` container, next to `arm_driver` and the other arm nodes:

```bash
make up                  # http://<jetson-ip>:8000 from any PC on the lab network
make logs S=web          # only the page's log lines
make restart             # after editing web/: restarts the system container; the arm is not zeroed or parked
```

There the page uses `ARM_SOURCE=ros`: it never opens the serial port, it is a client of `arm_driver`
([docker/README.md](../docker/README.md)). Without `ARM_SOURCE` or `ARM_MOCK=1` the server refuses to start: there
is no direct serial mode. Its lease is `arm_driver`'s control owner `web`, renewed every second while it lasts; the
temperature levels come from `/diagnostics` (`temperature_monitor`). A move is a stream of targets at most 15° ahead of the real angles, resent
every 100 ms, exactly like `braco_bringup/src/arm_moves.cpp`; if the page dies, the driver's 300 ms watchdog stops the
arm. Park uses the firmware limits and, like `make park`, releases the servos and enters rest mode once it arrives (STOP or a newer command before that cancels it). Wake up leaves rest mode
and powers the servos without moving the arm (rest mode is only entered with the *Rest on* button).

**On a PC** it runs in the `sim` container with `ARM_SOURCE=gazebo` (http://localhost:8000):

```bash
make up                  # the page shows the arm as not connected until the Gazebo bringup (#34) exists
```

The ROS 2 page then shows the PC's graph. The server finds the machine's role like the root Makefile does
(`SEAME_ROLE`, `COMPOSE_PROFILES`, then the CPU): `ARM_SOURCE=gazebo` refuses to start on the Jetson.

**Without the arm, ROS or docker** (mock data, for working on the page itself):

```bash
make web                 # http://localhost:8080, reloads when a file changes
```

## Tests

```bash
make ci          # with the host checks and lint; the tests alone: .venv/bin/python -m pytest web/tests -q
```

No arm, no ROS and no network needed:

- `tests/test_server.py`: HTTP routes, `/ws` (presence, lease, live mirror of the operator's panel, STOP for viewers,
  activity), docs path checks, GitHub sprint grouping (fake responses), cache, Gazebo source with fake messages.
- `tests/test_session.py`: the hub with a fake clock: one operator, commands locked for the others, STOP for
  everybody, one command at a time, idle expiry, reload vs closing the page, activity log; with a fake driver
  control: taking asks it, refused while the joystick has it, kept alive every tick, lost when forced away.
- `tests/test_server.py` also covers the camera with fake frames: online/offline, rate, staleness, snapshot and
  the MJPEG stream.
- `tests/test_ros_arm.py`: `RosArm` with a fake bridge: joint states, bounded steps resent inside the 300 ms watchdog,
  STOP and newer commands cancel a move, timeout, park (releases and rests on arrival, not when cancelled), wake (no movement), gripper/LED,
  `source: web` on every command, take/release control, losing it ends a move without STOP, temperature levels
  from `/diagnostics`; the ROS panels without ROS.

The host fallback targets the Jetson's Python 3.8. To test that version from a PC:

```bash
docker run --rm -v "$PWD":/src:ro python:3.8 bash -c 'cp -r /src /w && cd /w && rm -rf .venv && make .venv/.web-test-installed && .venv/bin/python -m pytest web/tests -q'
```

## Environment variables

| Variable | Default | Use |
|---|---|---|
| `ARM_SOURCE` | — | `ros`: the arm through arm_driver (the `real` container sets it); `gazebo`: Gazebo over ROS 2 (PC only). One of `ARM_SOURCE` / `ARM_MOCK` is required |
| `ARM_MOCK` | — | `1` uses mock data |
| `BRACO_ROS` | `1` | `0` never starts ROS 2 (health and graph stay empty) |
| `GZ_JOINT_NAMES` | myCobot 280 URDF names | joints in `/joint_states` for Gazebo, comma separated |
| `WEB_PORT` | `8000` | port of the page in the `real` / `sim` container (`docker/.env`) |
| `GITHUB_TOKEN` | — | optional; raises the GitHub API limit (60 req/h without) |
| `GITHUB_REPO` | `SEAME-pt/SEA-ME_Team2_braco` | repository of the sprints |

## Layout

```
web/
  server/app.py         FastAPI: /api/*, the /ws live connection, static files
  server/session.py     presence, control lease, one command at a time, activity log
  server/arm.py         RosArm (arm_driver), GazeboArm, MockArm
  server/ros.py         the one shared rclpy node (imported lazily: the server runs without ROS)
  server/ros_health.py  arm nodes and /diagnostics
  server/ros_graph.py   read-only graph explorer and topic echo
  server/camera.py      the camera node's JPEG frames as an MJPEG stream
  server/docs.py        finds, groups and reads every Markdown file in the repository
  server/github.py      milestones, SPRINT issues and sub-issues
  static/               plain HTML/CSS/JS, no build (three.js, marked and highlight.js from a CDN)
```

## API

Commands take the tab's id in the `X-Client-Id` header. Without control they answer **423**, while another command
runs **409**.

| Route | Description |
|---|---|
| `WS /ws` | the page's connection. First message `{"type": "hello", "id", "name", "page"}`; then the server sends `telemetry` (200 ms), `presence`, `lease`, `activity`, `ui`, `speed`, `health`, `graph` (ROS page), `echo` (watched topic); the page sends `ui`, `page`, `rename`, `watch` / `unwatch` |
| `WS /ws/telemetry` | bare snapshot stream every 200 ms (scripts, old pages) |
| `GET /api/status` | latest arm snapshot (`capabilities` lists what the source supports) |
| `POST /api/control/take` · `/release` | take / release control; take answers **409** while the joystick or zero/park has the arm |
| `GET /api/session` | clients, lease and activity log |
| `POST /api/stop` | STOP, **anyone** |
| `POST /api/pose/{park\|zero}` | move to a fixed pose at the shared speed; 409 while resting |
| `POST /api/wake` | leave rest mode and power on; the arm does not move |
| `POST /api/joint/{1-6}` `{angle}` | move one joint, inside its soft limit, at the shared speed |
| `POST /api/speed` `{value, unlocked}` | the one speed everybody shares (1-100), **operator only**. Above 80 it needs `unlocked: true` (the page asks the operator to accept a warning); releasing control puts a speed above 80 back to 80 |
| `POST /api/rest` `{on}` · `/api/gripper` `{closed}` · `/api/led` `{r, g, b}` | rest mode, gripper, Atom LED (ROS and mock sources) |
| `GET /api/ros/health` · `/graph` | the ROS 2 panels |
| `GET /api/camera/status` | `{online, fps, age, bytes, error}` of `/camera/image_raw/compressed` |
| `GET /api/camera/stream` | MJPEG (`multipart/x-mixed-replace`) of the camera, at most 15 fps per viewer, for an `<img>`; ends when the camera goes offline, **503** if it already is |
| `GET /api/camera/snapshot` | the newest frame as `image/jpeg`; **503** without one |
| `GET /api/docs` · `/api/docs/{path}` | document list / Markdown of one |
| `GET /api/sprints?refresh=1` | sprint data (`refresh` skips the cache) |

## Keeping it up to date

- **New document:** put the `.md` anywhere in the repository and it shows up (its group is set in `server/docs.py`).
- **New sprint:** create the issue titled `SPRINT N | Name` and add the tasks as sub-issues on GitHub.
- **Limits, PID or commands changed in `mycobot.hpp`:** update `static/js/specs.js`.
- **arm_driver's interface changed:** update `RosArm` in `server/arm.py` (its docstring lists the topics and services).
