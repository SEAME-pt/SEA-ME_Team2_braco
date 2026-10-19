# braco (web page)

The team's interface to the myCobot 280: live telemetry and control of the arm, its ROS 2 graph, the repository
docs and the sprints. It is served by the Jetson and opened from everybody's PC at the same time, and **every page
is in sync**: who is online, who controls the arm, what each person did, and the operator's control panel as it moves.

| Page | What it shows | Where it comes from |
|---|---|---|
| **Home** | arm summary, who is on the page, ROS 2 node health, activity log, current sprint, shortcuts | the other pages |
| **Arm** | 3D model, angles with soft/firmware limits, temperatures, cartesian pose, 60 s history, control (STOP, Park, Zero, wake up, per-joint sliders, gripper, LED, rest mode), activity, protocol table, limits and factory PID | `/ws` + `js/specs.js` |
| **ROS 2** | *Health*: arm_driver / temperature_monitor / joystick_teleop up or missing, driver services, `/diagnostics`. *Graph*: every node, topic (with live echo and rate), service and action. *Demos*: `/chatter`, `add_two_ints`, the `fibonacci` action with feedback and cancel, lifecycle talkers | the web server's ROS 2 node |
| **Docs** | every `.md` in the repository, grouped by topic, rendered | `GET /api/docs` |
| **Sprints** | milestone timeline, countdown, one card per `SPRINT N` issue with its sub-issues, and a page per sprint (`#/sprints/N`) | GitHub API, 5 min cache |

## Working together on the page

- **Your name** is asked once and remembered in the browser (change it with the 👤 button in the sidebar).
- **One operator at a time.** Click *Take control* in the sidebar. Everybody else sees "View only: <name> is in
  control", with your panel mirrored live (speed, step, the slider you are dragging). Every command except STOP
  needs control, and runs one at a time (a second one while the first is running is refused as *busy*).
- **STOP works for everybody**, operator or not.
- **Speed is shared**: one value held by the server, shown to everybody, changed only by the operator (viewers see
  it locked). It is capped at **80**; going above asks the operator to accept a warning, and the fast range is
  dropped again when control is released.
- Control is released by the operator, when the operator's page is closed (5 s later, so a reload keeps it), or
  after **60 s** without a command or a touch of the control panel. The sidebar shows the countdown.
- **Activity log**: commands, STOPs, failures, control changes, joins and leaves, for everybody (Home and Arm pages).
  Other people's actions also pop up as toasts.

Presence and the lease live in the server's memory: run **one** uvicorn process (`--workers 1`, the default).

## Run

**On the Jetson** the page comes with the arm container, which already runs `arm_driver` and the other arm nodes:

```bash
cd docker
make build ROLE=jetson   # once: the image now includes fastapi/uvicorn
make ws-build ROLE=jetson
make arm-up              # http://<jetson-ip>:8000 from any PC on the lab network
make arm-basics          # optional: the ros2_basics demos for ROS 2 > Demos
make arm-web-restart      # after editing web/: restarts only the page, the arm keeps running (control must be taken again)
```

There the page uses `ARM_SOURCE=ros`: it never opens the serial port, it is a client of `arm_driver`
([docker/README.md](../docker/README.md)). A move is a stream of targets at most 15° ahead of the real angles, resent
every 100 ms, exactly like `seame_arm/scripts/arm_moves.py`; if the page dies, the driver's 300 ms watchdog stops the
arm. Park uses the firmware limits and leaves the servos on; it does not enter rest mode. Wake up leaves rest mode
and powers the servos without moving the arm (rest mode is only entered with the *Rest on* button).

**On a PC, without the arm** (mock data, no ROS):

```bash
make web-mock          # http://localhost:8000
```

**With Gazebo** (PC only, inside the `pc-dev` container where ROS 2 is):

```bash
cd docker
make build          # once
make web-sim        # http://localhost:8000, with the simulation already running in Gazebo
```

The ROS 2 page then shows the PC's graph. The server finds the machine's role like the docker Makefile does
(`SEAME_ROLE`, `COMPOSE_PROFILES`, then the CPU): `ARM_SOURCE=gazebo` refuses to start on the Jetson.

**Fallback: pymycobot straight to the serial port** (only while the arm container is down, the port has one owner):

```bash
make web               # or make web-start / web-stop / web-logs, in the background
```

## Tests

```bash
make web-test
```

No arm, no ROS and no network needed:

- `tests/test_server.py`: HTTP routes, `/ws` (presence, lease, live mirror of the operator's panel, STOP for viewers,
  activity), docs path checks, GitHub sprint grouping (fake responses), cache, Gazebo source with fake messages.
- `tests/test_session.py`: the hub with a fake clock: one operator, commands locked for the others, STOP for
  everybody, one command at a time, idle expiry, reload vs closing the page, activity log.
- `tests/test_ros_arm.py`: `RosArm` with a fake bridge: joint states, bounded steps resent inside the 300 ms watchdog,
  STOP and newer commands cancel a move, timeout, park (no release, no rest), wake (no movement), gripper/LED; the ROS panels without ROS.
- `tests/test_emulated_arm.py`: the **real pymycobot** against an emulated arm on a pseudo-terminal (the fallback
  source). Skipped if pymycobot is not installed.

The host fallback targets the Jetson's Python 3.8. To test that version from a PC:

```bash
docker run --rm -v "$PWD":/src:ro python:3.8 bash -c 'cp -r /src /w && cd /w && rm -rf .venv && make web-test'
```

## Environment variables

| Variable | Default | Use |
|---|---|---|
| `ARM_SOURCE` | — | `ros`: the arm through arm_driver (the arm container sets it); `gazebo`: Gazebo over ROS 2 (PC only); unset: pymycobot on the serial port |
| `ARM_MOCK` | — | `1` uses mock data |
| `BRACO_ROS` | `1` | `0` never starts ROS 2 (health, graph and demos stay empty) |
| `GZ_JOINT_NAMES` | myCobot 280 URDF names | joints in `/joint_states` for Gazebo, comma separated |
| `ARM_PORT` / `ARM_BAUD` | `/dev/ttyTHS1` / `1000000` | serial port for the pymycobot fallback |
| `WEB_PORT` | `8000` | port in the arm container (`docker/.env`) |
| `GITHUB_TOKEN` | — | optional; raises the GitHub API limit (60 req/h without) |
| `GITHUB_REPO` | `SEAME-pt/SEA-ME_Team2_braco` | repository of the sprints |

## Layout

```
web/
  server/app.py         FastAPI: /api/*, the /ws live connection, static files
  server/session.py     presence, control lease, one command at a time, activity log
  server/arm.py         RosArm (arm_driver), GazeboArm, ArmSource (pymycobot), MockArm
  server/ros.py         the one shared rclpy node (imported lazily: the server runs without ROS)
  server/ros_health.py  arm nodes and /diagnostics
  server/ros_graph.py   read-only graph explorer and topic echo
  server/ros_demos.py   chatter, add_two_ints, fibonacci action, lifecycle talkers
  server/docs.py        finds, groups and reads every Markdown file in the repository
  server/github.py      milestones, SPRINT issues and sub-issues
  static/               plain HTML/CSS/JS, no build (three.js, marked and highlight.js from a CDN)
```

## API

Commands take the tab's id in the `X-Client-Id` header. Without control they answer **423**, while another command
runs **409**.

| Route | Description |
|---|---|
| `WS /ws` | the page's connection. First message `{"type": "hello", "id", "name", "page"}`; then the server sends `telemetry` (200 ms), `presence`, `lease`, `activity`, `ui`, `speed`, `health`, `demos`, `graph` (ROS page), `echo` (watched topic); the page sends `ui`, `page`, `rename`, `watch` / `unwatch` |
| `WS /ws/telemetry` | bare snapshot stream every 200 ms (scripts, old pages) |
| `GET /api/status` | latest arm snapshot (`capabilities` lists what the source supports) |
| `POST /api/control/take` · `/release` | take / release control |
| `GET /api/session` | clients, lease and activity log |
| `POST /api/stop` | STOP, **anyone** |
| `POST /api/pose/{park\|zero}` | move to a fixed pose at the shared speed; 409 while resting |
| `POST /api/wake` | leave rest mode and power on; the arm does not move |
| `POST /api/joint/{1-6}` `{angle}` | move one joint, inside its soft limit, at the shared speed |
| `POST /api/speed` `{value, unlocked}` | the one speed everybody shares (1-100), **operator only**. Above 80 it needs `unlocked: true` (the page asks the operator to accept a warning); releasing control puts a speed above 80 back to 80 |
| `POST /api/rest` `{on}` · `/api/gripper` `{closed}` · `/api/led` `{r, g, b}` | rest mode, gripper, Atom LED (ROS and mock sources) |
| `GET /api/ros/health` · `/graph` · `/demos` | the ROS 2 panels |
| `POST /api/ros/demos/add_two_ints` `{a, b}` | call the service |
| `POST /api/ros/demos/fibonacci` `{order}` · `/fibonacci/cancel` | send / cancel a goal |
| `POST /api/ros/demos/lifecycle/{node}` `{transition}` | configure, activate, deactivate, cleanup, shutdown |
| `GET /api/docs` · `/api/docs/{path}` | document list / Markdown of one |
| `GET /api/sprints?refresh=1` | sprint data (`refresh` skips the cache) |

## Keeping it up to date

- **New document:** put the `.md` anywhere in the repository and it shows up (its group is set in `server/docs.py`).
- **New sprint:** create the issue titled `SPRINT N | Name` and add the tasks as sub-issues on GitHub.
- **Limits, PID or commands changed in `mycobot.hpp`:** update `static/js/specs.js`.
- **arm_driver's interface changed:** update `RosArm` in `server/arm.py` (its docstring lists the topics and services).
