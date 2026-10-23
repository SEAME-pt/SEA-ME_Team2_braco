# Review — PR #64: ROS 2 workspace and package layout (#30)

Branch `30-ros-2-workspace-and-package-layout` → `main`, head `90971aa`.
126 files, +5529 / −5758. CI on the head commit: `host` ✅, `ros` ✅.

## Scope

The PR does more than the title says. Because PR #63 (#55) was merged into this branch and not into `main`, it also carries:

- the single control owner (`ControlLease`, `/arm/take_control`, `/arm/control_owner`);
- auto-rest and park on overheat (`AutoRest`, `temperature_monitor` forks `park`);
- servo voltages;
- the `braco_camera` node and the MJPEG stream on the page;
- the Control page;
- `make bench`.

On top of that come the layout work for #30: `libs/cpp` → `ros2_ws/src/mycobot_cpp`, removal of `tests/`, `playground/` and `.python-version`, and the architecture doc. The description lists all of this, but reviewers should know that most of the risky code here is from #55.

**Verdict: request changes.** One blocker in the docs, which this PR is mostly about. One safety issue in auto-rest. A few smaller items.

---

## Must fix

### 1. The ADR rename broke every link to the ADRs (≈35 links in 19 files)

Commit `90971aa` renamed `docs/ADR-0002-controle-unico.md` → `ADR-0002-unique-control.md` and `docs/ADR-0003-repouso-por-temperatura.md` → `ADR-0003-rest-per-temp.md`. No reference was updated:

```
git grep -l -e controle-unico -e repouso-por-temperatura
```

The hits include [docs/architecture.md:18](docs/architecture.md#L18) (7 links), `docs/architecture.html` (7), `docs/safety.md`, `docs/arm-rest-mode.md`, `docker/README.md`, the `braco_driver` and `braco_teleop` READMEs, `web/README.md` and the header comments of `control.hpp`, `auto_rest.hpp`, `arm_driver.cpp`, `temperature_monitor.cpp`, `joystick_teleop.cpp`, `bench.sh`, `app.py`, `arm.py` and `session.py`.

Every ADR link in the architecture page, on GitHub and in the page's Docs view, is now a 404. The commit message ("add ADR-0002 … and ADR-0003") also does not describe what the commit does, which is a rename.

**Fix:** run a `sed` over the repo for both old names. Then add a link check to `make ci` so this cannot happen silently again: a grep for `](…\.md)` targets that do not exist would do.

### 2. Auto-rest has no hysteresis: a servo hovering at 64–65 °C re-parks the arm on every crossing

[auto_rest.hpp:35](ros2_ws/src/braco_driver/include/braco_driver/auto_rest.hpp#L35) ends the episode on the first reading below `TEMP_HOT_C`, and the next reading ≥ 65 starts a new one. Temperatures arrive every 2 s at 1 °C resolution, so jitter at the threshold is the normal case for a hot arm.

Each new episode runs `park` again ([temperature_monitor.cpp:184](ros2_ws/src/braco_driver/src/temperature_monitor.cpp#L184)), and `park` does the following:

- calls `setRest(false)` first ([park.cpp:35](ros2_ws/src/braco_bringup/src/park.cpp#L35)), so rest mode is briefly **off** on a hot arm;
- if the released joints have sagged more than 2° from the park pose, sends `ALL_ANGLES`, which **re-energises the hot servos**;
- releases the joints again and sets rest mode.

This also contradicts ADR-0003's goal of "no fight with a person who resumes on purpose". An operator who resumes at 64–65 °C is overruled on the next upward crossing.

**Fix:** end the episode only below `TEMP_HOT_C - N` (for example below `TEMP_WARM_C`, or 60 °C). Add a gtest for 65 → 64 → 65 producing a single rest. Separately, consider having the hot-path park skip `setRest(false)` and the move when the driver is already in rest mode and the arm is within tolerance of the park pose.

---

## Should fix

### 3. Entering rest mode does not stop the arm, and it disables the watchdog stop

`setReadOnly(true)` → `clearPending()` sets `moving_ = false` but never sends STOP ([arm_driver.cpp:211-223](ros2_ws/src/braco_driver/src/arm_driver.cpp#L211-L223), [363-369](ros2_ws/src/braco_driver/src/arm_driver.cpp#L363-L369)). Because `moving_` is now false, `checkWatchdog()` will not stop the arm either. A joint that was moving keeps going to its last target, which can be up to 15° ahead for web/park steps.

The comment at [temperature_monitor.cpp:234](ros2_ws/src/braco_driver/src/temperature_monitor.cpp#L234) says "arm_driver stops the arm and refuses motion", which is not what happens. `release_servos` has the same pattern.

**Fix:** in `setReadOnly(true)`, set `stop_requested_ = true` under `state_mutex_` (as the forced `take` already does).

### 4. A command already taken by the serial thread is still sent after rest mode / a forced take

`serviceCommands()` swaps the pending command out under `state_mutex_`, releases the lock, then sends it ([arm_driver.cpp:424-465](ros2_ws/src/braco_driver/src/arm_driver.cpp#L424-L465)). If `set_read_only true` or a forced `take_control` lands in between, one more motion frame goes out after the driver has "refused" motion.

This window is small and #3's STOP would follow it, but a re-check of `read_only_` just before `sendAngles`/`sendAngle` closes it for free.

### 5. `make bench` step 3 cannot fail, and step 5 can leak into step 6

- **Step 3** ([bench.sh:91-104](scripts/bench.sh#L91-L104)): each `ros2 topic pub --once` through `docker compose exec` takes about 1–2 s to start. By the time the non-owner STOP is published, the driver's 300 ms watchdog has already stopped the arm, so the operator always sees it stop. To test the STOP path, stream the JOINTS command (`-r 10` in the background) and publish the STOP while it runs.
- **Step 5** ([bench.sh:127-143](scripts/bench.sh#L127-L143)): `kill $FAKE_PID` kills the local `docker compose exec` client, not necessarily the `ros2 topic pub` inside the container, which runs until its `timeout 45`. Fake 66 °C readings can then continue into step 6.

  The fakes also interleave with the real 2 s readings, which end the episode every time (see #2). The result: repeated parks during step 5, and a likely spurious FAIL "rest mode came back after resume" in step 6. Run the publisher with `docker compose exec … sh -c 'exec timeout …'` and stop it with `pkill -f` in the container. Or, better, add a parameter to `temperature_monitor` for a test input topic.
- **`cleanup`** ([bench.sh:61-66](scripts/bench.sh#L61-L66)) always calls `set_read_only false` on exit. If the arm really went hot during the bench, or the operator aborts with Ctrl+C right after step 5, the bench silently undoes a real auto-rest. Restore the rest state the bench found at the start instead.

### 6. The web page holds the arm's control for up to 60 s without moving

`Hub.tick()` renews `/arm/take_control` every second while a lease is held ([session.py:363](web/server/session.py#L363)), and the lease's idle timeout is 60 s. In practice `control_idle_s: 5.0` never applies to the page. A person who opens the Control page and walks away blocks the joystick for a minute, unless they know to hold button 8.

This may be intended. If so, state it in ADR-0002 and the `braco.yaml` comment, which currently says "the control owner loses it after this long without motion". If not, renew only while the page is actually commanding.

### 7. The whole host `/dev` is bind-mounted read-write into `real`

`- /dev:/run/hostdev` ([compose.yml:139](docker/compose.yml#L139)). The cgroup rule limits which **device nodes** can be opened. But the mount also exposes the host's `/dev/shm`, `/dev/mqueue` and any other non-device files in `/dev`, writable. Suggestions:

- mount it `:ro` (opening a char device for writing is not blocked by a read-only mount, so V4L2 still works); or
- mount only `/dev/v4l` and require a `v4l/by-id/...` name in `CAMERA_DEVICE`.

---

## Minor / nits

- **Orphaned park on monitor respawn.** `temperature_monitor` is `respawn=True`. If it dies while its forked `park` runs, the child keeps running and the new monitor has fresh `AutoRest` state, so a hot reading starts a second `park` concurrently. Both use source `"script"`, so both are accepted. Low risk (same target), but consider `prctl(PR_SET_PDEATHSIG, SIGTERM)` in the child, or killing the process group on shutdown.
- **`take_pending_` can stick** ([joystick_teleop.cpp:209](ros2_ws/src/braco_teleop/src/joystick_teleop.cpp#L209)). If a `take_control` request is never answered, the joystick never asks again until the node respawns. A timestamp with a timeout is enough. Mostly theoretical, since the launch shuts down when `arm_driver` exits.
- **`release_servos` / `power_on` / `set_gripper` are not owner-gated.** ADR-0002 relies on clients only calling them while they own the control. That is fine for now, but `release_servos` drops the arm. Note it in the ADR's consequences, or add `source` to those requests later (#40/#43?).
- **Docs HTML runs same-origin.** [docs.js:100](web/static/js/views/docs.js#L100) loads `docs/*.html` with `srcdoc` and no `sandbox`, so any script in a docs page runs with the app's origin. `architecture.html` has no `<script>` today; `sandbox="allow-same-origin"` (no `allow-scripts`) keeps the link rewriting working and closes this.
- **`format-check` now installs the whole dev venv.** It depends on `$(VENV)/.installed` ([ci.mk:80](mk/ci.mk#L80)), which installs `requirements-dev.txt` (pymycobot, numpy) on every fresh clone before the pre-push hook can run. Depending only on the formatter binaries would keep the hook light.
- **`PYTHON ?= python3`** replaces the pinned `.python-version` while CI pins 3.11. A dev on 3.12/3.13 now gets a venv that CI never tests. It is probably fine, but say so in `dev-setup.md`.
- **`AutoRest::active()`** is true for the whole hot episode, also when the operator has resumed. The LCD then still shows "REST: SERVO HOT" while the arm is accepting motion. Cosmetic.
- **Stale `%ld` for an `int`** in `arm_driver`'s "serial port … baud" log. This is pre-existing, but the line is in the diff context and `-Wformat` will flag it.

---

## What I checked

- Read the full diff of `braco_driver` (`arm_driver`, `temperature_monitor`, `control.hpp`, `auto_rest.hpp`, tests), `braco_bringup` (`arm_moves`, `park`, `zero`, launch, yaml), `joystick_teleop`, `braco_camera`, `web/server` (`arm.py`, `session.py`, `app.py`, `camera.py`, `docs.py`), `scripts/bench.sh`, `docker/`, `mk/` and CI.
- The `libs/cpp` → `mycobot_cpp` move and the removals of `tests/` and `playground/`: no remaining references outside the history notes in `dev-setup.md` and `architecture.md`.
- CI is green on `90971aa`. I did not run the web tests locally (the local `.venv` has a broken pytest), and nothing was run on hardware.

## What is good

- `ControlLease` and `AutoRest` are kept free of ROS and are unit-tested. That is the right shape for this logic.
- Taking and releasing control publishes the owner under the same lock that the idle expiry uses, so `/arm/control_owner` cannot go out of order.
- The joystick never sends STOP while it is not the owner, and the page cancels its stream without a STOP when it loses control. Both are subtle and both are right.
- The launch now shuts down when `arm_driver` exits, instead of leaving a headless system running.
- The repository root is much easier to read, and `architecture.md`/`.html` are thorough. Once the links in #1 are fixed they will be the best entry point to the project.
