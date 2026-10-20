# Roadmap: issue #16 "CI: Github Action build"

Branch: `16-ci-gh-action` · PR: [#51](https://github.com/SEAME-pt/SEA-ME_Team2_braco/pull/51) · Goal: every PR builds
the workspace automatically (green CI/CD pipeline).

_Last updated: 2026-10-20._

## 1. Status

| Issue #16 checklist item | State | Notes |
|---|---|---|
| Workflow with ROS 2 container, rosdep and colcon build | **Done** | `.github/workflows/ci.yml`, jobs `host` and `ros`; green on PR #51 |
| Cache dependencies to keep runs fast | **Done** | pip, BuildKit layers (`type=gha`), ccache + colcon `build/`; `ros` job 5 m 56 s cold → ~1 m 30 s warm |
| Require the check on main | **Not done** | Repo setting, needs admin rights; possible now that `host` and `ros` have run |

Delivered on the branch:

- `docker/Makefile` merged into the single root `Makefile`; `docker/README.md` and docs updated. Tracked as sub-issue
  [#52 "Merge Makefiles"](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/52) (closed, completed).
- flake8/pep257 fixes in `seame_arm` (`arm.launch.py`, `arm_moves.py`, `zero.py`).
- `PYTHON` override, host-architecture detection, host Python pinned to 3.11 (`.python-version`).
- Hardware-free `host-check`; `make ci` (= `host` job) and `make ci-image ci-ros` (= `ros` job).
- `origin/main` (PR #50, web page) merged; its `web*`, `arm-web-*`, `arm-basics` and `web-sim` targets ported.
- `.github/workflows/ci.yml`, CI section and badge in `docs/dev-setup.md`.

## 2. Blockers found

1. ~~**`.python-version` is missing from the repo.**~~ Added (`3.11`); `make host-check` passes from a clean tree.
2. ~~**The branch is 17 commits behind `main`.**~~ Merged in 95cdb5f. Conflicts in `.gitignore`, `Makefile` and
   `docker/Makefile` (modify/delete) resolved; all web targets kept with `##` help strings. Stale `cd docker` /
   "docker/Makefile" references in `web/` updated.
3. ~~`make lint` skips shellcheck and hadolint silently.~~ The `host` job installs shellcheck. hadolint can no longer be
   skipped (from `temp_notes.md`: it was skipped on dev machines without the binary): `make lint` falls back to the
   `HADOLINT_IMAGE` (`hadolint/hadolint:v2.12.0`) through docker, which lint already needs for the compose files. CI
   uses the same image, so the version lives only in the Makefile. Verified: binary, image, first pull, and a bad
   Dockerfile failing lint.
4. ~~`requirements.txt` pins on Python 3.11.~~ `numpy==1.24.4` and `pyserial==3.4` install on the runner's 3.11.

Found while implementing:

5. ~~**`make lint` fails without `docker/.env`.**~~ The `arm` service's `group_add` has `${I2C_GID:-${USER_GID}}` and
   `${INPUT_GID:-${USER_GID}}`; with both unset they collapse to the same GID and compose rejects the duplicate. The
   `host` job runs `make env` first, as every user does. **Still open for the robot:** a Jetson with neither an `i2c`
   nor an `input` group would hit the same error at `make arm-up`.
6. ~~**Flaky `web/tests/test_emulated_arm.py::test_arm_stops_answering`** (2 failures in 10 full-suite runs).~~ It was
   always skipped on main (its venv had no pymycobot); the merged venv runs it. The test asserted the usual notice
   time (< 5 s, measured 3.1 s), but the poller's worst case is 6 missed reads × 1.5 s ≈ 9.2 s (measured 1 in 40), when
   the arm goes silent right after an answered `get_angles`. The bound now follows that worst case; `arm.py` unchanged.

## 3. Steps to finish (in order)

1. [x] **Add `.python-version`** containing `3.11`. Verified from a clean tree: `make host-check`.
2. [x] **Merge `origin/main` into the branch** and resolve the conflicts; `make lint` and `make host-check` rerun.
3. [x] **Create `.github/workflows/ci.yml`**, triggered on `pull_request`, `push` to `main` and `workflow_dispatch`, with
       `concurrency` (cancels superseded PR runs only) and `permissions: contents: read`:
   - [x] Job `host`: checkout, `setup-python` (`python-version-file: .python-version`, `cache: pip`), shellcheck +
         `make env`, `make host-check`, `make lint`, `make web-test`. Locally: `make env && make ci`.
   - [x] Job `ros`: the Dockerfile's `base` stage (Ubuntu 26.04 + ROS 2 Lyrical, no GUI/Gazebo/MoveIt), then
         `rosdep install`, `colcon build`, `colcon test` of `docker/ws/src` + `libs`. Locally: `make ci-image ci-ros`
         (5 packages, 83 tests, 0 failures, 11 skipped).
4. [x] **Cache dependencies**: pip via `setup-python`; Docker layers via `docker/build-push-action` with
       `type=gha`; ccache + colcon `build/` via `actions/cache` (key reset when `docker/docker/` changes). apt/rosdep is
       not cached separately: every package the workspace needs is already in the cached `base` image.

       | Run | `host` | `ros` |
       |---|---|---|
       | Cold ([39417217737](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/runs/39417217737)) | 57 s | 5 m 56 s |
       | Warm ([39426515641](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/runs/39426515641)) | 49 s | 1 m 33 s |
       | Warm ([39427082498](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/runs/39427082498)) | 55 s | 1 m 30 s |

5. [x] **Validate**:
   - [x] Green run: [39426515641](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/runs/39426515641).
   - [x] Red run on purpose (`pc-dev` without its `:` in `docker/compose/dri.yml`, 65f7784): `host` failed at
         `make lint` in 19 s, [39426768638](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/runs/39426768638).
   - [x] Reverted (e5ae956), green again: [39427082498](https://github.com/SEAME-pt/SEA-ME_Team2_braco/actions/runs/39427082498).
   - [ ] Push the flaky-test fix (blocker 6, e3b169d) and the hadolint fallback, and get a green run. e3b169d's subject
         starts with a space (`" test: …"`), which `.githooks/commit-msg` rejects; hooks are not enabled in this clone
         (`make init`). Reword it before pushing.
6. [x] **Docs**: "Continuous Integration (CI)" section and status badge in `docs/dev-setup.md`; `make ci-image ci-ros`
       in `docker/README.md`.
7. [x] **Finish the PR**: #51 retitled `ci: GitHub Action build`; body follows `.github/PULL_REQUEST_TEMPLATE.md` with
       `Closes #16`, the run links and the times from step 4. Its "green run including the flaky-test fix" box is
       ticked once that run exists.
8. [ ] **After merge**: an admin enables branch protection on `main` and marks the `host` and `ros` jobs as required
       status checks (third checklist item of the issue).

## 4. Definition of Done for the PR

**Manual tests**
- [x] `make ci` passes locally from a clean tree (after `make env`); `make ci-image ci-ros` passes locally.
- [x] Link to a green GitHub Action run, and to the intentional red run from step 5.
- [x] Cold vs warm run time noted.

**Quality & documentation**
- [x] `docs/dev-setup.md` and `docker/README.md` describe the CI and how to reproduce it locally.
- [x] Merge conflicts with `main` resolved; no web targets lost from the Makefile.
- [ ] Review approved.

## 5. Open questions

- ~~Which ROS 2 image should CI use?~~ The project Dockerfile's `base` stage: same ROS distro and packages as both real
  images, and with `type=gha` layer caching it costs ~1 m 30 s on a warm run.
- ~~Should the arm64/Jetson image stay out of CI?~~ Yes: manual `make jetson-cross` (QEMU too slow).
- Who has admin rights to set the required check on `main`?
- Blocker 5 on the robot: give `I2C_GID` / `INPUT_GID` fallbacks that cannot collide, or have `make env` fail when a
  group is missing on the Jetson? (Out of scope for #16.)
- The merge commit 95cdb5f is titled `test: deliberate problem`; squash-merging #51 with the PR title would keep it out
  of `main`'s history.
