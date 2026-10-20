# Roadmap: issue #16 "CI: Github Action build"

Branch: `16-ci-gh-action` · Goal: every PR builds the workspace automatically (green CI/CD pipeline).

## 1. Status

| Issue #16 checklist item | State | Notes |
|---|---|---|
| Workflow with ROS 2 container, rosdep and colcon build | **Not done** | `.github/` only holds `PULL_REQUEST_TEMPLATE.md`; no `workflows/` yet |
| Cache dependencies to keep runs fast | **Not done** | Depends on the workflow existing |
| Require the check on main | **Not done** | Repo setting, needs admin rights; only possible after the workflow has run once |

Already delivered on the branch (6 commits ahead of the merge-base):

- `docker/Makefile` merged into the single root `Makefile`; `docker/README.md` and docs updated.
- flake8/pep257 fixes in `seame_arm` (`arm.launch.py`, `arm_moves.py`, `zero.py`).
- `PYTHON` override, host-architecture detection, host Python pinned to 3.11.
- Hardware-free `host-check` and `ci: host-check lint` targets (the local equivalent of the future Action).

## 2. Blockers found

1. **`.python-version` is missing from the repo.** `Makefile` reads it (`PY_VERSION := $(shell cat .python-version)`, and
   `$(VENV)/.installed` depends on it), so `make host-venv`, `host-check` and `ci` fail on a clean checkout and in CI.
2. **The branch is 17 commits behind `main`** (main now contains merged PR #50, the web page). main changed both
   `Makefile` (`web*`, `web-deps`, `web-test`) and `docker/Makefile` (`arm-web-logs`, `arm-web-restart`, `arm-basics`,
   `web-sim`), while this branch deleted `docker/Makefile`. Expect conflicts. main also adds `web/requirements*.txt`
   and `web/tests/` (pytest) that CI can run.
3. `make lint` skips shellcheck and hadolint silently when they are not installed, so CI must install them or the
   lint job proves less than it appears to.
4. `requirements.txt` pins `numpy==1.24.4` and `pyserial==3.4`; confirm they install on Python 3.11 on the runner.

## 3. Steps to finish (in order)

1. [ ] **Add `.python-version`** containing `3.11` and commit it. Verify from a clean clone: `make host-check`.
2. [ ] **Merge `origin/main` into the branch** and resolve the conflicts: port the `web*`, `arm-web-*`, `arm-basics` and
       `web-sim` targets into the merged root `Makefile` (keep the `##` help comments so `make help` lists them), fix
       paths for the files moved from `docker/Makefile`, then rerun `make lint` and `make host-check`.
3. [ ] **Create `.github/workflows/ci.yml`**, triggered on `pull_request` and `push` to `main`, with `concurrency`
       cancelling superseded runs and `permissions: contents: read`:
   - [ ] Job `host`: `actions/checkout`, `actions/setup-python` (`python-version-file: .python-version`, `cache: pip`),
         install `shellcheck`/`hadolint`, run `make ci`, then `make web-test`.
   - [ ] Job `ros`: build in the ROS 2 container (the `pc-dev` stage of `docker/docker/Dockerfile`, or the matching
         `ros:` base image), run `rosdep install`, `colcon build` and `colcon test` for `docker/ws/src` and `libs`.
         Reuse `make ws-deps ws-build ws-test` / `make run CMD=...` so CI and local use the same commands.
4. [ ] **Cache dependencies**: pip (via `setup-python`), apt/rosdep, ccache and colcon `build/` with `actions/cache`,
       Docker layers with `docker/build-push-action` and `cache-from/cache-to: type=gha`. Record a cold and a warm run time
       in the PR description.
5. [ ] **Validate**: run `make ci` locally, push, get a green run; then push a deliberate failure (e.g. a lint error) to
       confirm the check goes red, and revert it.
6. [ ] **Docs**: add a short "CI" section to `docs/dev-setup.md` (what runs, how to reproduce locally with `make ci`) and
       a status badge; update `docker/README.md` if the container commands changed.
7. [ ] **Open the PR**: title `ci: GitHub Action build`, body following `.github/PULL_REQUEST_TEMPLATE.md`, containing
       `Closes #16`. Commit messages must pass `.githooks/commit-msg` (conventional style as in the existing history).
8. [ ] **After merge**: an admin enables branch protection on `main` and marks the `host` and `ros` jobs as required
       status checks (third checklist item of the issue).

## 4. Definition of Done for the PR

**Manual tests**
- [ ] `make ci` passes locally from a clean clone.
- [ ] Link to a green GitHub Action run, and to the intentional red run from step 5.
- [ ] Cold vs warm run time noted.

**Quality & documentation**
- [ ] `docs/dev-setup.md` (and `docker/README.md` if needed) describe the CI and how to reproduce it locally.
- [ ] Merge conflicts with `main` resolved; no web targets lost from the Makefile.
- [ ] Review approved.

## 5. Open questions

- Which ROS 2 image should CI use? The Dockerfile targets ubuntu:26.04 with ROS Lyrical; building it in CI is slow
  without layer caching, a prebuilt `ros:` base may be enough.
- Should the arm64/Jetson image stay out of CI (QEMU build is slow)? Suggested: yes, keep it manual (`make jetson-cross`).
- Who has admin rights to set the required check on `main`?
