# mk/ci.mk - what .github/workflows/ci.yml runs, reproducible here: ci ci-ros lint.
#
#   host job   make ci       host checks (venv, imports, C++ hello-world build), lint, web page tests
#   ros job    make ci-ros   rosdep, colcon build and colcon test of ros2_ws/src + libs (make ci-image builds its image)
#
# No robot and no display needed.

# The `ros` job builds the workspace in a throw-away container from the Dockerfile's `base` stage: everything the
# packages depend on, without the GUI/Gazebo/MoveIt layers of the PC image. The image's user gets your UID/GID so
# the bind-mounted ros2_ws stays writable (GitHub runners are UID 1001).
# ci.yml builds the same image with docker/build-push-action (layer cache): keep its target and build-args in step.
# CI caches ros2_ws/build, where test results live: delete the old ones first, or a removed test would still count.
# colcon test exits 0 even when tests fail; colcon test-result is what fails the job.
CI_TARGET ?= base
CI_IMAGE  ?= $(IMAGE_PREFIX)/ci:$(IMAGE_TAG)
CI_ROS_SH := set -e; rosdep update --rosdistro "$$ROS_DISTRO" >/dev/null 2>&1 || echo "rosdep update failed - using the rosdep cache from the image"; \
             sudo apt-get update -qq; rosdep install --from-paths $(WS_BASE_PATHS) --ignore-src -y; \
             $(COLCON); \
             colcon test-result --delete-yes >/dev/null 2>&1 || true; \
             colcon test --base-paths $(WS_BASE_PATHS); colcon test-result --verbose
BUILDX_MISSING := docker buildx not found: the Dockerfile needs BuildKit (apt install docker-buildx, or docker-buildx-plugin from Docker's repo)

# shellcheck and hadolint: a local binary if installed, else the pinned image through docker, so no check is ever
# skipped (lint needs docker anyway, for the compose files).
SHELLCHECK_IMAGE := koalaman/shellcheck:v0.10.0
# SC1091: scripts source /etc/os-release and ROS setup files that exist only in the image (and not at all in the
# scratch-based shellcheck image), so following them would make the result depend on where lint runs.
SHELLCHECK_FLAGS := -x -e SC1091
SHELLCHECK_FILES := $(SCRIPTS_DIR)/*.sh $(TESTS_DIR)/*.sh
HADOLINT_IMAGE   := hadolint/hadolint:v2.12.0
HADOLINT_FLAGS   := $(addprefix --ignore ,DL3008 DL3013 DL3059 DL3006 DL3042 DL3045 SC1091 SC2086 SC2016)

.PHONY: ci ci-image ci-ros lint _host-check

##@ CI (.github/workflows/ci.yml)

ci: _host-check lint $(VENV)/.web-test-installed ## The `host` CI job: host checks, lint, web page tests
	$(VENV)/bin/python -m pytest web/tests -q

ci-image: ## Build the `ros` CI job's image (Dockerfile stage CI_TARGET=base) as CI_IMAGE, for your UID/GID
	@docker buildx version >/dev/null 2>&1 || { echo "$(BUILDX_MISSING)" >&2; exit 1; }
	docker buildx build --load --target $(CI_TARGET) --build-arg USER_UID=$$(id -u) --build-arg USER_GID=$$(id -g) -t $(CI_IMAGE) $(DOCKER_DIR)

ci-ros: ## The `ros` CI job: rosdep + colcon build + colcon test in a throw-away container (CI_IMAGE built if missing)
	@docker image inspect $(CI_IMAGE) >/dev/null 2>&1 || $(MAKE) --no-print-directory ci-image
	docker run --rm -e CCACHE_DIR=/ws/.ccache -e CCACHE_MAXSIZE=500M -v $(CURDIR)/$(WS_DIR):/ws \
	    -v $(CURDIR)/$(LIBS_DIR):$(CONTAINER_LIBS):ro $(CI_IMAGE) bash -c '$(CI_ROS_SH)'

lint: $(ENV_FILE) ## Compose files (both machines, GPU overrides), shellcheck and hadolint (binaries or their docker images)
	@echo "== compose =="
	@for r in pc jetson; do COMPOSE_PROFILES=$$r $(DC) config -q && echo "  $(COMPOSE_BASE) ($$r): ok"; done
	@for g in nvidia dri; do COMPOSE_PROFILES=pc docker compose --project-directory $(DOCKER_DIR) -f $(COMPOSE_BASE) -f $(COMPOSE_DIR)/$$g.yml config -q && echo "  $(COMPOSE_DIR)/$$g.yml: ok"; done
	@echo "== shell scripts =="
	@if command -v shellcheck >/dev/null 2>&1; then shellcheck $(SHELLCHECK_FLAGS) $(SHELLCHECK_FILES) && echo "  shellcheck: clean"; \
	else \
	  { docker image inspect $(SHELLCHECK_IMAGE) >/dev/null 2>&1 || docker pull -q $(SHELLCHECK_IMAGE) >/dev/null; } && \
	  docker run --rm -v "$(CURDIR)":/mnt:ro -w /mnt $(SHELLCHECK_IMAGE) $(SHELLCHECK_FLAGS) $(SHELLCHECK_FILES) && \
	  echo "  shellcheck ($(SHELLCHECK_IMAGE)): clean"; fi
	@echo "== Dockerfile =="
	@if command -v hadolint >/dev/null 2>&1; then hadolint $(HADOLINT_FLAGS) $(DOCKERFILE) && echo "  hadolint: clean"; \
	else \
	  { docker image inspect $(HADOLINT_IMAGE) >/dev/null 2>&1 || docker pull -q $(HADOLINT_IMAGE) >/dev/null; } && \
	  docker run --rm -i $(HADOLINT_IMAGE) hadolint $(HADOLINT_FLAGS) - < $(DOCKERFILE) && echo "  hadolint ($(HADOLINT_IMAGE)): clean"; fi

# Hardware-free checks on the host: the venv from requirements.txt, imports, the C++ hello-world build.
_host-check: $(VENV)/.installed
	$(VENV)/bin/python -c 'import ast; ast.parse(open("tests/hello_world.py").read())'
	$(VENV)/bin/python -c 'import numpy, pymycobot, serial'
	cmake -S $(CPP_DIR) -B $(CPP_BUILD)
	cmake --build $(CPP_BUILD)

$(VENV)/.installed: requirements.txt .python-version
	@$(PYTHON) -c 'import sys; v="%d.%d" % sys.version_info[:2]; sys.exit(0 if v == "$(PY_VERSION)" else "need Python $(PY_VERSION), $(PYTHON) is " + v + " (install it, e.g. uv python install $(PY_VERSION), or set PYTHON=/path/to/python$(PY_VERSION))")'
	rm -rf $(VENV)
	$(PYTHON) -m venv $(VENV)
	$(PIP) install --upgrade pip
	$(PIP) install -r requirements.txt
	touch $@
