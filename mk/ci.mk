# mk/ci.mk - what .github/workflows/ci.yml runs, reproducible here: ci ci-ros lint.
#
#   host job   make ci       host checks (venv, imports, C++ hello-world build), lint, web page tests
#   ros job    make ci-ros   rosdep, colcon build and colcon test of ros2_ws/src + libs + examples
#
# No robot and no display needed.

# The `ros` job builds the workspace in a throw-away container from the Dockerfile's `base` stage: everything the
# packages depend on, without the GUI/Gazebo/MoveIt layers of the PC image. The image's user gets your UID/GID so
# the bind-mounted ros2_ws stays writable (GitHub runners are UID 1001). CI builds the same image with its own cache.
CI_TARGET ?= base
CI_IMAGE  ?= $(IMAGE_PREFIX)/ci:$(IMAGE_TAG)
CI_ROS_SH := set -e; rosdep update --rosdistro "$$ROS_DISTRO" >/dev/null 2>&1 || true; \
             sudo apt-get update -qq; rosdep install --from-paths $(WS_BASE_PATHS) --ignore-src -y; \
             $(COLCON); colcon test --base-paths $(WS_BASE_PATHS); colcon test-result --verbose
HADOLINT_IGNORE := DL3008 DL3013 DL3059 DL3006 DL3042 DL3045 SC1091 SC2086 SC2016

.PHONY: ci ci-ros lint _host-check

##@ CI (.github/workflows/ci.yml)

ci: _host-check lint $(VENV)/.web-test-installed ## The `host` CI job: host checks, lint, web page tests
	$(VENV)/bin/python -m pytest web/tests -q

ci-ros: ## The `ros` CI job: rosdep + colcon build + colcon test in a throw-away container (CI_IMAGE built if missing)
	@docker image inspect $(CI_IMAGE) >/dev/null 2>&1 || \
	  docker build --target $(CI_TARGET) --build-arg USER_UID=$$(id -u) --build-arg USER_GID=$$(id -g) -t $(CI_IMAGE) $(DOCKER_DIR)
	docker run --rm -e CCACHE_DIR=/ws/.ccache -e CCACHE_MAXSIZE=500M -v $(CURDIR)/$(WS_DIR):/ws \
	    -v $(CURDIR)/$(LIBS_DIR):$(CONTAINER_LIBS):ro -v $(CURDIR)/$(EXAMPLES_DIR):$(CONTAINER_EXAMPLES) \
	    $(CI_IMAGE) bash -c '$(CI_ROS_SH)'

lint: $(ENV_FILE) ## Compose files (both machines, GPU overrides), shellcheck and hadolint (each if installed)
	@echo "== compose =="
	@for r in pc jetson; do COMPOSE_PROFILES=$$r $(DC) config -q && echo "  $(COMPOSE_BASE) ($$r): ok"; done
	@for g in nvidia dri; do COMPOSE_PROFILES=pc docker compose --project-directory $(DOCKER_DIR) -f $(COMPOSE_BASE) -f $(COMPOSE_DIR)/$$g.yml config -q && echo "  $(COMPOSE_DIR)/$$g.yml: ok"; done
	@echo "== shell scripts =="
	@if command -v shellcheck >/dev/null 2>&1; then shellcheck -x $(SCRIPTS_DIR)/*.sh $(TESTS_DIR)/*.sh && echo "  shellcheck: clean"; else echo "  shellcheck not installed - skipped"; fi
	@echo "== Dockerfile =="
	@if command -v hadolint >/dev/null 2>&1; then hadolint $(addprefix --ignore ,$(HADOLINT_IGNORE)) $(DOCKERFILE) && echo "  hadolint: clean"; else echo "  hadolint not installed - skipped"; fi

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
