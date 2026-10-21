# mk/dev.mk - working on the code, in the `dev` container: shell run build test, plus web and clean.
#
# `dev` is the same image as the system, with the workspace mounted at /ws, so a build here is what the system runs
# after `make restart`. PC: started by `make up` or by these commands, and kept up. Jetson: a throw-away container
# per command (`run --rm`), gone when the command ends.

# `make web`: the page on this machine with mock data (no arm, no ROS, no docker). Not WEB_PORT, which the
# PC's `sim` container already uses.
WEB_DEV_PORT ?= 8080

.PHONY: shell run build test web clean

##@ Development (in the dev container)

shell: _image _dev_up ## Open a shell in dev (ROS and the workspace sourced; one per terminal)
	$(DEV_SHELL)

run: _image _dev_up ## Run CMD in dev: make run CMD='ros2 topic list'
	@test -n "$$BRACO_CMD" || { echo "usage: make run CMD='ros2 topic list'" >&2; exit 2; }
	$(call dev_cmd) bash -c "$$BRACO_CMD"

build: _image _dev_up ## colcon build of ros2_ws/src and libs (COLCON_ARGS=...; make restart to run it)
	$(call dev_cmd,$(BUILD_ENV)) $(COLCON)
	@touch $(BUILD_STAMP) $(REBUILT)

test: _image _dev_up ## colcon test, then the results (unit tests and ament lint checks; PC only)
	@[ "$(ROLE)" != jetson ] || { echo "the Jetson builds without tests (BUILD_TESTING=OFF): run make test on the PC, or make ci-ros" >&2; exit 1; }
	$(call dev_cmd) bash -c 'colcon test --base-paths $(WS_BASE_PATHS) && colcon test-result --verbose'

web: $(VENV)/.web-installed ## Serve the page with mock arm data on http://localhost:8080 (no arm, no ROS, no docker)
	ARM_MOCK=1 BRACO_ROS=0 $(VENV)/bin/uvicorn web.server.app:app --host 127.0.0.1 --port $(WEB_DEV_PORT) --reload

clean: _role ## Delete colcon's build/ install/ log/ and the ROS logs (~/.ros/log, here and in the containers)
	rm -rf $(WS_DIR)/build $(WS_DIR)/install $(WS_DIR)/log $(HOME)/.ros/log
	@if docker image inspect $(IMAGE) >/dev/null 2>&1; then \
	  $(COMPOSE) run --rm --no-deps -T $(DEV) bash -c 'rm -rf "$$HOME/.ros/log"' && echo "cleared ~/.ros/log in the containers"; fi

# ---- host venv for the page (also used by mk/ci.mk) ------------------------------------------
# web and the web tests only need the web packages, so they also work where requirements.txt (pymycobot, numpy)
# cannot be installed: the venv prefers $(PYTHON) and falls back to python3.
$(VENV)/bin/python:
	$(or $(shell command -v $(PYTHON) 2>/dev/null),python3) -m venv $(VENV)
	$(PIP) install --upgrade pip

$(VENV)/.web-installed: $(VENV)/bin/python web/requirements.txt
	$(PIP) install -r web/requirements.txt
	touch $@

$(VENV)/.web-test-installed: $(VENV)/.web-installed web/requirements-test.txt
	$(PIP) install -r web/requirements-test.txt
	touch $@
