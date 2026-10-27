# mk/dev.mk - working on the code, in the `dev` container: shell run build test sim-gui, plus clean.
#
# `dev` is the same image as the system, with the workspace mounted at /ws, so a build here is what the system runs
# after `make restart`. PC: started by `make up` or by these commands, and kept up. Jetson: a throw-away container
# per command (`run --rm`), gone when the command ends.

.PHONY: shell run build test sim-gui clean

##@ Development (in the dev container)

shell: _image _dev_up ## Open a shell in dev (ROS and the workspace sourced; one per terminal)
	$(DEV_SHELL)

run: _image _dev_up ## Run CMD in dev: make run CMD='ros2 topic list'
	@test -n "$$BRACO_CMD" || { echo "usage: make run CMD='ros2 topic list'" >&2; exit 2; }
	$(call dev_cmd) bash -c "$$BRACO_CMD"

build: _image _dev_up ## colcon build of ros2_ws/src (COLCON_ARGS=...; make restart to run it)
	$(call dev_cmd,$(BUILD_ENV)) $(COLCON)
	@touch $(BUILD_STAMP) $(REBUILT)

test: _image _dev_up ## colcon test, then the results (unit tests and ament lint checks; PC only)
	@[ "$(ROLE)" != jetson ] || { echo "the Jetson builds without tests (BUILD_TESTING=OFF): run make test on the PC, or make ci-ros" >&2; exit 1; }
	$(call dev_cmd) bash -c 'colcon test --base-paths $(WS_BASE_PATHS) && colcon test-result --verbose'

# A Gazebo window on the simulation the `sim` container already runs (its GZ_PARTITION), not a second simulation:
# a second gazebo.launch.py on the same ROS domain fights it for /clock and /controller_manager.
SIM_GZ_ENV := -e GZ_PARTITION=braco-sim -e GZ_SIM_RESOURCE_PATH=/ws/install/braco_description/share
sim-gui: _image _dev_up ## PC: open the Gazebo window on the running simulation (make up SIM=1 first; Ctrl-C closes only the window)
	@[ "$(ROLE)" = pc ] || { echo "sim-gui: the simulation runs on the PC" >&2; exit 1; }
	@[ -n "$$($(COMPOSE) ps -q --status running sim)" ] || { echo "sim is not running: make up SIM=1 first" >&2; exit 1; }
	$(call dev_cmd,$(SIM_GZ_ENV)) ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=-g

clean: _role ## Delete colcon's build/ install/ log/ and the ROS logs (~/.ros/log, here and in the containers)
	rm -rf $(WS_DIR)/build $(WS_DIR)/install $(WS_DIR)/log $(HOME)/.ros/log
	@if docker image inspect $(IMAGE) >/dev/null 2>&1; then \
	  $(COMPOSE) run --rm --no-deps -T $(DEV) bash -c 'rm -rf "$$HOME/.ros/log"' && echo "cleared ~/.ros/log in the containers"; fi

# ---- host venv for the page's tests (mk/ci.mk) -----------------------------------------------
# The page and its tests only need web/requirements*.txt (no arm library, no ROS), so they also work where
# requirements-dev.txt (pymycobot, numpy) cannot be installed.
$(VENV)/bin/python:
	$(PYTHON) -m venv $(VENV)
	$(PIP) install --upgrade pip

$(VENV)/.web-installed: $(VENV)/bin/python web/requirements.txt
	$(PIP) install -r web/requirements.txt
	touch $@

$(VENV)/.web-test-installed: $(VENV)/.web-installed web/requirements-test.txt
	$(PIP) install -r web/requirements-test.txt
	touch $@
