# mk/dev.mk - working on the code, in the `dev` container: shell run build test sim-gui gaze-gui rviz-gui, plus clean.
#
# `dev` is the same image as the system, with the workspace mounted at /ws, so a build here is what the system runs
# after `make restart`. PC: started by `make up` or by these commands, and kept up. Jetson: a throw-away container
# per command (`run --rm`), gone when the command ends.

.PHONY: shell run build test sim-gui gaze-gui rviz-gui clean

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

# Windows on the simulation the `sim` container already runs (its GZ_PARTITION), not a second simulation: a second
# gazebo.launch.py on the same ROS domain fights it for /clock and /controller_manager. In the background: make returns
# at once, closing a window ends its process, closing them leaves the simulation running. Each one opens only once.
# Their output goes to ~/.ros/log in dev.
SIM_GZ_ENV := -e GZ_PARTITION=braco-sim -e GZ_SIM_RESOURCE_PATH=/ws/install/braco_description/share
GAZE_GUI := ros2 launch braco_description gazebo_gui.launch.py
RVIZ_GUI := ros2 launch braco_moveit_config rviz.launch.py
define gui_checks
	@[ "$(ROLE)" = pc ] || { echo "$@: the simulation runs on the PC" >&2; exit 1; }
	@[ -n "$$($(COMPOSE) ps -q --status running sim)" ] || { echo "sim is not running: make up SIM=1 first" >&2; exit 1; }
endef
# $(1): the command, $(2): extra options of exec
define gui_start
	@if $(COMPOSE) exec -T $(DEV) pgrep -f '$(1)' >/dev/null; then echo "$@: already open"; \
	else $(COMPOSE) exec -d $(2) $(DEV) ros_entrypoint.sh $(1) && echo "$@: opening in the background (close the window to end it)"; fi
endef

gaze-gui: _image _dev_up ## PC: open the Gazebo window on the running simulation, in the background
	$(gui_checks)
	$(call gui_start,$(GAZE_GUI),$(SIM_GZ_ENV))

rviz-gui: _image _dev_up ## PC: open RViz + MoveIt on the running simulation, in the background (needs SIM_MOVEIT=true)
	$(gui_checks)
	@$(COMPOSE) exec -T sim pgrep -f moveit_ros_move_group/move_group >/dev/null || { \
	  echo "move_group is not running: set SIM_MOVEIT=true in docker/.env, then make restart SIM=1" >&2; exit 1; }
	$(call gui_start,$(RVIZ_GUI))

sim-gui: gaze-gui rviz-gui ## PC: open both windows on the running simulation, in the background (make up SIM=1 first)

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
