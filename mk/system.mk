# mk/system.mk - turn the braco system on and off: up down restart status logs, and the web page: web web-stop.
#
# `make up` is idempotent, every step is skipped when there is nothing to do:
#   1. docker/.env          created if missing: asks once "dev or jetson" (any command does; make role asks again)
#   2. host checks          only problems are printed (make doctor shows everything)
#   3. image                built only if missing (make image rebuilds it)
#   4. colcon build         only when a file under ros2_ws/src changed since the last build
#   5. start                Jetson: `real`. PC: `dev`, plus `sim` with SIM=1. The system is restarted if it was running
#                           and step 4 rebuilt
#   6. wait                 until the system publishes /joint_states (UP_TIMEOUT seconds; not on a plain PC `make up`)
# The web page is separate: `make web` adds it to the running container (`real` on the Jetson, `dev` on the PC).

# Jetson: the driver and the servos. PC: Gazebo, the model and the controllers (first start loads the meshes).
UP_TIMEOUT ?= 90
XHOST      ?= 1
PARK       ?= 0

# Sources colcon builds from (git's view: tracked and new files, minus .gitignore; plain find outside a git checkout).
WS_SOURCES  := $(wildcard $(shell git ls-files -co --exclude-standard $(WS_SRC) 2>/dev/null \
                 || find $(WS_SRC) -type f ! -name '*.pyc'))
# Written after a successful colcon build; colcon's build/ is the right place (make clean removes both).
BUILD_STAMP := $(WS_DIR)/build/.braco-built
# Left by a build that ran while the system was up, so `up` knows to restart it.
REBUILT     := $(WS_DIR)/build/.braco-rebuilt

.PHONY: up down restart status logs web web-stop _checks _image _dev_up _xhost

##@ System (the real arm on the Jetson, the simulation on the PC)

up: _image | $(WS_DIRS) ## Turn on: Jetson the arm (real); PC only dev, SIM=1 also the simulation. No web page (make web). Builds only if needed
	@$(MAKE) -s --no-print-directory $(BUILD_STAMP)
	@was_up=$(if $(UP_SYSTEM),$$($(COMPOSE) ps -q --status running $(UP_SYSTEM) 2>/dev/null)); \
	had_web=$$(curl -fsS -m 2 -o /dev/null http://localhost:$(WEB_PORT)/api/status 2>/dev/null && echo 1); \
	$(COMPOSE) up -d $(UP_SERVICES) || exit 1; \
	if [ -n "$$was_up" ] && [ -f $(REBUILT) ]; then echo "workspace rebuilt: restarting $(UP_SYSTEM)"; $(COMPOSE) restart $(UP_SYSTEM); \
	  [ "$(UP_SYSTEM)" != "$(WEB_CONTAINER)" ] || [ -z "$$had_web" ] || touch $(WEB_AGAIN); fi; \
	rm -f $(REBUILT)
	@$(MAKE) --no-print-directory _xhost
	@$(if $(UP_SYSTEM),COMPOSE_PROFILES=$(ROLE) DC="$(DC)" SYSTEM=$(SYSTEM) UP_TIMEOUT=$(UP_TIMEOUT) sh -c "$$WAIT_SH",echo "dev is up: make shell   (simulation: make up SIM=1; web page: make web)")
	@if [ -f $(WEB_AGAIN) ]; then rm -f $(WEB_AGAIN); echo "the restart stopped the web page: starting it again"; $(MAKE) --no-print-directory web; fi

down: _role ## Turn the system off (PARK=1 parks the arm first; the caches are kept)
	@if [ "$(PARK)" = 1 ] && [ "$(ROLE)" = jetson ] && [ -n "$$($(COMPOSE) ps -q --status running $(SYSTEM))" ]; then \
	  echo "parking the arm"; $(SYS_EXEC) ros2 run braco_bringup park || exit 1; fi
	$(COMPOSE) down --remove-orphans

restart: _role ## Recreate the system container (PC: dev; SIM=1: sim) and start it like up; make web again for the page
	$(COMPOSE) rm -sf $(or $(UP_SYSTEM),$(DEV))
	@$(MAKE) --no-print-directory up

status: _role ## Containers, CPU/RAM per container, the arm as the page sees it (and a few s of tegrastats on the Jetson)
	@$(COMPOSE) ps
	@ids=$$($(COMPOSE) ps -q); if [ -n "$$ids" ]; then echo; docker stats --no-stream $$ids; fi
	@echo; printf 'page  http://localhost:$(WEB_PORT)  /api/status: '; \
	  curl -fsS -m 3 http://localhost:$(WEB_PORT)/api/status 2>/dev/null | cut -c1-300 || echo "not running (make web)"; echo
	@if [ "$(ROLE)" = jetson ] && command -v tegrastats >/dev/null 2>&1; then \
	  echo; echo "tegrastats (3 s):"; timeout 3 tegrastats --interval 1000 || true; fi

# S=web shows only the page (in the container it runs in); any other S is a pattern, e.g. S=arm_driver or S='ERROR|WARN'.
# On the PC without `sim` the log is dev's (where the page runs).
LOGS_web := uvicorn|web:|braco|Application|HTTP
logs: _role ## Follow the system's log (S=web, S=arm_driver, S=temperature_monitor, S=joystick_teleop, S=camera, any pattern)
	@svc=$(SYSTEM); if [ "$(S)" = web ]; then svc=$(WEB_CONTAINER); \
	elif [ "$(ROLE)" = pc ] && [ -z "$$($(COMPOSE) ps -q --status running $(SYSTEM) 2>/dev/null)" ]; then svc=$(DEV); fi; \
	if [ -n "$(S)" ]; then \
	  $(COMPOSE) logs -f --tail=300 $$svc | grep --line-buffered -iE '$(or $(LOGS_$(S)),$(S))'; \
	else $(COMPOSE) logs -f --tail=100 $$svc; fi

##@ Web page (separate from make up)

# The page runs inside the machine's running container: `real` on the Jetson (ARM_SOURCE=ros, the real arm), `dev` on
# the PC (ARM_SOURCE=gazebo: the simulation, make up SIM=1 or gazebo.launch.py in a dev shell). web/restart.sh starts it
# (or replaces a running one: after a change in web/, make web again). It stops with the container (make down).
# MOCK=1: synthetic arm data instead (no arm, no Gazebo needed), for working on the page itself.
web: _role ## Start (or restart) the web page in the running container (Jetson: real, PC: dev, on Gazebo); MOCK=1: mock arm data
	@[ -n "$$($(COMPOSE) ps -q --status running $(WEB_CONTAINER) 2>/dev/null)" ] || { \
	  echo "$(WEB_CONTAINER) is not running: make up first"; $(MAKE) --no-print-directory up || exit 1; }
	@$(COMPOSE) exec -T -e ARM_SOURCE=$(WEB_SOURCE) $(if $(filter 1,$(MOCK)),-e ARM_MOCK=1) $(WEB_CONTAINER) \
	  ros_entrypoint.sh bash /opt/braco/web/restart.sh
	@WEB_PORT=$(WEB_PORT) sh -c "$$WEB_WAIT_SH"

web-stop: _role ## Stop the web page (the container and the arm keep running)
	@[ -z "$$($(COMPOSE) ps -q --status running $(WEB_CONTAINER) 2>/dev/null)" ] || \
	  $(COMPOSE) exec -T $(WEB_CONTAINER) bash /opt/braco/web/restart.sh stop

# ---- the steps of `up` (.env is created by the rule in mk/setup.mk) ---------------------------
_checks: _role $(ENV_FILE)
	@QUIET=1 $(DOCTOR_RUN) || { echo "fix the [FAIL] lines above (make doctor shows every check)" >&2; exit 1; }

_image: _checks
	@docker image inspect $(IMAGE) >/dev/null 2>&1 || { echo "image $(IMAGE) not found: building it (once)"; $(COMPOSE) build $(DEV); }

# PC: starts `dev` if it is not running (on the Jetson dev commands are `run --rm`: nothing to start), and lets it
# draw on the display (Gazebo, RViz). Commands that need dev depend on `_image _dev_up` (in that order).
_dev_up: | $(WS_DIRS)
	@[ "$(ROLE)" = jetson ] || [ -n "$$($(COMPOSE) ps -q --status running $(DEV) 2>/dev/null)" ] || $(COMPOSE) up -d $(DEV)
	@$(MAKE) --no-print-directory _xhost

$(BUILD_STAMP): $(WS_SOURCES)
	@echo "workspace changed: colcon build"
	@$(MAKE) --no-print-directory _dev_up
	$(call dev_cmd,$(BUILD_ENV)) $(COLCON)
	@touch $@ $(REBUILT)

# The grant lasts only for the X session (Wayland: Xwayland starts empty at every login), so it is checked on every
# up and every dev command. Without it Gazebo's window aborts: "Authorization required ... could not connect to display".
_xhost:
	@if [ "$(ROLE)" = pc ] && [ "$(XHOST)" = 1 ] && [ -n "$$DISPLAY" ] && command -v xhost >/dev/null 2>&1 \
	  && ! xhost 2>/dev/null | grep -q '^LOCAL:'; then \
	  xhost +local: >/dev/null && echo "xhost +local:   (containers may draw on your display; XHOST=0 skips it)"; fi

# Up = the system publishes /joint_states (arm_driver on the Jetson, Gazebo's joint_state_broadcaster with SIM=1),
# asked inside the container itself.
define WAIT_SH
printf 'waiting for the arm (/joint_states, %s s max) ' "$$UP_TIMEOUT"
end=$$(( $$(date +%s) + UP_TIMEOUT ))
while :; do
  if [ -z "$$($$DC ps -q --status running $$SYSTEM)" ]; then
    echo; echo "$$SYSTEM stopped. Its last lines:" >&2; $$DC logs --tail=30 $$SYSTEM >&2; exit 1
  fi
  if $$DC exec -T $$SYSTEM ros_entrypoint.sh timeout 4 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
    echo " ok"; break; fi
  if [ "$$(date +%s)" -ge "$$end" ]; then
    echo; echo "no /joint_states from $$SYSTEM in $$UP_TIMEOUT s. See: make logs" >&2; exit 1; fi
  printf '.'; sleep 1
done
echo "braco is up   (web page: make web; make status, make logs, make down)"
endef
export WAIT_SH

# After make web: wait until the page answers, then print its URL.
define WEB_WAIT_SH
url=http://localhost:$$WEB_PORT/api/status
t=0
while [ "$$t" -lt 30 ]; do
  body=$$(curl -fsS -m 2 "$$url" 2>/dev/null) && break
  sleep 1; t=$$((t + 1))
done
[ -n "$$body" ] || { echo "the page does not answer on port $$WEB_PORT. See: make logs S=web" >&2; exit 1; }
if printf '%s' "$$body" | grep -q '"connected": *true'; then state="sees the arm"; else state="not connected to the arm yet"; fi
ip=$$(hostname -I 2>/dev/null | awk '{print $$1}')
echo "web page: http://$${ip:-localhost}:$$WEB_PORT   ($$state; make web-stop stops it)"
endef
export WEB_WAIT_SH
