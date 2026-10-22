# mk/system.mk - turn the braco system on and off: up down restart status logs.
#
# `make up` is idempotent, every step is skipped when there is nothing to do:
#   1. docker/.env          created if missing (make setup does the same)
#   2. host checks          only problems are printed (make doctor shows everything)
#   3. image                built only if missing (make image rebuilds it)
#   4. colcon build         only when a file under ros2_ws/src changed since the last build
#   5. start                the system (`real` or `sim`, plus `dev` on the PC); restarted if it was running and step 4 rebuilt
#   6. wait                 until the page's /api/status says connected (UP_TIMEOUT seconds)
#   7. print the page URL

# The PC does not wait long: its simulated arm cannot connect until the Gazebo bringup (#34) exists.
UP_TIMEOUT ?= $(if $(filter jetson,$(ROLE)),90,20)
XHOST      ?= 1
PARK       ?= 0

# Sources colcon builds from (git's view: tracked and new files, minus .gitignore; plain find outside a git checkout).
WS_SOURCES  := $(wildcard $(shell git ls-files -co --exclude-standard $(WS_SRC) 2>/dev/null \
                 || find $(WS_SRC) -type f ! -name '*.pyc'))
# Written after a successful colcon build; colcon's build/ is the right place (make clean removes both).
BUILD_STAMP := $(WS_DIR)/build/.braco-built
# Left by a build that ran while the system was up, so `up` knows to restart it.
REBUILT     := $(WS_DIR)/build/.braco-rebuilt

.PHONY: up down restart status logs _checks _image _dev_up _xhost

##@ System (the real arm on the Jetson, the simulation on the PC)

up: _image | $(WS_DIRS) ## Turn the system on: .env, checks, image and build only if needed, start, wait, URL
	@$(MAKE) -s --no-print-directory $(BUILD_STAMP)
	@was_up=$$($(COMPOSE) ps -q --status running $(SYSTEM) 2>/dev/null); \
	$(COMPOSE) up -d $(UP_SERVICES) || exit 1; \
	if [ -n "$$was_up" ] && [ -f $(REBUILT) ]; then echo "workspace rebuilt: restarting $(SYSTEM)"; $(COMPOSE) restart $(SYSTEM); fi; \
	rm -f $(REBUILT)
	@$(MAKE) --no-print-directory _xhost
	@COMPOSE_PROFILES=$(ROLE) DC="$(DC)" ROLE=$(ROLE) SYSTEM=$(SYSTEM) WEB_PORT=$(WEB_PORT) UP_TIMEOUT=$(UP_TIMEOUT) sh -c "$$WAIT_SH"

down: _role ## Turn the system off (PARK=1 parks the arm first; the caches are kept)
	@if [ "$(PARK)" = 1 ] && [ "$(ROLE)" = jetson ] && [ -n "$$($(COMPOSE) ps -q --status running $(SYSTEM))" ]; then \
	  echo "parking the arm"; $(SYS_EXEC) ros2 run braco_bringup park || exit 1; fi
	$(COMPOSE) down --remove-orphans

restart: _role ## Restart the system container (the arm is not zeroed or parked), then like up
	$(COMPOSE) rm -sf $(SYSTEM)
	@$(MAKE) --no-print-directory up

status: _role ## Containers, CPU/RAM per container, the arm as the page sees it (and a few s of tegrastats on the Jetson)
	@$(COMPOSE) ps
	@ids=$$($(COMPOSE) ps -q); if [ -n "$$ids" ]; then echo; docker stats --no-stream $$ids; fi
	@echo; printf 'page  http://localhost:$(WEB_PORT)  /api/status: '; \
	  curl -fsS -m 3 http://localhost:$(WEB_PORT)/api/status 2>/dev/null | cut -c1-300 || echo "no answer"; echo
	@if [ "$(ROLE)" = jetson ] && command -v tegrastats >/dev/null 2>&1; then \
	  echo; echo "tegrastats (3 s):"; timeout 3 tegrastats --interval 1000 || true; fi

# S=web shows only the page; any other S is a pattern, e.g. S=arm_driver or S='ERROR|WARN'.
LOGS_web := uvicorn|braco|web page|Application|HTTP
logs: _role ## Follow the system's log (S=web, S=arm_driver, S=temperature_monitor, S=joystick_teleop, any pattern)
	@if [ -n "$(S)" ]; then \
	  $(COMPOSE) logs -f --tail=300 $(SYSTEM) | grep --line-buffered -iE '$(or $(LOGS_$(S)),$(S))'; \
	else $(COMPOSE) logs -f --tail=100 $(SYSTEM); fi

# ---- the steps of `up` (.env is created by the rule in mk/setup.mk) ---------------------------
_checks: _role $(ENV_FILE)
	@QUIET=1 $(DOCTOR_RUN) || { echo "fix the [FAIL] lines above (make doctor shows every check)" >&2; exit 1; }

_image: _checks
	@docker image inspect $(IMAGE) >/dev/null 2>&1 || { echo "image $(IMAGE) not found: building it (once)"; $(COMPOSE) build $(DEV); }

# PC: starts `dev` if it is not running (on the Jetson dev commands are `run --rm`: nothing to start).
# Commands that need dev depend on `_image _dev_up` (in that order).
_dev_up: | $(WS_DIRS)
	@[ "$(ROLE)" = jetson ] || [ -n "$$($(COMPOSE) ps -q --status running $(DEV) 2>/dev/null)" ] || $(COMPOSE) up -d $(DEV)

$(BUILD_STAMP): $(WS_SOURCES)
	@echo "workspace changed: colcon build"
	@$(MAKE) --no-print-directory _dev_up
	$(call dev_cmd,$(BUILD_ENV)) $(COLCON)
	@touch $@ $(REBUILT)

_xhost:
	@if [ "$(ROLE)" = pc ] && [ "$(XHOST)" = 1 ] && [ -n "$$DISPLAY" ] && command -v xhost >/dev/null 2>&1; then \
	  xhost +local: >/dev/null && echo "xhost +local:   (containers may draw on your display; XHOST=0 skips it)"; fi

define WAIT_SH
url=http://localhost:$$WEB_PORT/api/status
printf 'waiting for the page to see the arm (%s s max) ' "$$UP_TIMEOUT"
t=0; body=""
while [ "$$t" -lt "$$UP_TIMEOUT" ]; do
  if [ -z "$$($$DC ps -q --status running $$SYSTEM)" ]; then
    echo; echo "$$SYSTEM stopped. Its last lines:" >&2; $$DC logs --tail=30 $$SYSTEM >&2; exit 1
  fi
  body=$$(curl -fsS -m 2 "$$url" 2>/dev/null)
  if printf '%s' "$$body" | grep -q '"connected": *true'; then echo " connected"; break; fi
  printf '.'; sleep 2; t=$$((t + 2))
done
if [ "$$t" -ge "$$UP_TIMEOUT" ]; then
  echo; err=$$(printf '%s' "$$body" | sed -n 's/.*"error": *"\([^"]*\)".*/\1/p')
  if [ -z "$$body" ]; then why="the page does not answer on port $$WEB_PORT"; else why="not connected: $${err:-no reason given}"; fi
  if [ "$$ROLE" = jetson ]; then echo "the arm did not come up in $$UP_TIMEOUT s ($$why). See: make logs" >&2; exit 1; fi
  echo "warning: $$why (the simulated arm needs Gazebo, issue #34). See: make logs"
fi
ip=$$(hostname -I 2>/dev/null | awk '{print $$1}')
echo "braco is up: http://$${ip:-localhost}:$$WEB_PORT   (make status, make logs, make down)"
endef
export WAIT_SH
