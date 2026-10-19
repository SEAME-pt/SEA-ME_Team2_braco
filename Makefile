VENV	:= .venv
PIP		:= $(VENV)/bin/pip
CPP_DIR	:= tests
CPP_BUILD	:= $(CPP_DIR)/build

all: python cpp

python:
	$(VENV)/bin/python tests/hello_world.py

cpp:
	cmake -S $(CPP_DIR) -B $(CPP_BUILD)
	cmake --build $(CPP_BUILD)
	$(CPP_BUILD)/hello_world

clean:
	rm -rf $(VENV) $(CPP_BUILD)

init: venv
	git config core.hooksPath .githooks

$(VENV)/.installed: requirements.txt
	python3 -m venv $(VENV)
	$(PIP) install --upgrade pip
	$(PIP) install -r requirements.txt
	touch $@

venv: $(VENV)/.installed

# ---- braco web page (web/README.md) ---------------------------------------------
# On the Jetson the page normally runs INSIDE the arm container (cd docker && make arm-up)
# and talks to the arm through ROS 2. `make web` / web-start below are the fallback that opens
# the serial port directly with pymycobot: only use them while the arm container is down.
# web-mock and web-test only need the web packages, so they also work on a PC
# where requirements.txt (pymycobot, numpy) cannot be installed.
$(VENV)/bin/python:
	python3 -m venv $(VENV)
	$(PIP) install --upgrade pip

$(VENV)/.web-installed: $(VENV)/bin/python web/requirements.txt
	$(PIP) install -r web/requirements.txt
	touch $@

web-deps: $(VENV)/.web-installed

web: venv web-deps
	$(VENV)/bin/uvicorn web.server.app:app --host 0.0.0.0 --port 8000

# ---- Web: background control (start / stop / restart / reinstall) -------------
# web-start runs the server detached, with its PID in $(WEB_PID) and output in
# $(WEB_LOG), so the terminal stays free. `make web` still runs in the foreground.
WEB_PORT	:= 8000
WEB_PID	:= $(VENV)/web.pid
WEB_LOG	:= $(VENV)/web.log
WEB_CMD	:= $(VENV)/bin/uvicorn web.server.app:app --host 0.0.0.0 --port $(WEB_PORT)

web-start: venv web-deps
	@if [ -f $(WEB_PID) ] && kill -0 $$(cat $(WEB_PID)) 2>/dev/null; then \
		echo "web already running (pid $$(cat $(WEB_PID))). Use: make web-restart"; \
	else \
		nohup setsid $(WEB_CMD) > $(WEB_LOG) 2>&1 & echo $$! > $(WEB_PID); \
		sleep 2; \
		if kill -0 $$(cat $(WEB_PID)) 2>/dev/null; then \
			echo "web started (pid $$(cat $(WEB_PID))) on port $(WEB_PORT). Logs: make web-logs"; \
		else \
			echo "web failed to start. Last log lines:"; tail -n 20 $(WEB_LOG); rm -f $(WEB_PID); exit 1; \
		fi; \
	fi

web-stop:
	@if [ -f $(WEB_PID) ] && kill -0 $$(cat $(WEB_PID)) 2>/dev/null; then \
		kill $$(cat $(WEB_PID)); \
		for i in 1 2 3 4 5; do kill -0 $$(cat $(WEB_PID)) 2>/dev/null || break; sleep 1; done; \
		kill -9 $$(cat $(WEB_PID)) 2>/dev/null || true; \
		echo "web stopped"; \
	else \
		echo "web not running (from web-start)"; \
	fi
	@rm -f $(WEB_PID)
	@# Also free the port if a foreground `make web` / stray uvicorn still holds it.
	@pkill -f "[u]vicorn web.server.app:app" 2>/dev/null || true

web-restart: web-stop web-start

# Wipes the whole venv (fixes corrupted/partial installs) and reinstalls, then starts.
web-reinstall: web-stop
	rm -rf $(VENV)
	$(MAKE) venv web-deps
	$(MAKE) web-start

web-status:
	@if [ -f $(WEB_PID) ] && kill -0 $$(cat $(WEB_PID)) 2>/dev/null; then \
		echo "web running (pid $$(cat $(WEB_PID))) on port $(WEB_PORT)"; \
	else echo "web stopped"; fi

web-logs:
	@tail -n 50 -f $(WEB_LOG)

web-mock: web-deps
	ARM_MOCK=1 $(VENV)/bin/uvicorn web.server.app:app --host 127.0.0.1 --port 8000 --reload

$(VENV)/.web-test-installed: $(VENV)/.web-installed web/requirements-test.txt
	$(PIP) install -r web/requirements-test.txt
	touch $@

web-test: $(VENV)/.web-test-installed
	$(VENV)/bin/python -m pytest web/tests -q

.PHONY: all python cpp clean init venv web-deps web web-mock web-test \
	web-start web-stop web-restart web-reinstall web-status web-logs
