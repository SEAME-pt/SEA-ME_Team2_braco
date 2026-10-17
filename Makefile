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

# ---- Web dashboard (web/README.md) --------------------------------------------
$(VENV)/.web-installed: $(VENV)/.installed web/requirements.txt
	$(PIP) install -r web/requirements.txt
	touch $@

web-deps: $(VENV)/.web-installed

web: web-deps
	$(VENV)/bin/uvicorn web.server.app:app --host 0.0.0.0 --port 8000

web-mock: web-deps
	ARM_MOCK=1 $(VENV)/bin/uvicorn web.server.app:app --host 127.0.0.1 --port 8000 --reload

$(VENV)/.web-test-installed: $(VENV)/.web-installed web/requirements-test.txt
	$(PIP) install -r web/requirements-test.txt
	touch $@

web-test: $(VENV)/.web-test-installed
	$(VENV)/bin/python -m pytest web/tests -q

.PHONY: all python cpp clean init venv web-deps web web-mock web-test
