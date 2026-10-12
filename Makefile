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

.PHONY: all python cpp clean init venv
