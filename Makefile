VENV	:= .venv
PYTHON	:= $(VENV)/bin/python
PIP		:= $(VENV)/bin/pip

all: venv

clean:
	rm -rf $(VENV)

init: venv
	git config core.hooksPath .githooks

$(VENV)/.installed: requirements.txt
	python3 -m venv $(VENV)
	$(PIP) install --upgrade pip
	$(PIP) install -r requirements.txt
	touch $@

venv: $(VENV)/.installed

.PHONY: init clean
