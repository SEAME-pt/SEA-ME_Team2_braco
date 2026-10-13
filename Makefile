VENV	:= .venv
PIP		:= $(VENV)/bin/pip
CPP_DIR	:= tests
CPP_BUILD	:= $(CPP_DIR)/build

.DEFAULT_GOAL := help

ROS_DISTRO	?= lyrical
IMAGE		?= braco-ros-smoke
TAG			?= dev
REGISTRY	?=
DOMAIN_ID	?= 42
REF			= $(if $(REGISTRY),$(REGISTRY)/)$(IMAGE):$(TAG)
UNAME_S		:= $(shell uname -s)
DOCKER_RUN	= docker run --rm --init -e ROS_DOMAIN_ID=$(DOMAIN_ID)
BUILD_ARGS	= -f docker/Dockerfile.smoke --build-arg ROS_DISTRO=$(ROS_DISTRO) -t $(REF)
BUILDX		= docker buildx build --platform linux/amd64,linux/arm64 $(BUILD_ARGS)

help:		## list targets
	@grep -hE '^[a-z-]+:.*##' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  %-20s %s\n", $$1, $$2}'

all: python cpp	## run python and C++ hello world

python:		## python hello world
	$(VENV)/bin/python tests/hello_world.py

cpp:		## C++ hello world
	cmake -S $(CPP_DIR) -B $(CPP_BUILD)
	cmake --build $(CPP_BUILD)
	$(CPP_BUILD)/hello_world

clean:		## remove venv and C++ build
	rm -rf $(VENV) $(CPP_BUILD)

init: venv	## venv + git hooks
	git config core.hooksPath .githooks

$(VENV)/.installed: requirements.txt
	python3 -m venv $(VENV)
	$(PIP) install --upgrade pip
	$(PIP) install -r requirements.txt
	touch $@

venv: $(VENV)/.installed	## create python venv

info:		## host, kernel, Docker, runc, libseccomp
	@echo "OS:          $(UNAME_S)"
	@echo "arch:        $$(uname -m)"
	@echo "kernel:      $$(uname -r)"
	@echo "docker:      $$(docker version --format '{{.Server.Version}}')"
	@docker version --format '{{range .Server.Components}}{{if eq .Name "runc"}}runc:        {{.Version}}{{end}}{{end}}' 2>/dev/null || true
	@dpkg -s libseccomp2 2>/dev/null | sed -n 's/^Version: /libseccomp2:  /p' || true

build:		## build smoke image for the native architecture
	docker build $(BUILD_ARGS) .

test-local: build	## talker + listener in ONE container, PASS/FAIL
	$(DOCKER_RUN) $(REF) smoke-local.sh

test-net-talker:	## talker with --network host (Linux only)
ifeq ($(UNAME_S),Darwin)
	@echo "Not supported on macOS: Docker Desktop runs containers in a VM, host networking is layer 4 only (TCP/UDP) and DDS multicast discovery is not expected to cross it. Run this on a Linux host."; exit 1
else
	$(DOCKER_RUN) --network host $(REF) ros2 run demo_nodes_cpp talker
endif

test-net-listener:	## listener with --network host, PASS/FAIL (Linux only)
ifeq ($(UNAME_S),Darwin)
	@echo "Not supported on macOS (see test-net-talker)."; exit 1
else
	@$(DOCKER_RUN) --network host $(REF) sh -c 'PYTHONUNBUFFERED=1 timeout 30 ros2 run demo_nodes_py listener 2>&1' | grep -m1 "I heard" \
		&& echo "PASS: heard talker across the network" \
		|| { echo "FAIL: nothing heard within 30s"; exit 1; }
endif

buildx:		## multi-arch build (amd64+arm64), no push
	$(BUILDX) .

push:		## multi-arch build and push (needs REGISTRY=)
	@test -n "$(REGISTRY)" || { echo "set REGISTRY, e.g. make push REGISTRY=ghcr.io/<org>"; exit 1; }
	$(BUILDX) --push .

.PHONY: help all python cpp clean init venv info build test-local test-net-talker test-net-listener buildx push
