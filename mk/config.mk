# mk/config.mk - paths, which machine this is, and the docker compose command. Included first by the Makefile.
#
# Every path the other mk/*.mk files use is defined here, so moving a file means editing one line.
# (Only the `## ...` help strings repeat a few names literally, because `make help` reads the files as text.)

# ---- project layout -------------------------------------------------------------------------
DOCKER_DIR   := docker
ENV_FILE     := $(DOCKER_DIR)/.env
ENV_EXAMPLE  := $(DOCKER_DIR)/.env.example
COMPOSE_BASE := $(DOCKER_DIR)/compose.yml
COMPOSE_DIR  := $(DOCKER_DIR)/compose
DOCKERFILE   := $(DOCKER_DIR)/Dockerfile
CONFIG_DIR   := $(DOCKER_DIR)/config
SCRIPTS_DIR  := $(DOCKER_DIR)/scripts
TESTS_DIR    := $(DOCKER_DIR)/tests
SYSTEMD_UNIT := $(DOCKER_DIR)/systemd/braco.service
SYSTEMD_DEST := /etc/systemd/system/braco.service
WS_DIR       := ros2_ws
WS_SRC       := $(WS_DIR)/src
WS_BAGS      := $(WS_DIR)/bags
# The plain C++ arm library (no ament linters): formatted by clang-format like the code outside ros2_ws/src.
CPP_PKG      := $(WS_SRC)/mycobot_cpp
# Where compose.yml mounts docker/tests inside the containers.
CONTAINER_TESTS    := /opt/seame/tests
# Every package colcon builds lives in ros2_ws/src (mycobot_cpp included), mounted at /ws.
WS_BASE_PATHS      := src

# Folders that must exist before a container bind-mounts ros2_ws. Created by make as YOUR user;
# if Docker creates them instead they end up owned by root.
WS_DIRS := $(WS_SRC) $(WS_BAGS)

# Files that must be present for the project to work (checked by `make doctor`).
PROJECT_FILES := $(COMPOSE_BASE) $(ENV_EXAMPLE) $(DOCKERFILE) $(CONFIG_DIR)/colcon-defaults.yaml $(SYSTEMD_UNIT) \
                 $(TESTS_DIR)/talker-listener.sh $(addprefix $(COMPOSE_DIR)/,nvidia.yml dri.yml) \
                 $(addprefix $(SCRIPTS_DIR)/,apt-install-optional.sh env.sh install-plotjuggler.sh \
                                             install-ros-apt-source.sh ros_entrypoint.sh smoke-test.sh)

# Host side (no docker): Python venv for the web page and its tests (web/requirements*.txt) and the dev tools
# (requirements-dev.txt: the formatters, plus the container's arm libraries for the editor).
PYTHON     ?= python3
VENV       := .venv
PIP        := $(VENV)/bin/pip

# ---- which machine is this? ------------------------------------------------------------------
# Read KEY from .env. The shell environment wins over .env, exactly as it does for docker compose.
dotenv = $(strip $(shell sed -n 's/^$(1)=//p' $(ENV_FILE) 2>/dev/null | tail -n 1))

# Probed once. UNAME_M and IS_JETSON can be overridden to simulate another host: make help UNAME_M=aarch64 IS_JETSON=1
# aarch64 alone does not mean "Jetson" (Raspberry Pi, ARM servers, VMs on Apple Silicon), so a Jetson is arm64
# AND carries a Tegra marker.
UNAME_M ?= $(shell uname -m)
ARCH    := $(if $(filter aarch64 arm64,$(UNAME_M)),arm64,amd64)
ifndef IS_JETSON
IS_JETSON := $(shell { grep -qsai jetson /proc/device-tree/model || test -f /etc/nv_tegra_release; } && echo 1)
endif
DETECTED_ROLE := $(if $(and $(filter arm64,$(ARCH)),$(IS_JETSON)),jetson,pc)

# ROLE=pc (workstation, simulation) or ROLE=jetson (the Nano, real arm). From, in order: ROLE=... on the command
# line, COMPOSE_PROFILES in the environment, COMPOSE_PROFILES in docker/.env, then detected from this machine.
# ROLE and GPU are generic names, so only the command line may set them (never a stray env var).
ifneq ($(origin ROLE),command line)
ROLE := $(or $(COMPOSE_PROFILES),$(call dotenv,COMPOSE_PROFILES),$(DETECTED_ROLE))
endif
ifneq ($(origin GPU),command line)
GPU :=
endif

IMAGE_PREFIX ?= $(or $(call dotenv,IMAGE_PREFIX),braco-image)
IMAGE_TAG    ?= $(or $(call dotenv,IMAGE_TAG),lyrical)
MYCOBOT_PORT ?= $(or $(call dotenv,MYCOBOT_PORT),/dev/ttyTHS1)
WEB_PORT     ?= $(or $(call dotenv,WEB_PORT),8000)

# The two compose services of each machine (docker/compose.yml): `dev` for the tools, and the system itself.
SYSTEM_pc     := sim
SYSTEM_jetson := real
SYSTEM        := $(SYSTEM_$(ROLE))
DEV           := dev
IMAGE         := $(IMAGE_PREFIX)/$(ROLE):$(IMAGE_TAG)

# Every compose call goes through docker/ (so .env and ../ros2_ws resolve) with this machine's profile.
# compose.yml also uses COMPOSE_PROFILES to pick the image of `dev`, which is why it is passed explicitly.
# GPU=nvidia|dri adds the matching override file (PC only); without it, compose uses COMPOSE_FILE from .env.
DC      := docker compose --project-directory $(DOCKER_DIR) $(if $(GPU),-f $(COMPOSE_BASE) -f $(COMPOSE_DIR)/$(GPU).yml)
COMPOSE := COMPOSE_PROFILES=$(ROLE) $(DC)

# No TTY in CI (GitHub Actions sets CI=true) or when stdin is not a terminal.
NOTTY ?= $(if $(or $(CI),$(shell [ -t 0 ] || echo 1)),-T)

# `dev` on the PC stays up (several shells, RViz, PlotJuggler in one container). On the Jetson it never runs on its
# own: every dev command is a throw-away `run --rm`, so normally only `real` uses the Nano's RAM.
ifeq ($(ROLE),jetson)
DEV_MODE    := run --rm --no-deps
UP_SERVICES := $(SYSTEM)
else
DEV_MODE    := exec
UP_SERVICES := $(DEV) $(SYSTEM)
endif
# A command in dev ($(1): extra options such as -e VAR=value). Through the image entrypoint, so ROS and the workspace
# are sourced (a bare `exec` would not).
dev_cmd   = $(COMPOSE) $(DEV_MODE) $(NOTTY) $(1) $(DEV) ros_entrypoint.sh
DEV_SHELL = $(COMPOSE) $(DEV_MODE) $(DEV) bash
SYS_EXEC := $(COMPOSE) exec $(NOTTY) $(SYSTEM) ros_entrypoint.sh

# The Jetson never simulates: it builds only what the real system runs, braco_bringup and its dependencies (driver,
# teleop, interfaces, mycobot_cpp). A package the real arm needs later goes in braco_bringup's package.xml as an
# exec_depend; a simulation package (Gazebo, MoveIt) never does, or the Jetson builds it again. The PC builds everything.
# The Nano is slow and has 4 GB of RAM: one package at a time, two compiler jobs (the ccache limit is in compose.yml),
# Release without tests (they run on the PC and in CI). --cmake-args REPLACES the cmake-args of colcon-defaults.yaml,
# so the ccache launchers are repeated here.
ifeq ($(ROLE),jetson)
WS_PACKAGES := --packages-up-to braco_bringup
COLCON_ARGS ?= --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
               -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
BUILD_ENV   := -e MAKEFLAGS=-j2
endif
COLCON = colcon build --base-paths $(WS_BASE_PATHS) $(WS_PACKAGES) $(COLCON_ARGS)

FORCE ?= 0

# CMD reaches the shell exactly as typed. Without $(value ...) make would expand `$LANG` or `$(id -u)` itself.
export BRACO_CMD = $(value CMD)

define confirm
@if [ "$(FORCE)" != 1 ]; then printf '%s [y/N] ' "$(1)"; read -r ans; [ "$$ans" = y ] || { echo "aborted"; exit 1; }; fi
endef

# ---- checks shared by the targets (not listed in `make help`) ---------------------------------
.PHONY: _role _jetson

_role:
	@case "$(ROLE)" in pc|jetson) ;; *) echo "ROLE must be 'pc' or 'jetson' (got '$(ROLE)')" >&2; exit 1;; esac
	@if [ -n "$(GPU)" ]; then \
	  case "$(GPU)" in nvidia|dri) ;; *) echo "GPU must be 'nvidia' or 'dri' (got '$(GPU)')" >&2; exit 1;; esac; \
	  [ "$(ROLE)" = pc ] || { echo "GPU only applies to the PC" >&2; exit 1; }; fi
	@if [ "$(ROLE)" = jetson ] && [ "$(ARCH)" != arm64 ] && [ "$(FORCE)" != 1 ]; then \
	  echo "ROLE=jetson needs an arm64 host, this one is $(UNAME_M). Build the image here with 'make jetson-image', or FORCE=1." >&2; exit 1; fi

_jetson: _role
	@[ "$(ROLE)" = jetson ] || { echo "this command needs the real arm: run it on the Jetson (this is ROLE=$(ROLE))" >&2; exit 1; }

$(WS_DIRS):
	@mkdir -p $@
