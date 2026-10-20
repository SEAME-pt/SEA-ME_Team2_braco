# SEA:ME 2.0 - arm class: the one Makefile of the repo.
#
#   make              list all targets
#   make env          create docker/.env once (UID/GID, groups, profile)
#   make up           build if needed and start this machine's container
#   make shell        open a shell inside it
#   make smoke        verify the install
#   make ci           host checks, lint, web tests (the `host` CI job; `make ci-image ci-ros` is the `ros` job)
#
# Run everything from the repo root. Two groups of targets:
#   docker workflow   up, shell, build, ws-build, arm-*, lint, clean ...   (docker compose, project dir docker/)
#   host workflow     host-venv, host-python, host-cpp, host-check, host-clean, init   (venv + CMake hello-world on this machine)
#
# Which machine is this?  ROLE=pc (workstation) or ROLE=jetson (the Nano).
# Resolved from, in order: ROLE=... on the command line, $COMPOSE_PROFILES, COMPOSE_PROFILES in docker/.env, then
# detected from this machine (arm64 + Tegra marker = jetson, anything else = pc). Runtime targets refuse
# ROLE=jetson on a non-arm64 host (FORCE=1 bypasses); see `make info`.
#
# Project layout - every path used below is defined in the next block, so moving a file means editing one line:
#
#   Makefile  requirements.txt  tests/                            the root: host venv and C++ hello-world
#   docker/     docker-compose.yml  .env.example  README.md       the compose project (every docker compose call
#                                                                 below passes --project-directory docker)
#   docker/compose/   nvidia.yml dri.yml                          GPU overrides
#   docker/docker/    Dockerfile  config/  scripts/               image definition (stages: base, jetson-driver, pc-dev)
#   docker/tests/     talker-listener.sh                          ROS 2 checks; mounted into the containers (no rebuild)
#   docker/systemd/   seame-arm.service                           boot unit for the arm services (make install-service)
#   libs/             the team's C++ library (mounted read-only at /opt/seame/libs; colcon builds it as mycobot_cpp)
#   created by make when needed:  docker/.env   docker/ws/src   docker/ws/bags   docker/dist/

.DEFAULT_GOAL := help

# ---- project layout (edit paths here, nowhere else) -------------------------------------------
# (Only the `## ...` help strings repeat a few paths literally, because `make help` reads this file as text.)
PROJECT      := seame-arm-docker
DOCKER_DIR   := docker
IMAGE_DIR    := $(DOCKER_DIR)/docker
ENV_FILE     := $(DOCKER_DIR)/.env
ENV_EXAMPLE  := $(DOCKER_DIR)/.env.example
COMPOSE_BASE := $(DOCKER_DIR)/docker-compose.yml
COMPOSE_DIR  := $(DOCKER_DIR)/compose
DOCKERFILE   := $(IMAGE_DIR)/Dockerfile
TL_TESTS_DIR := $(DOCKER_DIR)/tests
TL_SCRIPT    := $(TL_TESTS_DIR)/talker-listener.sh
# Where docker-compose.yml mounts docker/tests inside the containers (no trailing comment on the next line: it would leak spaces into the value).
CONTAINER_TESTS := /opt/seame/tests
# Where docker-compose.yml mounts ../libs; colcon also looks for packages there.
CONTAINER_LIBS := /opt/seame/libs
WS_BASE_PATHS  := src $(CONTAINER_LIBS)
SYSTEMD_UNIT := $(DOCKER_DIR)/systemd/seame-arm.service
SYSTEMD_DEST := /etc/systemd/system/seame-arm.service
CONFIG_DIR   := $(IMAGE_DIR)/config
SCRIPTS_DIR  := $(IMAGE_DIR)/scripts
WS_DIR       := $(DOCKER_DIR)/ws
WS_SRC       := $(WS_DIR)/src
WS_BAGS      := $(WS_DIR)/bags
DIST_DIR     := $(DOCKER_DIR)/dist
ARCHIVE      := $(DIST_DIR)/$(PROJECT).tar

# Host workflow (no docker): Python venv and the CMake hello-world.
# The interpreter version lives in .python-version (read by CI and uv/pyenv too);
# numpy==1.24.4 in requirements.txt has no wheels past 3.11.
PY_VERSION   := $(shell cat .python-version)
PYTHON       ?= python$(PY_VERSION)
VENV         := .venv
PIP          := $(VENV)/bin/pip
CPP_DIR      := tests
CPP_BUILD    := $(CPP_DIR)/build
# The braco web page on the host (make web / web-start); WEB_PORT is set with the other options below.
WEB_PID      := $(VENV)/web.pid
WEB_LOG      := $(VENV)/web.log
WEB_CMD       = $(VENV)/bin/uvicorn web.server.app:app --host 0.0.0.0 --port $(WEB_PORT)

# Folders that must exist before a container bind-mounts ./ws. Created by `make dirs` (and automatically by
# `make up` / `make run`) as YOUR user; if Docker creates them instead they end up root-owned.
WS_DIRS := $(WS_SRC) $(WS_BAGS)

# Files that must be present for the project to work (checked by `make doctor`).
PROJECT_FILES := $(COMPOSE_BASE) $(ENV_EXAMPLE) $(DOCKERFILE) $(CONFIG_DIR)/colcon-defaults.yaml $(TL_SCRIPT) $(SYSTEMD_UNIT) \
                 $(addprefix $(COMPOSE_DIR)/,nvidia.yml dri.yml) \
                 $(addprefix $(SCRIPTS_DIR)/,apt-install-optional.sh env.sh install-plotjuggler.sh \
                                             install-ros-apt-source.sh ros_entrypoint.sh smoke-test.sh)

# ---- configuration ---------------------------------------------------------------------------
# Read KEY from .env. The shell environment wins over .env, exactly as it does for docker compose.
dotenv = $(strip $(shell sed -n 's/^$(1)=//p' $(ENV_FILE) 2>/dev/null | tail -n 1))

# What machine is this? Probed once. UNAME_M and IS_JETSON can be overridden to simulate another host, e.g.
#   make info UNAME_M=aarch64 IS_JETSON=1
# aarch64 alone does not mean "Jetson" (Raspberry Pi, ARM servers, Linux VMs on Apple Silicon report it too),
# so a Jetson is arm64 AND carries a Tegra marker.
UNAME_S ?= $(shell uname -s)
UNAME_M ?= $(shell uname -m)
ARCH    := $(if $(filter aarch64 arm64,$(UNAME_M)),arm64,amd64)
ifndef IS_JETSON
IS_JETSON := $(shell { grep -qsai jetson /proc/device-tree/model || test -f /etc/nv_tegra_release; } && echo 1)
endif
DETECTED_ROLE := $(if $(and $(filter arm64,$(ARCH)),$(IS_JETSON)),jetson,pc)
# Informational only: GPU stays opt-in (an empty GPU defers to COMPOSE_FILE in .env).
GPU_HINT := $(if $(shell command -v nvidia-smi 2>/dev/null),nvidia,$(if $(wildcard /dev/dri),dri))

# ROLE and GPU are generic names, so only the command line may set them (never a stray env var).
ifneq ($(origin ROLE),command line)
ROLE := $(or $(COMPOSE_PROFILES),$(call dotenv,COMPOSE_PROFILES),$(DETECTED_ROLE))
ROLE_SRC := $(if $(COMPOSE_PROFILES),environment,$(if $(call dotenv,COMPOSE_PROFILES),$(ENV_FILE),detected))
else
ROLE_SRC := command line
endif
ifneq ($(origin GPU),command line)
GPU :=
endif

IMAGE_PREFIX ?= $(or $(call dotenv,IMAGE_PREFIX),seame)
IMAGE_TAG    ?= $(or $(call dotenv,IMAGE_TAG),lyrical)
MYCOBOT_PORT ?= $(or $(call dotenv,MYCOBOT_PORT),/dev/ttyTHS1)

SERVICE_pc     := pc-dev
SERVICE_jetson := jetson-driver
SERVICE        := $(SERVICE_$(ROLE))

# Every docker compose call goes through the project directory, so docker/.env, ./ws and ../libs resolve as before.
DC := docker compose --project-directory $(DOCKER_DIR)

# Always-on arm (profile "arm", Jetson only): one container running the seame_arm ROS 2 launch.
ARM_SERVICES := arm
# colcon --symlink-install makes this file a symlink to /ws/src/... (a container path), so on the host `test -f`
# follows it into nothing and fails: also accept the link itself.
ARM_BUILT    := $(WS_DIR)/install/seame_arm/share/seame_arm/package.xml
ARM          := $(DC) --profile arm

# GPU=nvidia|dri adds the matching override file (PC only). Without it, compose uses COMPOSE_FILE from .env.
COMPOSE_FILES := $(if $(GPU),-f $(COMPOSE_BASE) -f $(COMPOSE_DIR)/$(GPU).yml)
COMPOSE       := $(DC) $(COMPOSE_FILES) --profile $(ROLE)

# No TTY in CI (GitHub Actions sets CI=true). Override with NOTTY=-T or NOTTY= when needed.
ifdef CI
NOTTY ?= -T
endif

# EXEC goes through the image entrypoint, so ROS and your workspace are sourced (a bare `exec` would not).
EXEC := $(COMPOSE) exec $(NOTTY) $(SERVICE) ros_entrypoint.sh
RUN1 := $(COMPOSE) run --rm $(NOTTY) $(SERVICE)

XHOST       ?= 1
FORCE       ?= 0
PING_TOPIC  ?= /seame_ping
WEB_PORT    ?= 8000
COLCON_ARGS ?= $(if $(filter jetson,$(ROLE)),--parallel-workers 2)
JETSON_TAG  := $(IMAGE_TAG)-arm64
NANO        ?=

# CMD reaches the shell exactly as typed. Without $(value ...) make would expand `$LANG` or `$(id -u)` itself.
export SEAME_CMD = $(value CMD)

# Talker/listener options (TOPIC, COUNT, TIMEOUT): honoured only when given on the command line, because
# names this generic may well exist as unrelated environment variables.
TL_ENV := $(foreach v,TOPIC COUNT TIMEOUT,$(if $(filter command line,$(origin $(v))),-e $(v)=$($(v))))
# Runs through the image entrypoint (ROS sourced) with a TTY, so Ctrl-C reaches the nodes.
TL     = $(COMPOSE) exec $(NOTTY) $(TL_ENV) $(SERVICE) ros_entrypoint.sh bash $(CONTAINER_TESTS)/talker-listener.sh

define confirm
@if [ "$(FORCE)" != 1 ]; then printf '%s [y/N] ' "$(1)"; read -r ans; [ "$$ans" = y ] || { echo "aborted"; exit 1; }; fi
endef

.PHONY: all ci ci-image ci-ros init host-venv host-python host-cpp host-check host-clean _jetson help info doctor env dirs lint tar xhost build rebuild pull push up down restart ps logs config \
        arm-up arm-down arm-ps arm-logs arm-web-logs arm-basics arm-restart arm-web-restart arm-park arm-zero arm-rest arm-resume arm-temps install-service uninstall-service service-status \
        shell exec run smoke talker-listener talker talker-stop listener ping-pub ping-echo ws-deps ws-build ws-test ws-clean web-sim _pc_only \
        web-deps web web-start web-stop web-restart web-re web-reinstall web-status web-logs web-mock web-test \
        clean distclean binfmt jetson-cross jetson-ship _role _arch _xhost

##@ Setup

help: ## Show this help
	@printf 'SEA:ME arm class - docker compose helper\n  current: ROLE=%s (%s)  service=%s  GPU=%s  host=%s\n' "$(ROLE)" "$(ROLE_SRC)" "$(SERVICE)" "$(or $(GPU),none)" "$(UNAME_S)/$(ARCH)"
	@printf '  usage:   make <target> [ROLE=pc|jetson] [GPU=nvidia|dri] [CMD=...] [NANO=user@host]   (host targets: make host-*)\n'
	@awk 'BEGIN {FS = ":.*## "} /^[a-zA-Z0-9_-]+:.*## / {printf "  \033[36m%-16s\033[0m %s\n", $$1, $$2} /^##@/ {printf "\n\033[1m%s\033[0m\n", substr($$0, 5)}' $(MAKEFILE_LIST)

env: _role ## Create docker/.env from docker/.env.example (UID/GID, groups and profile filled in for this host)
	@if [ "$$(id -u)" = 0 ]; then echo "Run 'make env' as your normal user, not root/sudo (root's UID 0 would break the image's user)." >&2; exit 1; fi
	@if [ -f $(ENV_FILE) ]; then echo "$(ENV_FILE) already exists - leaving it alone"; else \
	  cp $(ENV_EXAMPLE) $(ENV_FILE) && \
	  sed -e "s/^USER_UID=.*/USER_UID=$$(id -u)/" -e "s/^USER_GID=.*/USER_GID=$$(id -g)/" \
	      -e "s/^COMPOSE_PROFILES=.*/COMPOSE_PROFILES=$(ROLE)/" $(ENV_FILE) > $(ENV_FILE).tmp && mv $(ENV_FILE).tmp $(ENV_FILE) && \
	  for pair in DIALOUT_GID:dialout VIDEO_GID:video RENDER_GID:render I2C_GID:i2c INPUT_GID:input; do \
	    var=$${pair%%:*}; grp=$${pair##*:}; gid=$$(getent group $$grp 2>/dev/null | cut -d: -f3); \
	    if [ -n "$$gid" ]; then sed "s/^$$var=.*/$$var=$$gid/" $(ENV_FILE) > $(ENV_FILE).tmp && mv $(ENV_FILE).tmp $(ENV_FILE); fi; \
	  done; \
	  echo "created $(ENV_FILE) for ROLE=$(ROLE) (uid $$(id -u), gid $$(id -g)) - review it, then: make up"; fi

info: _role ## Show what this Makefile resolved (role, service, images, compose command)
	@echo "ROLE          $(ROLE)  (from $(ROLE_SRC))"
	@echo "host          $(UNAME_S) $(UNAME_M) -> $(ARCH)  jetson marker: $(or $(IS_JETSON),no)  detected role: $(DETECTED_ROLE)"
	@echo "GPU hint      $(or $(GPU_HINT),none)  (opt in with GPU=$(or $(GPU_HINT),nvidia|dri), PC only)"
	@echo "service       $(SERVICE)"
	@echo "image         $(IMAGE_PREFIX)/$(SERVICE):$(IMAGE_TAG)"
	@echo "compose cmd   $(COMPOSE)"
	@echo "serial port   $(MYCOBOT_PORT)  (jetson only)"
	@echo "compose files $(COMPOSE_BASE) $(addprefix $(COMPOSE_DIR)/,nvidia.yml dri.yml)"
	@echo "dockerfile    $(DOCKERFILE)   scripts: $(SCRIPTS_DIR)/   config: $(CONFIG_DIR)/   tests: $(TL_TESTS_DIR)/"
	@echo "workspace     $(WS_SRC)  bags: $(WS_BAGS)   archive: $(ARCHIVE)"

define DOCTOR_SH
fail=0
ok()   { printf '  [ ok ] %s\n' "$$*"; }
warn() { printf '  [warn] %s\n' "$$*"; }
bad()  { printf '  [FAIL] %s\n' "$$*"; fail=1; }
echo "Host checks for ROLE=$$ROLE"
if command -v docker >/dev/null 2>&1; then
  ok "$$(docker --version)"
  if docker info >/dev/null 2>&1; then ok "docker daemon reachable"; else bad "cannot reach the docker daemon (running? are you in the docker group?)"; fi
else
  bad "docker is not installed"
fi
v=$$(docker compose version --short 2>/dev/null | sed 's/^v//')
if [ -z "$$v" ]; then
  bad "docker compose v2 plugin not found"
elif [ "$$(printf '%s\n2.20.0\n' "$$v" | sort -V | head -n 1)" = "2.20.0" ]; then
  ok "docker compose $$v (2.20+ required)"
else
  bad "docker compose $$v is too old (this project needs Compose 2.20+)"
fi
missing=""
for f in $$PROJECT_FILES; do [ -e "$$f" ] || missing="$$missing $$f"; done
if [ -z "$$missing" ]; then ok "all project files present"; else bad "missing project files:$$missing"; fi
if [ -f "$$ENV_FILE" ]; then ok "$$ENV_FILE present"; else warn "$$ENV_FILE missing - run: make env ROLE=$$ROLE"; fi
uid=$$(sed -n 's/^USER_UID=//p' "$$ENV_FILE" 2>/dev/null | tail -n 1)
if [ -n "$$uid" ] && [ "$$uid" != "$$(id -u)" ]; then warn "USER_UID=$$uid in $$ENV_FILE but your uid is $$(id -u): the workspace may not be writable from the container"; fi
for d in $$WS_DIRS; do
  if [ -d "$$d" ]; then
    if [ -w "$$d" ]; then ok "$$d present and writable"; else bad "$$d is not writable by you (created by Docker as root? fix: sudo chown -R $$(id -u):$$(id -g) $$d)"; fi
  else warn "$$d missing - make dirs (or make up) creates it"; fi
done
case "$$ROLE" in
  pc)
    if [ -n "$$IS_JETSON" ]; then warn "this host looks like a Jetson but ROLE=pc: did you mean ROLE=jetson?"; fi
    if [ -n "$$DISPLAY" ] && [ -d /tmp/.X11-unix ]; then ok "DISPLAY=$$DISPLAY and X11 socket directory present"; else warn "no DISPLAY or X11 socket: RViz2 and Gazebo windows will not open"; fi
    if command -v xhost >/dev/null 2>&1; then ok "xhost available"; else warn "xhost not installed (apt install x11-xserver-utils)"; fi
    ;;
  jetson)
    if [ "$$ARCH" = arm64 ]; then ok "arm64 host"; else bad "host is $$UNAME_M, not arm64: ROLE=jetson cannot run here (build the image with make jetson-cross)"; fi
    if [ -e "$$MYCOBOT_PORT" ]; then ok "serial device $$MYCOBOT_PORT present"; else warn "serial device $$MYCOBOT_PORT not found (the container will not start without it)"; fi
    if [ "$$MYCOBOT_PORT" = /dev/ttyTHS1 ] && command -v systemctl >/dev/null 2>&1 && systemctl is-active --quiet nvgetty 2>/dev/null; then
      warn "nvgetty is active and holds /dev/ttyTHS1: sudo systemctl disable --now nvgetty"
    fi
    if [ -e /dev/i2c-0 ]; then ok "/dev/i2c-0 present (LCD for temperature_monitor)"; else warn "/dev/i2c-0 not found: temperature_monitor will not start (set LCD_I2C_BUS in .env)"; fi
    if [ -e /dev/input/js0 ]; then ok "joystick /dev/input/js0 present"; else warn "no joystick at /dev/input/js0 (joystick_teleop waits for it)"; fi
    ;;
esac
exit $$fail
endef
export DOCTOR_SH

doctor: _role ## Check this host and project: docker, compose, project files, .env, folders, display/serial
	@ROLE=$(ROLE) ARCH=$(ARCH) UNAME_M=$(UNAME_M) IS_JETSON=$(IS_JETSON) MYCOBOT_PORT=$(MYCOBOT_PORT) ENV_FILE=$(ENV_FILE) WS_DIRS="$(WS_DIRS)" PROJECT_FILES="$(PROJECT_FILES)" sh -c "$$DOCTOR_SH"

xhost: ## (pc) Let local containers draw on your X11/XWayland display (xhost +local:)
	@if command -v xhost >/dev/null 2>&1; then xhost +local:; else echo "xhost not found: apt install x11-xserver-utils"; exit 1; fi

##@ Project folders, checks, packaging

dirs: $(WS_DIRS) ## Create the folders the project needs (docker/ws/src, docker/ws/bags) as your user

$(WS_DIRS):
	@mkdir -p $@ && echo "created $@"

# hadolint: a local binary if installed, else this image through docker (CI uses the image too), so the
# Dockerfile check is never skipped (lint needs docker anyway, for the compose files).
HADOLINT_IMAGE := hadolint/hadolint:v2.12.0
HADOLINT_FLAGS := --ignore DL3008 --ignore DL3013 --ignore DL3059 --ignore DL3006 --ignore DL3042 --ignore DL3045 \
                  --ignore SC1091 --ignore SC2086 --ignore SC2016

lint: _role ## Check the project: compose files, scripts and tests (shellcheck), Dockerfile (hadolint, or its docker image)
	@echo "== compose =="
	@$(DC) --profile pc --profile jetson --profile arm config -q && echo "  $(COMPOSE_BASE): ok (all profiles)"
	@for g in nvidia dri; do $(DC) -f $(COMPOSE_BASE) -f $(COMPOSE_DIR)/$$g.yml --profile pc config -q && echo "  $(COMPOSE_DIR)/$$g.yml: ok"; done
	@echo "== shell scripts =="
	@if command -v shellcheck >/dev/null 2>&1; then shellcheck -x $(SCRIPTS_DIR)/*.sh $(TL_TESTS_DIR)/*.sh && echo "  shellcheck: clean"; else echo "  shellcheck not installed - skipped"; fi
	@echo "== Dockerfile =="
	@if command -v hadolint >/dev/null 2>&1; then hadolint $(HADOLINT_FLAGS) $(DOCKERFILE) && echo "  hadolint: clean"; \
	else \
	  { docker image inspect $(HADOLINT_IMAGE) >/dev/null 2>&1 || docker pull -q $(HADOLINT_IMAGE) >/dev/null; } && \
	  docker run --rm -i $(HADOLINT_IMAGE) hadolint $(HADOLINT_FLAGS) - < $(DOCKERFILE) && echo "  hadolint ($(HADOLINT_IMAGE)): clean"; fi

TAR ?= tar
tar: ## Pack docker/ into docker/dist/seame-arm-docker.tar (leaves out .env, ws/ and dist/; needs GNU tar)
	@$(TAR) --version 2>/dev/null | grep -q 'GNU tar' || { echo "make tar needs GNU tar (macOS: brew install gnu-tar, then make tar TAR=gtar)" >&2; exit 1; }
	@mkdir -p $(DIST_DIR)
	$(TAR) -C $(DOCKER_DIR) -cf $(CURDIR)/$(ARCHIVE) --sort=name --owner=0 --group=0 --numeric-owner \
	    --exclude='./.env' --exclude='./.env.tmp' --exclude='./ws' --exclude='./dist' --exclude='./.git' \
	    --transform='s,^\.,$(PROJECT),' .
	@echo "wrote $(ARCHIVE) ($$(du -h $(ARCHIVE) | cut -f1)); unpack with: tar -xf $(ARCHIVE)"

##@ Build and run (the service is chosen by ROLE)

build: _role _arch ## Build the image for this machine (BUILD_ARGS='--progress=plain' etc.)
	$(COMPOSE) build $(BUILD_ARGS) $(SERVICE)

rebuild: BUILD_ARGS := --no-cache --pull
rebuild: build ## Rebuild from scratch (no cache, fresh ubuntu:26.04) - slow

pull: _role _arch ## Pull this machine's prebuilt image from the registry in IMAGE_PREFIX (then: make up)
	docker pull $(IMAGE_PREFIX)/$(SERVICE):$(IMAGE_TAG)

push: _role _arch ## Push this machine's image (IMAGE_PREFIX must be a registry path)
	$(COMPOSE) push $(SERVICE)

up: _role _arch _xhost | $(WS_DIRS) ## Start this machine's container in the background (builds first if missing)
	$(COMPOSE) up -d $(SERVICE)

down: _role ## Stop and remove the containers (named volumes are kept)
	$(COMPOSE) down --remove-orphans

restart: _role _arch ## Restart the container
	$(COMPOSE) restart $(SERVICE)

ps: _role ## Show container status
	$(COMPOSE) ps

logs: _role ## Follow the container log
	$(COMPOSE) logs -f --tail=100 $(SERVICE)

config: _role ## Print the fully resolved compose configuration
	$(COMPOSE) config

##@ Inside the container

shell: _role up ## Open an interactive shell (ROS and your workspace sourced)
	$(COMPOSE) exec $(SERVICE) bash

exec: _role up ## Run CMD in the running container: make exec CMD='ros2 topic list'
	@test -n "$$SEAME_CMD" || { echo "usage: make exec CMD='ros2 topic list'" >&2; exit 2; }
	$(EXEC) bash -c "$$SEAME_CMD"

run: _role _arch | $(WS_DIRS) ## Run CMD in a throw-away container (CI style): make run CMD='colcon build'
	@test -n "$$SEAME_CMD" || { echo "usage: make run CMD='colcon build'" >&2; exit 2; }
	$(RUN1) bash -c "$$SEAME_CMD"

smoke: _role up ## Verify the install: packages, pymycobot, RMW loopback (seame-smoke-test)
	$(EXEC) seame-smoke-test

ping-pub: _role up ## Publish a test message on PING_TOPIC - run on one machine ...
	$(EXEC) ros2 topic pub -r 2 $(PING_TOPIC) std_msgs/msg/String "{data: 'hello from $(ROLE)'}"

ping-echo: _role up ## ... and echo it on the other, to prove the PC and Jetson see each other
	$(EXEC) ros2 topic echo $(PING_TOPIC) std_msgs/msg/String

##@ ROS 2 talker/listener test (works on the PC and on the Jetson)

talker-listener: _role up ## Self-test: C++ talker + Python listener in this container, PASS/FAIL (run on each machine)
	$(TL) selftest

talker: _role up ## Two-machine test, step 1: run the C++ talker here (Ctrl-C stops it)
	$(TL) talker

listener: _role up ## Two-machine test, step 2: on the OTHER machine, run the checked Python listener (COUNT=0: no check)
	$(TL) listener

talker-stop: _role up ## Stop any talker left running in this container (e.g. one started without a terminal)
	@$(EXEC) bash -c 'if pkill -INT -f "[d]emo_nodes_cpp[/ ]talker"; then echo "stopped the talker(s) in $(SERVICE)"; else echo "no talker running in $(SERVICE)"; fi'

##@ Workspace (docker/ws/ is mounted at /ws in the container on both machines)

ws-deps: _role up ## rosdep-install the dependencies of docker/ws/src (lives in the running container only)
	$(EXEC) bash -c 'rosdep update >/dev/null 2>&1 || true; sudo apt-get update && rosdep install --from-paths $(WS_BASE_PATHS) --ignore-src -y'

ws-build: _role up ## colcon build of docker/ws/src + libs (adds --parallel-workers 2 on the Jetson; COLCON_ARGS=...)
	$(EXEC) colcon build --base-paths $(WS_BASE_PATHS) $(COLCON_ARGS)

ws-test: _role up ## colcon test, then print the results (includes ament lint checks)
	$(EXEC) bash -c 'colcon test --base-paths $(WS_BASE_PATHS) && colcon test-result --verbose'

ws-clean: ## Delete docker/ws/build, docker/ws/install and docker/ws/log
	rm -rf $(WS_DIR)/build $(WS_DIR)/install $(WS_DIR)/log

##@ Arm on the Jetson (one container: driver, temperature, teleop + web page, all under ROS 2)

arm-up: _jetson | $(WS_DIRS) ## Start the arm (build the workspace first: make ws-build ROLE=jetson)
	@test -e $(ARM_BUILT) || test -L $(ARM_BUILT) || { echo "seame_arm is not built yet: make ws-build ROLE=jetson" >&2; exit 1; }
	$(ARM) up -d $(ARM_SERVICES)

arm-down: _jetson ## Stop the arm and remove the container (does not park it: make arm-park first); the dev container is not touched
	$(ARM) stop $(ARM_SERVICES)
	$(ARM) rm --force $(ARM_SERVICES)

arm-ps: _jetson ## Status of the arm container
	$(ARM) ps $(ARM_SERVICES)

arm-logs: _jetson ## Follow the arm logs (driver, temperature and teleop nodes, web page)
	$(ARM) logs -f --tail=100 $(ARM_SERVICES)

arm-restart: _jetson ## Restart the arm container after make ws-build (the arm is not zeroed or parked)
	$(ARM) restart $(ARM_SERVICES)

# These run inside the already-running arm container (ROS sourced, workspace mounted),
# so nothing has to be stopped and the serial port keeps a single owner.
ARM_EXEC := $(ARM) exec $(NOTTY) arm ros_entrypoint.sh

arm-park: _jetson ## Fold the arm into the park pose, release the joints and enter rest mode (driver keeps reading temperatures)
	$(ARM_EXEC) ros2 run seame_arm park.py

arm-zero: _jetson ## Leave rest mode and move the arm to the zero pose
	$(ARM_EXEC) ros2 run seame_arm zero.py

arm-rest: _jetson ## Rest mode on: motion refused, temperatures keep being published
	$(ARM_EXEC) ros2 service call /arm/set_read_only std_srvs/srv/SetBool "{data: true}"

arm-resume: _jetson ## Rest mode off: motion accepted again (joints stay free until zero or power_on)
	$(ARM_EXEC) ros2 service call /arm/set_read_only std_srvs/srv/SetBool "{data: false}"

arm-temps: _jetson ## Print the servo temperatures once (works in rest mode)
	$(ARM_EXEC) ros2 topic echo --once /arm/servo_temperatures

arm-web-logs: _jetson ## Follow only the braco web page lines of the arm logs (the page: http://<jetson>:$(WEB_PORT))
	$(ARM) logs -f --tail=200 $(ARM_SERVICES) | grep --line-buffered -iE 'uvicorn|braco|web page|Application|HTTP'

arm-web-restart: _jetson ## Restart only the braco web page inside the running arm (no zero, no park; everyone must take control again)
	$(ARM_EXEC) env WEB_PORT=$(WEB_PORT) bash /opt/braco/web/restart.sh

arm-basics: _jetson ## Run the ros2_basics demos in the arm container for the page's ROS 2 > Demos tab (Ctrl-C stops them)
	$(ARM_EXEC) bash -c 'ros2 run ros2_basics_py managed_talker & ros2 run ros2_basics_cpp managed_talker & \
	  trap "kill %1 %2 2>/dev/null" EXIT; ros2 launch ros2_basics_py basics.launch.py'

install-service: _jetson ## Start the arm services at boot (systemd unit seame-arm, needs sudo)
	@test -e $(ARM_BUILT) || test -L $(ARM_BUILT) || { echo "seame_arm is not built yet: make ws-build ROLE=jetson" >&2; exit 1; }
	sed -e "s|@WORKDIR@|$(CURDIR)/$(DOCKER_DIR)|g" -e "s|@DOCKER@|$$(command -v docker)|g" $(SYSTEMD_UNIT) | sudo tee $(SYSTEMD_DEST) >/dev/null
	sudo systemctl daemon-reload
	sudo systemctl enable --now seame-arm.service
	@echo "installed $(SYSTEMD_DEST): the arm services now start at boot (make service-status)"

uninstall-service: ## Remove the boot unit and stop the arm services
	-sudo systemctl disable --now seame-arm.service
	sudo rm -f $(SYSTEMD_DEST)
	sudo systemctl daemon-reload

service-status: ## Show the boot unit and the arm services
	-systemctl status --no-pager seame-arm.service
	$(ARM) ps $(ARM_SERVICES)

##@ Web dashboard on Gazebo (PC only)

_pc_only: _role
	@test "$(ROLE)" = pc || { echo "this target is PC only (ROLE=$(ROLE)); on the Jetson the page comes with 'make arm-up' (port $(WEB_PORT))" >&2; exit 1; }

web-sim: _role _pc_only up ## Serve the braco web page from pc-dev, showing and commanding Gazebo (WEB_PORT=8000)
	$(EXEC) bash -c 'cd /opt/braco && ARM_SOURCE=gazebo exec python3 -m uvicorn web.server.app:app --host 0.0.0.0 --port $(WEB_PORT) --workers 1'

##@ Jetson image from a PC (no Docker build on the Nano)

binfmt: ## One-time: register QEMU so this PC can build arm64 images
	@[ "$(ARCH)" != arm64 ] || { echo "this host is already arm64: QEMU is not needed" >&2; exit 1; }
	docker run --privileged --rm tonistiigi/binfmt --install arm64

jetson-cross: ## Cross-build the Jetson image on this PC (arm64 via QEMU, slow); tag <IMAGE_TAG>-arm64
	@[ "$(ARCH)" != arm64 ] || { echo "this host is already arm64: build natively with make build ROLE=jetson" >&2; exit 1; }
	IMAGE_TAG=$(JETSON_TAG) DOCKER_DEFAULT_PLATFORM=linux/arm64 $(DC) --profile jetson build jetson-driver

jetson-ship: ## Copy the cross-built image to the Nano: make jetson-ship NANO=user@nano.local
	@test -n "$(NANO)" || { echo "usage: make jetson-ship NANO=user@host" >&2; exit 2; }
	docker save $(IMAGE_PREFIX)/jetson-driver:$(JETSON_TAG) | gzip | ssh $(NANO) 'gunzip | docker load'
	@echo "On the Nano set IMAGE_TAG=$(JETSON_TAG) in $(ENV_FILE), then: make up ROLE=jetson"

##@ Host (Python venv, C++ hello-world on this machine, no docker)

all: host-python host-cpp ## Run the Python and C++ hello-world checks

host-python: host-venv ## Run the Python hello-world in the venv
	$(VENV)/bin/python tests/hello_world.py

host-cpp: ## Configure, build and run the C++ hello-world
	cmake -S $(CPP_DIR) -B $(CPP_BUILD)
	cmake --build $(CPP_BUILD)
	$(CPP_BUILD)/hello_world

host-check: host-venv ## Hardware-free checks: Python imports and compile, C++ build without running
	$(VENV)/bin/python -c 'import ast; ast.parse(open("tests/hello_world.py").read())'
	$(VENV)/bin/python -c 'import numpy, pymycobot, serial'
	cmake -S $(CPP_DIR) -B $(CPP_BUILD)
	cmake --build $(CPP_BUILD)

host-clean: ## Remove the venv and the C++ build (host only; containers: make clean)
	rm -rf $(VENV) $(CPP_BUILD)

init: host-venv ## Create the venv and enable the git hooks
	git config core.hooksPath .githooks

$(VENV)/.installed: requirements.txt .python-version
	@$(PYTHON) -c 'import sys; v="%d.%d" % sys.version_info[:2]; sys.exit(0 if v == "$(PY_VERSION)" else "need Python $(PY_VERSION), $(PYTHON) is " + v + " (install it, e.g. uv python install $(PY_VERSION), or set PYTHON=/path/to/python$(PY_VERSION))")'
	rm -rf $(VENV)
	$(PYTHON) -m venv $(VENV)
	$(PIP) install --upgrade pip
	$(PIP) install -r requirements.txt
	touch $@

host-venv: $(VENV)/.installed ## Create the venv and install requirements.txt

##@ Continuous integration (.github/workflows/ci.yml; no robot, no display needed)

ci: host-check lint web-test ## The `host` CI job: host checks, lint, web page tests

# The `ros` CI job builds the workspace in a throw-away container from the Dockerfile's `base` stage: it has
# everything the packages in docker/ws/src and libs depend on, without the GUI/Gazebo/MoveIt layers of pc-dev.
# The image's user gets your UID/GID so the bind-mounted docker/ws stays writable (GitHub runners are UID 1001).
CI_TARGET ?= base
CI_IMAGE  ?= $(IMAGE_PREFIX)/ci:$(IMAGE_TAG)
CI_ROS_SH := set -e; rosdep update --rosdistro "$$ROS_DISTRO" >/dev/null 2>&1 || true; \
             sudo apt-get update -qq; rosdep install --from-paths $(WS_BASE_PATHS) --ignore-src -y; \
             colcon build --base-paths $(WS_BASE_PATHS) $(COLCON_ARGS); \
             colcon test --base-paths $(WS_BASE_PATHS); colcon test-result --verbose

ci-image: ## Build the `ros` CI job's image (Dockerfile stage CI_TARGET=base) as CI_IMAGE, for your UID/GID
	docker build --target $(CI_TARGET) --build-arg USER_UID=$$(id -u) --build-arg USER_GID=$$(id -g) -t $(CI_IMAGE) $(IMAGE_DIR)

ci-ros: ## The `ros` CI job: rosdep, colcon build and colcon test of docker/ws/src + libs in a throw-away CI_IMAGE container
	@docker image inspect $(CI_IMAGE) >/dev/null 2>&1 || { echo "image $(CI_IMAGE) not found: make ci-image" >&2; exit 1; }
	docker run --rm -e CCACHE_DIR=/ws/.ccache -e CCACHE_MAXSIZE=500M -v $(CURDIR)/$(WS_DIR):/ws -v $(CURDIR)/libs:$(CONTAINER_LIBS):ro \
	    $(CI_IMAGE) bash -c '$(CI_ROS_SH)'

##@ Braco web page (web/README.md; on the Jetson it runs inside the arm container: make arm-up)

# `make web` / web-start are the fallback that opens the serial port directly with pymycobot: only use them while
# the arm container is down. web-mock and web-test only need the web packages, so they also work on a PC where
# requirements.txt (pymycobot, numpy) cannot be installed: the venv prefers $(PYTHON) and falls back to python3.
$(VENV)/bin/python:
	$(or $(shell command -v $(PYTHON) 2>/dev/null),python3) -m venv $(VENV)
	$(PIP) install --upgrade pip

$(VENV)/.web-installed: $(VENV)/bin/python web/requirements.txt
	$(PIP) install -r web/requirements.txt
	touch $@

$(VENV)/.web-test-installed: $(VENV)/.web-installed web/requirements-test.txt
	$(PIP) install -r web/requirements-test.txt
	touch $@

web-deps: $(VENV)/.web-installed ## Install the web page's packages (fastapi, uvicorn) into the venv

web: host-venv web-deps ## Serve the page in the foreground on WEB_PORT, pymycobot straight to the serial port
	$(WEB_CMD)

# web-start runs the server detached, with its PID in $(WEB_PID) and output in $(WEB_LOG), so the terminal stays free.
web-start: host-venv web-deps ## Same as web, in the background (web-stop, web-restart, web-status, web-logs)
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

web-stop: ## Stop the background page (and any stray uvicorn holding the port)
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

web-restart: web-stop web-start ## Restart the background page

# The braco page runs inside the arm container on the Jetson. The arm is not zeroed or parked.
web-re: ## (jetson) Restart the arm container with its page; only the page: make arm-web-restart
	$(MAKE) arm-restart

web-reinstall: web-stop ## Wipe the venv (fixes corrupted/partial installs), reinstall, then web-start
	rm -rf $(VENV)
	$(MAKE) host-venv web-deps
	$(MAKE) web-start

web-status: ## Is the background page running?
	@if [ -f $(WEB_PID) ] && kill -0 $$(cat $(WEB_PID)) 2>/dev/null; then \
		echo "web running (pid $$(cat $(WEB_PID))) on port $(WEB_PORT)"; \
	else echo "web stopped"; fi

web-logs: ## Follow the background page's log
	@tail -n 50 -f $(WEB_LOG)

web-mock: web-deps ## Serve the page with mock arm data on localhost (no arm, no ROS)
	ARM_MOCK=1 $(VENV)/bin/uvicorn web.server.app:app --host 127.0.0.1 --port $(WEB_PORT) --reload

web-test: $(VENV)/.web-test-installed ## Run the web page's tests (pytest web/tests, no arm needed)
	$(VENV)/bin/python -m pytest web/tests -q


##@ Cleanup

clean: _role ## Stop containers AND delete the cache volumes (ccache, Gazebo cache, ROS logs)
	$(call confirm,Remove containers and named volumes for ROLE=$(ROLE)?)
	$(COMPOSE) down -v --remove-orphans

distclean: _role ## clean + remove the built images + ws build artefacts (full reset; FORCE=1 skips the prompt)
	$(call confirm,Remove containers and volumes and images and ws build output?)
	$(COMPOSE) down -v --remove-orphans --rmi all
	rm -rf $(WS_DIR)/build $(WS_DIR)/install $(WS_DIR)/log

# ---- internal helpers (not listed in `make help`) ----
_role:
	@case "$(ROLE)" in pc|jetson) ;; *) echo "ROLE must be 'pc' or 'jetson' (got '$(ROLE)'). Use: make <target> ROLE=pc" >&2; exit 1;; esac
	@if [ -n "$(GPU)" ]; then \
	  case "$(GPU)" in nvidia|dri) ;; *) echo "GPU must be 'nvidia' or 'dri' (got '$(GPU)')" >&2; exit 1;; esac; \
	  [ "$(ROLE)" = pc ] || { echo "GPU only applies to ROLE=pc" >&2; exit 1; }; \
	fi

# Runtime targets need an image that matches the host CPU: the jetson image is arm64-only.
_arch:
	@if [ "$(ROLE)" = jetson ] && [ "$(ARCH)" != arm64 ] && [ "$(FORCE)" != 1 ]; then \
	  echo "ROLE=jetson needs an arm64 host, this one is $(UNAME_M). Use ROLE=pc, build with 'make jetson-cross' (then 'make jetson-ship NANO=...'), or FORCE=1 to run anyway." >&2; exit 1; fi

_jetson: _arch
	@[ "$(ROLE)" = jetson ] || { echo "arm services run on the Jetson only (ROLE=$(ROLE)); use ROLE=jetson" >&2; exit 1; }

_xhost:
	@if [ "$(ROLE)" = pc ] && [ "$(XHOST)" = 1 ] && command -v xhost >/dev/null 2>&1; then \
	  echo "xhost +local:   (allows local containers to use your display; XHOST=0 to skip)"; xhost +local: >/dev/null; fi
