# mk/setup.mk - preparing a machine: setup doctor image boot-on boot-off jetson-image reset.

IMAGE_ARGS ?=
NANO       ?=
JETSON_TAG := $(IMAGE_TAG)-arm64

.PHONY: setup doctor image boot-on boot-off jetson-image reset

##@ Setup (once per machine)

setup: _role $(ENV_FILE) | $(WS_DIRS) ## docker/.env for this machine, git hooks, folders, then doctor
	git config core.hooksPath .githooks
	@$(DOCTOR_RUN)

doctor: _role ## Check this machine: docker, compose, project files, .env, folders, devices, Jetson RAM/power/swap
	@$(DOCTOR_RUN)

image: _role $(ENV_FILE) ## (Re)build this machine's image (IMAGE_ARGS='--no-cache --pull' for a clean one; slow)
	$(COMPOSE) build $(IMAGE_ARGS) $(DEV)

boot-on: _jetson ## Start the system at boot (systemd unit braco; needs sudo and a built workspace)
	@test -f $(BUILD_STAMP) || { echo "the workspace is not built yet: make up" >&2; exit 1; }
	sed -e "s|@WORKDIR@|$(CURDIR)/$(DOCKER_DIR)|g" -e "s|@DOCKER@|$$(command -v docker)|g" $(SYSTEMD_UNIT) | sudo tee $(SYSTEMD_DEST) >/dev/null
	sudo systemctl daemon-reload
	sudo systemctl enable --now braco.service
	@echo "installed $(SYSTEMD_DEST): the system now starts at boot (make status)"

boot-off: ## Do not start the system at boot any more (removes the unit; the system keeps running until make down)
	-sudo systemctl disable braco.service
	sudo rm -f $(SYSTEMD_DEST)
	sudo systemctl daemon-reload

jetson-image: _role ## On a PC: build the Jetson image (arm64 via QEMU, slow); NANO=user@host also copies it there
	@[ "$(ARCH)" != arm64 ] || { echo "this host is arm64: build natively with make image ROLE=jetson" >&2; exit 1; }
	@[ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ] || docker run --privileged --rm tonistiigi/binfmt --install arm64
	COMPOSE_PROFILES=jetson IMAGE_TAG=$(JETSON_TAG) DOCKER_DEFAULT_PLATFORM=linux/arm64 \
	  $(DC) build $(IMAGE_ARGS) $(DEV)
	@if [ -n "$(NANO)" ]; then \
	  echo "copying $(IMAGE_PREFIX)/jetson:$(JETSON_TAG) to $(NANO) as $(IMAGE_PREFIX)/jetson:$(IMAGE_TAG)"; \
	  docker save $(IMAGE_PREFIX)/jetson:$(JETSON_TAG) | gzip | \
	    ssh $(NANO) 'gunzip | docker load && docker tag $(IMAGE_PREFIX)/jetson:$(JETSON_TAG) $(IMAGE_PREFIX)/jetson:$(IMAGE_TAG)' && \
	  echo "done. On the Nano: make up"; \
	else echo "built $(IMAGE_PREFIX)/jetson:$(JETSON_TAG). Copy it with: make jetson-image NANO=user@host"; fi

reset: _role ## Full reset: clean + remove the containers, cache volumes and images (FORCE=1 skips the prompt)
	$(call confirm,Remove the containers, volumes, images and build output of ROLE=$(ROLE)?)
	$(COMPOSE) down -v --remove-orphans --rmi all
	rm -rf $(WS_DIR)/build $(WS_DIR)/install $(WS_DIR)/log $(HOME)/.ros/log

# ---- docker/.env, created once from .env.example with this machine's ids -----------------------
$(ENV_FILE): $(ENV_EXAMPLE)
	@if [ "$$(id -u)" = 0 ]; then echo "Run make as your normal user, not root/sudo (UID 0 would break the image's user)." >&2; exit 1; fi
	@if [ -f $@ ]; then echo "$(ENV_EXAMPLE) is newer than $@: compare them (touch $@ to silence this)"; touch $@; else \
	  sed -e "s/^USER_UID=.*/USER_UID=$$(id -u)/" -e "s/^USER_GID=.*/USER_GID=$$(id -g)/" \
	      -e "s/^COMPOSE_PROFILES=.*/COMPOSE_PROFILES=$(ROLE)/" $(ENV_EXAMPLE) > $@.tmp && \
	  for pair in DIALOUT_GID:dialout VIDEO_GID:video RENDER_GID:render I2C_GID:i2c INPUT_GID:input; do \
	    var=$${pair%%:*}; grp=$${pair##*:}; gid=$$(getent group $$grp 2>/dev/null | cut -d: -f3); \
	    if [ -n "$$gid" ]; then sed "s/^$$var=.*/$$var=$$gid/" $@.tmp > $@.tmp2 && mv $@.tmp2 $@.tmp; \
	    elif [ "$(ROLE)" = jetson ] && { [ $$grp = i2c ] || [ $$grp = input ]; }; then \
	      echo "warning: no '$$grp' group on this host - $$var left empty, the real container cannot use the device (see make doctor)" >&2; fi; \
	  done; mv $@.tmp $@; \
	  echo "created $@ for ROLE=$(ROLE) (uid $$(id -u), gid $$(id -g))"; fi

# ---- make doctor (QUIET=1 prints only the problems: what `make up` uses) ------------------------
DOCTOR_RUN = ROLE=$(ROLE) ARCH=$(ARCH) UNAME_M=$(UNAME_M) IS_JETSON=$(IS_JETSON) MYCOBOT_PORT=$(MYCOBOT_PORT) \
             ENV_FILE=$(ENV_FILE) WS_DIRS="$(WS_DIRS)" PROJECT_FILES="$(PROJECT_FILES)" sh -c "$$DOCTOR_SH"

define DOCTOR_SH
fail=0
ok()   { [ -n "$$QUIET" ] || printf '  [ ok ] %s\n' "$$*"; }
warn() { printf '  [warn] %s\n' "$$*"; }
bad()  { printf '  [FAIL] %s\n' "$$*"; fail=1; }
[ -n "$$QUIET" ] || echo "Checks for ROLE=$$ROLE"
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
if docker buildx version >/dev/null 2>&1; then ok "docker buildx $$(docker buildx version | cut -d' ' -f2) (BuildKit)"; else bad "docker buildx not found: the Dockerfile needs BuildKit (apt install docker-buildx, or docker-buildx-plugin from Docker's repo)"; fi
command -v curl >/dev/null 2>&1 && ok "curl available" || warn "curl not installed: make up cannot wait for the page (apt install curl)"
missing=""
for f in $$PROJECT_FILES; do [ -e "$$f" ] || missing="$$missing $$f"; done
if [ -z "$$missing" ]; then ok "all project files present"; else bad "missing project files:$$missing"; fi
if [ -f "$$ENV_FILE" ]; then ok "$$ENV_FILE present"; else warn "$$ENV_FILE missing - make setup (or make up) creates it"; fi
uid=$$(sed -n 's/^USER_UID=//p' "$$ENV_FILE" 2>/dev/null | tail -n 1)
if [ -n "$$uid" ] && [ "$$uid" != "$$(id -u)" ]; then warn "USER_UID=$$uid in $$ENV_FILE but your uid is $$(id -u): the workspace may not be writable from the container"; fi
for d in $$WS_DIRS; do
  if [ -d "$$d" ]; then
    if [ -w "$$d" ]; then ok "$$d present and writable"; else bad "$$d is not writable by you (created by Docker as root? fix: sudo chown -R $$(id -u):$$(id -g) $$d)"; fi
  else ok "$$d missing - make up creates it"; fi
done
case "$$ROLE" in
  pc)
    if [ -n "$$IS_JETSON" ]; then warn "this host looks like a Jetson but ROLE=pc: did you mean ROLE=jetson?"; fi
    if [ -n "$$DISPLAY" ] && [ -d /tmp/.X11-unix ]; then ok "DISPLAY=$$DISPLAY and X11 socket directory present"; else ok "no DISPLAY: RViz2 and Gazebo windows will not open (the page still works)"; fi
    ;;
  jetson)
    if [ "$$ARCH" = arm64 ]; then ok "arm64 host"; else bad "host is $$UNAME_M, not arm64: the real arm runs on the Jetson (build its image here with make jetson-image)"; fi
    if [ -e "$$MYCOBOT_PORT" ]; then ok "serial device $$MYCOBOT_PORT present"; else bad "serial device $$MYCOBOT_PORT not found (the real container cannot start without it)"; fi
    if [ "$$MYCOBOT_PORT" = /dev/ttyTHS1 ] && command -v systemctl >/dev/null 2>&1 && systemctl is-active --quiet nvgetty 2>/dev/null; then
      warn "nvgetty is active and holds /dev/ttyTHS1: sudo systemctl disable --now nvgetty"
    fi
    lcd=$$(sed -n 's/^LCD_I2C_BUS=//p' "$$ENV_FILE" 2>/dev/null | tail -n 1)
    if [ -e "$${lcd:-/dev/i2c-0}" ]; then ok "$${lcd:-/dev/i2c-0} present (LCD of temperature_monitor)"; else bad "$${lcd:-/dev/i2c-0} not found: the real container cannot start (set LCD_I2C_BUS in .env)"; fi
    if [ -e /dev/input/js0 ]; then ok "joystick /dev/input/js0 present"; else ok "no joystick at /dev/input/js0 (joystick_teleop waits for it)"; fi
    cam=$$(sed -n 's/^CAMERA_DEVICE=//p' "$$ENV_FILE" 2>/dev/null | tail -n 1)
    if [ -e "/dev/$${cam:-video0}" ]; then ok "camera /dev/$${cam:-video0} present"; else ok "no camera at /dev/$${cam:-video0} (the camera node waits for it)"; fi
    # The real container reaches the LCD and joystick through these groups; unset, compose gives it a placeholder.
    for pair in I2C_GID:i2c INPUT_GID:input; do
      var=$${pair%%:*}; grp=$${pair##*:}
      gid=$$(getent group "$$grp" 2>/dev/null | cut -d: -f3)
      envgid=$$(sed -n "s/^$$var=//p" "$$ENV_FILE" 2>/dev/null | tail -n 1)
      if [ -z "$$gid" ]; then warn "no '$$grp' group on this host: the real container gets a placeholder group and cannot open the device (create the group and a udev rule, then set $$var in $$ENV_FILE)"
      elif [ "$$envgid" != "$$gid" ]; then warn "$$var=$${envgid:-<empty>} in $$ENV_FILE but the host's '$$grp' gid is $$gid: set $$var=$$gid (make setup never overwrites .env)"
      else ok "group $$grp ($$gid) matches $$var in $$ENV_FILE"; fi
    done
    # The Nano has 4 GB: a desktop takes ~1 GB of it, 5 W mode halves the CPU, and colcon needs swap to finish.
    if systemctl is-active --quiet display-manager 2>/dev/null || pgrep -x Xorg >/dev/null 2>&1; then
      warn "a desktop session is running (~1 GB of RAM): sudo systemctl set-default multi-user.target && sudo reboot"
    else ok "no desktop session"; fi
    mode=$$(nvpmodel -q 2>/dev/null | sed -n 's/^NV Power Mode: *//p' | head -n 1)
    if [ -z "$$mode" ]; then warn "could not read the power mode (nvpmodel -q)"
    elif [ "$$mode" = MAXN ]; then ok "power mode MAXN (10 W)"
    else warn "power mode is $$mode, not MAXN (10 W): sudo nvpmodel -m 0 (needs the barrel-jack supply)"; fi
    if [ "$$(awk 'NR > 1' /proc/swaps 2>/dev/null | wc -l)" -gt 0 ]; then ok "swap: $$(awk 'NR > 1 {s += $$3} END {printf "%d MB", s / 1024}' /proc/swaps)"
    else warn "no swap: colcon builds can run out of memory (sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile)"; fi
    ;;
esac
exit $$fail
endef
export DOCTOR_SH
