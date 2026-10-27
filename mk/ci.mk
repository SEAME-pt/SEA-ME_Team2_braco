# mk/ci.mk - what .github/workflows/ci.yml runs, reproducible here: ci ci-ros lint format format-check.
#
#   host job   make ci       format check, lint, web page tests
#   ros job    make ci-ros   rosdep, colcon build and colcon test of ros2_ws/src (make ci-image builds its image)
#              make cppcheck cppcheck on the C++ packages (same image)
#              make tidy     clang-tidy on the C++ packages, from the ci-ros build (same image)
#   coverage   make coverage the GoogleTests only, built with --coverage, then a gcovr report (same image)
#
# No robot and no display needed.

# The `ros` job builds the workspace in a throw-away container from the Dockerfile's `base` stage: everything the
# packages depend on, without the GUI/Gazebo/MoveIt layers of the PC image. The image's user gets your UID/GID so
# the bind-mounted ros2_ws stays writable (GitHub runners are UID 1001).
# ci.yml builds the same image with docker/build-push-action (layer cache): keep its target and build-args in step.
# CI caches ros2_ws/build, where test results live: delete the old ones first, or a removed test would still count.
# colcon test exits 0 even when tests fail; colcon test-result is what fails the job.
CI_TARGET ?= base
CI_IMAGE  ?= $(IMAGE_PREFIX)/ci:$(IMAGE_TAG)
CI_ROS_SH := set -e; rosdep update --rosdistro "$$ROS_DISTRO" >/dev/null 2>&1 || echo "rosdep update failed - using the rosdep cache from the image"; \
             sudo apt-get update -qq; rosdep install --from-paths $(WS_BASE_PATHS) --ignore-src -y; \
             $(COLCON); \
             colcon test-result --delete-yes >/dev/null 2>&1 || true; \
             colcon test --base-paths $(WS_BASE_PATHS); colcon test-result --verbose
CI_RUN = docker run --rm -e CCACHE_DIR=/ws/.ccache -e CCACHE_MAXSIZE=500M -v $(CURDIR)/$(WS_DIR):/ws $(CI_IMAGE)

# coverage: a Debug build with --coverage in its own tree, ros2_ws/coverage/ - not in build/, where it would replace
# ci-ros's cached build and colcon test-result would count its tests twice. --cmake-args replaces the cmake-args of
# colcon-defaults.yaml, so the ccache launchers are repeated. gcov adds to the .gcda counters of the last run: they
# are deleted first. Only the gtest label runs (no linters); a failing test stops before the report. The table is cut
# before gcovr's Missing column (every uncovered line number): the HTML report has them.
COVERAGE_PKGS := mycobot_cpp braco_driver braco_teleop braco_bringup
COV_DIRS      := --build-base coverage/build --install-base coverage/install
COV_FLAGS     := -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage \
                 -DCMAKE_EXE_LINKER_FLAGS=--coverage -DCMAKE_SHARED_LINKER_FLAGS=--coverage \
                 -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
CI_COV_SH := set -eo pipefail; sudo apt-get update -qq; sudo apt-get install -y -qq gcovr >/dev/null; \
             rosdep install --from-paths $(WS_BASE_PATHS) --ignore-src -y >/dev/null; \
             colcon --log-base coverage/log build --base-paths $(WS_BASE_PATHS) --packages-up-to $(COVERAGE_PKGS) \
                 $(COV_DIRS) --cmake-args $(COV_FLAGS); \
             find coverage/build -name "*.gcda" -delete; \
             colcon test-result --test-result-base coverage/build --delete-yes >/dev/null 2>&1 || true; \
             colcon --log-base coverage/log test --base-paths $(WS_BASE_PATHS) --packages-select $(COVERAGE_PKGS) \
                 $(COV_DIRS) --ctest-args -L gtest; \
             colcon test-result --test-result-base coverage/build --verbose; \
             mkdir -p coverage/html; \
             gcovr -r /ws/src coverage/build --exclude ".*/test/.*" --txt --print-summary \
                 --html-details coverage/html/index.html | cut -c1-64

# cppcheck: run directly, because ament_cppcheck skips every file ("performance issues" with cppcheck >= 2.0, it
# feeds it the ROS headers too). Here only the packages' own include dirs are given, so it takes a second. All of
# $(WS_SRC) is checked, mycobot_cpp included (it has no ament linters). Findings fail the target; a false positive
# gets an inline `// cppcheck-suppress <id>` with the reason. Off, as matters of taste rather than defects:
# useStlAlgorithm (a std::transform in place of a two-line loop) and useInitializationList (the nodes declare their
# parameters, then create publishers, clients and the serial port in the constructor body, as rclcpp code does; the
# check flags whichever assignments come first, so moving them only exposes the next ones).
CPPCHECK_INCLUDES := $(patsubst $(WS_DIR)/%,-I %,$(wildcard $(WS_SRC)/*/include $(WS_SRC)/*/includes))
CPPCHECK_FLAGS    := --enable=warning,style,performance,portability --std=c++17 --language=c++ --inline-suppr \
                     --error-exitcode=1 --quiet --suppress=missingIncludeSystem --suppress=useStlAlgorithm \
                     --suppress=useInitializationList
CI_CPPCHECK_SH    := cppcheck $(CPPCHECK_FLAGS) -j"$$(nproc)" $(CPPCHECK_INCLUDES) $(WS_BASE_PATHS) && echo "cppcheck: clean"

# tidy: clang-tidy (checks in ros2_ws/.clang-tidy) on the sources of ros2_ws/src, from the compile_commands.json that
# ci-ros's build leaves in build/<pkg> (colcon-defaults.yaml exports it): run make ci-ros first. It is installed at run
# time, like gcovr, so the image does not carry it. Generated code (braco_interfaces) lives in build/, not src/, and is
# skipped. A package's failure does not stop the others; the exit status is the worst.
CI_TIDY_SH := ls build/*/compile_commands.json >/dev/null 2>&1 || { echo "no build in $(WS_DIR)/build: run make ci-ros first" >&2; exit 1; }; \
              sudo apt-get update -qq; sudo apt-get install -y -qq clang-tidy >/dev/null; \
              rc=0; for db in build/*/compile_commands.json; do \
                run-clang-tidy -quiet -p "$$(dirname "$$db")" -j "$$(nproc)" "^/ws/src/" 2>/dev/null || rc=1; \
              done; exit $$rc

BUILDX_MISSING := docker buildx not found: the Dockerfile needs BuildKit (apt install docker-buildx, or docker-buildx-plugin from Docker's repo)

# shellcheck and hadolint: a local binary if installed, else the pinned image through docker, so no check is ever
# skipped (lint needs docker anyway, for the compose files).
SHELLCHECK_IMAGE := koalaman/shellcheck:v0.10.0
# SC1091: scripts source /etc/os-release and ROS setup files that exist only in the image (and not at all in the
# scratch-based shellcheck image), so following them would make the result depend on where lint runs.
SHELLCHECK_FLAGS := -x -e SC1091
# Every shell script git tracks, so a new one is checked without editing this line; the hooks have no extension.
SHELLCHECK_FILES = $(shell git ls-files '*.sh') $(addprefix .githooks/,commit-msg pre-push)
HADOLINT_IMAGE   := hadolint/hadolint:v2.12.0
HADOLINT_FLAGS   := $(addprefix --ignore ,DL3008 DL3013 DL3059 DL3006 DL3042 DL3045 SC1091 SC2086 SC2016)

# format / format-check: clang-format (C++) and ruff (Python, .ruff.toml) on the files git tracks. The ROS 2
# packages in $(WS_SRC) are left to ament_uncrustify and ament_flake8 (make test), except $(CPP_PKG), which has no
# ament linters. format-check is the pre-push hook.
CPP_GLOBS = '*.cpp' '*.hpp' '*.h' '*.cc'
CPP_FILES = $(shell git ls-files -- $(CPP_GLOBS) ':!$(WS_SRC)') $(shell git ls-files -- $(addprefix $(CPP_PKG)/,$(CPP_GLOBS)))
PY_FILES  = $(shell git ls-files '*.py')
# Use the venv's pinned ruff and clang-format (make setup) even when the venv is not activated.
format format-check: export PATH := $(CURDIR)/$(VENV)/bin:$(PATH)

.PHONY: ci ci-image ci-ros coverage cppcheck tidy lint format format-check

##@ CI (.github/workflows/ci.yml)

ci: format-check lint $(VENV)/.web-test-installed ## The `host` CI job: format check, lint, web page tests
	$(VENV)/bin/python -m pytest web/tests -q

ci-image: ## Build the `ros` CI job's image (Dockerfile stage CI_TARGET=base) as CI_IMAGE, for your UID/GID
	@docker buildx version >/dev/null 2>&1 || { echo "$(BUILDX_MISSING)" >&2; exit 1; }
	docker buildx build --load --target $(CI_TARGET) --build-arg USER_UID=$$(id -u) --build-arg USER_GID=$$(id -g) -t $(CI_IMAGE) $(DOCKER_DIR)

ci-ros: ## The `ros` CI job: rosdep + colcon build + colcon test in a throw-away container (CI_IMAGE built if missing)
	@docker image inspect $(CI_IMAGE) >/dev/null 2>&1 || $(MAKE) --no-print-directory ci-image
	$(CI_RUN) bash -c '$(CI_ROS_SH)'

coverage: ## GoogleTests of ros2_ws with coverage (gcovr), in the ci-ros container; report in ros2_ws/coverage/html
	@docker image inspect $(CI_IMAGE) >/dev/null 2>&1 || $(MAKE) --no-print-directory ci-image
	$(CI_RUN) bash -c '$(CI_COV_SH)'

cppcheck: ## cppcheck on every C++ package of ros2_ws/src (ament_cppcheck skips them all), in the ci-ros image
	@docker image inspect $(CI_IMAGE) >/dev/null 2>&1 || $(MAKE) --no-print-directory ci-image
	$(CI_RUN) bash -c '$(CI_CPPCHECK_SH)'

tidy: ## clang-tidy on the C++ packages of ros2_ws/src (ros2_ws/.clang-tidy), from the build of make ci-ros
	@docker image inspect $(CI_IMAGE) >/dev/null 2>&1 || $(MAKE) --no-print-directory ci-image
	$(CI_RUN) bash -c '$(CI_TIDY_SH)'

lint: $(ENV_FILE) $(VENV)/.installed ## Compose files, ruff check, shellcheck and hadolint (binaries or their docker images)
	@echo "== compose =="
	@for r in pc jetson; do COMPOSE_PROFILES=$$r $(DC) config -q && echo "  $(COMPOSE_BASE) ($$r): ok"; done
	@COMPOSE_PROFILES=jetson I2C_GID= INPUT_GID= $(DC) config -q && echo "  $(COMPOSE_BASE) (jetson): ok without i2c/input groups"
	@for g in nvidia dri; do COMPOSE_PROFILES=pc docker compose --project-directory $(DOCKER_DIR) -f $(COMPOSE_BASE) -f $(COMPOSE_DIR)/$$g.yml config -q && echo "  $(COMPOSE_DIR)/$$g.yml: ok"; done
	@echo "== Python =="
	@$(VENV)/bin/ruff check -q $(PY_FILES) && echo "  ruff check: clean"
	@echo "== shell scripts =="
	@if command -v shellcheck >/dev/null 2>&1; then shellcheck $(SHELLCHECK_FLAGS) $(SHELLCHECK_FILES) && echo "  shellcheck: clean"; \
	else \
	  { docker image inspect $(SHELLCHECK_IMAGE) >/dev/null 2>&1 || docker pull -q $(SHELLCHECK_IMAGE) >/dev/null; } && \
	  docker run --rm -v "$(CURDIR)":/mnt:ro -w /mnt $(SHELLCHECK_IMAGE) $(SHELLCHECK_FLAGS) $(SHELLCHECK_FILES) && \
	  echo "  shellcheck ($(SHELLCHECK_IMAGE)): clean"; fi
	@echo "== Dockerfile =="
	@if command -v hadolint >/dev/null 2>&1; then hadolint $(HADOLINT_FLAGS) $(DOCKERFILE) && echo "  hadolint: clean"; \
	else \
	  { docker image inspect $(HADOLINT_IMAGE) >/dev/null 2>&1 || docker pull -q $(HADOLINT_IMAGE) >/dev/null; } && \
	  docker run --rm -i $(HADOLINT_IMAGE) hadolint $(HADOLINT_FLAGS) - < $(DOCKERFILE) && echo "  hadolint ($(HADOLINT_IMAGE)): clean"; fi

format: $(VENV)/.installed ## Fix the formatting of all C++ (clang-format) and Python (ruff) files
	@clang-format -i $(CPP_FILES)
	@ruff format -q $(PY_FILES)
	@printf '\033[0;32mOK:\033[0m all files formatted\n'

format-check: $(VENV)/.installed ## Check the formatting without changing files (used by the pre-push hook)
	@clang-format --dry-run --Werror $(CPP_FILES)
	@ruff format --check -q $(PY_FILES)
	@printf '\033[0;32mOK:\033[0m formatting is correct\n'

# The dev tools on the host (requirements-dev.txt): the formatters, and the container's arm libraries for the editor.
$(VENV)/.installed: $(VENV)/bin/python requirements-dev.txt
	$(PIP) install -r requirements-dev.txt
	touch $@
