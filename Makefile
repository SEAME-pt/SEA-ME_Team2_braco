# braco - SEA:ME 2.0 arm class: the one Makefile of the repo. Run it from the repo root.
#
#   make up        turn the system on: the real arm on the Jetson, the simulation on the PC
#   make down      turn it off (PARK=1 parks the arm first)
#   make           list every command
#
# Each machine runs two containers (docker/compose.yml): `dev` for the tools (shell, build, test) and the system
# itself, `real` on the Jetson or `sim` on the PC. The machine (ROLE=pc|jetson) comes from COMPOSE_PROFILES in
# docker/.env, or is detected (arm64 + Tegra = jetson); ROLE=... on the command line overrides it.
#
# The commands live in mk/, one file per topic:
#   mk/config.mk   paths, machine detection, compose command
#   mk/system.mk   up down restart status logs
#   mk/arm.mk      zero park rest resume temps                  (Jetson)
#   mk/dev.mk      shell run build test web clean
#   mk/setup.mk    setup doctor image boot-on boot-off jetson-image reset
#   mk/net.mk      net-test net-talk net-listen
#   mk/ci.mk       ci ci-image ci-ros lint format format-check

.DEFAULT_GOAL := help

include mk/config.mk
include mk/system.mk
include mk/arm.mk
include mk/dev.mk
include mk/setup.mk
include mk/net.mk
include mk/ci.mk

.PHONY: help
help: ## Show this help
	@printf 'braco - this machine: ROLE=%s (system: %s, image %s)\n' "$(ROLE)" "$(SYSTEM)" "$(IMAGE)"
	@printf 'usage: make <command> [ROLE=pc|jetson] [GPU=nvidia|dri] [CMD=...] [S=...] [PARK=1] [FORCE=1]\n'
	@awk 'BEGIN {FS = ":.*## "} /^[a-zA-Z0-9_-]+:.*## / {printf "  \033[36m%-13s\033[0m %s\n", $$1, $$2} /^##@/ {printf "\n\033[1m%s\033[0m\n", substr($$0, 5)}' $(MAKEFILE_LIST)
