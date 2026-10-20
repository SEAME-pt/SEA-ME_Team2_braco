# shellcheck shell=bash
# Sourced (never executed) by the entrypoint and by interactive shells.
# Layers, in order: ROS distro -> your workspace overlay (if it has been built).

_seame_source() {
    if [ -f "$1" ]; then
        # shellcheck disable=SC1090
        . "$1"
    fi
    return 0
}

_seame_source "/opt/ros/${ROS_DISTRO}/setup.bash"
_seame_source "${SEAME_WS:-/ws}/install/setup.bash"

unset -f _seame_source
