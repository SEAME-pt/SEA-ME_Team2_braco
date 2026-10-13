#!/usr/bin/env bash
# ros2_baseline_check.sh - read-only pre-flight for the myCobot 280 Jetson Nano
# ROS2 setup (see docs/ros2.md, Phase 0).
#
# SAFETY: this script only reads system information. It never opens the serial
# port, never powers servos and never sends motion commands. Safe to run anytime.
#
# Usage (run ON the Jetson):
#     bash initial_tests/ros2_baseline_check.sh
#
# Exit code: 0 when nothing FAILs, 1 otherwise (WARNs do not fail the run).

set -u

OK_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

section() { printf '\n== %s ==\n' "$1"; }
ok()      { OK_COUNT=$((OK_COUNT + 1));     printf '  [ OK ]  %s\n' "$1"; }
warn()    { WARN_COUNT=$((WARN_COUNT + 1)); printf '  [WARN]  %s\n' "$1"; }
fail()    { FAIL_COUNT=$((FAIL_COUNT + 1)); printf '  [FAIL]  %s\n' "$1"; }
info()    { printf '  [info]  %s\n' "$1"; }
have()    { command -v "$1" >/dev/null 2>&1; }

printf 'myCobot 280 JN - ROS2 baseline check\n'
printf 'host: %s   user: %s   date: %s\n' "$(hostname)" "${USER:-?}" "$(date)"
printf 'NOTE: read-only script. No motion commands are sent.\n'

# ---------------------------------------------------------------- OS / board
section "OS / board / resources"

if have lsb_release; then
    info "$(lsb_release -ds 2>/dev/null)"
    case "$(lsb_release -rs 2>/dev/null)" in
        20.04) ok "Ubuntu 20.04 (expected for ROS2 Foxy on this image)" ;;
        *)     warn "Ubuntu not 20.04 - ROS2 distro may differ from Foxy, adapt setup.bash paths" ;;
    esac
else
    warn "lsb_release not available"
fi

info "kernel: $(uname -r)   arch: $(uname -m)"

if [ -r /etc/nv_tegra_release ]; then
    info "JetPack: $(head -n1 /etc/nv_tegra_release)"
else
    warn "no /etc/nv_tegra_release - not a Jetson board or JetPack layout changed"
fi

if have free; then
    info "RAM: $(free -h | awk '/^Mem:/ {print $2 " total, " $7 " available"}')"
fi
info "disk /: $(df -h / | awk 'NR==2 {print $4 " free of " $2}')"

# ------------------------------------------------------------------- network
section "Network (needed for remote ROS2 / laptop offload)"
ip_list=$(hostname -I 2>/dev/null | xargs || true)
if [ -n "$ip_list" ]; then
    ok "IP address(es): $ip_list"
else
    warn "no IP address - WiFi/Ethernet down? (only matters for Phase 5 / remote work)"
fi

# ----------------------------------------------------------------------- ROS
section "ROS2 environment"

if [ -z "${ROS_DISTRO:-}" ]; then
    sourced=""
    for setup in /opt/ros/*/setup.bash; do
        if [ -f "$setup" ]; then
            set +u; # shellcheck disable=SC1090
            . "$setup"; set -u
            sourced="$setup"
            break
        fi
    done
    if [ -n "$sourced" ]; then
        warn "ROS env was not sourced; script sourced $sourced automatically"
    fi
fi

if [ -n "${ROS_DISTRO:-}" ]; then
    ok "ROS_DISTRO=$ROS_DISTRO"
else
    fail "ROS2 not set up: no environment in \$ROS_DISTRO and nothing sourceable under /opt/ros/"
fi

info "installed distros: $(ls /opt/ros/ 2>/dev/null | tr '\n' ' ')"
info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset, default 0>}   ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}   RMW=${RMW_IMPLEMENTATION:-<default>}"

if have ros2; then
    ok "ros2 CLI available"
else
    fail "ros2 CLI not found on PATH (source /opt/ros/<distro>/setup.bash)"
fi

# ------------------------------------------------------------------ workspace
section "Workspace ~/colcon_ws"

WS="$HOME/colcon_ws"
if [ -d "$WS/src" ]; then
    ok "workspace source tree exists: $WS/src"
    info "top-level entries: $(ls "$WS/src" 2>/dev/null | head -n 12 | tr '\n' ' ')"
else
    fail "no $WS/src - workspace missing (official image: user 'er', ~/colcon_ws)"
fi

if [ -f "$WS/install/setup.bash" ]; then
    set +u; # shellcheck disable=SC1091
    . "$WS/install/setup.bash"; set -u
    ok "workspace install/setup.bash sourced"
else
    warn "no $WS/install/setup.bash - run 'colcon build --symlink-install' inside $WS"
fi

if have ros2; then
    pkgs=$(ros2 pkg list 2>/dev/null | grep -i mycobot || true)
    if [ -n "$pkgs" ]; then
        ok "mycobot ROS2 packages visible:"
        printf '          %s\n' $pkgs
    else
        warn "no mycobot packages visible - workspace not built/sourced?"
    fi
fi

# ----------------------------------------------------------------- pymycobot
section "pymycobot library"

if have python3; then
    ver=$(python3 -c 'import pymycobot; print(pymycobot.__version__)' 2>/dev/null || true)
    if [ -n "$ver" ]; then
        ok "pymycobot $ver"
        info "bridge node (mycobot_communication) expects pymycobot <= 3.5.3 on the foxy branch"
        info "slider demo (mycobot_280jn) expects pymycobot >= 3.6.1 on the foxy branch"
        info "the two official demos disagree - use whichever matches your image"
    else
        warn "pymycobot not importable with python3 (pip install pymycobot in the ROS env if needed)"
    fi
else
    warn "python3 not found"
fi

# --------------------------------------------------------------------- serial
section "Arm serial link"

if [ -e /dev/ttyTHS1 ]; then
    ok "$(ls -l /dev/ttyTHS1 | awk '{print $1, $3, $4, $NF}')"
    if [ -r /dev/ttyTHS1 ] && [ -w /dev/ttyTHS1 ]; then
        ok "/dev/ttyTHS1 is readable/writable by $USER"
    else
        fail "/dev/ttyTHS1 not accessible by $USER - add user to 'dialout' and re-login"
    fi
else
    fail "/dev/ttyTHS1 missing - check the UART device tree / model variant"
fi

if id -nG "${USER:-}" 2>/dev/null | grep -qw dialout; then
    ok "user ${USER:-?} is in the 'dialout' group"
else
    warn "user ${USER:-?} is NOT in 'dialout' - serial access may fail"
fi

# -------------------------------------------------------------------- display
section "GUI / display"

if [ -n "${DISPLAY:-}" ]; then
    ok "DISPLAY=$DISPLAY (rviz2 / slider GUI can open)"
else
    warn "DISPLAY is unset - rviz2 and slider demos need the robot desktop or 'export DISPLAY=:0'"
fi

# ---------------------------------------------------------------------- doctor
section "ros2 doctor (optional, can take ~30 s)"

if have ros2; then
    doctor_out=$(timeout 45 ros2 doctor 2>&1 || true)
    if [ -n "$doctor_out" ]; then
        printf '%s\n' "$doctor_out" | tail -n 12 | sed 's/^/  /'
        if printf '%s\n' "$doctor_out" | grep -qi 'error\|fail'; then
            warn "ros2 doctor reported issues - read its output above"
        else
            ok "ros2 doctor ran"
        fi
    else
        warn "ros2 doctor produced no output (timed out or unsupported on this distro)"
    fi
else
    info "skipped (no ros2 CLI)"
fi

# --------------------------------------------------------------------- summary
section "Summary"
printf '  OK: %d   WARN: %d   FAIL: %d\n' "$OK_COUNT" "$WARN_COUNT" "$FAIL_COUNT"

if [ "$FAIL_COUNT" -gt 0 ]; then
    printf '  Result: FAIL - fix the [FAIL] items above (see docs/ros2.md section 4).\n'
    exit 1
fi

printf '  Result: PASS - ready for docs/ros2.md Phase 1 (middleware smoke test).\n'
exit 0
