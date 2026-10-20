#!/usr/bin/env bash
# arm_routine.sh - entrypoint of the arm container: the ROS 2 launch plus the braco web page.
#
#   container starts  -> ros2 launch seame_arm arm.launch.py (driver, temperature, teleop)
#                        the braco web page (uvicorn on WEB_PORT, ARM_SOURCE=ros), if /opt/braco is mounted
#   container stops   -> the web page, then the launch, are stopped
#
# The arm is NOT moved when the container starts or stops: zero and park are manual
# (make arm-zero / make arm-park).
#
# The web page is a plain client of arm_driver: if it crashes only a line is logged, the arm
# keeps running (and the driver's watchdog stops any move the page was streaming).
#
# "Stops" means docker stop / make arm-down / systemctl stop braco / a normal shutdown of
# the Jetson: Docker sends SIGTERM and the trap below stops the page and the launch.
# Arguments are passed on to the launch (port:=... lcd_bus:=...).
set -u

launch_pid=""
web_pid=""
WEB_DIR=${WEB_DIR:-/opt/braco}
WEB_PORT=${WEB_PORT:-8000}

on_stop() {
    trap - TERM INT
    echo "arm_routine: stopping"
    if [ -n "$web_pid" ]; then
        kill -TERM "$web_pid" 2>/dev/null
        wait "$web_pid" 2>/dev/null
    fi
    # SIGINT = Ctrl-C of ros2 launch
    kill -INT "$launch_pid" 2>/dev/null
    wait "$launch_pid" 2>/dev/null
    exit 0
}
trap on_stop TERM INT

echo "arm_routine: starting the arm nodes"
ros2 launch seame_arm arm.launch.py "$@" &
launch_pid=$!

# One process (--workers 1): presence and the control lease live in its memory
if [ -f "$WEB_DIR/web/server/app.py" ]; then
    echo "arm_routine: starting the braco web page on port $WEB_PORT"
    (cd "$WEB_DIR" && ARM_SOURCE=ros exec python3 -m uvicorn web.server.app:app \
        --host 0.0.0.0 --port "$WEB_PORT" --workers 1 --no-access-log) &
    web_pid=$!
else
    echo "arm_routine: $WEB_DIR/web not mounted - no web page"
fi

echo "arm_routine: ready"
# Exits if the launch dies (driver gone): the container stops
wait "$launch_pid"
echo "arm_routine: the launch ended"
exit 1
