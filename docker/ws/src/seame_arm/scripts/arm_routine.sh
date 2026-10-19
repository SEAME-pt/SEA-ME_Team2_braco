#!/usr/bin/env bash
# arm_routine.sh - entrypoint of the arm container: the ROS 2 launch plus the boot/shutdown routine.
#
#   container starts  -> ros2 launch seame_arm arm.launch.py (driver, temperature, teleop)
#                        the braco web page (uvicorn on WEB_PORT, ARM_SOURCE=ros), if /opt/braco is mounted
#                        then zero.py (all joints to 0) once the driver answers
#   container stops   -> park.py (fold into the park pose, release the joints),
#                        then the web page, then the launch is stopped
#
# The web page is a plain client of arm_driver: if it crashes only a line is logged, the arm
# keeps running (and the driver's watchdog stops any move the page was streaming).
#
# "Stops" means docker stop / make arm-down / systemctl stop seame-arm / a normal
# shutdown of the Jetson: Docker sends SIGTERM and the trap below runs park.py while the
# driver is still up (it is stopped last). Nothing runs on a power cut or kill -9.
# The container only exists after make arm-up or make install-service, so neither
# script runs before that. Arguments are passed on to the launch (port:=... lcd_bus:=...).
set -u

launch_pid=""
zero_pid=""
web_pid=""
WEB_DIR=${WEB_DIR:-/opt/braco}
WEB_PORT=${WEB_PORT:-8000}

on_stop() {
    trap - TERM INT
    echo "arm_routine: stopping - parking the arm"
    # A zero move still running is cancelled first (the driver stops the arm)
    if [ -n "$zero_pid" ] && kill -0 "$zero_pid" 2>/dev/null; then
        kill -INT "$zero_pid" 2>/dev/null
        wait "$zero_pid" 2>/dev/null
    fi
    ros2 run seame_arm park.py || echo "arm_routine: park failed"
    # The page stayed up during park so everybody saw it; now it can go
    if [ -n "$web_pid" ]; then
        kill -TERM "$web_pid" 2>/dev/null
        wait "$web_pid" 2>/dev/null
    fi
    # Park is done: now the launch may take the nodes down (SIGINT = Ctrl-C of ros2 launch)
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

# In the background, so a stop request is handled at once (bash defers traps
# while a foreground command runs). zero.py waits for the driver by itself.
echo "arm_routine: moving the arm to zero"
ros2 run seame_arm zero.py &
zero_pid=$!
wait "$zero_pid" || echo "arm_routine: zero failed"
zero_pid=""

echo "arm_routine: ready - park runs when the container stops"
# Exits if the launch dies (driver gone): the container stops and restarts by policy
wait "$launch_pid"
echo "arm_routine: the launch ended"
exit 1
