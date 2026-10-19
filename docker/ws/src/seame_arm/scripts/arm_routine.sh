#!/usr/bin/env bash
# arm_routine.sh - entrypoint of the arm container: the ROS 2 launch plus the boot/shutdown routine.
#
#   container starts  -> ros2 launch seame_arm arm.launch.py (driver, temperature, teleop)
#                        then zero.py (all joints to 0) once the driver answers
#   container stops   -> park.py (fold into the park pose, release the joints),
#                        then the launch is stopped
#
# "Stops" means docker stop / make arm-down / systemctl stop seame-arm / a normal
# shutdown of the Jetson: Docker sends SIGTERM and the trap below runs park.py while the
# driver is still up (it is stopped last). Nothing runs on a power cut or kill -9.
# The container only exists after make arm-up or make install-service, so neither
# script runs before that. Arguments are passed on to the launch (port:=... lcd_bus:=...).
set -u

launch_pid=""
zero_pid=""

on_stop() {
    trap - TERM INT
    echo "arm_routine: stopping - parking the arm"
    # A zero move still running is cancelled first (the driver stops the arm)
    if [ -n "$zero_pid" ] && kill -0 "$zero_pid" 2>/dev/null; then
        kill -INT "$zero_pid" 2>/dev/null
        wait "$zero_pid" 2>/dev/null
    fi
    ros2 run seame_arm park.py || echo "arm_routine: park failed"
    # Park is done: now the launch may take the nodes down (SIGINT = Ctrl-C of ros2 launch)
    kill -INT "$launch_pid" 2>/dev/null
    wait "$launch_pid" 2>/dev/null
    exit 0
}
trap on_stop TERM INT

echo "arm_routine: starting the arm nodes"
ros2 launch seame_arm arm.launch.py "$@" &
launch_pid=$!

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
