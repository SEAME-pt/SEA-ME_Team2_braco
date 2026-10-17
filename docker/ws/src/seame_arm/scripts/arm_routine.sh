#!/usr/bin/env bash
# arm_routine.sh - boot and shutdown routine of the arm (the arm-routine container).
#
#   container starts  -> zero.py  (all joints to 0)
#   container stops   -> park.py  (fold into the park pose, release the joints)
#
# "Stops" means docker stop / make arm-down / systemctl stop seame-arm / a normal
# shutdown of the Jetson: Docker sends SIGTERM and the trap below runs park.py.
# Nothing runs on a power cut or kill -9. The container only exists after
# make arm-up or make install-service, so neither script runs before that.
set -u

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
    exit 0
}
trap on_stop TERM INT

echo "arm_routine: starting - moving the arm to zero"
# In the background, so a stop request is handled at once (bash defers traps
# while a foreground command runs)
ros2 run seame_arm zero.py &
zero_pid=$!
wait "$zero_pid" || echo "arm_routine: zero failed"
zero_pid=""

echo "arm_routine: ready - park runs when the container stops"
sleep infinity &
wait $!
