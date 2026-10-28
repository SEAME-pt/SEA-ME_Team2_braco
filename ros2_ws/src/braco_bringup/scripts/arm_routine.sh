#!/usr/bin/env bash
# arm_routine.sh - entrypoint of the system containers (`real` on the Jetson, `sim` on the PC):
# the ROS 2 launch, nothing else.
#
#   container starts  -> ros2 launch braco_bringup braco.launch.py sim:=$SIM
#                          SIM=false (real): driver, temperature, teleop, camera
#                          SIM=true  (sim):  Gazebo + controllers
#   container stops   -> the launch is stopped
#
# The braco web page is not started here: `make web` adds it to the running container (web_routine.sh).
#
# The arm is NOT moved when the container starts or stops: zero and park are manual
# (make zero / make park, or make down PARK=1).
#
# "Stops" means docker stop / make down / systemctl stop braco / a normal shutdown of
# the Jetson: Docker sends SIGTERM and the trap below stops the launch.
# Arguments are passed on to the launch (port:=... lcd_bus:=...).
set -u

launch_pid=""
SIM=${SIM:-false}

# Reached through the trap below, which shellcheck does not follow (SC2317 before 0.11, SC2329 from 0.11).
# shellcheck disable=SC2317,SC2329
on_stop() {
    trap - TERM INT
    echo "arm_routine: stopping"
    # SIGINT = Ctrl-C of ros2 launch
    kill -INT "$launch_pid" 2>/dev/null
    wait "$launch_pid" 2>/dev/null
    exit 0
}
trap on_stop TERM INT

echo "arm_routine: starting the arm nodes (sim:=$SIM)"
ros2 launch braco_bringup braco.launch.py sim:="$SIM" "$@" &
launch_pid=$!

echo "arm_routine: ready"
# Exits if the launch dies (driver or Gazebo gone): the container stops
wait "$launch_pid"
echo "arm_routine: the launch ended"
exit 1
