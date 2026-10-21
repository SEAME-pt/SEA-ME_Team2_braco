# mk/arm.mk - the real arm, through the running `real` container (Jetson): zero park rest resume temps demos.
#
# These run inside the system container (ROS sourced, workspace mounted), so nothing has to be stopped and the
# serial port keeps its single owner, arm_driver.

.PHONY: zero park rest resume temps demos _system_up

##@ Arm (Jetson, with the system up)

zero: _jetson _system_up ## Leave rest mode and move the arm to the zero pose
	$(SYS_EXEC) ros2 run braco_bringup zero.py

park: _jetson _system_up ## Fold the arm, release the joints and enter rest mode (temperatures keep being read)
	$(SYS_EXEC) ros2 run braco_bringup park.py

rest: _jetson _system_up ## Rest mode on: motion refused, temperatures keep being published
	$(SYS_EXEC) ros2 service call /arm/set_read_only std_srvs/srv/SetBool "{data: true}"

resume: _jetson _system_up ## Rest mode off: motion accepted again (joints stay free until zero or power_on)
	$(SYS_EXEC) ros2 service call /arm/set_read_only std_srvs/srv/SetBool "{data: false}"

temps: _jetson _system_up ## Print the servo temperatures once (works in rest mode)
	$(SYS_EXEC) ros2 topic echo --once /arm/servo_temperatures

demos: _role _system_up ## Run the ros2_basics demos for the page's ROS 2 > Demos tab (Ctrl-C stops them)
	$(SYS_EXEC) bash -c 'ros2 run ros2_basics_py managed_talker & ros2 run ros2_basics_cpp managed_talker & \
	  trap "kill %1 %2 2>/dev/null" EXIT; ros2 launch ros2_basics_py basics.launch.py'

_system_up: _role
	@[ -n "$$($(COMPOSE) ps -q --status running $(SYSTEM) 2>/dev/null)" ] || { echo "the system is not running: make up" >&2; exit 1; }
