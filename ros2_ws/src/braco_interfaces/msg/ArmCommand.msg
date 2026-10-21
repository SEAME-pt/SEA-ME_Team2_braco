# Command for arm_driver on /arm/command. The newest command replaces any
# that has not been sent yet ("latest wins").
#
# Only the control owner moves the arm (see srv/TakeControl): motion from any
# other source is ignored. STOP is always obeyed, whoever sends it.
uint8 STOP=0         # stop all joints now; drops pending commands
uint8 JOINTS=1       # send each entry of targets (one serial command per joint)
uint8 ALL_ANGLES=2   # send angles_deg to all six joints at speed

uint8 type
JointTarget[] targets
float64[6] angles_deg
uint8 speed
string source        # who sends it: "web", "joystick", "script" or "mission"
