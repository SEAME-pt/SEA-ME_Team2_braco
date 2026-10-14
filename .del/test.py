from pymycobot import MyCobot280

mc = MyCobot280('/dev/ttyTHS1', 1000000)

# Gets the current angle of all joints
angles = mc.release_all_servos()
mc.focus_servo(2)
print(angles)