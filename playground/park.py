import time
from pymycobot import MyCobot280

# WARNING: opens the serial port directly. With the arm services up (the arm_driver node owns
# /dev/ttyTHS1) this corrupts both sides. Use `make park` (in docker/) instead.

mc = MyCobot280("/dev/ttyTHS1", 1000000)
time.sleep(1)

SPEED = 30

target_angles = [0, 131, -150, 146, 4.5, 0]
mc.send_angles(target_angles, SPEED)
time.sleep(3)

