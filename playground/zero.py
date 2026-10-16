import time
from pymycobot import MyCobot280

mc = MyCobot280("/dev/ttyTHS1", 1000000)
time.sleep(1)

SPEED = 30

target_angles = [0, 0, 0, 0, 0, 0]
mc.send_angles(target_angles, SPEED)

time.sleep(3)

