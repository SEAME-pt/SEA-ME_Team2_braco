import time
from pymycobot import MyCobot280

mc = MyCobot280("/dev/ttyTHS1", 1000000)
time.sleep(1)
for j in range(1, 7):
    print("J%d: min %s  max %s" % (j, mc.get_joint_min_angle(j), mc.get_joint_max_angle(j)))
    time.sleep(0.1)
