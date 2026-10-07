from pymycobot import MyCobot280
import time

# Initialize serial connection on the Jetson Nano
mc = MyCobot280('/dev/ttyTHS1', 1000000)
time.sleep(1)

# Power on and enable servo torque
mc.send_angles([0, 0, 0, 0, 0, 0], 40)
time.sleep(3)

mc.send_angle(3, 70, 40)
time.sleep(3)

mc.send_angle(4, -70, 40)
time.sleep(3)

mc.send_angle(1, 90, 40)
time.sleep(3)

mc.send_angle(5, -90, 40)
time.sleep(3)

mc.send_angle(5, 90, 40)
time.sleep(3)
