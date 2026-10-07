import time
from pymycobot import MyCobot280

mc = MyCobot280("/dev/ttyTHS1", 1000000)
time.sleep(1)

SPEED = 30

print("Before:", mc.get_angles())

for motor in range(1, 7):
    print(f"motor {motor} → 0°")
    mc.send_angle(motor, 0, SPEED)
    time.sleep(1)

print("After:", mc.get_angles())
