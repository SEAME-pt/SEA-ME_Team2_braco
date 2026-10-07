from pymycobot import MyCobot280
import time

# Initialize serial connection on the Jetson Nano
mc = MyCobot280('/dev/ttyTHS1', 1000000)
time.sleep(1)

# Power on and enable servo torque
mc.power_on()
time.sleep(0.5)

print("Current position:", mc.get_angles())

print("Moving to zero position...")
mc.send_angles([0, 0, 0, 0, 0, 0], 25)
time.sleep(4)

print("Moving Joint 3 to 70°...")
mc.send_angle(3, 70, 25)
time.sleep(3)

print("Moving Joint 4 to -70°...")
mc.send_angle(4, -70, 25)
time.sleep(3)

print("Moving Joint 1 to 90°...")
mc.send_angle(1, 90, 25)
time.sleep(3)

print("Moving Joint 5 to -90°...")
mc.send_angle(5, -90, 25)
time.sleep(3)

print("Moving Joint 5 to 90°...")
mc.send_angle(5, 90, 25)
time.sleep(3)

print("Sequence finished! Final position:", mc.get_angles())
