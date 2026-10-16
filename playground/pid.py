
#Import dependent library files.
import time
from pymycobot import *

#Define data address and data.

#Define the serial port of the robotic arm，Please check your robot model and fill in the corresponding content.
# _port = '/dev/ttyAMA0'
_port = '/dev/ttyTHS1'
_baud = 1000000

#Instantiate a hardware serial port. 
try:
    mc = MyCobot280(_port, _baud)
except Exception as e:
    print(e)
    print("Error: The current serial port can not be used, please check whether the serial port serial number is correct, after confirming that there is no error, please re-run the program.")
    exit()


#Loop modification and display modification results.
#joint_id = 1 - 7.
mc.set_servo_data(4, 23, 5)
mc.set_servo_data(4, 22, 10)
mc.set_servo_data(4, 21, 37)
mc.set_servo_data(2, 23, 5)
mc.set_servo_data(2, 22, 10)
mc.set_servo_data(2, 21, 48)
exit()
