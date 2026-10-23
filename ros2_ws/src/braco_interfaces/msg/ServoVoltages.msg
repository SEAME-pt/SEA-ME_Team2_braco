# Servo supply voltages of the myCobot 280, published by arm_driver.
# valid = false when the arm did not answer (volts is then all zero).
std_msgs/Header header
float32[6] volts   # J1..J6, volts
bool valid
