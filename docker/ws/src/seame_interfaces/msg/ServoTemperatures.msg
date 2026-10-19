# Servo temperatures of the myCobot 280, published by arm_driver.
# valid = false when the arm did not answer (celsius is then all zero).
std_msgs/Header header
int16[6] celsius   # J1..J6, degrees Celsius
bool valid
