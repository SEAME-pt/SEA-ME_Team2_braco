# One joint goal for arm_driver.
uint8 joint        # 1..6
float64 angle_deg  # clamped to JOINT_LIMITS (mycobot.hpp)
uint8 speed        # 1..100 (percent)
