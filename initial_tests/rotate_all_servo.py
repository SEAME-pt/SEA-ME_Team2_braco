from pymycobot import MyCobot280
import time

# Inicialização para Jetson Nano
mc = MyCobot280('/dev/ttyTHS1', 1000000)

# Robot arm recovery
mc.send_angles([0, 0, 0, 0, 0, 0], 40)
time.sleep(3)

# Control joint 3 to move 70°
mc.send_angle(3, 70, 40)
time.sleep(3)

# Control joint 4 to move -70°
mc.send_angle(4, -70, 40)
time.sleep(3)

# Control joint 1 to move 90°
mc.send_angle(1, 90, 40)
time.sleep(3)

# Control joint 5 to move -90°
mc.send_angle(5, -90, 40)
time.sleep(3)

# Control joint 5 to move 90°
mc.send_angle(5, 90, 40)
time.sleep(3)
