from pymycobot import MyCobot280
import time

# Inicializa conexão serial no Jetson Nano
mc = MyCobot280('/dev/ttyTHS1', 1000000)
time.sleep(1)

# 1. Energiza e ativa o torque dos servos
mc.power_on()
time.sleep(0.5)

print("Posição atual:", mc.get_angles())

# 2. Retorna para a posição zero (posição inicial neutra)
print("Movendo para posição zero...")
mc.send_angles([0, 0, 0, 0, 0, 0], 25)
time.sleep(4)

# 3. Mover junta 3 para 70°
print("Movendo Junta 3 para 70°...")
mc.send_angle(3, 70, 25)
time.sleep(3)

# 4. Mover junta 4 para -70°
print("Movendo Junta 4 para -70°...")
mc.send_angle(4, -70, 25)
time.sleep(3)

# 5. Mover junta 1 para 90°
print("Movendo Junta 1 para 90°...")
mc.send_angle(1, 90, 25)
time.sleep(3)

# 6. Mover junta 5 para -90°
print("Movendo Junta 5 para -90°...")
mc.send_angle(5, -90, 25)
time.sleep(3)

# 7. Mover junta 5 para 90°
print("Movendo Junta 5 para 90°...")
mc.send_angle(5, 90, 25)
time.sleep(3)

print("Sequência finalizada! Posição final:", mc.get_angles())
