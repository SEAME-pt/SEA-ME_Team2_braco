from pymycobot import MyCobot280
import time

# Inicializa conexão serial no Jetson Nano
mc = MyCobot280('/dev/ttyTHS1', 1000000)
time.sleep(1)

mc.power_on()
time.sleep(0.5)

VEL = 30  # velocidade moderada (0 a 100)


def passo(nome, angulos, espera=2):
    print(f'-> {nome}')
    mc.send_angles(angulos, VEL)
    time.sleep(espera)


print('Posição atual:', mc.get_angles())

# Posição inicial
passo('Posição inicial', [0, 0, 0, 0, 0, 0], 4)

# 1. Balanço lateral da base
for _ in range(2):
    passo('Balança para a direita', [40, 0, 0, 0, 0, 0])
    passo('Balança para a esquerda', [-40, 0, 0, 0, 0, 0])
passo('Centro', [0, 0, 0, 0, 0, 0])

# 2. Reverência
passo('Reverência', [0, 30, -40, 0, 30, 0], 2.5)
passo('Levanta', [0, 0, 0, 0, 0, 0], 2.5)

# 3. Ondinha com o punho
for _ in range(3):
    passo('Punho para cima', [0, 0, 0, 0, 60, 0], 1.5)
    passo('Punho para baixo', [0, 0, 0, 0, -60, 0], 1.5)
passo('Punho no centro', [0, 0, 0, 0, 0, 0])

# 4. Giro do punho
passo('Gira o punho', [0, 0, 0, 0, 0, 120], 3)
passo('Gira o punho ao contrário', [0, 0, 0, 0, 0, -120], 4)
passo('Punho no centro', [0, 0, 0, 0, 0, 0])

# 5. Passo de dança combinado
for _ in range(2):
    passo('Combinado A', [30, 20, -30, 40, 40, 60], 2.5)
    passo('Combinado B', [-30, -20, 30, -40, -40, -60], 2.5)

# 6. Final
passo('Pose final', [0, 40, -60, 0, 40, 0], 3)
passo('Volta ao zero', [0, 0, 0, 0, 0, 0], 4)

print('Dança finalizada! Posição final:', mc.get_angles())
