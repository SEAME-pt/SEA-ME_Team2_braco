import time
from pymycobot import MyCobot280

mc = MyCobot280("/dev/ttyTHS1", 1000000)
time.sleep(1)

SPEED = 30

print("Before:", mc.get_angles())

# Envia o comando para todas as 6 juntas se moverem simultaneamente
target_angles = [0, 0, 0, 0, 0, 0]
mc.send_angles(target_angles, SPEED)

# Aguarda o movimento completar antes de ler os ângulos novamente
# (sleep fixo ou verificação com is_moving)
time.sleep(3)

print("After:", mc.get_angles())
