from pymycobot import MyCobot280
import time

mc = MyCobot280('/dev/ttyTHS1', 1000000)
time.sleep(1)

mc.power_on()
time.sleep(0.5)

mc.set_color(255, 0, 0)    # vermelho
time.sleep(2)

mc.set_color(0, 255, 0)    # verde
time.sleep(2)

mc.set_color(0, 0, 255)    # azul
time.sleep(2)

mc.set_color(0, 0, 0)      # apagado
mc.set_color(255, 255, 255)      # Ligar tudo
