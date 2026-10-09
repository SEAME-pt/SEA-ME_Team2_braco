from machine import Pin
from neopixel import NeoPixel

np = NeoPixel(Pin(27), 25)   # LEDs do ATOM Matrix no pino 27

# 5x5, lido linha por linha (índices 0 a 24)
smile = [
    0, 1, 0, 1, 0,
    0, 0, 0, 0, 0,
    1, 0, 0, 0, 1,
    0, 1, 1, 1, 0,
    0, 0, 0, 0, 0,
]

BRILHO = 0.08                # 0.0 a 1.0, baixo para não ofuscar
COR = (255, 255, 0)          # amarelo

def escala(cor, k):
    return tuple(int(c * k) for c in cor)

def desenha(bitmap, cor):
    for i, v in enumerate(bitmap):
        np[i] = escala(cor, BRILHO) if v else (0, 0, 0)
    np.write()

desenha(smile, COR)
