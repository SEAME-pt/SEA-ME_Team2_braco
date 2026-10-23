#!/usr/bin/env python3
import os
import struct
import sys

JOYSTICK_DEVICE = '/dev/input/js0'

EVENT_FORMAT = 'IhBB'  # time (ms), value (int16), type (uint8), number (uint8)
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

TYPE_BUTTON = 0x01
TYPE_AXIS = 0x02
TYPE_INIT = 0x80


def main():
    if not os.path.exists(JOYSTICK_DEVICE):
        print(f'Erro: Dispositivo {JOYSTICK_DEVICE} não encontrado.')
        print('Verifique se o comando está emparelhado/conectado.')
        sys.exit(1)

    print(f'=== Monitorizando {JOYSTICK_DEVICE} ===')
    print('Mova os analógicos e pressione botões para ver os IDs.')
    print('Pressione Ctrl+C para encerrar.\n')

    with open(JOYSTICK_DEVICE, 'rb') as js:
        while True:
            data = js.read(EVENT_SIZE)
            if not data:
                break

            _, value, ev_type, number = struct.unpack(EVENT_FORMAT, data)

            # Ignora o dump de inicialização de hardware
            if ev_type & TYPE_INIT:
                continue

            if ev_type == TYPE_AXIS:
                pct = (value / 32767.0) * 100
                print(
                    f'[EIXO]   ID: {number:<2} | Valor bruto: {value:<6} | Posição: {pct:>6.1f}%'
                )

            elif ev_type == TYPE_BUTTON:
                estado = 'PRESSIONADO' if value == 1 else 'SOLTO'
                print(f'[BOTÃO]  ID: {number:<2} | Estado: {estado}')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nLeitura encerrada.')
