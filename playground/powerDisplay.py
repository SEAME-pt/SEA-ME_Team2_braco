import time
from smbus2 import SMBus

# I2C Bus on Jetson Nano (Pins 00/01)
bus = SMBus(0)

# Addresses
LCD_ADDR = 0x3E
RGB_ADDR = 0x60  # or 0x62 depending on revision


def lcd_send_cmd(cmd):
    try:
        bus.write_byte_data(LCD_ADDR, 0x80, cmd)
    except Exception:
        pass


def lcd_send_data(data):
    try:
        bus.write_byte_data(LCD_ADDR, 0x40, data)
    except Exception:
        pass


def set_rgb(r, g, b):
    for addr in [0x60, 0x62, 0x6B]:
        try:
            # Mode 1 init
            bus.write_byte_data(addr, 0x00, 0x00)
            # Output enable
            bus.write_byte_data(addr, 0x08, 0xAA)
            # PWM registers (B, G, R or R, G, B depending on chip)
            bus.write_byte_data(addr, 0x04, r)
            bus.write_byte_data(addr, 0x03, g)
            bus.write_byte_data(addr, 0x02, b)
            return True
        except Exception:
            continue
    return False


def init_lcd():
    time.sleep(0.05)
    # 8-bit mode, 2 lines, 5x8 font
    lcd_send_cmd(0x38)
    time.sleep(0.005)
    lcd_send_cmd(0x38)
    time.sleep(0.001)
    lcd_send_cmd(0x38)
    lcd_send_cmd(0x38)
    # Display control: display on, cursor off, blink off
    lcd_send_cmd(0x0C)
    # Clear display
    lcd_send_cmd(0x01)
    time.sleep(0.005)
    # Entry mode
    lcd_send_cmd(0x06)


def write_string(text, line=1):
    if line == 1:
        lcd_send_cmd(0x80)
    elif line == 2:
        lcd_send_cmd(0xC0)
    for char in text:
        lcd_send_data(ord(char))


print('Initializing Waveshare LCD1602 RGB...')
init_lcd()
set_rgb(0, 255, 0)  # Green backlight

write_string('JETSON NANO', 1)
write_string('MYCOBOT 280', 2)
print('Test command completed!')
