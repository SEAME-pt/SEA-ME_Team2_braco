import time
from smbus2 import SMBus

bus = SMBus(0)
RGB_ADDR = 0x60

def test_v1_protocol():
    print("Testing Protocol 1 (Standard PCA9633)...")
    try:
        # Auto-increment register write:
        # MODE1 (0x00) -> 0x00 (normal mode)
        bus.write_byte_data(RGB_ADDR, 0x00, 0x00)
        # MODE2 (0x01) -> 0x05 (totem pole)
        bus.write_byte_data(RGB_ADDR, 0x01, 0x05)
        # LEDOUT (0x08) -> 0xAA (all LEDs PWM mode)
        bus.write_byte_data(RGB_ADDR, 0x08, 0xAA)
        
        # Test Green: PWM0/1/2 or PWM1/2/3
        for reg in [0x02, 0x03, 0x04, 0x05]:
            bus.write_byte_data(RGB_ADDR, reg, 0)
        bus.write_byte_data(RGB_ADDR, 0x03, 255) # Green
        print("  Sent Protocol 1 GREEN command.")
        return True
    except Exception as e:
        print(f"  Protocol 1 failed: {e}")
        return False

def test_v2_protocol():
    print("Testing Protocol 2 (Waveshare AiP / ST RGB)...")
    try:
        # Many Waveshare LCD1602 RGB modules use:
        # REG_RED = 0x04, REG_GREEN = 0x03, REG_BLUE = 0x02
        # with MODE1 = 0x00, MODE2 = 0x01, SUBADDR1 = 0x02...
        # Initialize
        bus.write_byte_data(RGB_ADDR, 0x00, 0x20) # Auto-increment
        bus.write_byte_data(RGB_ADDR, 0x08, 0xFF) # All outputs completely ON
        time.sleep(0.01)
        # Or individual color registers
        bus.write_byte_data(RGB_ADDR, 0x04, 0)    # R = 0
        bus.write_byte_data(RGB_ADDR, 0x03, 255)  # G = 255
        bus.write_byte_data(RGB_ADDR, 0x02, 0)    # B = 0
        print("  Sent Protocol 2 GREEN command.")
        return True
    except Exception as e:
        print(f"  Protocol 2 failed: {e}")
        return False

def test_raw_sweep():
    print("Sweeping all PWM registers (0x00 to 0x0A)...")
    try:
        # Wake up
        bus.write_byte_data(RGB_ADDR, 0x00, 0x00)
        bus.write_byte_data(RGB_ADDR, 0x01, 0x00)
        bus.write_byte_data(RGB_ADDR, 0x08, 0xAA)
        # Turn every PWM channel fully ON to see if any light turns on
        for reg in range(2, 6):
            bus.write_byte_data(RGB_ADDR, reg, 0xFF)
        print("  All channels set to 255.")
    except Exception as e:
        print(f"  Sweep failed: {e}")

print("--- Probing 0x60 Backlight ---")
test_v1_protocol()
time.sleep(1)

test_v2_protocol()
time.sleep(1)

test_raw_sweep()
print("Done. Did any color light up or flash?")
