from pymycobot import MyCobot280
import time

mc = MyCobot280('/dev/ttyTHS1', 1000000)
time.sleep(1)
mc.power_on()
time.sleep(0.5)

POSES = {
    "home":    [0, 0, 0, 0, 0, 0],
    "transit": [0, 30, -60, 30, 0, 0],
    "left":    [90, 30, -60, 30, 0, 0],
}

SEQUENCE = [
    ("home", 25),
    ("transit", 50),
    ("left", 50),
    ("transit", 50),
    ("home", 25),
]

def move_and_wait(angles, speed, timeout=10.0):
    mc.send_angles(angles, speed)
    start = time.time()
    while time.time() - start < timeout:
        if mc.is_in_position(angles, 0) == 1:
            return True
        time.sleep(0.1)
    return False

def run(sequence):
    for name, speed in sequence:
        target = POSES[name]
        if not move_and_wait(target, speed):
            print(f"Timeout at '{name}', aborting")
            mc.stop()
            return False
        actual = mc.get_angles()
        error = [round(a - t, 2) for a, t in zip(actual, target)]
        print(f"{name}: error per joint = {error}")
    return True

run(SEQUENCE)
