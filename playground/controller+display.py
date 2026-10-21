#!/usr/bin/env python3
"""
Joystick control of all 6 joints of the myCobot 280 (Jetson Nano)
with real-time stop-angle reporting on the Waveshare LCD1602 RGB display.
"""

import os
import signal
import struct
import sys
import time

from pymycobot import MyCobot280
from smbus2 import SMBus

SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 1000000
JOYSTICK_DEVICE = '/dev/input/js0'


# ---- LCD1602 Driver Helper --------------------------------------------------
class WaveshareLCD:
    LCD_ADDR = 0x3E
    RGB_ADDR = 0x60

    def __init__(self, bus_num=0):
        self.bus = None
        try:
            self.bus = SMBus(bus_num)
            self._init_rgb()
            self._init_lcd()
        except Exception as e:
            print(f'!!! LCD Init warning ({e}) - continuing without display')

    def _write_reg(self, addr, reg, val):
        try:
            self.bus.write_byte_data(addr, reg, val)
        except Exception:
            pass

    def _init_rgb(self):
        """Exact sequence from test_raw_sweep that successfully lit the backlight."""
        if not self.bus:
            return
        # Wake up & set register modes
        self._write_reg(self.RGB_ADDR, 0x00, 0x00)
        self._write_reg(self.RGB_ADDR, 0x01, 0x00)
        self._write_reg(self.RGB_ADDR, 0x08, 0xAA)
        # Turn every PWM channel fully ON (registers 0x02 to 0x05 at 0xFF)
        for reg in range(2, 6):
            self._write_reg(self.RGB_ADDR, reg, 0xFF)

    def _send_cmd(self, cmd):
        if not self.bus:
            return
        try:
            self.bus.write_byte_data(self.LCD_ADDR, 0x80, cmd)
        except Exception:
            pass

    def _send_data(self, data):
        if not self.bus:
            return
        try:
            self.bus.write_byte_data(self.LCD_ADDR, 0x40, data)
        except Exception:
            pass

    def _init_lcd(self):
        time.sleep(0.05)
        self._send_cmd(0x38)
        time.sleep(0.005)
        self._send_cmd(0x39)
        time.sleep(0.005)
        self._send_cmd(0x14)
        self._send_cmd(0x74)
        self._send_cmd(0x54)
        self._send_cmd(0x6F)
        time.sleep(0.2)
        self._send_cmd(0x38)
        self._send_cmd(0x0C)
        self._send_cmd(0x01)
        time.sleep(0.01)
        self._send_cmd(0x06)

    def write_line(self, row, text):
        if not self.bus:
            return
        self._send_cmd(0x80 if row == 1 else 0xC0)
        formatted = text.ljust(16)[:16]
        for char in formatted:
            self._send_data(ord(char))

    def show_angles(self, angles):
        """Format 6 joint angles across the two lines (exact 16 chars each)."""
        if not angles or len(angles) < 6:
            return
        l1 = f'1:{int(angles[0]):>3} 2:{int(angles[1]):>3} 3:{int(angles[2]):<3}'[:16]
        l2 = f'4:{int(angles[3]):>3} 5:{int(angles[4]):>3} 6:{int(angles[5]):<3}'[:16]
        self.write_line(1, l1)
        self.write_line(2, l2)

    def show_moving(self, text='MOVING...'):
        self.write_line(1, 'ARM MOVING...')
        self.write_line(2, text.center(16)[:16])


# ---- Speed modes (pymycobot speed units 1..100 = percent) ------------------
SPEED_MODES = [
    {
        'name': 'slow',
        'max_speed': 12,
        'lead_max': 5.0,
        'bumper_speed': 8,
        'color': (0, 0, 255),
    },
    {
        'name': 'medium',
        'max_speed': 30,
        'lead_max': 10.0,
        'bumper_speed': 15,
        'color': (0, 255, 0),
    },
    {
        'name': '50%',
        'max_speed': 50,
        'lead_max': 15.0,
        'bumper_speed': 22,
        'color': (128, 0, 255),
    },
    {
        'name': '75%',
        'max_speed': 75,
        'lead_max': 20.0,
        'bumper_speed': 30,
        'color': (255, 255, 255),
    },
    {
        'name': '100%',
        'max_speed': 100,
        'lead_max': 25.0,
        'bumper_speed': 40,
        'color': (255, 0, 0),
    },
]
START_MODE = 1

MIN_SPEED = 3
STICK_EXPO = 1.8
GRIPPER_SPEED = 50

LEAD_MIN_DEG = 1.0
SLOW_ZONE_DEG = 25.0
MIN_LIMIT_FACTOR = 0.15
AT_LIMIT_DEG = 0.3

HOME_SPEED = 60
HOME_LEAD_DEG = 15.0
HOME_TOLERANCE_DEG = 1.0
HOME_MAX_BAD_READS = 3

# ---- Stiffer J4 Hold Settings ----------------------------------------------
HOLD_TOLERANCE_DEG = 0.8
HOLD_RESEND_PERIOD = 0.1
HOLD_SPEED = 20

AXIS_DEADZONE = 5000
AXIS_MAX = 32767.0
BUMPERS_EXCLUSIVE = True

COMMAND_GAP = 0.05
LOOP_PERIOD = 0.02

AXIS_TO_JOINT = {
    0: (1, -1),
    1: (2, 1),
    3: (4, -1),
    2: (5, -1),
    7: (3, 1),
}

TRIGGER_AXES = {4, 5}
BUTTON_GRIPPER_CLOSE = 0
BUTTON_GRIPPER_OPEN = 1
BUTTON_HOME = 3
BUTTON_L1 = 6
BUTTON_R1 = 7
BUTTON_SPEED_MODE = 9
SPEED_BUTTON_GUARD_MS = 250

LIMITS = [None, 160, 130, 140, 120, 135, 170]

EVENT_FORMAT = 'IhBB'
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
TYPE_BUTTON = 0x01
TYPE_AXIS = 0x02
TYPE_INIT = 0x80


def read_events(fd):
    events = []
    while True:
        try:
            data = os.read(fd, EVENT_SIZE)
        except BlockingIOError:
            break
        if len(data) < EVENT_SIZE:
            break
        t_ms, value, ev_type, number = struct.unpack(EVENT_FORMAT, data)
        if ev_type & TYPE_INIT:
            continue
        events.append((t_ms, ev_type, number, value))
    return events


def stick_ratio(raw_value):
    magnitude = abs(raw_value)
    if magnitude <= AXIS_DEADZONE:
        return 0.0
    ratio = min(1.0, (magnitude - AXIS_DEADZONE) / (AXIS_MAX - AXIS_DEADZONE))
    return ratio**STICK_EXPO


def motion_for_ratio(ratio, mode):
    speed = MIN_SPEED + ratio * (mode['max_speed'] - MIN_SPEED)
    lead = LEAD_MIN_DEG + ratio * (mode['lead_max'] - LEAD_MIN_DEG)
    return speed, lead


def bumper_ratio(mode):
    span = float(mode['max_speed'] - MIN_SPEED)
    return max(0.0, min(1.0, (mode['bumper_speed'] - MIN_SPEED) / span))


def shape_near_limit(joint, angle, direction, speed, lead):
    lim = LIMITS[joint]
    remaining = (lim - angle) if direction > 0 else (angle + lim)
    if remaining <= AT_LIMIT_DEG:
        return None
    if remaining >= SLOW_ZONE_DEG:
        factor = 1.0
    else:
        factor = max(MIN_LIMIT_FACTOR, remaining / SLOW_ZONE_DEG)
    speed = max(1, int(round(speed * factor)))
    lead = min(lead * factor, remaining)
    return angle + direction * lead, speed


def read_angles(mc):
    try:
        angles = mc.get_angles()
    except Exception:
        return None
    if (
        isinstance(angles, list)
        and len(angles) == 6
        and all(isinstance(a, (int, float)) for a in angles)
    ):
        return angles
    return None


def set_led(mc, mode):
    r, g, b = mode['color']
    try:
        mc.set_color(r, g, b)
    except Exception as error:
        print('!!! could not set LED colour (%s)' % error)
    time.sleep(COMMAND_GAP)


def wanted_motion(axis_state, button_state, mode, blocked_axes=()):
    wanted = {}
    l1 = button_state.get(BUTTON_L1, 0) == 1
    r1 = button_state.get(BUTTON_R1, 0) == 1
    bumper_dir = -1 if (l1 and not r1) else (1 if (r1 and not l1) else 0)
    if bumper_dir:
        speed, lead = motion_for_ratio(bumper_ratio(mode), mode)
        wanted[6] = (bumper_dir, speed, lead)
        if BUMPERS_EXCLUSIVE:
            return wanted

    for axis, (joint, direction) in AXIS_TO_JOINT.items():
        if axis in blocked_axes:
            continue
        value = axis_state.get(axis, 0)
        ratio = stick_ratio(value)
        if ratio > 0.0:
            sign = direction if value > 0 else -direction
            speed, lead = motion_for_ratio(ratio, mode)
            wanted[joint] = (sign, speed, lead)

    return wanted


def quit_on_signal(signum, frame):
    raise KeyboardInterrupt


def main():
    debug = '--debug' in sys.argv

    signal.signal(signal.SIGTERM, quit_on_signal)
    signal.signal(signal.SIGHUP, quit_on_signal)

    print('1/3 Initializing LCD1602 RGB on I2C bus 0...')
    lcd = WaveshareLCD(bus_num=0)

    print('2/3 Opening joystick:', JOYSTICK_DEVICE)
    fd = os.open(JOYSTICK_DEVICE, os.O_RDONLY | os.O_NONBLOCK)

    print('3/3 Connecting to arm on:', SERIAL_PORT)
    mc = MyCobot280(SERIAL_PORT, BAUD_RATE)
    time.sleep(1)
    mc.power_on()
    time.sleep(0.5)

    init_angles = read_angles(mc)
    print('    Current angles:', init_angles)
    if init_angles:
        lcd.show_angles(init_angles)

    mode_index = START_MODE
    mode = SPEED_MODES[mode_index]
    set_led(mc, mode)

    print('Ready. Speed mode: %s. Button 9 changes speed, X homes, Ctrl+C quits.' % mode['name'])

    axis_state = {}
    button_state = {}
    active = set()
    hold = {}
    last_hold_send = {}
    limit_reported = set()
    homing = False
    bad_reads = 0
    axis_changed_ms = {}
    blocked_axes = set()
    speed_btn_ms = None
    was_moving = False

    def stop_arm():
        mc.stop()
        time.sleep(COMMAND_GAP)
        active.clear()
        hold.clear()

    try:
        while True:
            # 1. Drain joystick events
            home_clicked = False
            for t_ms, ev_type, number, value in read_events(fd):
                if ev_type == TYPE_AXIS:
                    if number in TRIGGER_AXES:
                        continue
                    axis_state[number] = value
                    axis_changed_ms[number] = t_ms
                    near_speed_btn = button_state.get(BUTTON_SPEED_MODE, 0) == 1 or (
                        speed_btn_ms is not None
                        and abs(t_ms - speed_btn_ms) <= SPEED_BUTTON_GUARD_MS
                    )
                    if near_speed_btn and abs(value) > AXIS_DEADZONE:
                        blocked_axes.add(number)
                    elif number in blocked_axes and abs(value) <= AXIS_DEADZONE:
                        blocked_axes.discard(number)
                elif ev_type == TYPE_BUTTON:
                    button_state[number] = value
                    if number == BUTTON_SPEED_MODE:
                        speed_btn_ms = t_ms
                        for ax, ax_ms in axis_changed_ms.items():
                            if (
                                t_ms - ax_ms <= SPEED_BUTTON_GUARD_MS
                                and abs(axis_state.get(ax, 0)) > AXIS_DEADZONE
                            ):
                                blocked_axes.add(ax)
                    if value == 1:
                        if number == BUTTON_GRIPPER_CLOSE:
                            mc.set_gripper_state(1, GRIPPER_SPEED)
                        elif number == BUTTON_GRIPPER_OPEN:
                            mc.set_gripper_state(0, GRIPPER_SPEED)
                        elif number == BUTTON_HOME:
                            home_clicked = True
                        elif number == BUTTON_SPEED_MODE:
                            mode_index = (mode_index + 1) % len(SPEED_MODES)
                            mode = SPEED_MODES[mode_index]
                            set_led(mc, mode)

            wanted = wanted_motion(axis_state, button_state, mode, blocked_axes)

            # 2. Homing transitions (click toggles, any jog input aborts)
            if home_clicked:
                if homing:
                    print('>>> homing cancelled')
                    homing = False
                    stop_arm()
                    stopped_angles = read_angles(mc)
                    if stopped_angles:
                        lcd.show_angles(stopped_angles)
                    was_moving = False
                else:
                    print('>>> homing (click X again or touch a stick to abort)')
                    homing = True
                    bad_reads = 0
                    active.clear()
                    hold.clear()
                    lcd.show_moving('HOMING ALL')

            if homing and wanted:
                print('>>> homing aborted by stick/bumper input')
                homing = False
                stop_arm()

            # 3. Homing routine
            if homing:
                was_moving = True
                angles = read_angles(mc)
                if angles is None:
                    bad_reads += 1
                    mc.stop()
                    time.sleep(COMMAND_GAP)
                    if bad_reads >= HOME_MAX_BAD_READS:
                        print('!!! no valid feedback, aborting home')
                        homing = False
                        was_moving = False
                        stopped_angles = read_angles(mc)
                        if stopped_angles:
                            lcd.show_angles(stopped_angles)
                    time.sleep(LOOP_PERIOD)
                    continue

                bad_reads = 0
                max_dev = max(abs(a) for a in angles)

                if max_dev <= HOME_TOLERANCE_DEG:
                    print('>>> home reached')
                    homing = False
                    stop_arm()
                    time.sleep(0.05)
                    final_home_angles = read_angles(mc) or angles
                    lcd.show_angles(final_home_angles)
                    was_moving = False
                else:
                    scale = min(1.0, HOME_LEAD_DEG / max_dev)
                    targets = [round(a * (1.0 - scale), 1) for a in angles]
                    mc.send_angles(targets, HOME_SPEED)
                    time.sleep(COMMAND_GAP)

                time.sleep(LOOP_PERIOD)
                continue

            # 4. Idle / Stop detection
            if not wanted:
                if active or was_moving:
                    print('>>> stop')
                    stop_arm()
                    stopped_angles = read_angles(mc)
                    if stopped_angles:
                        lcd.show_angles(stopped_angles)
                    was_moving = False

                hold.clear()
                limit_reported.clear()
                time.sleep(LOOP_PERIOD)
                continue

            # 5. Jogging mode
            angles = read_angles(mc)
            if angles is None:
                stop_arm()
                time.sleep(LOOP_PERIOD)
                continue

            if not was_moving:
                was_moving = True
                moving_names = ','.join(f'J{j}' for j in sorted(wanted.keys()))
                lcd.show_moving(moving_names)

            now = time.monotonic()
            moving = set(wanted)
            released = active - moving
            limit_reported &= moving

            # Hold idle joints
            for j in range(1, 7):
                if j in moving:
                    hold.pop(j, None)
                elif j not in hold:
                    hold[j] = angles[j - 1]

            for j in sorted(released):
                mc.send_angle(j, round(hold[j], 1), HOLD_SPEED)
                last_hold_send[j] = now
                time.sleep(COMMAND_GAP)

            for j in sorted(hold):
                if j in released:
                    continue
                drift = angles[j - 1] - hold[j]
                if (
                    abs(drift) > HOLD_TOLERANCE_DEG
                    and now - last_hold_send.get(j, 0.0) >= HOLD_RESEND_PERIOD
                ):
                    mc.send_angle(j, round(hold[j], 1), HOLD_SPEED)
                    last_hold_send[j] = now
                    time.sleep(COMMAND_GAP)

            # Move active joints
            for joint, (direction, speed, lead) in sorted(wanted.items()):
                shaped = shape_near_limit(joint, angles[joint - 1], direction, speed, lead)
                if shaped is None:
                    continue
                target, spd = shaped
                mc.send_angle(joint, round(target, 1), spd)
                time.sleep(COMMAND_GAP)

            active = moving
            time.sleep(LOOP_PERIOD)

    except KeyboardInterrupt:
        print('\nQuitting.')
    except OSError as error:
        print('\nJoystick disconnected (%s).' % error)
    finally:
        mc.stop()
        time.sleep(0.2)
        final_angles = read_angles(mc)
        if final_angles:
            lcd.show_angles(final_angles)
        print('Arm stopped. Final angles:', final_angles)
        os.close(fd)


if __name__ == '__main__':
    main()
