#!/usr/bin/env python3
"""
Joystick control of all 6 joints of the myCobot 280 (Jetson Nano).

Hold a control and the joint moves at constant speed. Release it and the arm
stops. On press the joint is sent towards its limit with send_angle(); on
release stop() halts the arm where it is.

Default layout (Xbox-style controller):
    Left stick   left / right   joint 1  base rotation
    Left stick   up / down      joint 2  shoulder
    Right stick  up / down      joint 3  elbow
    D-pad        up / down      joint 4  wrist pitch
    Right stick  left / right   joint 5  wrist rotation
    LB / RB                     joint 6  tool rotation
    A / B                       close / open gripper
    Y                           all joints back to 0
    Start                       quit

Usage:
    python3 joystick_all_joints.py           # control the arm
    python3 joystick_all_joints.py --calibrate            # map YOUR controller and fix directions
    python3 joystick_all_joints.py --calibrate --no-arm   # map the controls only, arm stays still
    python3 joystick_all_joints.py --test    # push controls and see what each one is mapped to (arm stays still)
    python3 joystick_all_joints.py --show    # print the mapping and check all joints
    python3 joystick_all_joints.py --debug   # control + print every joystick event

--calibrate saves joystick_config.json next to this script. When that file
exists it replaces the default layout. Delete the file to go back to default.
"""

import argparse
import json
import os
import select
import signal
import struct
import sys
import time

SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 1000000
JOYSTICK_DEVICE = '/dev/input/js0'
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'joystick_config.json')

SPEED = 25  # movement speed while a control is held (1 to 100)
HOME_SPEED = 25
GRIPPER_SPEED = 50
COMMAND_GAP = 0.05  # seconds between two serial commands

# Stick values go from -32767 to 32767. A direction turns on above PRESS and
# only turns off below RELEASE, so a stick resting near the edge does not
# flicker between moving and stopped.
# PRESS is well below the maximum because some sticks do not reach the full
# range on every side.
PRESS_LEVEL = 14000
RELEASE_LEVEL = 9000
TRIGGER_REST_LEVEL = 20000  # an axis resting beyond this is an analog trigger
HOLD_TIME = 0.25  # calibration: a control must be held this long
LIMIT_MARGIN = 3  # degrees from the limit that count as "at the limit"

# Slightly inside the mechanical limits. Index 0 is unused so joint N is LIMITS[N].
LIMITS = [None, 160, 130, 145, 140, 160, 175]

JOINT_NAMES = [
    None,
    'base rotation',
    'shoulder',
    'elbow',
    'wrist pitch',
    'wrist rotation',
    'tool rotation',
]

ACTIONS = [
    ('gripper_close', 'close gripper'),
    ('gripper_open', 'open gripper'),
    ('home', 'all joints back to 0'),
    ('quit', 'quit the program'),
]

NUDGE_DEG = 15  # test movement used by --calibrate
NUDGE_WAIT = 2.0


def axis(number, sign):
    return {'type': 'axis', 'number': number, 'sign': sign}


def button(number):
    return {'type': 'button', 'number': number}


# "pos" makes the joint angle increase, "neg" makes it decrease.
# The directions here are a best guess. Run --calibrate to confirm them on the arm.
DEFAULT_CONFIG = {
    'joints': {
        '1': {'pos': axis(0, -1), 'neg': axis(0, 1)},  # left stick X
        '2': {'pos': axis(1, 1), 'neg': axis(1, -1)},  # left stick Y
        '3': {'pos': axis(4, 1), 'neg': axis(4, -1)},  # right stick Y
        '4': {'pos': axis(7, 1), 'neg': axis(7, -1)},  # d-pad Y
        '5': {'pos': axis(3, -1), 'neg': axis(3, 1)},  # right stick X
        '6': {'pos': button(4), 'neg': button(5)},  # LB / RB
    },
    'actions': {
        'gripper_close': button(0),  # A
        'gripper_open': button(1),  # B
        'home': button(3),  # Y
        'quit': button(7),  # Start
    },
}

EVENT_FORMAT = 'IhBB'  # time (ms), value, type, number
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
TYPE_BUTTON = 0x01
TYPE_AXIS = 0x02
TYPE_INIT = 0x80


# ------------------------------------------------------------- joystick ----


class Pad:
    """Keeps the current state of every axis and button of the joystick."""

    def __init__(self, device, debug=False):
        self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
        self.debug = debug
        self.axes = {}
        self.buttons = {}
        self.rest = {}  # axis value reported at startup
        self.latched = {}  # axis directions currently considered pressed
        time.sleep(0.3)
        self.poll()

    def poll(self):
        """Read every waiting event and update the state."""
        while True:
            try:
                data = os.read(self.fd, EVENT_SIZE)
            except BlockingIOError:
                break
            if len(data) < EVENT_SIZE:
                break
            _, value, ev_type, number = struct.unpack(EVENT_FORMAT, data)
            is_init = bool(ev_type & TYPE_INIT)
            kind = ev_type & ~TYPE_INIT
            if kind == TYPE_AXIS:
                self.axes[number] = value
                if is_init:
                    self.rest[number] = value
                elif self.debug:
                    print('axis   %d = %d' % (number, value))
            elif kind == TYPE_BUTTON:
                self.buttons[number] = value
                if self.debug and not is_init:
                    print('button %d = %d' % (number, value))

    def is_trigger(self, number):
        """Analog triggers rest at one end of the range instead of the middle."""
        return abs(self.rest.get(number, 0)) > TRIGGER_REST_LEVEL

    def active(self, binding):
        if binding is None:
            return False
        if binding['type'] == 'button':
            return self.buttons.get(binding['number'], 0) == 1
        key = (binding['number'], binding['sign'])
        level = self.axes.get(binding['number'], 0) * binding['sign']
        was_on = self.latched.get(key, False)
        is_on = level > (RELEASE_LEVEL if was_on else PRESS_LEVEL)
        self.latched[key] = is_on
        return is_on

    def pushed_controls(self):
        """Return (strength, binding) for everything being pushed right now."""
        pushed = []
        for number, value in sorted(self.buttons.items()):
            if value == 1:
                pushed.append((40000, button(number)))
        for number, value in sorted(self.axes.items()):
            for sign in (1, -1):
                if self.is_trigger(number) and self.rest[number] * sign > 0:
                    continue  # that is just the trigger at rest
                if value * sign > PRESS_LEVEL:
                    pushed.append((value * sign, axis(number, sign)))
        return pushed

    def current_control(self):
        """Return the binding pushed the hardest right now, or None."""
        pushed = self.pushed_controls()
        if not pushed:
            return None
        return max(pushed, key=lambda item: item[0])[1]

    def close(self):
        os.close(self.fd)


def describe(binding):
    if binding is None:
        return '(none)'
    if binding['type'] == 'button':
        return 'button %d' % binding['number']
    return 'axis %d %s' % (binding['number'], '+' if binding['sign'] > 0 else '-')


# --------------------------------------------------------------- config ----


def load_config(path):
    if os.path.exists(path):
        with open(path) as config_file:
            print('Using mapping from', path)
            return json.load(config_file)
    print('Using the default Xbox-style mapping (no %s)' % os.path.basename(path))
    return DEFAULT_CONFIG


def check_config(config):
    """Print the mapping and report joints without a control. True if all OK."""
    ok = True
    used = {}
    print('')
    print('  joint               angle +        angle -')
    for joint in range(1, 7):
        entry = config['joints'].get(str(joint), {})
        pos, neg = entry.get('pos'), entry.get('neg')
        print(
            '  %d %-16s  %-13s  %-13s' % (joint, JOINT_NAMES[joint], describe(pos), describe(neg))
        )
        for label, binding in (('+', pos), ('-', neg)):
            if binding is None:
                print('  !! joint %d has no control for direction %s' % (joint, label))
                ok = False
                continue
            owner = 'joint %d %s' % (joint, label)
            if describe(binding) in used:
                print(
                    '  !! %s is used by both %s and %s'
                    % (describe(binding), used[describe(binding)], owner)
                )
                ok = False
            used[describe(binding)] = owner
    for key, label in ACTIONS:
        binding = config['actions'].get(key)
        print('  %-20s  %s' % (label, describe(binding)))
        if binding is not None and describe(binding) in used:
            print('  !! %s is also used by %s' % (describe(binding), used[describe(binding)]))
            ok = False
    print('')
    print(
        'All 6 joints have a control in both directions.'
        if ok
        else 'The mapping has problems, see the !! lines. Run --calibrate to fix it.'
    )
    print('')
    return ok


# ------------------------------------------------------------------ arm ----


def connect_arm():
    from pymycobot import MyCobot280

    mc = MyCobot280(SERIAL_PORT, BAUD_RATE)
    time.sleep(1)
    mc.power_on()
    time.sleep(0.5)
    return mc


def read_angles(mc):
    for _ in range(5):
        angles = mc.get_angles()
        if isinstance(angles, list) and len(angles) == 6:
            return angles
        time.sleep(0.2)
    return None


# ---------------------------------------------------------------- setup ----


def wait_release(pad):
    deadline = time.time() + 5
    while time.time() < deadline:
        pad.poll()
        if pad.current_control() is None:
            return
        time.sleep(0.02)


def wait_control(pad, can_skip, taken):
    """
    Wait for a control held steady for HOLD_TIME, so a bumped stick is not
    picked up by accident. Controls already in `taken` are refused.
    Enter on the keyboard skips (returns None) when can_skip is set.
    """
    wait_release(pad)
    candidate, since = None, 0.0
    while True:
        ready, _, _ = select.select([pad.fd, sys.stdin], [], [], 0.02)
        if sys.stdin in ready:
            sys.stdin.readline()
            if can_skip:
                return None
        pad.poll()
        control = pad.current_control()
        name = describe(control) if control is not None else None
        if name != candidate:
            candidate, since = name, time.time()
            continue
        if control is None or time.time() - since < HOLD_TIME:
            continue
        wait_release(pad)
        if name in taken:
            print('    %s is already used by %s, choose another control.' % (name, taken[name]))
            candidate = None
            continue
        return control


def nudge(mc, joint):
    """Move the joint a little and back. True if the first move increased the angle."""
    angles = read_angles(mc)
    if angles is None:
        return None
    start = angles[joint - 1]
    delta = NUDGE_DEG if start + NUDGE_DEG <= LIMITS[joint] else -NUDGE_DEG
    mc.send_angle(joint, round(start + delta, 1), SPEED)
    time.sleep(NUDGE_WAIT)
    mc.send_angle(joint, round(start, 1), SPEED)
    time.sleep(NUDGE_WAIT)
    return delta > 0


def run_setup(pad, config_path, use_arm=True):
    print('')
    print('CALIBRATION: choose the control for each joint.')
    mc = None
    if use_arm:
        print('After each choice the joint makes a small test move and comes back,')
        print('so keep the area around the arm clear.')
        try:
            mc = connect_arm()
        except Exception as error:
            print('Arm not available (%s). Directions will not be checked.' % error)
    else:
        print('The arm will not move. The control you push is saved as angle +.')

    config = {'joints': {}, 'actions': {}}
    taken = {}  # control name -> what it is already used for
    for joint in range(1, 7):
        owner = 'joint %d (%s)' % (joint, JOINT_NAMES[joint])
        print('')
        print(
            'Joint %d (%s): push and hold the control you want for it.'
            % (joint, JOINT_NAMES[joint])
        )
        first = wait_control(pad, False, taken)
        taken[describe(first)] = owner
        print('    got', describe(first))

        first_is_pos = True
        moved_pos = nudge(mc, joint) if mc is not None else None
        if moved_pos is None:
            if use_arm:
                print('    Could not test the direction on the arm, assuming angle +.')
        else:
            answer = input(
                '    The joint moved and came back. Should that push move it '
                'the way the FIRST move went? [Y/n] '
            )
            same = answer.strip().lower() not in ('n', 'no', 'nao')
            first_is_pos = moved_pos == same

        second = None
        if first['type'] == 'axis' and not pad.is_trigger(first['number']):
            second = axis(first['number'], -first['sign'])
            if describe(second) in taken:
                second = None
        if second is None:
            print('    Now push and hold the control for the opposite direction.')
            second = wait_control(pad, False, taken)
            print('    got', describe(second))
        taken[describe(second)] = owner

        config['joints'][str(joint)] = {
            'pos': first if first_is_pos else second,
            'neg': second if first_is_pos else first,
        }

    for key, label in ACTIONS:
        print('')
        print('Press the button to %s (or Enter on the keyboard to skip).' % label)
        chosen = wait_control(pad, True, taken)
        print('    got', describe(chosen))
        if chosen is not None:
            config['actions'][key] = chosen
            taken[describe(chosen)] = label

    check_config(config)
    with open(config_path, 'w') as config_file:
        json.dump(config, config_file, indent=2)
    print('Saved to', config_path)


# ----------------------------------------------------------------- test ----


def run_test(pad, config):
    """Print what every pushed control is mapped to. The arm is not used."""
    mapped = {}
    for joint in range(1, 7):
        entry = config['joints'].get(str(joint), {})
        for label, side in (('angle +', 'pos'), ('angle -', 'neg')):
            if entry.get(side) is not None:
                mapped.setdefault(describe(entry[side]), []).append(
                    'joint %d (%s) %s' % (joint, JOINT_NAMES[joint], label)
                )
    for key, label in ACTIONS:
        if config['actions'].get(key) is not None:
            mapped.setdefault(describe(config['actions'][key]), []).append(label)

    print('TEST: push each control to see what it does. The arm will not move.')
    print('Ctrl+C quits.')
    before = set()
    while True:
        pad.poll()
        now = set(describe(binding) for _, binding in pad.pushed_controls())
        for name in sorted(now - before):
            print('  %-10s -> %s' % (name, ' AND '.join(mapped.get(name, ['NOT MAPPED']))))
        before = now
        time.sleep(0.02)


# -------------------------------------------------------------- control ----


def wanted_motion(pad, config):
    """Translate what is held into {joint: direction}, direction is 1 or -1."""
    wanted = {}
    for joint in range(1, 7):
        entry = config['joints'].get(str(joint), {})
        pos = pad.active(entry.get('pos'))
        neg = pad.active(entry.get('neg'))
        if pos and not neg:
            wanted[joint] = 1
        elif neg and not pos:
            wanted[joint] = -1
    return wanted


def run_control(pad, config):
    print('Connecting to the arm on', SERIAL_PORT)
    mc = connect_arm()
    print('Current angles:', mc.get_angles())
    print('Ready. Hold a control to move, release to stop. Ctrl+C quits.')

    moving = {}  # joint -> direction currently commanded
    action_was = {}  # action -> was its button down on the last cycle

    try:
        while True:
            pad.poll()

            pressed = []
            for key, _ in ACTIONS:
                is_down = pad.active(config['actions'].get(key))
                if is_down and not action_was.get(key, False):
                    pressed.append(key)
                action_was[key] = is_down

            if 'quit' in pressed:
                print('>>> quit')
                break
            if 'home' in pressed:
                print('>>> all joints to 0')
                mc.stop()
                time.sleep(COMMAND_GAP)
                mc.send_angles([0, 0, 0, 0, 0, 0], HOME_SPEED)
                time.sleep(4)
                pad.poll()
                moving = {}
                continue
            if 'gripper_close' in pressed:
                print('>>> gripper close')
                mc.set_gripper_state(1, GRIPPER_SPEED)
            if 'gripper_open' in pressed:
                print('>>> gripper open')
                mc.set_gripper_state(0, GRIPPER_SPEED)

            wanted = wanted_motion(pad, config)
            if wanted != moving:
                # A joint was released or reversed: stop() halts every joint,
                # so the ones still held are sent again right after.
                if any(wanted.get(joint) != moving[joint] for joint in moving):
                    print('>>> stop')
                    mc.stop()
                    time.sleep(COMMAND_GAP)
                    moving = {}
                new_joints = [joint for joint in wanted if joint not in moving]
                angles = mc.get_angles() if new_joints else None
                if not (isinstance(angles, list) and len(angles) == 6):
                    angles = None
                for joint in new_joints:
                    direction = wanted[joint]
                    target = direction * LIMITS[joint]
                    entry = config['joints'][str(joint)]
                    control = describe(entry['pos' if direction > 0 else 'neg'])
                    print(
                        '>>> joint %d (%s) -> %d   [%s]'
                        % (joint, JOINT_NAMES[joint], target, control)
                    )
                    if angles is not None and abs(angles[joint - 1] - target) <= LIMIT_MARGIN:
                        print(
                            '    joint %d is already at this limit (%.0f), it cannot go '
                            'further that way' % (joint, angles[joint - 1])
                        )
                    mc.send_angle(joint, target, SPEED)
                    time.sleep(COMMAND_GAP)
                moving = dict(wanted)

            time.sleep(0.02)
    except KeyboardInterrupt:
        print('\nQuitting.')
    except OSError as error:
        print('\nJoystick lost (%s).' % error)
    finally:
        # Never leave the arm travelling towards a limit.
        mc.stop()
        time.sleep(0.2)
        print('Arm stopped. Final angles:', mc.get_angles())


def quit_on_signal(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description='Joystick control for the myCobot 280')
    parser.add_argument(
        '--calibrate',
        '--setup',
        dest='calibrate',
        action='store_true',
        help='map your controller and confirm each joint direction',
    )
    parser.add_argument(
        '--no-arm',
        action='store_true',
        help='with --calibrate: map the controls without moving the arm',
    )
    parser.add_argument(
        '--test',
        action='store_true',
        help='print what each pushed control is mapped to, arm stays still',
    )
    parser.add_argument(
        '--show',
        action='store_true',
        help='print the mapping, check all joints, and exit',
    )
    parser.add_argument('--debug', action='store_true', help='print every joystick event')
    parser.add_argument('--device', default=JOYSTICK_DEVICE)
    parser.add_argument('--config', default=CONFIG_FILE)
    args = parser.parse_args()

    # Closing the terminal or SSH session must also stop the arm.
    signal.signal(signal.SIGTERM, quit_on_signal)
    signal.signal(signal.SIGHUP, quit_on_signal)

    if args.show:
        check_config(load_config(args.config))
        return

    print('Opening joystick', args.device)
    pad = Pad(args.device, debug=args.debug)
    try:
        if args.calibrate:
            run_setup(pad, args.config, use_arm=not args.no_arm)
        else:
            config = load_config(args.config)
            check_config(config)
            if args.test:
                run_test(pad, config)
            else:
                run_control(pad, config)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        pad.close()


if __name__ == '__main__':
    main()
