#!/usr/bin/env python3
"""Search for an object with the myCobot 280 camera and look at it.

Usage:
    python3.8 search_object.py              -> list available object names
    python3.8 search_object.py --teach      -> record the search viewpoints by hand
    python3.8 search_object.py bottle       -> search for a bottle
    python3.8 search_object.py cell phone   -> names with spaces work too

A window shows the camera video with the detections. Press q to quit.
"""

import json
import os
import sys
import time

import cv2
from pymycobot import MyCobot280

# ---------------------------------------------------------------- settings

SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 1000000
CAMERA_INDEX = 0
MODEL_PATH = 'yolov8n.pt'
VIEWPOINTS_FILE = 'search_viewpoints.json'

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
INFERENCE_SIZE = 320

# Two thresholds: a weak detection is a "candidate" worth a closer look,
# a strong one (after centering) confirms the object.
CANDIDATE_CONFIDENCE = 0.25
MIN_CONFIDENCE = 0.5
CONFIRM_FRAMES = 3  # frames checked after centering on a candidate

SHOW_WINDOW = True  # set to False when running without a display (plain SSH)
WINDOW_NAME = 'Object search'

PAN_JOINT = 1  # rotates the whole arm left/right
TILT_JOINT = 4  # tilts the camera up/down

# Fallback sweep, used only when there is no taught viewpoints file.
# Joint order: [j1, j2, j3, j4, j5, j6], in degrees.
SEARCH_POSE = [0, 0, -45, -45, 0, 0]
PAN_MIN = -120
PAN_MAX = 120
TILT_ANGLES = [-45, -75]
CAMERA_HFOV = 60  # horizontal field of view of the camera, degrees
VIEW_OVERLAP = 0.3  # fraction of each view shared with the next one

MOVE_SPEED = 25
ASSUMED_DEG_PER_SEC = 20  # conservative, only used to compute move timeouts
ARRIVAL_TOLERANCE = 3.0  # degrees
SETTLE_TIME = 0.7  # seconds to wait after arriving at a viewpoint
CORRECTION_TIME = 0.8  # seconds to wait after each centering correction

# Centering: degrees of joint motion per pixel of error.
# If the arm moves AWAY from the object, flip the sign of that axis.
PAN_GAIN = -0.05
TILT_GAIN = 0.05
MAX_CORRECTION = 8.0  # max degrees per correction step
MAX_CENTERING_TRAVEL = 40  # max degrees away from the viewpoint while centering
DEADBAND_PIXELS = 40  # object counts as centered inside this radius
MAX_CENTERING_STEPS = 20  # give up centering after this many attempts


# ---------------------------------------------------------------- helpers


def clamp(value, low, high):
    return max(low, min(high, value))


def print_available_objects(model):
    names = sorted(model.names.values())
    print('Usage: python3.8 search_object.py <object name>')
    print('       python3.8 search_object.py --teach\n')
    print('Available objects:')
    for i in range(0, len(names), 4):
        print('  ' + ''.join(name.ljust(18) for name in names[i : i + 4]))


def find_class_id(model, wanted):
    for class_id, name in model.names.items():
        if name.lower() == wanted.lower():
            return class_id
    return None


def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def connect_arm():
    mc = MyCobot280(SERIAL_PORT, BAUD_RATE)
    time.sleep(1)
    return mc


def read_fresh_frame(cap):
    """Drop buffered frames so the image matches the current arm pose."""
    for _ in range(4):
        cap.grab()
    ok, frame = cap.read()
    return frame if ok else None


def read_angles(mc):
    """Return the six joint angles, or None if the arm gave an invalid reply."""
    angles = mc.get_angles()
    if isinstance(angles, list) and len(angles) == 6:
        return angles
    return None


def box_center(box):
    x1, y1, x2, y2 = box.xyxy[0].tolist()
    return (x1 + x2) / 2, (y1 + y2) / 2


def show(image):
    """Update the video window. Pressing q stops the program."""
    if not SHOW_WINDOW:
        return
    cv2.imshow(WINDOW_NAME, image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        raise KeyboardInterrupt


class Detector:
    """Reads frames, runs YOLO, draws the result and finds the wanted object."""

    def __init__(self, cap, model, class_id, wanted):
        self.cap = cap
        self.model = model
        self.class_id = class_id
        self.wanted = wanted
        self.status = 'starting'
        self.last_image = None

    def step(self, min_confidence=CANDIDATE_CONFIDENCE):
        """Process one frame. Return (frame, best_box); best_box is None if absent."""
        frame = read_fresh_frame(self.cap)
        if frame is None:
            return None, None

        result = self.model(frame, imgsz=INFERENCE_SIZE, conf=CANDIDATE_CONFIDENCE, verbose=False)[
            0
        ]
        targets = [
            box
            for box in result.boxes
            if int(box.cls) == self.class_id and float(box.conf) >= min_confidence
        ]
        best = max(targets, key=lambda box: float(box.conf)) if targets else None

        image = result.plot()  # all detections drawn
        height, width = image.shape[:2]
        cv2.drawMarker(image, (width // 2, height // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        if best is not None:
            x1, y1, x2, y2 = map(int, best.xyxy[0])
            strong = float(best.conf) >= MIN_CONFIDENCE
            color = (0, 255, 0) if strong else (0, 255, 255)  # green / yellow
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 4)
        cv2.putText(
            image,
            f'target: {self.wanted} | {self.status}',
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        self.last_image = image
        show(image)
        return frame, best

    def watch(self, seconds):
        """Keep the video running for a while."""
        end = time.time() + seconds
        while time.time() < end:
            self.step()


# ---------------------------------------------------------------- viewpoints


def default_viewpoints():
    """Grid computed from the camera field of view, in serpentine order."""
    step = CAMERA_HFOV * (1 - VIEW_OVERLAP)
    count = int((PAN_MAX - PAN_MIN) / step) + 2
    pans = [PAN_MIN + i * (PAN_MAX - PAN_MIN) / (count - 1) for i in range(count)]

    viewpoints = []
    for row, tilt in enumerate(TILT_ANGLES):
        ordered = pans if row % 2 == 0 else list(reversed(pans))
        for pan in ordered:
            pose = list(SEARCH_POSE)
            pose[PAN_JOINT - 1] = round(pan, 1)
            pose[TILT_JOINT - 1] = tilt
            viewpoints.append(pose)
    return viewpoints


def load_viewpoints():
    if os.path.exists(VIEWPOINTS_FILE):
        with open(VIEWPOINTS_FILE) as file:
            viewpoints = json.load(file)
        if viewpoints:
            print(f'Using {len(viewpoints)} taught viewpoints from {VIEWPOINTS_FILE}')
            return viewpoints
    print(
        'No taught viewpoints found, using the default grid (run with --teach to record your own)'
    )
    return default_viewpoints()


def teach():
    """Record viewpoints by moving the arm by hand while watching the camera."""
    mc = connect_arm()
    cap = open_camera()
    if cap is None:
        print('Could not open camera')
        return 1

    print('Teach mode: the servos will be released and the arm will go limp.')
    input('HOLD THE ARM with one hand, then press Enter... ')
    mc.release_all_servos()

    viewpoints = []
    try:
        if SHOW_WINDOW:
            print('Move the arm so the camera sees one part of the search area.')
            print('In the video window: SPACE = record viewpoint, q = finish.')
            while True:
                frame = read_fresh_frame(cap)
                if frame is None:
                    continue
                cv2.putText(
                    frame,
                    f'recorded: {len(viewpoints)} | SPACE=record q=finish',
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow(WINDOW_NAME, frame)
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    break
                if key == ord(' '):
                    record_viewpoint(mc, viewpoints)
        else:
            while True:
                answer = input('Move the arm, then Enter = record, q + Enter = finish: ')
                if answer.strip().lower() == 'q':
                    break
                record_viewpoint(mc, viewpoints)
    finally:
        # Lock the arm where it is so it does not drop.
        current = read_angles(mc)
        mc.power_on()
        time.sleep(0.5)
        if current is not None:
            mc.send_angles(current, MOVE_SPEED)
        cap.release()
        cv2.destroyAllWindows()

    if not viewpoints:
        print('Nothing recorded, file not changed')
        return 1
    with open(VIEWPOINTS_FILE, 'w') as file:
        json.dump(viewpoints, file, indent=2)
    print(f'Saved {len(viewpoints)} viewpoints to {VIEWPOINTS_FILE}')
    return 0


def record_viewpoint(mc, viewpoints):
    angles = read_angles(mc)
    if angles is None:
        print('Could not read the joint angles, try again')
        return
    viewpoints.append([round(angle, 1) for angle in angles])
    print(f'Viewpoint {len(viewpoints)}: {viewpoints[-1]}')


# ---------------------------------------------------------------- phases


def move_to(mc, detector, pose):
    """Send the arm to a pose and keep the video running until it arrives."""
    start = read_angles(mc)
    if start is not None:
        travel = max(abs(a - b) for a, b in zip(start, pose))
    else:
        travel = 180
    timeout = travel / ASSUMED_DEG_PER_SEC + 1.0

    mc.send_angles(pose, MOVE_SPEED)
    end = time.time() + timeout
    while time.time() < end:
        detector.step()
        current = read_angles(mc)
        if current is not None and all(
            abs(a - b) < ARRIVAL_TOLERANCE for a, b in zip(current, pose)
        ):
            break
    detector.watch(SETTLE_TIME)


def center_on(mc, detector, pose):
    """Center the candidate in the image. Return True if centered."""
    home_pan = pose[PAN_JOINT - 1]
    home_tilt = pose[TILT_JOINT - 1]
    pan, tilt = home_pan, home_tilt

    for _ in range(MAX_CENTERING_STEPS):
        frame, box = detector.step()
        if frame is None or box is None:
            continue

        center_x, center_y = box_center(box)
        error_x = center_x - frame.shape[1] / 2
        error_y = center_y - frame.shape[0] / 2
        if abs(error_x) < DEADBAND_PIXELS and abs(error_y) < DEADBAND_PIXELS:
            return True

        pan += clamp(error_x * PAN_GAIN, -MAX_CORRECTION, MAX_CORRECTION)
        tilt += clamp(error_y * TILT_GAIN, -MAX_CORRECTION, MAX_CORRECTION)
        pan = clamp(pan, home_pan - MAX_CENTERING_TRAVEL, home_pan + MAX_CENTERING_TRAVEL)
        tilt = clamp(tilt, home_tilt - MAX_CENTERING_TRAVEL, home_tilt + MAX_CENTERING_TRAVEL)

        mc.send_angle(PAN_JOINT, pan, MOVE_SPEED)
        mc.send_angle(TILT_JOINT, tilt, MOVE_SPEED)
        detector.watch(CORRECTION_TIME)
    return False


def confirm(detector):
    """After centering, require a strong detection to accept the object."""
    for _ in range(CONFIRM_FRAMES):
        _, box = detector.step(MIN_CONFIDENCE)
        if box is not None:
            return True
    return False


def search(mc, detector, viewpoints):
    """Visit each viewpoint, inspect candidates. Return True if the object is found."""
    for index, pose in enumerate(viewpoints, start=1):
        detector.status = f'searching {index}/{len(viewpoints)}'
        move_to(mc, detector, pose)

        frame, box = detector.step()
        if frame is None:
            print(f'viewpoint {index}: failed to read frame, skipping')
            continue
        if box is None:
            print(f'viewpoint {index}: nothing')
            continue

        print(f'viewpoint {index}: candidate ({float(box.conf):.2f}), taking a closer look')
        detector.status = 'checking candidate'
        centered = center_on(mc, detector, pose)
        if confirm(detector):
            if not centered:
                print('Confirmed, but could not center it perfectly')
            return True
        print(f'viewpoint {index}: candidate rejected')
    return False


# ---------------------------------------------------------------- main


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == '--teach':
        return teach()

    from ultralytics import YOLO  # imported here so --teach starts fast

    model = YOLO(MODEL_PATH)

    if len(sys.argv) < 2:
        print_available_objects(model)
        return 0

    wanted = ' '.join(sys.argv[1:])
    class_id = find_class_id(model, wanted)
    if class_id is None:
        print(f"Unknown object: '{wanted}'\n")
        print_available_objects(model)
        return 1

    viewpoints = load_viewpoints()

    cap = open_camera()
    if cap is None:
        print('Could not open camera')
        return 1

    mc = connect_arm()
    mc.power_on()
    time.sleep(0.5)

    detector = Detector(cap, model, class_id, wanted)
    exit_code = 1
    try:
        print(f'Searching for: {wanted}')
        if search(mc, detector, viewpoints):
            exit_code = 0
            print(f"Looking at '{wanted}'. Done.")
            detector.status = 'found'
            detector.step()
            if detector.last_image is not None:
                cv2.imwrite('found.jpg', detector.last_image)
        else:
            print(f"'{wanted}' not found in the search area")
    except KeyboardInterrupt:
        print('\nStopped by user')
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
