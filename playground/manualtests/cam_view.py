"""Open the camera and show live object detection with YOLO.

Usage:
    python3 camera_detect.py            # camera 0
    python3 camera_detect.py 1          # another camera index
Press 'q' to quit.
"""

import os
import sys
import time

# Show the window on the robot's own screen, even when started over SSH.
# Must be set before importing cv2.
ROBOT_DISPLAY = ':0'
os.environ['DISPLAY'] = ROBOT_DISPLAY
if 'XAUTHORITY' not in os.environ:
    for candidate in (
        f'/run/user/{os.getuid()}/gdm/Xauthority',
        os.path.expanduser('~/.Xauthority'),
    ):
        if os.path.exists(candidate):
            os.environ['XAUTHORITY'] = candidate
            break

import cv2
from ultralytics import YOLO

MODEL_PATH = 'yolov8n.pt'  # smallest model, best fit for the Jetson Nano
IMAGE_SIZE = 320  # smaller = faster, larger = more accurate
CONFIDENCE = 0.5  # minimum confidence to show a detection
WINDOW_NAME = 'Object detection'


def main():
    camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f'Error: could not open camera {camera_index}')
        return 1

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    model = YOLO(MODEL_PATH)
    last_time = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('Error: failed to read frame from camera')
                break

            results = model(frame, imgsz=IMAGE_SIZE, conf=CONFIDENCE, verbose=False)
            annotated = results[0].plot()  # frame with boxes, labels and scores

            now = time.time()
            fps = 1.0 / (now - last_time)
            last_time = now
            cv2.putText(
                annotated,
                f'FPS: {fps:.1f}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            cv2.imshow(WINDOW_NAME, annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == '__main__':
    sys.exit(main())
