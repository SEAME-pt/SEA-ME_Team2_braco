"""YOLO object detection streamed to the browser as MJPEG.

Usage:
    python3 yolo_stream.py                 # detect every class the model knows
    python3 yolo_stream.py bottle cup      # detect only these classes
    python3 yolo_stream.py --list          # print available class names
    python3 yolo_stream.py --export        # build a TensorRT engine (run once, on the Jetson)

Then open http://JETSON_IP:8080 on your computer.

Model selection: the newest model that loads wins. If a TensorRT engine
(.engine) exists next to a model, it is used instead of the .pt file.
"""

import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
from ultralytics import YOLO

CAMERA_INDEX = 0
PORT = 8080
# Tried in order. Older ones are fallbacks for old ultralytics versions.
MODEL_CANDIDATES = ['yolo26n.pt', 'yolo11n.pt', 'yolov8n.pt']
CONFIDENCE = 0.4
IMAGE_SIZE = 320  # smaller = faster on the Jetson Nano
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
JPEG_QUALITY = 70

INDEX_HTML = b"""<!doctype html>
<html>
<head><title>braco YOLO stream</title></head>
<body style="margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh">
<img src="/stream" style="max-width:100%;max-height:100vh">
</body>
</html>"""

latest_jpeg = None
frame_ready = threading.Condition()


def load_pt_model():
    """Load the newest .pt model this ultralytics version supports."""
    for path in MODEL_CANDIDATES:
        try:
            model = YOLO(path)
            return model, path
        except Exception as error:
            print(f'Could not load {path}: {error}')
    print('No model could be loaded. Try: pip install -U ultralytics')
    sys.exit(1)


def load_model():
    """Prefer a TensorRT engine if one was exported, else the .pt model."""
    for path in MODEL_CANDIDATES:
        engine_path = Path(path).with_suffix('.engine')
        if engine_path.exists():
            try:
                model = YOLO(str(engine_path), task='detect')
                print(f'Using TensorRT engine {engine_path}')
                return model
            except Exception as error:
                print(f'Could not load {engine_path}: {error}')
    model, path = load_pt_model()
    print(f'Using {path} (run with --export for a faster TensorRT engine)')
    return model


def export_engine():
    """Export the best available model to TensorRT with FP16."""
    model, path = load_pt_model()
    print(f'Exporting {path} to TensorRT, this takes several minutes...')
    model.export(format='engine', half=True, imgsz=IMAGE_SIZE)
    print('Done. Run the script again without --export to use the engine.')


def resolve_class_ids(model, class_names):
    """Translate class names into model ids. Exit if a name is unknown."""
    if not class_names:
        return None
    name_to_id = {name: class_id for class_id, name in model.names.items()}
    unknown = [name for name in class_names if name not in name_to_id]
    if unknown:
        print('Unknown classes:', ', '.join(unknown))
        print('Run with --list to see the available ones.')
        sys.exit(1)
    return [name_to_id[name] for name in class_names]


def detection_loop(model, class_ids):
    """Read frames, run YOLO, draw boxes and publish the latest JPEG."""
    global latest_jpeg

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print('Could not open camera', CAMERA_INDEX)
        return

    fps = 0.0
    while True:
        start = time.time()
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue

        results = model(frame, imgsz=IMAGE_SIZE, conf=CONFIDENCE, classes=class_ids, verbose=False)
        annotated = results[0].plot()

        elapsed = time.time() - start
        if elapsed > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / elapsed)
        cv2.putText(
            annotated, f'{fps:.1f} FPS', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        ok, jpg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            continue
        with frame_ready:
            latest_jpeg = jpg.tobytes()
            frame_ready.notify_all()


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_stream()
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(INDEX_HTML)))
            self.end_headers()
            self.wfile.write(INDEX_HTML)

    def send_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while True:
                with frame_ready:
                    frame_ready.wait(timeout=1.0)
                    jpeg = latest_jpeg
                if jpeg is None:
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *args):
        pass


def main():
    args = sys.argv[1:]

    if '--export' in args:
        export_engine()
        return

    model = load_model()

    if '--list' in args:
        print('Available classes:')
        print(', '.join(model.names.values()))
        return

    class_ids = resolve_class_ids(model, args)

    worker = threading.Thread(target=detection_loop, args=(model, class_ids), daemon=True)
    worker.start()

    print(f'Streaming on http://0.0.0.0:{PORT}')
    try:
        ThreadingHTTPServer(('0.0.0.0', PORT), StreamHandler).serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
