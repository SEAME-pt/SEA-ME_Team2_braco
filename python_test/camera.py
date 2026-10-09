import cv2

cap = cv2.VideoCapture(0)  # 0 = /dev/video0
if not cap.isOpened():
    raise RuntimeError("Não consegui abrir a câmera")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

ok, frame = cap.read()
if ok:
    cv2.imwrite("teste.jpg", frame)
    print("Frame salvo:", frame.shape)
else:
    print("Falha ao ler o frame")

cap.release()
