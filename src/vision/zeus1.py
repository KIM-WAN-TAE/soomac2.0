import cv2
from ultralytics import YOLO

model = YOLO('/home/ohheemin/Downloads/best (4).pt')

cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("cannot open the device")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("cannot open the frame")
        break

    results = model(frame)

    annotated_frame = frame.copy()

    for result in results[0].boxes:
        if result.conf >= 0.7: 
            annotated_frame = results[0].plot()

    resized_frame = cv2.resize(annotated_frame, (640, 480))
    cv2.imshow('Detecting pickle and tomato', resized_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
