from ultralytics import YOLO
import cv2
import math
import time

from constants import classNames
from utils import calculate_centers

print(classNames)


cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

model = YOLO("../YOLO Weights/yolov8n.pt")

classNames = ["person"]

while True:
    success, img = cap.read()
    img_width = img.shape[1]  # Get the width of the image

    results = model(img, stream=True)

    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert coordinates to integers
            w, h = x2 - x1, y2 - y1

            # Calculate centers
            screen_center_x, person_center_x = calculate_centers(img_width, x1, x2)

            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.imshow("Image", img)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
