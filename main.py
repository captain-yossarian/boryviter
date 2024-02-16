from ultralytics import YOLO

import cvzone
import math
from pymavlink import mavutil
from constants import classNames
from camera import Camera
from picamera2 import Picamera2, Preview
from connection import Connection
from camera import Camera

piCam = Camera()
connection = Connection()
connection.arm()


model = YOLO("../YOLO Weights/yolov8n.pt")


while True:
    img = piCam.capture_array()

    results = model(img, stream=True)

    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            w, h = x2 - x1, y2 - y1
            cvzone.cornerRect(img, (x1, y1, w, h))
            conf = math.ceil((box.conf[0] * 100)) / 100

            cls = box.cls[0]
            name = classNames[int(cls)]

            cvzone.putTextRect(
                img, f"{name} " f"{conf}", (max(0, x1), max(35, y1)), scale=2
            )


#     # cv2.imshow("Image", img)
#     # cv2.waitKey(1)
