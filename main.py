import time
from typing import Literal
from ultralytics import YOLO

import cvzone
import math
from pymavlink import mavutil
from constants import classNames

# from camera import Camera
# from picamera2 import Picamera2, Preview
from connection import Connection

# from camera import Camera
from utils import calculate_centers

# camera = Camera()
print("start camera")
# camera.start()
print("connection to RPI")
connection = Connection("simulator")
connection.turn_from(-1)
# connection.takeoff(2)
# time.sleep(3)
# connection.roll()


model = YOLO("../YOLO Weights/yolov8n.pt")


print("Learning model loaded")
# while True:
#     connection.receive_message_attitude()
#     img = camera.piCam.capture_array()

#     results = model(img, stream=True)

#     for r in results:
#         boxes = r.boxes
#         for box in boxes:
#             x1, y1, x2, y2 = box.xyxy[0]
#             x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
#             w, h = x2 - x1, y2 - y1
#             cvzone.cornerRect(img, (x1, y1, w, h))
#             conf = math.ceil((box.conf[0] * 100)) / 100
#             direction = calculate_centers(640, x1, x2)
#             print(f"direction:{direction}")
#             connection.turn_from(direction)
#             cls = box.cls[0]
#             name = "person"

#             cvzone.putTextRect(
#                 img, f"{name} " f"{conf}", (max(0, x1), max(35, y1)), scale=2
#             )


#     # cv2.imshow("Image", img)
#     # cv2.waitKey(1)
