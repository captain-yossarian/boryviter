from ultralytics import YOLO
import cv2
import math
import time
import numpy as np
import cv2 as cv
# from constants import classNames
from utils import calculate_centers
import cvzone
# print(classNames)

import cvzone


 
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
    
    
model = YOLO("../YOLO Weights/yolov8n.pt")
classNames = ["person"]

 # Function to initialize a tracker
def init_tracker(frame, bbox):
    tracker = cv.TrackerKCF_create()
    tracker.init(frame, bbox)
    return tracker

# Function to update tracker
def update_tracker(tracker, frame):
    success, bbox = tracker.update(frame)
    return success, bbox

# Function to calculate direction
def calculate_centers(frame_width, x1, x2):
    center_x = (x1 + x2) / 2
    frame_center_x = frame_width / 2
    if center_x < frame_center_x:
        return "Left"
    elif center_x > frame_center_x:
        return "Right"
    else:
        return "Center"

tracker = None

while True:
    success, img  =cap.read()
    if not success:
        break
    
    results = model(img, stream=True)
    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls = int(box.cls[0])
            if cls != 0:
                continue
            
            name = classNames[int(cls)]
            
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            w, h = x2-x1, y2-y1
            cvzone.cornerRect(img, (x1, y1, w, h))

            conf = math.ceil((box.conf[0]*100))/100
            direction=calculate_centers(img.shape[1], x1, x2)
            print(direction)

            # cvzone.putTextRect(img, f'{name} 'f'{conf}', (max(0,x1), max(35,y1)), scale = 0.5)
            
            if tracker is None:
                bbox = (x1, y1, w, h)
                tracker = init_tracker(img, bbox)
            else:
                # Update tracker
                success_tracker, bbox = update_tracker(tracker, img)
                if success_tracker:
                    # Draw tracker bounding box
                    x, y, w, h = [int(i) for i in bbox]
                    cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
    cv.imshow('window', img)
    # cv.imshow('frame', gray)
    if cv.waitKey(1) == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()