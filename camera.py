from  pymavlink import mavutil
import cv2
from picamera2 import Picamera2, Preview

piCam=cv2.VideoCapture(0)
# piCam=Picamera2()
# piCam.preview_configuration.main.size=(640,320)
# piCam.preview_configuration.main.format="RGB888"
# piCam.preview_configuration.align()
# piCam.configure('preview')
# piCam.start()

# master = mavutil.mavlink_connection("/dev/serial0",baud=115200)
# master.wait_heartbeat()

# print(f"Heartbeat {master.target_system}") 


face_detecalieantor = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

while True:
    im = piCam.capture_array()
    print("start camera")
 
    grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    faces = face_detector.detectMultiScale(grey, 1.1, 5)

    for (x, y, w, h) in faces:
        cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0))

 
        print("detect face")
 
    # cv2.imshow("Camera", im)
    # cv2.waitKey(1)