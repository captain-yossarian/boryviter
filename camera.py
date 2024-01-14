from  pymavlink import mavutil
import cv2
from picamera2 import Picamera2, Preview

piCam=Picamera2()
piCam.preview_configuration.main.size=(640,320)
piCam.preview_configuration.main.format="RGB888"
piCam.preview_configuration.align()
piCam.configure('preview')
piCam.start()

master = mavutil.mavlink_connection("/dev/serial0",baud=115200)
master.wait_heartbeat()

print(f"Heartbeat {master.target_system}")

def arm():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')


face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

while True:
    im = piCam.capture_array()
 
    grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    faces = face_detector.detectMultiScale(grey, 1.1, 5)

    for (x, y, w, h) in faces:
        cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0))

 
        print("detect face")
        arm()

    # cv2.imshow("Camera", im)
    # cv2.waitKey(1)