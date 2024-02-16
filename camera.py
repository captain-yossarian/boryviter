from picamera2 import Picamera2, Preview
import cv2


class Camera:
    def __init__(self):
        piCam = cv2.VideoCapture(0)
        piCam = Picamera2()
        piCam.preview_configuration.main.size = (640, 320)
        piCam.preview_configuration.main.format = "RGB888"
        piCam.preview_configuration.align()
        piCam.configure("preview")
        self.piCam = piCam

    def start(self):
        self.piCam.start()
