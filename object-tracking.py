import cv2 as cv
from TargetObject import ObjectStrategy, Point
import time


class ObjectTracking(object):
    # constructor function
    def __init__(self, vehicle, camera_id=0):
        self.vehicle = vehicle
        self.cap = cv.VideoCapture(camera_id)  # Initialize camera capture
        self.tracker = cv.TrackerCSRT_create()  # Initialize tracker with CSRT algorithm
        self.BB = None  # Bounding Box
        self.to = ObjectStrategy(vehicle)

    def track(self,frame):
        (success, box) = self.tracker.update(frame)
        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            xb = x + int(w / 2)
            yb = y + int(h / 2)
            self.to.currentPos.update(xb, yb)
        return success, frame

    def run(self):
        ret, frame = self.cap.read()
        f_height = frame.shape[0]
        f_width = frame.shape[1]

        while True:
            ret, frame = self.cap.read()

            if self.BB is not None:
                success, frame = self.track(frame)  # Track object
                # get current time
                now = time.time()

                xb = self.to.targetPos.x
                yb = self.to.targetPos.y

                cv.line(frame, (xb, 0), (xb, f_height), (255, 0, 0), 2)
                cv.line(frame, (0, yb), (f_width, yb), (255, 0, 0), 2)


                # exit immediately if it's been too soon since the last update
                if (now - self.to.last_update) > self.to.vel_update_rate:
                    self.to.last_update = now
                    print ('last update - ', now)

                    xb = self.to.currentPos.x
                    yb = self.to.currentPos.y
                    print('last update pos - ', xb, ' ', yb)

                    ## the main function
                    self.to.moveObject()


            cv.imshow("Frame", frame)
            key = cv.waitKey(1) & 0xFF

            if key == ord("c"):  # Select Region of Interest (ROI) to track
                self.BB = cv.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
                self.tracker.init(frame, self.BB)
                (x, y, w, h) = [int(v) for v in self.BB]
                xb=x+int(w/2)
                yb=y+int(h/2)
                self.to.targetPos.update(xb, yb)



            elif key == ord("q"):
                break
        self.cap.release()
        cv.destroyAllWindows()