import sys
import os
sys.path.insert(1, os.getcwd())

import time
import math
from BaseObject import Point

class ObjectStrategy(object):
    def __init__(self,vehicle):

        # connect to vehicle with dronekit
        # self.vehicle = self.get_vehicle_with_dronekit()
        self.camera_angle=78 #
        self.vehicle = vehicle

        self.targetPos = Point()
        self.currentPos = Point()
        # update time  range in seconds
        self.vel_update_rate = 2
        # initialised flag
        self.initialised = False
        # timer to intermittently update
        self.last_update = time.time()

    def getShift(self):
        shift = Point()
        shift.x = 3
        dx=self.currentPos.x- self.targetPos.x
        print('Stratagy shift dx - ', dx)
        dy = self.currentPos.y - self.targetPos.y
        awx=dx*(self.camera_angle/640)
        print('Stratagy shift awx - ', awx)
        y = shift.x * math.tan(awx/180 * math.pi)
        print('Stratagy shift y - ', y)
        shift.y=y
        return shift

    def moveObject(self):
        shift = self.getShift()
        if abs(shift.y) < 0.2:
            shift.y = 0
        self.send_mav_msg(shift)

    def send_mav_msg(self, shift):
        self.vehicle.velocy(shift)
