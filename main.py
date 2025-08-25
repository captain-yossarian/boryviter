from sqlite3 import connect
import time
from typing import Literal
from ultralytics import YOLO

import cvzone
import math
from pymavlink import mavutil
from constants import classNames
from camera import Camera

from picamera2 import Picamera2, Preview
from connection import Connection

from camera import Camera
from utils import calculate_centers

camera = Camera()
print("start camera")
camera.start()
print("connection to RPI")
connection = Connection("simulator")
