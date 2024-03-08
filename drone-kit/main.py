#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import functions
# for dronekit
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

# Set up option parsing to get connection string
from MavikObject import MavikObject
from ObjectTracking import ObjectTracking




vehicle = connect(connection_string, wait_ready=True)
mavik = MavikObject(vehicle)
ot = ObjectTracking(mavik)
ot.run()

