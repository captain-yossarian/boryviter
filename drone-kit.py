from dronekit import connect, VehicleMode
import time

# /dev/ttyAMA0
# "/dev/serial0"
vehicle = connect("/dev/serial0", baud=921600, wait_ready=True)
vehicle.mode = VehicleMode("ALTHOLD")
print(vehicle.mode.name)

vehicle.armed = True
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)
