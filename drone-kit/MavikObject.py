
from pymavlink import mavutil  # Needed for command message definitions

from BaseObject import Point
# definitions page
import time


class MavikObject(object):
    # constructor function
    def __init__(self, conection):
        # connect to vehicle with dronekit
        # self.vehicle = self.get_vehicle_with_dronekit()

        self.conection = conection

    def velocy(self, point, duration=2):
        speed_type = 0  # air speed
        msg = self.conection.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            8,
            4039,
            # mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            # 0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            point.x, point.y, point.z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        )

        # send command to vehicle
        # vehicle.send_mavlink(msg)
        # send command to vehicle on 1 Hz cycle
        now = time.time()
        first_update = now
        last_update = now
        step = 2

        self.conection.send_mavlink(msg)
        print('mavik point - x', point.x, ', y ', point.y, ', z ', point.z)

        # exit immediately if it's been too soon since the last update
        # while (now - first_update) < duration:
        #     now = time.time()
        #     if (now - last_update) > step:
        #         # for x in range(0, duration):
        #         self.conection.send_mavlink(msg)
        #         last_update = now
        #         # time.sleep(2)