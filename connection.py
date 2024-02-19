import math
from pymavlink import mavutil
import time

# "/dev/serial0",baud=921600,source_system=1
rpi_connection_string = "/dev/serial0"
simulator_connection_string = "udpin:localhost:14550"


# https://mavlink.io/en/messages/common.html#MAV_RESULT
class Connection:

    def __init__(self, connection):
        self.connection = mavutil.mavlink_connection(
            "/dev/serial0", baud=921600, source_system=1
        )
        # self.connection = mavutil.mavlink_connection("udpin:localhost:14550")
        print("Wait heartbeat")
        self.connection.wait_heartbeat()

        print(
            "Heartbeat from system (system %u component %u)"
            % (self.connection.target_system, self.connection.target_component)
        )
        msg = self.connection.recv_match(blocking=True)
        print(f"msg:{msg}")
        # self.connection.mav.command_long_send(
        #     self.connection.target_system,
        #     self.connection.target_component,
        #     mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
        #     1,
        #     0,
        #     0,
        #     0,
        #     0,
        #     0,
        #     0,
        #     0,
        # )

    #

    def send_log(self):
        self.connection.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_CRITICAL, b"This is just a message."
        )

    def arm(self):
        print("Waiting for the vehicle to arm")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

    # Turn relatively to 0
    def turn(self, angle):
        # Define the heading change in degrees (-180 to 180, negative for left)
        left_turn_angle = angle  # 90 degrees left turn

        # Send MAV_CMD_CONDITION_YAW command to set the desired heading
        self.connection.mav.command_long_send(
            self.connection.target_system,  # Target system ID
            self.connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Command ID
            0,  # Confirmation
            0,  # Desired heading in degrees
            0,  # Hold time (0 to continue indefinitely)
            0,  # Relative offset (1 for relative angle)
            0,  # Empty (ignored)
            0,  # Empty (ignored)
            0,  # Empty (ignored)
            0,  # Empty (ignored)
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        msg = self.connection.recv_match(blocking=True)
        print(f"msg:{msg}")

    def roll(self):
        # Define the roll angle in degrees (-180 to 180, negative for left)
        left_roll_angle = 180
        # send comment to change roll angle
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            50,
            0,
            0,
            0,
            0,
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

    def takeoff(self, meters):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            meters,
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

    # Returns rotation angle relatively to initial copter position
    def get_current_rotation(self):
        print("get current position")
        msg = self.connection.recv_match(type="ATTITUDE", blocking=True)
        print(f"msg:{msg}")
        if msg:
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            print(yaw)
            return yaw
        else:
            print("Failed to retrieve current rotation angle.")
            return None

    def turn_from(self, angle):
        print("TURN FUCK")
        current_angle = self.get_current_rotation()
        print(f"current angle {current_angle}")
        if not (current_angle is None):
            print("Run turn")
            self.turn(current_angle + angle)
        else:
            print("Failed to turn")
