import math
from pymavlink import mavutil
import time


class Connection:
    def __init__(self, connection_string="udpin:localhost:14550"):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.connection.wait_heartbeat()

        print(f"Heartbeat {self.connection.target_system}")

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

    def turn(self):
        # Define the heading change in degrees (-180 to 180, negative for left)
        left_turn_angle = 45  # 90 degrees left turn

        # Send MAV_CMD_CONDITION_YAW command to set the desired heading
        self.connection.mav.command_long_send(
            self.connection.target_system,  # Target system ID
            self.connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Command ID
            0,  # Confirmation
            left_turn_angle,  # Desired heading in degrees
            15,  # Hold time (0 to continue indefinitely)
            1,  # Relative offset (1 for relative angle)
            0,  # Empty (ignored)
            0,  # Empty (ignored)
            0,  # Empty (ignored)
            0,  # Empty (ignored)
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

    def roll(self):
        # Define the roll angle in degrees (-180 to 180, negative for left)
        left_roll_angle = 180
        # send comment to change roll angle
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            left_roll_angle,
            0,
            1,
            0,
            0,
            0,
            0,
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

    def takeoff(self):
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
            1,
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
