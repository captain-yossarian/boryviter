import math
from typing import Literal
from pymavlink import mavutil
import time

# "/dev/serial0",baud=921600,source_system=1
rpi_connection_string = "/dev/serial0"
simulator_connection_string = "udpin:localhost:14550"

_ = mavutil.mavlink


# https://mavlink.io/en/messages/common.html#MAV_RESULT
class Connection:

    def __init__(self, connection: Literal["rpi", "simulator"]):
        connection_string = (
            rpi_connection_string
            if connection == "rpi"
            else simulator_connection_string
        )
        print(f"connection: {connection_string}")
        self.connection = mavutil.mavlink_connection(
            connection_string, baud=921600, source_system=1
        )
        print("Wait heartbeat")
        self.connection.wait_heartbeat()

        print(
            "Heartbeat from system (system %u component %u)"
            % (self.connection.target_system, self.connection.target_component)
        )
        msg = self.connection.recv_match(blocking=True)

    def send(self, command, args):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            command,
            *args,
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        if msg is not None:
            if msg.result == 0:
                print(f"Command: ${command} is success")

    def send_log(self):
        self.connection.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_CRITICAL, b"This is just a message."
        )

    def arm(self):
        self.send(
            _.MAV_CMD_COMPONENT_ARM_DISARM,
            [
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            ],
        )

    # Turn relatively to 0
    def turn(self, angle):
        self.send(
            _.MAV_CMD_CONDITION_YAW,
            [
                0,  # Confirmation
                angle,  # Desired heading in degrees
                25,  # Hold time (0 to continue indefinitely)
                0,  # Relative offset (1 for relative angle)
                0,  # Empty (ignored)
                0,  # Empty (ignored)
                0,  # Empty (ignored)
                0,  # Empty (ignored)
            ],
        )

    def roll(self, angle):
        self.send(
            _.MAV_CMD_CONDITION_YAW,
            [
                0,
                angle,
                0,
                0,
                0,
                0,
                0,
                0,
            ],
        )

    def takeoff(self, meters):
        self.send(
            _.MAV_CMD_NAV_TAKEOFF,
            [
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                meters,
            ],
        )

    # Returns rotation angle relatively to initial copter position
    def get_current_rotation(self):
        msg = self.connection.recv_match(type="ATTITUDE", blocking=True)

        if msg:
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            return [roll, pitch, yaw]
        else:
            print("Failed to retrieve current rotation angle.")
            return [0, 0, 10]

    def turn_from(self, direction: Literal[-1, 0, 1]):
        roll, pitch, yaw = self.get_current_rotation()
        self.send(
            _.MAV_CMD_CONDITION_YAW,
            [
                0,
                yaw + direction * 10,
                # Speed during yaw change:[deg per second].
                10,
                direction,
                0,
                0,
                0,
                0,
            ],
        )

    # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#set-position-target-local-ned
    def fly_forward(self, forward_meters):
        # Send velocity command to fly forward (e.g., 1 m/s)
        vx = 1  # Velocity in x-direction (forward)
        vy = 0  # Velocity in y-direction (sideways)
        vz = 0  # Velocity in z-direction (vertical)
        yaw = 0  # Yaw angle (in radians)
        duration = 10  # Duration to fly forward (in seconds)
        self.connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10,
                self.connection.target_system,
                self.connection.target_component,
                # Positions are relative to the vehicle’s current position
                # use MAV_FRAME_LOCAL_NED for absolute position
                mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
                int(0b110111111000),
                forward_meters,  # X Position in meters (positive is forward or North)
                0,  # X Position in meters (positive is forward or North)
                -1,  # Z Position in meters (positive is down)
                0,  # X velocity in m/s (positive is forward or North)
                0,  # Y velocity in m/s (positive is right or East)
                0,  # Z velocity in m/s (positive is down)
                0,  # 	X acceleration in m/s/s (positive is forward or North)
                0,  # Y acceleration in m/s/s (positive is right or East)
                0,  # 	Z acceleration in m/s/s (positive is down)
                0,  # 	yaw or heading in radians (0 is forward or North)
                0,  # yaw rate in rad/s
            )
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

    def position(self):
        self.connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                10,
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                int(0b110111111000),
                int(-35.3629849 * 10**7),
                int(149.1649185 * 10**7),
                2,
                0,
                0,
                0,
                0,
                0,
                0,
                1.57,
                0.5,
            )
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
