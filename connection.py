import math
from typing import Literal
from pymavlink import mavutil
import time

# "/dev/serial0",baud=921600,source_system=1
rpi_connection_string = "/dev/serial0"
simulator_connection_string = "udpin:localhost:14550"
baud_rate = 57600
_ = mavutil.mavlink


# https://mavlink.io/en/messages/common.html#MAV_RESULT
class Connection:

    def __init__(self, connection: Literal["rpi", "simulator"]):
        connection_string = (
            rpi_connection_string
            if connection == "rpi"
            else simulator_connection_string
        )
        print(f"connection: {connection_string}, baud:{baud_rate}")
        self.connection = mavutil.mavlink_connection(
            connection_string, baud=baud_rate, source_system=1
        )
        print("Wait heartbeat")
        self.connection.wait_heartbeat()

        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            baud_rate,
            1,
        )

        message = self.connection.mav.command_long_encode(
            self.connection.target_system,  # Target system ID
            self.connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,  # param1: Message ID to be streamed
            1000000,  # param2: Interval in microseconds
            0,  # param3 (unused)
            0,  # param4 (unused)
            0,  # param5 (unused)
            0,  # param5 (unused)
            0,  # param6 (unused)
        )

        print(
            "Heartbeat from system (system %u component %u)"
            % (self.connection.target_system, self.connection.target_component)
        )
        msg = self.connection.recv_match(blocking=True)

        self.connection.mav.request_data_stream_send(
            self.connection.target_system,  # Target system ID
            self.connection.target_component,  # Target component ID
            _.MAV_DATA_STREAM_ALL,  # Request all data streams
            10,  # Request at 10 Hz (you can adjust this frequency as needed)
            1,  # Start sending immediately
        )

    def read_loop(self):
        while True:

            # grab a mavlink message
            msg = self.connection.recv_match(blocking=False)
            if not msg:
                return

            # handle the message based on its type
            msg_type = msg.get_type()

    def send(self, command, args):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            command,
            *args,
        )
        msg = self.connection.recv_match(type="COMMAND_ACK", blocking=False)
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
        print("GET YAW")
        msg = self.connection.recv_match(type="ATTITUDE", blocking=False)
        print("YAW RECEIVED")
        if msg:
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            return [roll, pitch, yaw]
        else:
            print("Failed to retrieve current rotation angle.")
            return [0, 0, 0]

    def turn_from(self, direction: Literal[-1, 0, 1]):
        roll, pitch, yaw = self.get_current_rotation()
        print(f"yaw: {yaw}")
        if direction == -1:
            print("rotate left")
        if direction == 1:
            print("rotate right")

        self.send(
            _.MAV_CMD_CONDITION_YAW,
            [
                0,
                yaw + 10,  # degrees
                20,  # Speed during yaw change:[deg per second].
                direction,  # direction [-1 0 1]
                1,  # absolute=0 or relative=1
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
                int(0b010111111000),
                5,  # fly forward in meters
                0,
                -1,  # height, negativs is positive
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        )
        # msg = self.connection.recv_match(type="COMMAND_ACK", blocking=True)
        # print(msg)

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
