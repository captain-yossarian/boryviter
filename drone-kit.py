import dronekit_sitl

sitl = dronekit_sitl.start_default()
from collections import abc
import collections

collections.MutableMapping = abc.MutableMapping
from dronekit import connect, VehicleMode, LocationGlobalRelative

import time


def check_pre_arm_conditions(vehicle):
    """
    Check all pre-arm conditions before attempting to arm.
    """
    print("Checking pre-arm conditions...")

    # Check if vehicle is ready
    if not vehicle.is_armable:
        print("Vehicle is not armable. Waiting for initialization...")
        return False

    # Check GPS lock
    if vehicle.gps_0.fix_type < 3:
        print("GPS not locked. Current fix type:", vehicle.gps_0.fix_type)
        print("Waiting for GPS lock...")
        return False

    # Check system health
    if vehicle.system_status.state != "STANDBY":
        print(
            "System not in STANDBY state. Current state:", vehicle.system_status.state
        )
        return False

    # Check if in GUIDED mode
    if vehicle.mode.name != "GUIDED":
        print("Vehicle not in GUIDED mode. Current mode:", vehicle.mode.name)
        return False

    # Check EKF status
    if hasattr(vehicle, "ekf_ok") and not vehicle.ekf_ok:
        print("EKF not ready")
        return False

    print("All pre-arm conditions met!")
    return True


def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")

    # Wait for vehicle to be ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    # Set to GUIDED mode first
    print("Setting to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(5)  # Give time for mode change
    print("MODE NAME", vehicle.mode.name)
    while not check_pre_arm_conditions(vehicle):
        time.sleep(1)

    print("Arming motors")
    try:
        vehicle.armed = True
    except Exception as e:
        print(f"Error arming vehicle: {e}")
        return False

    # Confirm vehicle armed before attempting to take off
    timeout = 30  # 30 second timeout
    start_time = time.time()
    while not vehicle.armed:
        print(" Waiting for arming...")
        if time.time() - start_time > timeout:
            print("Timeout waiting for arming")
            return False
        time.sleep(1)

    print("Taking off!")
    try:
        vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    except Exception as e:
        print(f"Error during takeoff: {e}")
        return False

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    return True


def change_altitude(vehicle, new_alt):
    current_location = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(
        current_location.lat, current_location.lon, new_alt
    )
    vehicle.simple_goto(target_location)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt <= new_alt:
            print("Reached target altitude")
            break
        time.sleep(1)


def main():
    """
    Main function to connect and fly the drone.
    """
    try:
        # Connect to the vehicle
        print("Connecting to vehicle...")
        # /dev/ttyAMA0
        # "/dev/serial0"
        vehicle = connect("tcp:127.0.0.1:5760", baud=921600, wait_ready=True)

        print(f"Vehicle connected: {vehicle}")
        print(f"Vehicle mode: {vehicle.mode.name}")
        print(f"Vehicle armed: {vehicle.armed}")
        print(f"GPS fix type: {vehicle.gps_0.fix_type}")
        print(f"System status: {vehicle.system_status.state}")

        # Attempt to arm and takeoff
        success = arm_and_takeoff(vehicle, 20)
        time.sleep(5)
        change_altitude(vehicle, 10)

        if success:
            print("Flight completed successfully!")
        else:
            print("Flight failed!")

    except Exception as e:
        print(f"Error connecting to vehicle: {e}")
        return False

    finally:
        # Always close the connection
        try:
            vehicle.close()
            print("Vehicle connection closed")
        except:
            pass


if __name__ == "__main__":
    main()
