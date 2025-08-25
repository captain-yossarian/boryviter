from collections import abc
import collections

collections.MutableMapping = abc.MutableMapping
from dronekit import connect, VehicleMode
import time


def test_mode_switching():
    """
    Test mode switching capabilities of the vehicle.
    """
    try:
        print("Connecting to vehicle...")
        vehicle = connect("tcp:127.0.0.1:5760", baud=921600, wait_ready=True)

        print("\n=== VEHICLE MODE TESTING ===")
        print(f"Vehicle: {vehicle}")
        print(f"Current mode: {vehicle.mode.name}")
        print(f"System status: {vehicle.system_status.state}")
        print(f"GPS fix type: {vehicle.gps_0.fix_type}")
        print(f"GPS satellites: {vehicle.gps_0.satellites_visible}")

        # List all available modes
        print(f"\n=== AVAILABLE MODES ===")
        try:
            available_modes = [mode.name for mode in vehicle.mode_mapping]
            print(f"Available modes: {available_modes}")
        except Exception as e:
            print(f"Could not get available modes: {e}")

        # Test mode switching
        print(f"\n=== MODE SWITCHING TEST ===")

        # Try switching to GUIDED mode
        print("Attempting to switch to GUIDED mode...")
        try:
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(3)
            print(f"Mode after GUIDED attempt: {vehicle.mode.name}")
        except Exception as e:
            print(f"Error switching to GUIDED: {e}")

        # Try switching to STABILIZE mode
        print("Attempting to switch to STABILIZE mode...")
        try:
            vehicle.mode = VehicleMode("STABILIZE")
            time.sleep(3)
            print(f"Mode after STABILIZE attempt: {vehicle.mode.name}")
        except Exception as e:
            print(f"Error switching to STABILIZE: {e}")

        # Try switching back to GUIDED
        print("Attempting to switch back to GUIDED mode...")
        try:
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(3)
            print(f"Mode after second GUIDED attempt: {vehicle.mode.name}")
        except Exception as e:
            print(f"Error switching to GUIDED: {e}")

        print(f"\n=== FINAL STATUS ===")
        print(f"Final mode: {vehicle.mode.name}")
        print(f"GPS fix type: {vehicle.gps_0.fix_type}")
        print(f"System status: {vehicle.system_status.state}")

        vehicle.close()

    except Exception as e:
        print(f"Error during testing: {e}")


if __name__ == "__main__":
    test_mode_switching()
