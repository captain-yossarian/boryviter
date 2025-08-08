from collections import abc
import collections

collections.MutableMapping = abc.MutableMapping
from dronekit import connect, VehicleMode
import time

def diagnose_vehicle():
    """
    Diagnose vehicle status and identify arming issues.
    """
    try:
        print("Connecting to vehicle for diagnosis...")
        vehicle = connect("tcp:127.0.0.1:5760", baud=921600, wait_ready=True)
        
        print("\n=== VEHICLE DIAGNOSTICS ===")
        print(f"Vehicle: {vehicle}")
        print(f"Vehicle mode: {vehicle.mode.name}")
        print(f"Vehicle armed: {vehicle.armed}")
        print(f"Is armable: {vehicle.is_armable}")
        print(f"System status: {vehicle.system_status.state}")
        
        # GPS Status
        print(f"\n=== GPS STATUS ===")
        print(f"GPS fix type: {vehicle.gps_0.fix_type}")
        print(f"GPS satellites: {vehicle.gps_0.satellites_visible}")
        print(f"GPS location: {vehicle.location.global_frame}")
        
        # EKF Status
        print(f"\n=== EKF STATUS ===")
        if hasattr(vehicle, 'ekf_ok'):
            print(f"EKF OK: {vehicle.ekf_ok}")
        else:
            print("EKF status not available")
        
        # Battery Status
        print(f"\n=== BATTERY STATUS ===")
        if hasattr(vehicle, 'battery'):
            print(f"Battery level: {vehicle.battery.level}%")
            print(f"Battery voltage: {vehicle.battery.voltage}V")
        else:
            print("Battery status not available")
        
        # Ground Start Detection
        print(f"\n=== GROUND START DETECTION ===")
        print("The 'GROUND START' error usually means:")
        print("1. Vehicle thinks it's on the ground")
        print("2. Safety checks preventing arming")
        print("3. Missing GPS lock or poor GPS signal")
        
        # Recommendations
        print(f"\n=== RECOMMENDATIONS ===")
        if vehicle.gps_0.fix_type < 3:
            print("❌ GPS not locked - Wait for GPS lock (fix_type >= 3)")
        else:
            print("✅ GPS is locked")
            
        if vehicle.mode.name != 'GUIDED':
            print(f"❌ Vehicle not in GUIDED mode - Current: {vehicle.mode.name}")
        else:
            print("✅ Vehicle in GUIDED mode")
            
        if not vehicle.is_armable:
            print("❌ Vehicle not armable - Check all pre-arm conditions")
        else:
            print("✅ Vehicle is armable")
        
        vehicle.close()
        
    except Exception as e:
        print(f"Error during diagnosis: {e}")

if __name__ == "__main__":
    diagnose_vehicle()
