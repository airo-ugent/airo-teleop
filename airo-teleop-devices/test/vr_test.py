"""Manual check: print VrTeleopDevice's raw state from a connected headset (USB adb).

Not a pytest test (it needs a headset); run it directly: python vr_test.py
"""

import time

from airo_teleop_devices.vr_teleop_device import VrTeleopDevice

if __name__ == "__main__":
    device = VrTeleopDevice()  # USB headset by default; ip_address="..." for network adb
    try:
        while True:
            action = device.get_raw_state()
            # e.g. [rx, ry, rz, rrx, rry, rrz, ..., A, B, X, Y, LJ, RJ, enabled]
            print(action)
            time.sleep(1 / 30)
    finally:
        device.disconnect()
