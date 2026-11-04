import time

import numpy as np

from airo_teleop_devices.vr_teleop_device import VrTeleopDevice

device = VrTeleopDevice()  # USB headset by default; ip_address="..." for network adb
try:
    while True:
        action = device.get_raw_state()
        # e.g. [rx, ry, rz, rrx, rry, rrz, ..., A, B, X, Y, LJ, RJ, enabled]
        print(action)
        time.sleep(1 / 30)
finally:
    device.disconnect()
