import time
from airo_teleop_devices.spacemouse_teleop_device import SpaceMouseTeleopDevice


try:
    device = SpaceMouseTeleopDevice()  # set buffer size.
    while True:
        action = device.get_raw_state()
        print(f"Action: {action.round(3)}")
        time.sleep(0.05)
except ValueError as e:
    print(e)
except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    if "device" in locals():
        device.close()
