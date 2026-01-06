from airo_robots.manipulators.hardware.ur_rtde import URrtde
from airo_robots.grippers.hardware.schunk_process import SchunkGripperProcess
import time


gripper = SchunkGripperProcess(usb_interface="/dev/ttyUSB1")
#print(gripper.get_current_width())
gripper.move(0.06)
while True:
    time.sleep(1)
