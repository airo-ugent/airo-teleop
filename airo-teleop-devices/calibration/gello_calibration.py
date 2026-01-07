import time

from loguru import logger
import numpy as np

from airo_teleop_devices.gello_teleop_device import GelloTeleopDevice

#=============Test configuration==================#
port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792DZ5-if00-port0"
# Place the UR arm in a known start joint pose and enter the joints here [radians]
start_joints = np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0, 0.0]) 
#=================================================#

gello_config = GelloTeleopDevice.GELLO_DEFAULT_CONFIG  
device = GelloTeleopDevice(gello_config=gello_config, port=port)
device.calibrate_joint_offsets(current_joints=start_joints)
device.calibrate_trigger()

input("Calibration done, you can copy-paste the calibration values. " \
"Press Enter to start reading calibrated gello values...")
while True:
    gello_joints = np.array(device.get_raw_state())
    logger.debug(f"joints [°]: {(gello_joints[:6]*180/np.pi).round(3)}, trigger [open=0, close=1]: {gello_joints[6].round(3)}")
    time.sleep(0.05)
