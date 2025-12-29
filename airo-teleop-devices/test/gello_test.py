import time

from loguru import logger
import numpy as np

from airo_teleop_devices.drivers.dynamixel_robot import DynamixelConfig
from airo_teleop_devices.gello_teleop_device import GelloTeleop


dynamixel_config = DynamixelConfig(
    joint_ids=[1, 2, 3, 4, 5, 6, 7],
    joint_offsets=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    joint_signs=[1, 1, -1, 1, 1, 1, 1],
    start_joints=None,
)
device = GelloTeleop(dynamixel_config=dynamixel_config, gripper_config=(7, 194, 152), 
                        port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792DZ5-if00-port0")
device.calibrate_trigger(gripper_id=7)
while True:
    gello_joints = np.array(device.get_raw_state())
    # log joints with 3 decimal places
    logger.debug(f"gello_joints={gello_joints.round(3)}")
    time.sleep(0.05)
