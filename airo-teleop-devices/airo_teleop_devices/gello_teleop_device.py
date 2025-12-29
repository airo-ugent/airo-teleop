from typing import Optional, Tuple

from loguru import logger
import numpy as np
import numpy.typing as npt

from airo_teleop_devices.drivers.dynamixel_robot import DynamixelRobot, DynamixelConfig
from airo_teleop_devices.teleop_device import TeleopDevice


class GelloTeleop(TeleopDevice):
    def __init__(self, port: str, dynamixel_config: DynamixelConfig, 
                 gripper_config: Optional[Tuple[int, float, float]]=None):
        """
        :param port: usb port to which Gello is connected, check your /dev/serial/by-id/ folder for the exact path
        :param dynamixel_config: see DynamixelConfig class in drivers/dynamixel_robot.py
        :param gripper_config: ([ID of gripper servo], [open position in servo degrees], [close position in servo degrees])
        """
        super().__init__()
        self.robot = DynamixelRobot(dynamixel_config=dynamixel_config, port=port, real=True, baudrate=57600)
        self.gripper_config = gripper_config

    def get_raw_state(self) -> npt.NDArray[np.float64]:
        joint_state = self.robot.get_joint_state()
        if self.gripper_config is not None:
            gripper_id, gripper_open, gripper_close = self.gripper_config
            gripper_index = self.robot._joint_ids.index(gripper_id)
            gripper_pos = joint_state[gripper_index]
            # map gripper_pos from [gripper_open, gripper_close] to [0, 1]
            gripper_pos_mapped = (gripper_pos - gripper_open) / (gripper_close - gripper_open)
            gripper_pos_mapped = min(max(0, gripper_pos_mapped), 1)
            joint_state[gripper_index] = gripper_pos_mapped
        return joint_state
    
    def calibrate_trigger(self, gripper_id: int):
        """
        Use this function once when integrating a Gello device in your experiment to find the open and 
        close dynamixel positions of the Gello trigger. You can then hardcode these values in the gripper_config
        parameter when initializing the GelloTeleop class.
        :param gripper_id: ID of the dynamixel servo controlling the Gello trigger
        """
        input("Please release the trigger and press Enter to continue...")
        open_pos = self.robot.get_joint_state()[self.robot._joint_ids.index(gripper_id)]
        input("Please pull the trigger fully and press Enter to continue...")
        close_pos = self.robot.get_joint_state()[self.robot._joint_ids.index(gripper_id)]
        self.gripper_config = (gripper_id, open_pos, close_pos)
        logger.info(f"Gripper calibrated: open_pos={open_pos}, close_pos={close_pos}")
