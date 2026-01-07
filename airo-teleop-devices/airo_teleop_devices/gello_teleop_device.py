from typing import Optional, Tuple

from dataclasses import dataclass
from loguru import logger
import numpy as np
import numpy.typing as npt

from airo_teleop_devices.drivers.dynamixel_robot import DynamixelRobot, DynamixelConfig
from airo_teleop_devices.teleop_device import TeleopDevice


@dataclass
class GelloConfig:
    """
    Configuration class for Gello teleop device. The gripper config is kept separate from the dynamixel config,
    because it represents an interpretation of a specific dynamixel joint as a gripper trigger, whereas 
    the other dynamixel settings can be applied to every joint.
    :param dynamixel_config: see DynamixelConfig class in drivers/dynamixel_robot.py
    :param gripper_config: ([ID of gripper servo], [open position in servo degrees], [close position in servo degrees])
    """
    dynamixel_config: DynamixelConfig
    gripper_config: Optional[Tuple[int, float, float]] = None


class GelloTeleopDevice(TeleopDevice):
    GELLO_DEFAULT_CONFIG = GelloConfig(dynamixel_config=DynamixelConfig(
                joint_ids=np.array([1, 2, 3, 4, 5, 6, 7]),  # Standard setting for all Gello devices
                joint_offsets=np.array([0.0]*7),  # Needs to be calibrated, see test > gello_calibration.py
                joint_signs=np.array([1, 1, -1, 1, 1, 1, 1]),  # Standard setting for all Gello devices
                start_joints=None
            ),
            gripper_config=None  # Needs to be calibrated, see test > gello_calibration.py
        )
    GELLO1_DEFAULT_CONFIG = GelloConfig(dynamixel_config=DynamixelConfig(
                joint_ids=np.array([1, 2, 3, 4, 5, 6, 7]),
                joint_offsets=np.array([40, 16, 25, 40, 15, 16, 0]) * np.pi / 16,
                joint_signs=np.array([1, 1, -1, 1, 1, 1, 1]),
                start_joints=np.array([0.0]*7)
            ),
            gripper_config=(7, -2.7642947390014405, -3.749831989386644)
        )
    GELLO2_DEFAULT_CONFIG = GelloConfig(dynamixel_config=DynamixelConfig(
                joint_ids=np.array([1, 2, 3, 4, 5, 6, 7]),
                joint_offsets=np.array([4*np.pi/2, 2*np.pi/2, 0*np.pi/2, -3*np.pi/2, 2*np.pi/2, 7*np.pi/2, 0]),
                joint_signs=np.array([1, 1, -1, 1, 1, 1, 1]),
                start_joints=np.array([0.0]*7)
            ),
            gripper_config=(7, -2.879793277, -3.595378259)
        )

    def __init__(self, port: str, gello_config: GelloConfig):
        """
        :param port: usb port to which Gello is connected, check your /dev/serial/by-id/ folder for the exact path
        :param gello_config: Configuration for the Gello device. Your physical Gello teleop device is labeled with 
        either "Gello1" or "Gello2", choose the according default config above or provide your own DynamixelConfig 
        and gripper_config.
        """
        super().__init__()
        self.dynamixel_config = gello_config.dynamixel_config
        self.gripper_config = gello_config.gripper_config
        self.robot = DynamixelRobot(dynamixel_config=gello_config.dynamixel_config, port=port, real=True, baudrate=57600)

    def get_raw_state(self) -> npt.NDArray[np.float64]:
        joint_state = self.robot.get_joint_state()
        if self.gripper_config is not None:
            gripper_id, gripper_open, gripper_close = self.gripper_config
            gripper_index = list(self.robot._joint_ids).index(gripper_id)
            gripper_pos = joint_state[gripper_index]
            # map gripper_pos from [gripper_open, gripper_close] to [0, 1]
            gripper_pos_mapped = (gripper_pos - gripper_open) / (gripper_close - gripper_open)
            gripper_pos_mapped = min(max(0, gripper_pos_mapped), 1)
            joint_state[gripper_index] = gripper_pos_mapped
        return joint_state
    
    def calibrate_trigger(self) -> None:
        """
        Use this function once when integrating a Gello device in your experiment to find the open and 
        close dynamixel positions of the Gello trigger. You can then hardcode these values in the gripper_config
        parameter when initializing the GelloTeleop class.
        :param gripper_id: ID of the dynamixel servo controlling the Gello trigger
        """
        gripper_id = 7  # Gello trigger is always joint 7
        gripper_idx = list(self.robot._joint_ids).index(gripper_id)
        input("[GelloTeleopDevice.calibrate_trigger] Please release the trigger and press Enter to continue...")
        open_pos = self.robot.get_joint_state()[gripper_idx]
        input("[GelloTeleopDevice.calibrate_trigger] Please pull the trigger fully and press Enter to continue...")
        close_pos = self.robot.get_joint_state()[gripper_idx]
        self.gripper_config = (gripper_id, open_pos, close_pos)
        logger.info(f"Gripper calibrated: open_pos={open_pos}, close_pos={close_pos}")

    def calibrate_joint_offsets(self, current_joints: np.ndarray) -> None:
        """
        https://github.com/wuphilipp/gello_software/blob/main/scripts/gello_get_offset.py
        """
        def get_error(offset: float, index: int, joint_state: np.ndarray) -> float:
            joint_sign_i = self.dynamixel_config.joint_signs[index]
            start_i = current_joints[index]
            joint_i = joint_sign_i * (joint_state[index] - offset)
            return np.abs(joint_i - start_i)

        input("[GelloTeleopDevice.calibrate_joint_offsets] Hold Gello in the same pose as the robot and press Enter to continue...")

        for _ in range(10):
            self.robot._driver.get_joints()  # warmup

        best_offsets = []
        curr_joints = self.robot._driver.get_joints()
        logger.debug(f"Current joints from dynamixels: {curr_joints}")
        for i in range(6):  # only first 6 joints of gello need an offset
            best_offset = 0
            best_error = 1e6
            for offset in np.linspace(
                -8 * np.pi, 8 * np.pi, 8 * 4 * 8 + 1
            ):  # intervals of pi/16
                error = get_error(offset, i, curr_joints)
                if error < best_error:
                    best_error = error
                    best_offset = offset
            best_offsets.append(best_offset)
        self.dynamixel_config.joint_offsets[:6] = np.array(best_offsets)
        self.robot.dynamixel_config.joint_offsets[:6] = np.array(best_offsets)
        self.robot.init_joint_offsets()  # re-initialize offsets to account for start_joints
        logger.info(f"Joint offsets calibrated: {self.dynamixel_config.joint_offsets}")
