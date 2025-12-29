from airo_teleop_agents.teleop_agent import TeleopAgent, TeleopAgentConfig
from airo_teleop_devices.gello_teleop_device import GelloTeleopDevice
from airo_teleop_devices.drivers.dynamixel_robot import DynamixelConfig
from airo_typing import HomogeneousMatrixType
from loguru import logger
import numpy.typing as npt
import numpy as np
import ur_analytic_ik


class Gello4UR_ParallelGripper(TeleopAgent):
    def __init__(self, config: TeleopAgentConfig, gello_usb_port: str, ur_type: str, 
                 gripper_opening_range: tuple[float, float], 
                 gello_trigger_range: tuple[float, float] | None=None, 
                 ur_start_joints: npt.NDArray[np.float64] | None=None):
        """
        Docstring for __init__
        
        :param self: Description
        :param config: Details whether to use joint space or Cartesian space, and whether to use delta actions
        :param gello_usb_port: COM port to which the Gello device is connected
        :param ur_type: "UR3e" or "UR5e"
        :param ur_start_joints: Starting joint positions for the UR robot
        :param gripper_opening_range: Given as (min_opening, max_opening) in meters
        :param gello_trigger_range: Raw dynamixel positions for fully open and fully closed trigger, e.g., (3.45, 2.53).
        """       
        gello = self._init_gello_device(ur_start_joints=ur_start_joints, 
                                       gello_usb_port=gello_usb_port,
                                       gello_trigger_range=gello_trigger_range)
        transform_func = self._build_transform_func(config, ur_type, gripper_opening_range)
        super().__init__(config=config, teleop_device=gello, transform_func=transform_func)

    def _init_gello_device(self, ur_start_joints: npt.NDArray[np.float64] | None, 
                          gello_usb_port: str,
                          gello_trigger_range: tuple[float, float] | None) -> GelloTeleopDevice:
        dynamixel_config = DynamixelConfig(
            joint_ids=[1, 2, 3, 4, 5, 6, 7],
            joint_offsets=np.array([40, 16, 25, 40, 15, 16, 0]) * np.pi / 16,
            joint_signs=[1, 1, -1, 1, 1, 1, 1],
            start_joints=np.concatenate([ur_start_joints, np.array([None])])  # append dummy for gripper
        )     
        gripper_id = 7
        if gello_trigger_range is not None:
            gello = GelloTeleopDevice(dynamixel_config=dynamixel_config, gripper_config=(gripper_id, gello_trigger_range[0], gello_trigger_range[1]), 
                            port=gello_usb_port)
        else:
            gello = GelloTeleopDevice(dynamixel_config=dynamixel_config, gripper_config=(gripper_id, 0, 0), 
                            port=gello_usb_port)
            logger.warning("Gello trigger range not provided, calibrating gripper trigger...")
            gello.calibrate_trigger(gripper_id=gripper_id)
        return gello

    def _build_transform_func(self, config, ur_type, gripper_opening_range):
        if config.use_joint_space:
            # Joint mapping should already be set up by the dynamixel_config
            def transform_func(raw_data: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
                teleop_action = raw_data.copy()
                # teleop_action[6] is the gripper mapped to [0, 1], with 0 = fully open, 1 = fully closed
                teleop_action[6] = gripper_opening_range[0] + (gripper_opening_range[1] - gripper_opening_range[0]) * (1 - teleop_action[6])
                return teleop_action
        else:
            if ur_type == "UR3e":
                fk_function = ur_analytic_ik.ur3e.forward_kinematics # type: ignore
            elif ur_type == "UR5e":
                fk_function = ur_analytic_ik.ur5e.forward_kinematics # type: ignore
            else:
                raise ValueError(f"Unsupported ur_type: {ur_type}")
            def transform_func(raw_data: npt.NDArray[np.float64]) -> tuple[HomogeneousMatrixType, float]:
                teleop_action = raw_data.copy()
                joint_positions = teleop_action[:6]
                ee_pose = fk_function(*joint_positions)
                # teleop_action[6] is the gripper mapped to [0, 1], with 0 = fully open, 1 = fully closed
                teleop_action[6] = gripper_opening_range[0] + (gripper_opening_range[1] - gripper_opening_range[0]) * (1 - teleop_action[6])
                return (ee_pose, teleop_action[6])
        return transform_func
    
class Gello4UR3(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str, config: TeleopAgentConfig):
        raise NotImplementedError

class Gello4UR3_Robotiq(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str, config: TeleopAgentConfig):
        raise NotImplementedError
    
    
class Gello4UR3_Schunk(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str, config: TeleopAgentConfig):
        raise NotImplementedError
    
class Gello4UR5(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str, config: TeleopAgentConfig):
        raise NotImplementedError
    
class Gello4UR5_Robotiq(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str, config: TeleopAgentConfig):
        raise NotImplementedError
    
class Gello4UR5_Schunk(Gello4UR_ParallelGripper):
    def __init__(self, config: TeleopAgentConfig, gello_usb_port: str,
                 gello_trigger_range: tuple[float, float] | None=None, 
                 ur_start_joints: npt.NDArray[np.float64] | None=None):
        super().__init__(config=config, gello_usb_port=gello_usb_port, ur_type="UR5e", 
                         gripper_opening_range=(0.0, 0.08),  # TODO: draw from Schunk specs in airo_robots
                         gello_trigger_range=gello_trigger_range,
                         ur_start_joints=ur_start_joints)
