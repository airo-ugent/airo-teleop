from airo_teleop_agents.teleop_agent import TeleopAgent
from airo_robots.manipulators.hardware.ur_rtde import URrtde
from airo_teleop_devices.gello_teleop_device import GelloTeleopDevice
from airo_teleop_devices.drivers.dynamixel_robot import DynamixelConfig
from airo_typing import HomogeneousMatrixType
from loguru import logger
import numpy.typing as npt
import numpy as np
import ur_analytic_ik


class Gello4UR_ParallelGripper(TeleopAgent):
    def __init__(self, gello_usb_port: str, 
                 ur_robot: URrtde,
                 gripper_opening_range: tuple[float, float], 
                 gello_trigger_range: tuple[float, float] | None=None, 
                 use_joint_space: bool=True):
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
        self.gello_usb_port = gello_usb_port
        self.ur_robot = ur_robot
        self.gripper_opening_range = gripper_opening_range
        self.gello_trigger_range = gello_trigger_range
        self.use_joint_space = use_joint_space

        gello = self._init_gello_device()
        transform_func = self._build_transform_func()
        super().__init__(teleop_device=gello, transform_func=transform_func)

    def _init_gello_device(self) -> GelloTeleopDevice:
        input("[Gello4UR_ParallelGripper._init_gello_device] Hold gello in similar pose as robot and press Enter to continue...")
        ur_start_joints = self.ur_robot.get_joint_configuration()
        dynamixel_config = DynamixelConfig(
            joint_ids=[1, 2, 3, 4, 5, 6, 7],
            joint_offsets=np.array([40, 16, 25, 40, 15, 16, 0]) * np.pi / 16,
            joint_signs=[1, 1, -1, 1, 1, 1, 1],
            start_joints=np.concatenate([ur_start_joints, np.array([None])])  # append dummy for gripper
        )     
        gripper_id = 7
        if self.gello_trigger_range is not None:
            gello = GelloTeleopDevice(dynamixel_config=dynamixel_config, gripper_config=(gripper_id, self.gello_trigger_range[0], self.gello_trigger_range[1]), 
                            port=self.gello_usb_port)
        else:
            gello = GelloTeleopDevice(dynamixel_config=dynamixel_config, gripper_config=(gripper_id, 0, 0), 
                            port=self.gello_usb_port)
            logger.warning("Gello trigger range not provided, calibrating gripper trigger...")
            gello.calibrate_trigger(gripper_id=gripper_id)
        return gello

    def _build_transform_func(self):
        if self.use_joint_space:
            # Joint mapping should already be set up by the dynamixel_config
            def transform_func(raw_data: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: # type: ignore
                teleop_action = raw_data.copy()
                # teleop_action[6] is the gripper mapped to [0, 1], with 0 = fully open, 1 = fully closed
                teleop_action[6] = self.gripper_opening_range[0] + (self.gripper_opening_range[1] - self.gripper_opening_range[0]) * (1 - teleop_action[6])
                return teleop_action
        else:
            forward_kinematics_dict = {
                URrtde.URModels.UR3: ur_analytic_ik.ur3.forward_kinematics,
                URrtde.URModels.UR3e: ur_analytic_ik.ur3e.forward_kinematics,
                URrtde.URModels.UR5e: ur_analytic_ik.ur5e.forward_kinematics
            }
            ur_type = self.ur_robot.model
            fk_function = forward_kinematics_dict[ur_type]
            def transform_func(raw_data: npt.NDArray[np.float64]) -> tuple[HomogeneousMatrixType, float]:
                teleop_action = raw_data.copy()
                joint_positions = teleop_action[:6]
                ee_pose = fk_function(*joint_positions)
                # teleop_action[6] is the gripper mapped to [0, 1], with 0 = fully open, 1 = fully closed
                teleop_action[6] = self.gripper_opening_range[0] + (self.gripper_opening_range[1] - self.gripper_opening_range[0]) * (1 - teleop_action[6])
                return (ee_pose, teleop_action[6])
        return transform_func
    
class Gello4UR3(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str):
        raise NotImplementedError

class Gello4UR3_Robotiq(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str):
        raise NotImplementedError
    
    
class Gello4UR3_Schunk(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str):
        raise NotImplementedError
    
class Gello4UR5(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str):
        raise NotImplementedError
    
class Gello4UR5_Robotiq(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str):
        raise NotImplementedError
    
class Gello4UR5_Schunk(Gello4UR_ParallelGripper):
    def __init__(self, gello_usb_port: str,
                 ur_robot: URrtde,
                 gello_trigger_range: tuple[float, float] | None=None, 
                 use_joint_space: bool=True,):
        super().__init__(gello_usb_port=gello_usb_port, 
                         ur_robot=ur_robot,
                         gripper_opening_range=(0.0, 0.08),  # TODO: draw from Schunk specs in airo_robots
                         gello_trigger_range=gello_trigger_range,
                         use_joint_space=use_joint_space)
