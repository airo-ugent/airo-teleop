from airo_teleop_agents.teleop_agent import TeleopAgent
from airo_teleop_devices.gello_teleop_device import GelloTeleopDevice, GelloConfig
from airo_robots.manipulators.hardware.ur_rtde import URrtde
from airo_robots.grippers.hardware.parallel_position_gripper import ParallelPositionGripper
from airo_teleop_devices.drivers.dynamixel_robot import DynamixelConfig
from airo_typing import HomogeneousMatrixType
from loguru import logger
import numpy.typing as npt
import numpy as np
import ur_analytic_ik


class Gello4UR(TeleopAgent):
    def __init__(self, gello_usb_port: str, 
                 gello_config: GelloConfig, 
                 ur_robot: URrtde,
                 use_joint_space: bool=True):
        """
        This class implements a teleoperation agent for any combination of Gello teleop device, UR robot, 
        and parallel gripper (e.g. Robotiq 2F-85 and Schunk EGK40).
        :param gello_usb_port: COM port to which the Gello device is connected
        :param ur_robot: airo_robots.manipulators.hardware.ur_rtde.URrtde object representing the UR robot to be teleoperated
        :param gello_config: Configuration for the Gello device
        :param use_joint_space: Whether to use joint space control or tool space control
        """       
        self.gello_usb_port = gello_usb_port
        self.ur_robot = ur_robot
        self.gello_config = gello_config
        self.use_joint_space = use_joint_space

        gello = self._init_gello_device()
        transform_func = self._build_transform_func()
        super().__init__(teleop_device=gello, transform_func=transform_func)

    def _init_gello_device(self) -> GelloTeleopDevice:
        input("[Gello4UR_ParallelGripper._init_gello_device] Hold gello in similar pose as robot and press Enter to continue...")
        ur_start_joints = self.ur_robot.get_joint_configuration()
        self.gello_config.dynamixel_config.start_joints = ur_start_joints
        gello = GelloTeleopDevice(gello_config=self.gello_config, port=self.gello_usb_port)
        return gello

    def _build_transform_func(self):
        if self.use_joint_space:
            # Joint mapping should already be set up by the dynamixel_config
            def transform_func(raw_data: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: # type: ignore
                teleop_action = raw_data.copy()
                return teleop_action[:6]
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
                return ee_pose
        return transform_func


class Gello4UR_ParallelGripper(Gello4UR):
    def __init__(self, gello_usb_port: str, 
                 gello_config: GelloConfig, 
                 ur_robot: URrtde,
                 gripper: ParallelPositionGripper, 
                 use_joint_space: bool=True):
        """
        This class implements a teleoperation agent for any combination of Gello teleop device, UR robot, 
        and parallel gripper (e.g. Robotiq 2F-85 and Schunk EGK40).
        :param gello_usb_port: COM port to which the Gello device is connected
        :param ur_robot: airo_robots.manipulators.hardware.ur_rtde.URrtde object representing the UR robot to be teleoperated
        :param gripper: airo_robots.grippers.hardware.parallel_position_gripper.ParallelPositionGripper object representing the gripper attached to the UR robot
        :param gello_config: Configuration for the Gello device
        :param use_joint_space: Whether to use joint space control or tool space control
        """       
        
        self.gripper = gripper
        self.gripper_opening_range = (self.gripper.gripper_specs.min_width, self.gripper.gripper_specs.max_width)
        super().__init__(gello_usb_port=gello_usb_port, 
                         gello_config=gello_config, 
                         ur_robot=ur_robot,
                         use_joint_space=use_joint_space)

    def _init_gello_device(self) -> GelloTeleopDevice:
        gello = super()._init_gello_device()
        if self.gello_config.gripper_config is None:
            logger.warning("Gello gripper config not provided, calibrating gello trigger...")
            gello.calibrate_trigger()
        return gello

    def _build_transform_func(self):
        super_transform_func = super()._build_transform_func()
        if self.use_joint_space:
            def transform_func(raw_data: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: # type: ignore
                joint_action = super_transform_func(raw_data)
                # Map gripper from [0, 1] to [max_width, min_width]
                gripper_action = self.gripper_opening_range[0] + (self.gripper_opening_range[1] - self.gripper_opening_range[0]) * (1 - raw_data[6])
                #concatenate joint action and gripper action
                teleop_action = np.concatenate((joint_action, [gripper_action]))
                return teleop_action
        else:
            def transform_func(raw_data: npt.NDArray[np.float64]) -> tuple[HomogeneousMatrixType, float]:
                ee_pose = super_transform_func(raw_data)
                gripper_action = self.gripper_opening_range[0] + (self.gripper_opening_range[1] - self.gripper_opening_range[0]) * (1 - raw_data[6])
                return (ee_pose, gripper_action)
        return transform_func
