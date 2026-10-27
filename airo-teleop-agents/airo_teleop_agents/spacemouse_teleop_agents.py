from airo_teleop_agents.teleop_agent import TeleopAgent
from airo_robots.manipulators.position_manipulator import PositionManipulator
from airo_robots.grippers.parallel_position_gripper import ParallelPositionGripper
from airo_teleop_devices.spacemouse_teleop_device import SpaceMouseTeleopDevice
from airo_typing import HomogeneousMatrixType
from typing import Callable, Literal, Tuple
import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as scpRotation


class SpaceMouse4PositionManipulator(TeleopAgent):
    def __init__(self, 
                 position_manipulator: PositionManipulator | None,
                 translation_scale: float = 0.02, 
                 rotation_scale: float = 0.02, 
                 deadzone: Tuple[float, float]=(0.1, 0.1), 
                 enabled_axes: list[bool]=[True]*6 + [False]*2,
                 action_mode: Literal["absolute", "delta"] = "absolute"):
        """
        Teleop agent using a SpaceMouse to control a PositionManipulator in tool space.
        Teleop agent using a SpaceMouse to control a PositionManipulator in tool space and a ParallelPositionGripper.
        TODO: currently, the speed of the teleoperated robot will depend on the loop frequency with which this TeleopAgent is polled:
        a translation step (obtained by spacemouse translation value * translation_scale) at 5 Hz vs at 25 Hz is vastly different behaviour.
        We could attempt to pass linear and joint speeds along with a loop frequency as arguments, from which we derive the needed
        translation_scale, rotation_scale.
        
        :param position_manipulator: airo_robots PositionManipulator object, like URrtde. Only used in "absolute" 
            action_mode, can be None in "delta" action_mode.
        :param translation_scale: Real-world displacement per spacemouse action in meters.
        :param rotation_scale: Real-world rotation per spacemouse action in radians.
        :param deadzone: (translation, rotation) deadzone values. If output of spacemouse is below this value, 
            it is set to 0.
        :param enabled_axes: Boolean array detailing which SpaceMouse axes are enabled. This allows e.g. to only teleop translations. 
            Can be set dynamically (simply do SpaceMouse4PositionManipulator.enabled_axes = new_setting). 
            By default, last two axes disabled because these are the buttons that would typically contorl a gripper.
            The value of the last two elements of enabled_axes, however, doesn't actually matter, the button 
            state is simply not used in the transform function of this TeleopAgent.
        :param action_mode: "absolute" returns the new 4x4 TCP pose, obtained by applying the delta to the current TCP pose 
            of the position_manipulator. "delta" returns the delta itself as [dx, dy, dz, drx, dry, drz], without reading 
            the robot: dx/dy/dz in meters in the base frame, drx/dry/drz XYZ euler angles in radians, to be applied as 
            R.from_euler("xyz", [drx, dry, drz]).as_matrix() @ current_rotation (i.e. around base-frame axes through the EEF).
        """
        if action_mode not in ("absolute", "delta"):
            raise ValueError(f"action_mode must be 'absolute' or 'delta', got {action_mode!r}")
        if action_mode == "absolute" and position_manipulator is None:
            raise ValueError("a position_manipulator is required in 'absolute' action_mode")
        self.action_mode = action_mode
        self.position_manipulator = position_manipulator
        self.enabled_axes = enabled_axes
        self.translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        spacemouse_device = SpaceMouseTeleopDevice(deadzone=deadzone)
        super().__init__(teleop_device=spacemouse_device, 
                         transform_func=self._build_transform_func())
        
    def _build_transform_func(self) -> Callable:
        def transform_func(raw_data) -> HomogeneousMatrixType | npt.NDArray[np.float64]:
            # Convert spacemouse action to homogeneous matrix action
            # we take the action to consist of a delta position, delta rotation and delta gripper width.
            # the delta rotation is interpreted as expressed in a frame with the same orientation as the base frame but with the origin at the EEF.
            # in this way, when rotating the spacemouse, the robot eef will not move around, while at the same time the axes of orientation
            # do not depend on the current orientation of the EEF.

            # the delta position is intepreted in the world frame and also applied on the EEF frame.
            raw_data = np.where(self.get_enable_axes(), raw_data, 0)
            translation_delta = raw_data[:3] * self.translation_scale
            rpy_delta = raw_data[3:6] * self.rotation_scale

            if self.action_mode == "delta":
                return np.concatenate([translation_delta, rpy_delta])

            current_pose = self.position_manipulator.get_tcp_pose()
            current_translation = current_pose[:3, 3]
            current_rotation_matrix = current_pose[:3, :3]

            new_translation = current_translation + translation_delta
            # Rotation is now interpreted as euler and not as rotvec similar to Diffusion Policy.
            # However, rotvec seems more principled (related to twist)
            new_rotation_matrix = scpRotation.from_euler("xyz", rpy_delta).as_matrix() @ current_rotation_matrix
            new_pose = np.eye(4)
            new_pose[:3, :3] = new_rotation_matrix
            new_pose[:3, 3] = new_translation

            return new_pose
        return transform_func
    
    def get_enable_axes(self) -> list[bool]:
        return self.enabled_axes
    

class SpaceMouse4PositionManipulator_ParallelGripper(SpaceMouse4PositionManipulator):
    def __init__(self, 
                 position_manipulator: PositionManipulator | None,
                 gripper: ParallelPositionGripper | None, 
                 translation_scale: float=0.02, 
                 rotation_scale: float=0.02, 
                 gripper_step_size: float | None=0.01,
                 deadzone: Tuple[float, float]=(0.1, 0.1), 
                 enabled_axes: list[bool]=[True]*8,
                 action_mode: Literal["absolute", "delta"] = "absolute"):
        """
        Teleop agent using a SpaceMouse to control a PositionManipulator in tool space and a ParallelPositionGripper.
        TODO: currently, the speed of the teleoperated robot will depend on the loop frequency with which this TeleopAgent is polled:
        a translation step (obtained by spacemouse translation value * translation_scale) at 5 Hz vs at 25 Hz is vastly different behaviour.
        We could attempt to pass linear and joint speeds along with a loop frequency as arguments, from which we derive the needed
        translation_scale, rotation_scale and gripper_step_size.

        :param position_manipulator: airo_robots PositionManipulator object, like URrtde. Can be None in "delta" action_mode.
        :param gripper: airo_robots ParallelPositionGripper object, like Robotiq2F85 or SchunkEGK40. Can be None in "delta" action_mode.
        :param translation_scale: Real-world displacement per spacemouse action in meters.
        :param rotation_scale: Real-world rotation per spacemouse action in radians.
        :param gripper_step_size: Step size for gripper when the spacemouse buttons are pressed, in meters. If None, the action will 
        be set to either the minimum or maximum width of the ParallelPositionGripper.
        :param deadzone: (translation, rotation) deadzone values. If output of spacemouse is below this value, 
            it is set to 0.
        :param enabled_axes: Boolean array detailing which SpaceMouse axes are enabled. This allows e.g. to only teleop translations. 
            Can be set dynamically (simply do SpaceMouse4PositionManipulator.enabled_axes = new_setting).
        :param action_mode: "absolute" returns a (4x4 TCP pose, gripper width) tuple. "delta" returns a flat array 
            [dx, dy, dz, drx, dry, drz, dgripper] (see SpaceMouse4PositionManipulator), where dgripper is -gripper_step_size 
            (button 0, close), +gripper_step_size (button 1, open) or 0. Requires a gripper_step_size.
        """
        if action_mode == "absolute" and gripper is None:
            raise ValueError("a gripper is required in 'absolute' action_mode")
        if action_mode == "delta" and gripper_step_size is None:
            raise ValueError("a gripper_step_size is required in 'delta' action_mode")
        self.gripper = gripper
        self.gripper_step_size = gripper_step_size
        super().__init__(
                 position_manipulator=position_manipulator,
                 translation_scale=translation_scale, 
                 rotation_scale=rotation_scale, 
                 deadzone=deadzone, 
                 enabled_axes=enabled_axes,
                 action_mode=action_mode)
        
    def _build_transform_func(self) -> Callable:
        super_transform_func = super()._build_transform_func()
        def transform_func(raw_data) -> Tuple[HomogeneousMatrixType, float] | npt.NDArray[np.float64]:
            manipulator_action = super_transform_func(raw_data)
            raw_data = np.where(self.get_enable_axes(), raw_data, 0)
            if self.action_mode == "delta":
                gripper_delta = -self.gripper_step_size if raw_data[6] else self.gripper_step_size if raw_data[7] else 0.0
                return np.concatenate([manipulator_action, [gripper_delta]])
            current_gripper_opening = self.gripper.get_current_width()
            if raw_data[6]:
                new_gripper_opening = current_gripper_opening - self.gripper_step_size if self.gripper_step_size is not None else self.gripper.gripper_specs.min_width
            elif raw_data[7]:
                new_gripper_opening = current_gripper_opening + self.gripper_step_size if self.gripper_step_size is not None else self.gripper.gripper_specs.max_width
            else:
                new_gripper_opening = current_gripper_opening
            return manipulator_action, new_gripper_opening
        return transform_func