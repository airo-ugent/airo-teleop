from dataclasses import dataclass
from airo_teleop_devices.teleop_device import TeleopDevice
import numpy as np
import numpy.typing as npt


@dataclass
class TeleopAgentConfig:
    use_joint_space: bool = True  # Expected to be handled in TeleopAgent.transform_func
    use_deltas: bool | npt.NDArray[np.bool]  = False  # If bool, applies to all dimensions; if array, specifies per dimension


class TeleopAgent:
    '''
    Takes the raw state output of a TeleopDevice and transforms it to a robot action using the provided transform_func.
    The transform_func is expected to be set up properly in accordance with the TeleopConfig (e.g., joint space vs Cartesian space).
    Use of deltas is handled in get_action, based on the setting in the TeleopConfig.

    TODO: consider if multiple teleop devices should be supported within a single teleop agent, for example:
    Gello controls Mobi's manipulator, while Spacemouse controls the base. As it stands, this would require
    separate TeleopAgents for each device.
    '''
    def __init__(self, config: TeleopAgentConfig, teleop_device: TeleopDevice, transform_func=lambda x: x):
        self.transform_func = transform_func
        self.config = config
        self.teleop_device = teleop_device

    def get_action(self, current_robot_state: npt.NDArray[np.float64] | None = None) -> npt.NDArray[np.float64]:
        teleop_device_state = self.teleop_device.get_raw_state()
        action = self.transform_func(teleop_device_state)
        if np.array(self.config.use_deltas).any():
            assert current_robot_state is not None, "current_robot_state must be provided when using delta actions."
            assert current_robot_state.shape == action.shape, "Shape of current_robot_state must match shape of teleop device output."
            if isinstance(self.config.use_deltas, bool):
                return action - current_robot_state
            else:
                assert self.config.use_deltas.shape == current_robot_state.shape, "Shape of TeleopConfig.use_deltas must match shape of current_robot_state."
                return np.where(self.config.use_deltas, action - current_robot_state, action)
        else:
            return action
    
