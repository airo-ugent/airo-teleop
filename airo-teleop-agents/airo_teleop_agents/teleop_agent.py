from dataclasses import dataclass
from airo_teleop_devices.teleop_device import TeleopDevice
import numpy as np
import numpy.typing as npt
from airo_typing import HomogeneousMatrixType


class TeleopAgent:
    '''
    Takes the raw state output of a TeleopDevice and transforms it to a robot action using the provided transform_func.
    The transform_func is expected to be set up properly in accordance with the TeleopConfig (e.g., joint space vs Cartesian space).
    Use of deltas is handled in get_action, based on the setting in the TeleopConfig.
    '''
    def __init__(self, teleop_device: TeleopDevice, transform_func=lambda x: x):
        self.transform_func = transform_func
        self.teleop_device = teleop_device

    def get_action(self) -> npt.NDArray[np.float64] | tuple[HomogeneousMatrixType, float]:
        teleop_device_state = self.teleop_device.get_raw_state()
        action = self.transform_func(teleop_device_state)
        return action
