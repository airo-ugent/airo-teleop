from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
from typing import Dict, Optional, List
from loguru import logger
from airo_teleop_devices.drivers.dynamixel_driver import DynamixelDriver, FakeDynamixelDriver


@dataclass
class DynamixelConfig:
    joint_ids: npt.NDArray[np.int8]
    joint_offsets: npt.NDArray[np.float64]
    joint_signs: npt.NDArray[np.int8]
    start_joints: Optional[npt.NDArray[np.float64]] = None

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets) == len(self.joint_signs)


class DynamixelRobot:
    """A class representing a dynamixel robot."""

    def __init__(self,
            dynamixel_config: DynamixelConfig,
            real: bool = True,
            port: str = "/dev/ttyUSB0",
            baudrate: int = 57600):
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99
        self._joint_ids = dynamixel_config.joint_ids
        logger.debug(f"Attempting to connect to port: {port}")
        if real:
            self._driver = DynamixelDriver(self._joint_ids, port=port, baudrate=baudrate)
            self._driver.set_torque_mode(False)
        else:
            self._driver = FakeDynamixelDriver(self._joint_ids)

        self.dynamixel_config = dynamixel_config
        joint_signs = dynamixel_config.joint_signs
        if joint_signs is None:
            self._joint_signs = np.ones(len(self._joint_ids))
        else:
            self._joint_signs = np.array(joint_signs)
        self.init_joint_offsets()
        assert len(self._joint_ids) == len(self._joint_offsets), (
            f"joint_ids: {len(self._joint_ids)}, " f"joint_offsets: {len(self._joint_offsets)}"
        )
        assert len(self._joint_ids) == len(self._joint_signs), (
            f"joint_ids: {len(self._joint_ids)}, " f"joint_signs: {len(self._joint_signs)}"
        )
        assert np.all(np.abs(self._joint_signs) == 1), f"joint_signs: {self._joint_signs}"

    def init_joint_offsets(self) -> None:
        joint_ids = self.dynamixel_config.joint_ids
        joint_offsets = self.dynamixel_config.joint_offsets
        start_joints = self.dynamixel_config.start_joints

        if joint_offsets is None:
            self._joint_offsets = np.zeros(len(joint_ids))
        else:
            self._joint_offsets = np.array(joint_offsets)

        if start_joints is not None:
            # loop through all joints and add +- 2pi to the joint offsets to get the closest to start joints
            current_joints = self.get_joint_state()
            new_joint_offsets = np.zeros(np.array(current_joints).shape)
            assert len(current_joints) == len(start_joints), f"current_joints: {len(current_joints)}, start_joints: {len(start_joints)}"
            for idx, (c_joint, s_joint, joint_offset) in enumerate(
                zip(current_joints, start_joints, self._joint_offsets)
            ):
                if s_joint is not None:
                    new_joint_offsets[idx] = (
                        np.pi * 2 * np.round((-s_joint + c_joint) / (2 * np.pi)) * self._joint_signs[idx] + joint_offset
                    )
            self._joint_offsets = np.array(new_joint_offsets)


    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()

        if self._last_pos is None:
            self._last_pos = pos
        else:
            # exponential smoothing
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        self._driver.set_joints((joint_state + self._joint_offsets).tolist())

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}


if __name__ == "__main__":
    dynamixel_config = DynamixelConfig(
        joint_ids=[1, 2, 3, 4, 5, 6, 7],
        joint_offsets=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        joint_signs=[1, 1, -1, 1, 1, 1, 1],
        start_joints=None,
    )
    robot = DynamixelRobot(
        dynamixel_config=dynamixel_config,
        port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792DZ5-if00-port0",
        baudrate=57600,
        real=True,
    )
    print(robot.get_joint_state())
