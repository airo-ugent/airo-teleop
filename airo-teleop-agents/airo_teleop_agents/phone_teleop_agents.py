from typing import Callable, Sequence
import time
import warnings

import numpy as np

from airo_teleop_agents.teleop_agent import TeleopAgent
from airo_teleop_devices.phone_teleop_device import PhoneTeleopDevice
from airo_robots.manipulators.position_manipulator import PositionManipulator
from airo_typing import HomogeneousMatrixType
from airo_teleop_phone.rotation import Rotation


class Phone4PositionManipulator(TeleopAgent):
    def __init__(
        self,
        position_manipulator: PositionManipulator,
        phone_config,
        translation_scale: float = 0.5,
        rotation_scale: float = 1.0,
        phone_forward_axis: str = "-x",
        enable_settle_time_s: float = 0.25,
        max_translation_step_m: float = 0.03,
        max_rotation_step_rad: float = 0.35,
        enabled_axes: Sequence[bool] | None = None,
        auto_connect: bool = True,
    ) -> None:
        """
        Teleop agent that maps a phone's calibrated pose to a PositionManipulator TCP pose.

        :param position_manipulator: airo_robots PositionManipulator object, like URrtde
        :param phone_config: airo_teleop_phone.PhoneConfig instance
        :param translation_scale: Scale factor for translation deltas in meters
        :param rotation_scale: Scale factor for rotation deltas in radians
        :param phone_forward_axis: Axis of the base frame the phone top points to at init: "+x", "+y", "-x", "-y"
        :param enable_settle_time_s: Seconds to ignore updates after enable to avoid pose jumps
        :param max_translation_step_m: Max translation step per update in meters
        :param max_rotation_step_rad: Max rotation step per update in radians
        :param enabled_axes: Boolean array for enabling xyz and rpy axes
        :param auto_connect: If True, connect and calibrate during initialization
        """
        self.position_manipulator = position_manipulator
        self.translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        self.enabled_axes = list(enabled_axes) if enabled_axes is not None else [True] * 6
        self.phone_forward_axis = phone_forward_axis
        self.enable_settle_time_s = enable_settle_time_s
        self.max_translation_step_m = max_translation_step_m
        self.max_rotation_step_rad = max_rotation_step_rad

        warnings.warn(
            "Do not cover the phone camera with your fingers during teleop/calibration.",
            UserWarning,
        )

        self._translation_map, self._rotation_map = self._build_axis_maps(phone_forward_axis)

        self._reference_pose: HomogeneousMatrixType | None = None
        self._command_when_disabled: HomogeneousMatrixType | None = None
        self._prev_enabled = False
        self._enable_time_s: float | None = None
        self._last_command_pose: HomogeneousMatrixType | None = None

        phone_device = PhoneTeleopDevice(phone_config=phone_config, auto_connect=auto_connect)
        super().__init__(teleop_device=phone_device, transform_func=self._build_transform_func())

    def _build_transform_func(self) -> Callable:
        def transform_func(raw_data: np.ndarray) -> HomogeneousMatrixType:
            if raw_data.shape[0] < 7:
                return self.position_manipulator.get_tcp_pose()
            # print(raw_data)
            pos = raw_data[:3]
            rotvec = raw_data[3:6]
            enabled = bool(raw_data[6] > 0.5)

            pos = np.where(self.get_enabled_axes()[:3], pos, 0.0)
            rotvec = np.where(self.get_enabled_axes()[3:6], rotvec, 0.0)

            if enabled:
                current_pose = self.position_manipulator.get_tcp_pose()
                if not self._prev_enabled or self._reference_pose is None:
                    self._reference_pose = current_pose.copy()
                ref_pose = self._reference_pose if self._reference_pose is not None else current_pose

                if not self._prev_enabled:
                    self._enable_time_s = time.monotonic()
                    self._last_command_pose = ref_pose.copy()
              
                mapped_translation = (self._translation_map @ pos) * self.translation_scale
                mapped_rotvec = (self._rotation_map @ rotvec) * self.rotation_scale

                delta_rot = Rotation.from_rotvec(mapped_rotvec).as_matrix()
                desired = np.eye(4, dtype=float)
                desired[:3, :3] = delta_rot @ ref_pose[:3, :3]
                desired[:3, 3] = ref_pose[:3, 3] + mapped_translation

                if self._enable_time_s is not None:
                    time_since_enable = time.monotonic() - self._enable_time_s
                    if time_since_enable < self.enable_settle_time_s:
                        desired = (
                            self._last_command_pose.copy()
                            if self._last_command_pose is not None
                            else ref_pose.copy()
                        )
                    else:
                        self._enable_time_s = None

                if self._last_command_pose is not None:
                    translation_delta = desired[:3, 3] - self._last_command_pose[:3, 3]
                    delta_norm = float(np.linalg.norm(translation_delta))
                    if self.max_translation_step_m > 0 and delta_norm > self.max_translation_step_m:
                        desired[:3, 3] = self._last_command_pose[:3, 3] + (
                            translation_delta * (self.max_translation_step_m / delta_norm)
                        )

                    rotation_delta = desired[:3, :3] @ self._last_command_pose[:3, :3].T
                    rotation_delta_rotvec = Rotation.from_matrix(rotation_delta).as_rotvec()
                    rotation_delta_angle = float(np.linalg.norm(rotation_delta_rotvec))
                    if self.max_rotation_step_rad > 0 and rotation_delta_angle > self.max_rotation_step_rad:
                        rotation_delta_rotvec = rotation_delta_rotvec * (
                            self.max_rotation_step_rad / rotation_delta_angle
                        )
                        rotation_delta_clamped = Rotation.from_rotvec(rotation_delta_rotvec).as_matrix()
                        desired[:3, :3] = rotation_delta_clamped @ self._last_command_pose[:3, :3]

                self._last_command_pose = desired.copy()
                self._command_when_disabled = desired.copy()
            else:
                if self._command_when_disabled is None:
                    self._command_when_disabled = self.position_manipulator.get_tcp_pose().copy()
                desired = self._command_when_disabled.copy()
                self._last_command_pose = desired.copy()
                self._enable_time_s = None

            self._prev_enabled = enabled
            return desired

        return transform_func

    @staticmethod
    def _build_axis_maps(phone_forward_axis: str) -> tuple[np.ndarray, np.ndarray]:
        axis_to_yaw = {
            "-x": 0.0,
            "+x": np.pi,
            "+y": -np.pi / 2.0,
            "-y": np.pi / 2.0,
        }
        if phone_forward_axis not in axis_to_yaw:
            raise ValueError(
                "phone_forward_axis must be one of '+x', '+y', '-x', '-y'."
            )

        yaw = axis_to_yaw[phone_forward_axis]
        cos_yaw = float(np.cos(yaw))
        sin_yaw = float(np.sin(yaw))
        yaw_rot = np.array(
            [
                [cos_yaw, -sin_yaw, 0.0],
                [sin_yaw, cos_yaw, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        # Base mapping when phone top points toward -x in the base frame.
        translation_map_base = np.array(
            [
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        rotation_map_base = np.array(
            [
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        translation_map = yaw_rot @ translation_map_base
        rotation_map = yaw_rot @ rotation_map_base
        return translation_map, rotation_map

    def get_enabled_axes(self) -> list[bool]:
        return self.enabled_axes
