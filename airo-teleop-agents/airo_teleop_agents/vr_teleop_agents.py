"""Teleop agents for a Meta Quest VR headset (airo_teleop_devices.vr_teleop_device.VrTeleopDevice).

One hand controller drives one arm with a clutch:

* Hold the GRIP button to move. On pressing it, the controller pose and the robot's TCP pose are taken as the
  reference; while it is held, the TCP follows the controller's motion relative to that reference. Releasing it
  freezes the robot, so the hand can be repositioned (like lifting a mouse) before grabbing again.
* Translations and rotations are expressed in the robot base frame: moving the controller forward (in the headset's
  tracking frame) moves the TCP along `operator_forward_axis` of the robot base, and rotating the controller about
  the vertical rotates the TCP about the base's vertical. Recenter the headset (hold the Oculus/Meta button) while
  facing that robot axis, so the headset's forward matches it.
* Hold the TRIGGER for precision ("sniper") mode: the motion is scaled down by `precision_scale` (default 0.25) on
  top of the normal scales, for fine positioning; pressing or releasing it mid-stroke does not move the robot.
* The gripper (ParallelGripper variant) keeps the width it has at start; holding B (Y on the left controller) opens
  it step by step, holding A (X) closes it. So starting the agent never opens a gripper that holds something.
  (gripper_mode "analog" / "binary" use the trigger instead.)

Safety: the robot only moves while the device reports a fresh, valid pose (its `enabled` flag) AND the grip is held;
a pose stream that dies, goes invalid or freezes mid-motion freezes the robot at its last command. The first
`enable_settle_time_s` after engaging keeps re-taking the reference (no jump from a pose still settling), and each
update moves at most `max_translation_step_m` / `max_rotation_step_rad` from the previous command.
"""

import time
from typing import Callable, Literal, Sequence, Tuple

import numpy as np
from loguru import logger
from airo_robots.grippers.parallel_position_gripper import ParallelPositionGripper
from airo_robots.manipulators.position_manipulator import PositionManipulator
from airo_typing import HomogeneousMatrixType
from scipy.spatial.transform import Rotation

from airo_teleop_agents.teleop_agent import TeleopAgent
from airo_teleop_devices.vr_teleop_device import VrTeleopDevice

# indices into VrTeleopDevice's 27-dim raw state (see its docstring)
_POSE_INDEX = {"right": 0, "left": 6}
_TRIGGER_INDEX = {"right": 12, "left": 14}
_GRIP_INDEX = {"right": 13, "left": 15}
_BUTTON_INDEX = {"A": 20, "B": 21, "X": 22, "Y": 23}
_ENABLED_INDEX = 26

# yaw (about the vertical) from the headset's tracking frame (x forward, y left, z up) to the robot base frame
_FORWARD_AXIS_YAW = {"+x": 0.0, "+y": np.pi / 2.0, "-x": np.pi, "-y": -np.pi / 2.0}


def yaw_matrix(yaw: float) -> np.ndarray:
    c, s = float(np.cos(yaw)), float(np.sin(yaw))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def forward_axis_to_rotation(operator_forward_axis: str | float) -> np.ndarray:
    """Rotation from the headset tracking frame to the robot base frame.

    `operator_forward_axis`: the robot base axis the headset's forward points along ("+x", "-x", "+y", "-y"), or the
    yaw in radians of the headset's forward axis in the base frame."""
    if isinstance(operator_forward_axis, str):
        if operator_forward_axis not in _FORWARD_AXIS_YAW:
            raise ValueError(f"operator_forward_axis must be one of {sorted(_FORWARD_AXIS_YAW)} or a yaw in radians")
        return yaw_matrix(_FORWARD_AXIS_YAW[operator_forward_axis])
    return yaw_matrix(float(operator_forward_axis))


class VrClutchMapper:
    """The pure pose mapping of the VR agents (no device, no robot): raw state + current TCP pose -> target pose.

    Kept separate so it can be unit tested with synthetic raw states."""

    # while disengaged, a measured TCP pose more than this many step limits from the held command replaces it
    REBASE_FACTOR = 2.0

    def __init__(
        self,
        hand: Literal["right", "left"] = "right",
        operator_forward_axis: str | float = "+x",
        translation_scale: float = 1.0,
        rotation_scale: float = 1.0,
        grip_engage: float = 0.6,
        grip_release: float = 0.4,
        enable_settle_time_s: float = 0.2,
        max_translation_step_m: float = 0.03,
        max_rotation_step_rad: float = 0.3,
        enabled_axes: Sequence[bool] | None = None,
        precision_scale: float | None = 0.25,
        precision_engage: float = 0.6,
        precision_release: float = 0.4,
    ) -> None:
        if hand not in _POSE_INDEX:
            raise ValueError("hand must be 'right' or 'left'")
        if not 0.0 <= grip_release <= grip_engage <= 1.0:
            raise ValueError("need 0 <= grip_release <= grip_engage <= 1")
        self.hand = hand
        self.vr_to_base = forward_axis_to_rotation(operator_forward_axis)
        self.translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        self.grip_engage = grip_engage
        self.grip_release = grip_release
        self.enable_settle_time_s = enable_settle_time_s
        self.max_translation_step_m = max_translation_step_m
        self.max_rotation_step_rad = max_rotation_step_rad
        self.enabled_axes = np.asarray(enabled_axes if enabled_axes is not None else [True] * 6, dtype=bool)
        if self.enabled_axes.shape != (6,):
            raise ValueError("enabled_axes needs 6 entries: x, y, z, rx, ry, rz (robot base frame)")
        if precision_scale is not None and not 0.0 < precision_scale <= 1.0:
            raise ValueError("precision_scale must be in (0, 1], or None to disable precision mode")
        if not 0.0 <= precision_release <= precision_engage <= 1.0:
            raise ValueError("need 0 <= precision_release <= precision_engage <= 1")
        self.precision_scale = precision_scale
        self.precision_engage = precision_engage
        self.precision_release = precision_release
        self.reset()

    def reset(self) -> None:
        """Forget the clutch state: the next call holds the current TCP pose until the grip is pressed again."""
        self.engaged = False
        self.ever_engaged = False
        self._engage_time: float | None = None
        self._controller_reference: HomogeneousMatrixType | None = None
        self._tcp_reference: HomogeneousMatrixType | None = None
        self._last_command: HomogeneousMatrixType | None = None
        self.precision = False
        self._rebase_warned = False

    def controller_pose(self, raw_state: np.ndarray) -> HomogeneousMatrixType | None:
        """This hand's controller pose (headset frame), or None if the device has no valid pose for it."""
        if raw_state.shape[0] <= _ENABLED_INDEX or raw_state[_ENABLED_INDEX] < 0.5:
            return None
        start = _POSE_INDEX[self.hand]
        block = raw_state[start : start + 6]
        if not np.any(block):  # the device zeroes an invalid hand's block
            return None
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_rotvec(block[3:6]).as_matrix()
        pose[:3, 3] = block[:3]
        return pose

    def grip(self, raw_state: np.ndarray) -> float:
        """This hand's grip value, 0 for a raw state too short to be the device's 27-dim layout."""
        if raw_state.shape[0] <= _ENABLED_INDEX:
            return 0.0
        return float(raw_state[_GRIP_INDEX[self.hand]])

    def trigger(self, raw_state: np.ndarray) -> float:
        """This hand's trigger value in [0, 1], 0 for a raw state too short to be the device's 27-dim layout."""
        if raw_state.shape[0] <= _ENABLED_INDEX:
            return 0.0
        return float(np.clip(raw_state[_TRIGGER_INDEX[self.hand]], 0.0, 1.0))

    def __call__(
        self, raw_state: np.ndarray, current_tcp_pose: HomogeneousMatrixType, now: float | None = None
    ) -> HomogeneousMatrixType:
        now = time.monotonic() if now is None else now
        raw_state = np.asarray(raw_state, dtype=np.float64)
        if self._last_command is None:
            self._last_command = np.array(current_tcp_pose, dtype=np.float64)

        controller = self.controller_pose(raw_state)
        grip = self.grip(raw_state)
        threshold = self.grip_release if self.engaged else self.grip_engage
        engaged = controller is not None and grip >= threshold

        if not engaged:
            self.engaged = False
            self._rebase_if_moved_away(current_tcp_pose)
            return self._last_command.copy()

        rising_edge = not self.engaged
        if rising_edge:
            self.engaged = True
            self.ever_engaged = True
            self._rebase_warned = False
            self._engage_time = now
        if rising_edge or now - self._engage_time < self.enable_settle_time_s:
            self.precision = self._precision_held(raw_state)
            # settling: keep re-taking the reference, so the stroke starts from the pose after the settle time, and
            # from where the robot really is (not the last command, which the robot may not have reached)
            self._controller_reference = controller
            self._tcp_reference = np.array(current_tcp_pose, dtype=np.float64)
            self._last_command = self._tcp_reference.copy()
            return self._last_command.copy()

        precision = self._precision_held(raw_state)
        if precision != self.precision:
            # the scale changes: restart the stroke from here, so the switch itself does not move the robot
            self._controller_reference = controller
            self._tcp_reference = self._last_command.copy()
            self.precision = precision
        desired = self._map(controller)
        self._last_command = self._clamp_step(self._last_command, desired)
        return self._last_command.copy()

    def _rebase_if_moved_away(self, current_tcp_pose: HomogeneousMatrixType) -> None:
        """While disengaged: if the arm is far from the held command, hold where it is instead.

        Something else moved the arm (another controller, freedrive, a protective stop) without reset(): holding the
        old command would make the next action jump back to it. "Far" is more than REBASE_FACTOR step limits (which
        the robot never lags by in normal tracking); a step limit of 0 (no limit) disables that half of the check.
        """
        assert self._last_command is not None
        measured = np.asarray(current_tcp_pose, dtype=np.float64)
        distance = float(np.linalg.norm(measured[:3, 3] - self._last_command[:3, 3]))
        angle = float(np.linalg.norm(Rotation.from_matrix(measured[:3, :3] @ self._last_command[:3, :3].T).as_rotvec()))
        too_far = (self.max_translation_step_m > 0 and distance > self.REBASE_FACTOR * self.max_translation_step_m) or (
            self.max_rotation_step_rad > 0 and angle > self.REBASE_FACTOR * self.max_rotation_step_rad
        )
        if not too_far:
            return
        if not self._rebase_warned:
            logger.warning(
                f"VR teleop ({self.hand} hand): the arm is {distance * 100:.1f} cm / {np.degrees(angle):.0f} deg away "
                "from the held command (moved by something else?); holding its measured pose instead"
            )
            self._rebase_warned = True
        self._last_command = measured.copy()

    def _precision_held(self, raw_state: np.ndarray) -> bool:
        """Precision mode: the trigger is held (with hysteresis); always False if precision_scale is None."""
        if self.precision_scale is None:
            return False
        threshold = self.precision_release if self.precision else self.precision_engage
        return self.trigger(raw_state) >= threshold

    def _map(self, controller: HomogeneousMatrixType) -> HomogeneousMatrixType:
        reference, tcp_reference = self._controller_reference, self._tcp_reference
        assert reference is not None and tcp_reference is not None
        factor = self.precision_scale if (self.precision and self.precision_scale is not None) else 1.0
        translation = self.vr_to_base @ (controller[:3, 3] - reference[:3, 3]) * self.translation_scale * factor
        # the controller's rotation since the reference, in the headset frame, re-expressed in the base frame
        delta_vr = controller[:3, :3] @ reference[:3, :3].T
        rotvec = Rotation.from_matrix(self.vr_to_base @ delta_vr @ self.vr_to_base.T).as_rotvec()
        rotvec = rotvec * self.rotation_scale * factor
        translation = np.where(self.enabled_axes[:3], translation, 0.0)
        rotvec = np.where(self.enabled_axes[3:], rotvec, 0.0)
        desired = np.eye(4)
        desired[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix() @ tcp_reference[:3, :3]
        desired[:3, 3] = tcp_reference[:3, 3] + translation
        return desired

    def _clamp_step(self, previous: HomogeneousMatrixType, desired: HomogeneousMatrixType) -> HomogeneousMatrixType:
        out = desired.copy()
        step = desired[:3, 3] - previous[:3, 3]
        norm = float(np.linalg.norm(step))
        if self.max_translation_step_m > 0 and norm > self.max_translation_step_m:
            out[:3, 3] = previous[:3, 3] + step * (self.max_translation_step_m / norm)
        rotation_step = Rotation.from_matrix(desired[:3, :3] @ previous[:3, :3].T).as_rotvec()
        angle = float(np.linalg.norm(rotation_step))
        if self.max_rotation_step_rad > 0 and angle > self.max_rotation_step_rad:
            rotation_step *= self.max_rotation_step_rad / angle
            out[:3, :3] = Rotation.from_rotvec(rotation_step).as_matrix() @ previous[:3, :3]
        return out


class Vr4PositionManipulator(TeleopAgent):
    def __init__(
        self,
        position_manipulator: PositionManipulator,
        hand: Literal["right", "left"] = "right",
        operator_forward_axis: str | float = "+x",
        translation_scale: float = 1.0,
        rotation_scale: float = 1.0,
        enable_settle_time_s: float = 0.2,
        max_translation_step_m: float = 0.03,
        max_rotation_step_rad: float = 0.3,
        enabled_axes: Sequence[bool] | None = None,
        precision_scale: float | None = 0.25,
        vr_device: VrTeleopDevice | None = None,
        **vr_device_kwargs,
    ) -> None:
        """
        Teleop agent that maps one Meta Quest hand controller to a PositionManipulator TCP pose (hold grip to move).
        Returns absolute 4x4 TCP poses in the robot base frame; see the module docstring.

        :param position_manipulator: airo_robots PositionManipulator object, like URrtde
        :param hand: the controller that drives the arm, "right" or "left"
        :param operator_forward_axis: robot base axis the headset's forward points along ("+x", "-x", "+y", "-y"), or
            its yaw in radians in the base frame
        :param translation_scale: TCP translation per metre of controller translation
        :param rotation_scale: TCP rotation per radian of controller rotation
        :param enable_settle_time_s: seconds after pressing the grip during which the reference is re-taken
        :param max_translation_step_m: max translation per update in metres (0 = no limit)
        :param max_rotation_step_rad: max rotation per update in radians (0 = no limit)
        :param enabled_axes: 6 booleans for x, y, z, rx, ry, rz in the robot base frame
        :param precision_scale: precision ("sniper") mode: while the trigger is held, translation and rotation are
            scaled by this on top of the scales above (pressing/releasing it mid-stroke does not move the robot);
            None disables it
        :param vr_device: an existing VrTeleopDevice (e.g. shared between two arms); else one is created with
            `vr_device_kwargs` (ip_address, port, apk_name, max_pose_age, max_frozen_s, ...). close() only closes a
            device the agent created, so the caller closes a device it passed in
        """
        self.position_manipulator = position_manipulator
        self.mapper = VrClutchMapper(
            hand=hand,
            operator_forward_axis=operator_forward_axis,
            translation_scale=translation_scale,
            rotation_scale=rotation_scale,
            enable_settle_time_s=enable_settle_time_s,
            max_translation_step_m=max_translation_step_m,
            max_rotation_step_rad=max_rotation_step_rad,
            enabled_axes=enabled_axes,
            precision_scale=precision_scale,
        )
        # close() only closes a device this agent created: a passed-in one may be shared (e.g. between two arms)
        self._owns_device = vr_device is None
        device = vr_device if vr_device is not None else VrTeleopDevice(**vr_device_kwargs)
        super().__init__(teleop_device=device, transform_func=self._build_transform_func())

    def _build_transform_func(self) -> Callable:
        def transform_func(raw_data: np.ndarray) -> HomogeneousMatrixType:
            return self.mapper(raw_data, self.position_manipulator.get_tcp_pose())

        return transform_func

    def reset(self) -> None:
        self.mapper.reset()

    def close(self) -> None:
        """Close the VR device if this agent created it; a device passed in as `vr_device` is the caller's."""
        if self._owns_device:
            self.teleop_device.close()


class Vr4PositionManipulator_ParallelGripper(Vr4PositionManipulator):
    def __init__(
        self,
        position_manipulator: PositionManipulator,
        gripper: ParallelPositionGripper,
        hand: Literal["right", "left"] = "right",
        gripper_mode: Literal["buttons", "analog", "binary"] = "buttons",
        gripper_step_m: float = 0.002,
        gripper_max_lead_m: float = 0.003,
        **kwargs,
    ) -> None:
        """
        As Vr4PositionManipulator, plus the gripper on the same controller. Returns a
        (4x4 TCP pose, gripper width in metres) tuple.

        :param gripper: airo_robots ParallelPositionGripper object, like Robotiq2F85 or SchunkEGK40
        :param gripper_mode:
            "buttons" (default): the gripper starts at the width it has and changes only while a button is held: the
            upper button (B on the right controller, Y on the left) opens it, the lower one (A / X) closes it. The
            target moves `gripper_step_m` per update (smooth motion) but never more than `gripper_max_lead_m` beyond
            the measured width, so fingers blocked by an object squeeze at most that much, and stop that close after
            the button is released. The trigger is the precision (slow motion) button, see precision_scale.
            "analog": the width follows the trigger (released = max width, pressed = min width), after the first grip
            press. "binary": as analog, trigger past half = min width, else max width. These two leave no button for
            precision mode, so it is off.
        :param gripper_step_m: width change per update in "buttons" mode
        :param gripper_max_lead_m: "buttons" mode: how far the target may run ahead of the measured width
        """
        if gripper_mode not in ("buttons", "analog", "binary"):
            raise ValueError("gripper_mode must be 'buttons', 'analog' or 'binary'")
        if gripper_mode == "buttons" and gripper_step_m <= 0:
            raise ValueError("gripper_step_m must be positive")
        if gripper_mode == "buttons" and gripper_max_lead_m <= 0:
            # with a lead <= 0 the target can never move past the fingers: the buttons would silently do nothing
            raise ValueError("gripper_max_lead_m must be positive")
        self.gripper = gripper
        self.gripper_mode = gripper_mode
        self.gripper_step_m = gripper_step_m
        self.gripper_max_lead_m = gripper_max_lead_m
        self._gripper_target: float | None = None
        open_button, close_button = ("B", "A") if hand == "right" else ("Y", "X")
        self._open_index = _BUTTON_INDEX[open_button]
        self._close_index = _BUTTON_INDEX[close_button]
        if gripper_mode != "buttons":
            if kwargs.get("precision_scale") is not None:
                raise ValueError(f"gripper_mode {gripper_mode!r} uses the trigger: precision mode is not available")
            kwargs["precision_scale"] = None  # the trigger drives the gripper
        super().__init__(position_manipulator, hand=hand, **kwargs)

    def reset(self) -> None:
        super().reset()
        self._gripper_target = None

    def gripper_width(self, trigger: float) -> float:
        specs = self.gripper.gripper_specs
        if self.gripper_mode == "binary":
            return specs.min_width if trigger > 0.5 else specs.max_width
        return specs.max_width - trigger * (specs.max_width - specs.min_width)

    def button_gripper_width(self, raw_data: np.ndarray) -> float:
        """'buttons' mode: hold the target; move it while open or close is held, at most max_lead past the fingers."""
        if self._gripper_target is None:
            self._gripper_target = float(self.gripper.get_current_width())
        if self.mapper.controller_pose(raw_data) is None:
            return self._gripper_target  # stale / invalid stream: its cached buttons may be stale too
        opening = raw_data[self._open_index] > 0.5
        closing = raw_data[self._close_index] > 0.5
        if opening != closing:
            specs = self.gripper.gripper_specs
            measured = float(self.gripper.get_current_width())
            if opening:
                target = min(max(self._gripper_target, measured) + self.gripper_step_m, measured + self.gripper_max_lead_m)
            else:
                target = max(min(self._gripper_target, measured) - self.gripper_step_m, measured - self.gripper_max_lead_m)
            self._gripper_target = float(np.clip(target, specs.min_width, specs.max_width))
        return self._gripper_target

    def _build_transform_func(self) -> Callable:
        pose_func = super()._build_transform_func()

        def transform_func(raw_data: np.ndarray) -> Tuple[HomogeneousMatrixType, float]:
            raw_data = np.asarray(raw_data, dtype=np.float64)
            pose = pose_func(raw_data)
            if self.gripper_mode == "buttons":
                return pose, self.button_gripper_width(raw_data)
            if not self.mapper.ever_engaged or self.mapper.controller_pose(raw_data) is None:
                # before the first grip press, or without a valid controller: leave the gripper where it is
                return pose, self.gripper.get_current_width()
            return pose, self.gripper_width(self.mapper.trigger(raw_data))

        return transform_func
