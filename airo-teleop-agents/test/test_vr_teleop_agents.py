"""Tests for the VR clutch mapping and the gripper mapping, with synthetic raw states (no headset, no robot)."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from airo_teleop_agents.vr_teleop_agents import (
    VrClutchMapper,
    Vr4PositionManipulator_ParallelGripper,
    forward_axis_to_rotation,
)

TCP = np.eye(4)
TCP[:3, 3] = [0.4, 0.1, 0.3]
TCP[:3, :3] = Rotation.from_euler("x", np.pi).as_matrix()  # tool pointing down


def raw_state(position=(0.0, 0.0, 1.0), rotvec=(0.0, 0.0, 0.0), grip=1.0, trigger=0.0, enabled=True, hand="right"):
    state = np.zeros(27)
    start = 0 if hand == "right" else 6
    state[start : start + 3] = position
    state[start + 3 : start + 6] = rotvec
    state[13 if hand == "right" else 15] = grip
    state[12 if hand == "right" else 14] = trigger
    state[26] = 1.0 if enabled else 0.0
    return state


def mapper(**kwargs):
    defaults = dict(
        operator_forward_axis="+x", enable_settle_time_s=0.0, max_translation_step_m=0.0, max_rotation_step_rad=0.0
    )
    return VrClutchMapper(**{**defaults, **kwargs})


def test_no_motion_without_grip_or_enabled():
    m = mapper()
    for state in (raw_state(grip=0.0), raw_state(enabled=False), raw_state(position=(0, 0, 0), grip=1.0)):
        np.testing.assert_allclose(m(state, TCP, now=0.0), TCP)
    assert not m.ever_engaged  # an all-zero pose block is an invalid hand, not a pose


def test_translation_follows_controller_in_base_frame():
    m = mapper(operator_forward_axis="+x")
    m(raw_state(position=(0.0, 0.0, 1.0)), TCP, now=0.0)  # engage: reference taken
    target = m(raw_state(position=(0.1, 0.02, 1.0)), TCP, now=0.1)
    np.testing.assert_allclose(target[:3, 3], TCP[:3, 3] + [0.1, 0.02, 0.0], atol=1e-12)
    np.testing.assert_allclose(target[:3, :3], TCP[:3, :3], atol=1e-12)


def test_forward_axis_rotates_the_mapping():
    m = mapper(operator_forward_axis="-y", translation_scale=0.5)
    m(raw_state(position=(0.0, 0.0, 1.0)), TCP, now=0.0)
    target = m(raw_state(position=(0.2, 0.0, 1.0)), TCP, now=0.1)  # 20 cm forward in the headset frame
    np.testing.assert_allclose(target[:3, 3], TCP[:3, 3] + [0.0, -0.1, 0.0], atol=1e-12)


def test_rotation_is_applied_in_the_base_frame():
    m = mapper()
    m(raw_state(rotvec=(0.0, 0.0, 0.0)), TCP, now=0.0)
    target = m(raw_state(rotvec=(0.0, 0.0, 0.3)), TCP, now=0.1)  # controller yaws 0.3 rad about vertical
    expected = Rotation.from_rotvec([0.0, 0.0, 0.3]).as_matrix() @ TCP[:3, :3]
    np.testing.assert_allclose(target[:3, :3], expected, atol=1e-12)
    np.testing.assert_allclose(target[:3, 3], TCP[:3, 3], atol=1e-12)


def test_clutch_release_freezes_and_regrab_starts_from_measured_pose():
    m = mapper()
    m(raw_state(position=(0.0, 0.0, 1.0)), TCP, now=0.0)
    moved = m(raw_state(position=(0.1, 0.0, 1.0)), TCP, now=0.1)
    # release and move the hand back: the target stays
    frozen = m(raw_state(position=(0.0, 0.0, 1.0), grip=0.0), TCP, now=0.2)
    np.testing.assert_allclose(frozen, moved)
    # re-grab: the new stroke starts at the robot's measured pose
    measured = moved.copy()
    measured[0, 3] -= 0.005  # the robot lags 5 mm behind the command
    start = m(raw_state(position=(0.0, 0.0, 1.0)), measured, now=0.3)
    np.testing.assert_allclose(start, measured)
    target = m(raw_state(position=(0.05, 0.0, 1.0)), measured, now=0.4)
    np.testing.assert_allclose(target[:3, 3], measured[:3, 3] + [0.05, 0.0, 0.0], atol=1e-12)


def test_grip_hysteresis():
    m = mapper(grip_engage=0.6, grip_release=0.4)
    m(raw_state(grip=0.5), TCP, now=0.0)
    assert not m.engaged  # below the engage threshold
    m(raw_state(grip=0.7), TCP, now=0.1)
    m(raw_state(grip=0.5), TCP, now=0.2)
    assert m.engaged  # above the release threshold
    m(raw_state(grip=0.3), TCP, now=0.3)
    assert not m.engaged


def test_settle_time_retakes_the_reference():
    m = mapper(enable_settle_time_s=0.2)
    m(raw_state(position=(0.0, 0.0, 1.0)), TCP, now=0.0)
    # the controller pose still moves during the settle time: no motion, reference follows
    np.testing.assert_allclose(m(raw_state(position=(0.05, 0.0, 1.0)), TCP, now=0.1), TCP)
    target = m(raw_state(position=(0.06, 0.0, 1.0)), TCP, now=0.3)
    np.testing.assert_allclose(target[:3, 3], TCP[:3, 3] + [0.01, 0.0, 0.0], atol=1e-12)


def test_step_limits():
    m = mapper(max_translation_step_m=0.01, max_rotation_step_rad=0.05)
    m(raw_state(), TCP, now=0.0)
    target = m(raw_state(position=(0.5, 0.0, 1.0), rotvec=(0.0, 0.0, 1.0)), TCP, now=0.1)
    assert np.linalg.norm(target[:3, 3] - TCP[:3, 3]) == pytest.approx(0.01)
    angle = np.linalg.norm(Rotation.from_matrix(target[:3, :3] @ TCP[:3, :3].T).as_rotvec())
    assert angle == pytest.approx(0.05)


def test_enabled_axes_mask_base_frame_axes():
    m = mapper(enabled_axes=[True, True, False, False, False, True])
    m(raw_state(), TCP, now=0.0)
    target = m(raw_state(position=(0.1, 0.1, 1.1), rotvec=(0.2, 0.0, 0.2)), TCP, now=0.1)
    np.testing.assert_allclose(target[:3, 3], TCP[:3, 3] + [0.1, 0.1, 0.0], atol=1e-12)
    expected = Rotation.from_rotvec([0.0, 0.0, 0.2]).as_matrix() @ TCP[:3, :3]
    np.testing.assert_allclose(target[:3, :3], expected, atol=1e-12)


def test_left_hand_reads_its_own_block():
    m = mapper(hand="left")
    m(raw_state(hand="left"), TCP, now=0.0)
    right_only = raw_state(position=(0.3, 0.0, 1.0))  # right hand moves, left block is zero = invalid
    np.testing.assert_allclose(m(right_only, TCP, now=0.1), TCP)


def test_forward_axis_validation():
    with pytest.raises(ValueError):
        forward_axis_to_rotation("z")
    np.testing.assert_allclose(forward_axis_to_rotation(np.pi / 2), forward_axis_to_rotation("+y"), atol=1e-12)


class _Specs:
    min_width, max_width = 0.0, 0.08


class _FakeGripper:
    gripper_specs = _Specs()

    def get_current_width(self):
        return 0.03


class _FakeRobot:
    def get_tcp_pose(self):
        return TCP.copy()


class _FakeDevice:
    def __init__(self):
        self.state = raw_state(grip=0.0, trigger=0.0)

    def get_raw_state(self):
        return self.state

    def close(self):
        pass


@pytest.mark.parametrize("mode, trigger, width", [("analog", 0.25, 0.06), ("binary", 0.4, 0.08), ("binary", 0.6, 0.0)])
def test_gripper_follows_trigger_only_after_first_grip(mode, trigger, width):
    device = _FakeDevice()
    agent = Vr4PositionManipulator_ParallelGripper(
        _FakeRobot(), _FakeGripper(), gripper_mode=mode, vr_device=device, enable_settle_time_s=0.0
    )
    device.state = raw_state(grip=0.0, trigger=trigger)
    pose, gripper = agent.get_action()
    assert gripper == 0.03  # never engaged: the gripper stays where it is
    device.state = raw_state(grip=1.0, trigger=trigger)
    agent.get_action()
    device.state = raw_state(grip=0.0, trigger=trigger)  # released clutch: the trigger still drives the gripper
    pose, gripper = agent.get_action()
    assert gripper == pytest.approx(width)
    np.testing.assert_allclose(pose, TCP)


class _WidthGripper(_FakeGripper):
    def __init__(self, width):
        self.width = width

    def get_current_width(self):
        return self.width


def _buttons(**pressed):
    state = raw_state(grip=0.0)
    for name, index in {"A": 20, "B": 21, "X": 22, "Y": 23}.items():
        state[index] = 1.0 if pressed.get(name) else 0.0
    return state


def test_buttons_mode_keeps_the_held_width_and_moves_smoothly_with_a_bounded_lead():
    device, gripper = _FakeDevice(), _WidthGripper(0.03)
    agent = Vr4PositionManipulator_ParallelGripper(
        _FakeRobot(), gripper, vr_device=device, gripper_step_m=0.002, gripper_max_lead_m=0.003,
        enable_settle_time_s=0.0,
    )  # fmt: skip
    device.state = _buttons()
    assert agent.get_action()[1] == pytest.approx(0.03)  # starts at the width it holds, nothing pressed
    device.state = raw_state(grip=1.0, trigger=0.0)  # grip + released trigger: must NOT open in buttons mode
    assert agent.get_action()[1] == pytest.approx(0.03)
    device.state = _buttons(A=True)  # close
    assert agent.get_action()[1] == pytest.approx(0.028)
    gripper.width = 0.029  # the fingers follow, lagging: the target keeps moving at its own pace
    assert agent.get_action()[1] == pytest.approx(0.026)
    # the fingers are blocked by the plug at 0.029: the target stops max_lead below them
    assert agent.get_action()[1] == pytest.approx(0.026)
    assert agent.get_action()[1] == pytest.approx(0.026)
    device.state = _buttons()  # released: the target holds
    assert agent.get_action()[1] == pytest.approx(0.026)
    device.state = _buttons(B=True)  # open: starts from the fingers, not from the squeezed target
    assert agent.get_action()[1] == pytest.approx(0.031)
    gripper.width = 0.0795
    agent._gripper_target = 0.0795
    assert agent.get_action()[1] == pytest.approx(0.08)  # clamped to max_width
    device.state = _buttons(A=True, B=True)  # both: no change
    assert agent.get_action()[1] == pytest.approx(0.08)
    device.state = _buttons(A=True)
    device.state[26] = 0.0  # stale stream: hold the target
    assert agent.get_action()[1] == pytest.approx(0.08)


def test_buttons_mode_left_hand_uses_x_and_y():
    device, gripper = _FakeDevice(), _WidthGripper(0.03)
    agent = Vr4PositionManipulator_ParallelGripper(
        _FakeRobot(), gripper, hand="left", vr_device=device, gripper_step_m=0.002
    )
    device.state = _buttons(A=True)  # right-hand button: ignored
    device.state[6:9] = [0.0, 0.0, 1.0]  # a valid left pose
    assert agent.get_action()[1] == pytest.approx(0.03)
    device.state = _buttons(Y=True)
    device.state[6:9] = [0.0, 0.0, 1.0]
    assert agent.get_action()[1] == pytest.approx(0.032)


def test_precision_mode_scales_motion_without_a_jump_on_switching():
    m = mapper(precision_scale=0.25)
    m(raw_state(position=(0.0, 0.0, 1.0)), TCP, now=0.0)
    normal = m(raw_state(position=(0.1, 0.0, 1.0)), TCP, now=0.1)
    np.testing.assert_allclose(normal[:3, 3], TCP[:3, 3] + [0.1, 0.0, 0.0], atol=1e-12)
    # press the trigger at the same hand pose: nothing moves
    switched = m(raw_state(position=(0.1, 0.0, 1.0), trigger=1.0), TCP, now=0.2)
    np.testing.assert_allclose(switched, normal, atol=1e-12)
    assert m.precision
    # 8 cm more hand motion -> 2 cm robot motion; rotation scaled too
    fine = m(raw_state(position=(0.18, 0.0, 1.0), rotvec=(0.0, 0.0, 0.4), trigger=1.0), TCP, now=0.3)
    np.testing.assert_allclose(fine[:3, 3], normal[:3, 3] + [0.02, 0.0, 0.0], atol=1e-12)
    expected = Rotation.from_rotvec([0.0, 0.0, 0.1]).as_matrix() @ TCP[:3, :3]
    np.testing.assert_allclose(fine[:3, :3], expected, atol=1e-9)
    # release the trigger: no jump, then full scale again
    released = m(raw_state(position=(0.18, 0.0, 1.0), rotvec=(0.0, 0.0, 0.4), trigger=0.0), TCP, now=0.4)
    np.testing.assert_allclose(released, fine, atol=1e-12)
    later = m(raw_state(position=(0.28, 0.0, 1.0), rotvec=(0.0, 0.0, 0.4), trigger=0.0), TCP, now=0.5)
    np.testing.assert_allclose(later[:3, 3], fine[:3, 3] + [0.1, 0.0, 0.0], atol=1e-12)


def test_precision_mode_held_from_the_start_of_a_stroke():
    m = mapper(precision_scale=0.5)
    m(raw_state(position=(0.0, 0.0, 1.0), trigger=1.0), TCP, now=0.0)
    target = m(raw_state(position=(0.1, 0.0, 1.0), trigger=1.0), TCP, now=0.1)
    np.testing.assert_allclose(target[:3, 3], TCP[:3, 3] + [0.05, 0.0, 0.0], atol=1e-12)


def test_precision_mode_off_when_disabled_or_the_trigger_drives_the_gripper():
    m = mapper(precision_scale=None)
    m(raw_state(trigger=1.0), TCP, now=0.0)
    target = m(raw_state(position=(0.1, 0.0, 1.0), trigger=1.0), TCP, now=0.1)
    np.testing.assert_allclose(target[:3, 3], TCP[:3, 3] + [0.1, 0.0, 0.0], atol=1e-12)
    agent = Vr4PositionManipulator_ParallelGripper(_FakeRobot(), _FakeGripper(), gripper_mode="analog",
                                                   vr_device=_FakeDevice())  # fmt: skip
    assert agent.mapper.precision_scale is None
    with pytest.raises(ValueError, match="precision mode"):
        Vr4PositionManipulator_ParallelGripper(
            _FakeRobot(), _FakeGripper(), gripper_mode="binary", precision_scale=0.25, vr_device=_FakeDevice()
        )
