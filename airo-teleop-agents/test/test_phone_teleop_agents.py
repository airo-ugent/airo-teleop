"""Tests for the phone agent's precision ("sniper") mode, with a fake phone device and robot (no hardware)."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from airo_teleop_agents import phone_teleop_agents
from airo_teleop_agents.phone_teleop_agents import Phone4PositionManipulator

TCP = np.eye(4)
TCP[:3, 3] = [0.4, 0.1, 0.3]
TCP[:3, :3] = Rotation.from_euler("x", np.pi).as_matrix()


class FakePhoneDevice:
    def __init__(self, phone_config=None, auto_connect=True):
        self.last_raw_inputs = {}


class FakeRobot:
    def __init__(self, pose):
        self.pose = pose.copy()

    def get_tcp_pose(self):
        return self.pose.copy()


@pytest.fixture
def agent(monkeypatch):
    monkeypatch.setattr(phone_teleop_agents, "PhoneTeleopDevice", FakePhoneDevice)
    with pytest.warns(UserWarning):
        return Phone4PositionManipulator(
            FakeRobot(TCP),
            phone_config=None,
            translation_scale=1.0,
            rotation_scale=1.0,
            phone_forward_axis="-x",  # identity-like map up to the x/y sign flip
            enable_settle_time_s=0.0,
            max_translation_step_m=0.0,
            max_rotation_step_rad=0.0,
        )


def step(agent, pos=(0.0, 0.0, 0.0), rotvec=(0.0, 0.0, 0.0), enabled=True, precision=False):
    agent.teleop_device.last_raw_inputs = {"precision": precision}
    return agent.transform_func(np.array([*pos, *rotvec, 1.0 if enabled else 0.0]))


def test_precision_scales_translation_and_rotation(agent):
    step(agent, precision=True)  # enable, with precision on from the start
    pose = step(agent, pos=(0.0, 0.0, 0.04), rotvec=(0.0, 0.0, 0.4), precision=True)
    np.testing.assert_allclose(pose[:3, 3], TCP[:3, 3] + [0.0, 0.0, 0.01])
    expected_rot = Rotation.from_rotvec([0.0, 0.0, 0.1]).as_matrix() @ TCP[:3, :3]
    np.testing.assert_allclose(pose[:3, :3], expected_rot, atol=1e-9)


def test_switching_precision_does_not_move_and_restarts_the_stroke(agent):
    step(agent)
    moved = step(agent, pos=(0.0, 0.0, 0.08), rotvec=(0.0, 0.0, 0.2))
    np.testing.assert_allclose(moved[:3, 3], TCP[:3, 3] + [0.0, 0.0, 0.08])

    # switching on at the same phone pose: no jump
    switched = step(agent, pos=(0.0, 0.0, 0.08), rotvec=(0.0, 0.0, 0.2), precision=True)
    np.testing.assert_allclose(switched, moved, atol=1e-9)

    # further phone motion is scaled from the switch point
    fine = step(agent, pos=(0.0, 0.0, 0.12), rotvec=(0.0, 0.0, 0.6), precision=True)
    np.testing.assert_allclose(fine[:3, 3], moved[:3, 3] + [0.0, 0.0, 0.01])
    expected_rot = Rotation.from_rotvec([0.0, 0.0, 0.1]).as_matrix() @ moved[:3, :3]
    np.testing.assert_allclose(fine[:3, :3], expected_rot, atol=1e-9)

    # switching off again: no jump, full scale from there
    off = step(agent, pos=(0.0, 0.0, 0.12), rotvec=(0.0, 0.0, 0.6))
    np.testing.assert_allclose(off, fine, atol=1e-9)
    coarse = step(agent, pos=(0.0, 0.0, 0.14), rotvec=(0.0, 0.0, 0.6))
    np.testing.assert_allclose(coarse[:3, 3], fine[:3, 3] + [0.0, 0.0, 0.02])


def test_precision_ignored_when_disabled(monkeypatch, agent):
    agent.precision_scale = None
    step(agent, precision=True)
    pose = step(agent, pos=(0.0, 0.0, 0.04), precision=True)
    np.testing.assert_allclose(pose[:3, 3], TCP[:3, 3] + [0.0, 0.0, 0.04])


def test_invalid_precision_scale(monkeypatch):
    monkeypatch.setattr(phone_teleop_agents, "PhoneTeleopDevice", FakePhoneDevice)
    with pytest.raises(ValueError):
        Phone4PositionManipulator(FakeRobot(TCP), phone_config=None, precision_scale=0.0)
