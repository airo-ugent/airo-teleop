"""Tests for VrTeleopDevice and the Meta Quest reader without a headset: a fake reader, and a local subprocess in place
of `adb logcat` (no adb is ever started)."""

import sys
import threading

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip("ppadb")  # airo_teleop_vr imports it (nothing connects at import)

from airo_teleop_devices.vr_teleop_device import VrTeleopDevice, _pose_is_valid  # noqa: E402
from airo_teleop_vr.reader import MetaQuestReader, TeleopAppNotInstalledError, is_proper_transform  # noqa: E402

TAG = "wE9ryARX"


def pose(position=(0.1, 0.2, 1.0), rotvec=(0.0, 0.0, 0.3)):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    transform[:3, 3] = position
    return transform


class FakeReader:
    """Duck-typed MetaQuestReader: poses in ROS coordinates, set directly by the test."""

    def __init__(self, **kwargs):
        self.transforms = {"right": pose(), "left": None}
        self.trigger = {"right": 0.0, "left": 0.0}
        self.thread_alive = False
        self.age = 0.0

    def run(self):
        self.thread_alive = True

    def stop(self):
        self.thread_alive = False

    def pose_stream_age(self):
        return self.age

    def get_hand_controller_transform_ros(self, hand="right"):
        transform = self.transforms[hand]
        return None if transform is None else transform.copy()

    def get_trigger_value(self, hand="right"):
        return self.trigger[hand]

    def get_grip_value(self, hand="right"):
        return 0.0

    def get_joystick_value(self, hand="right"):
        return (0.0, 0.0)

    def get_button_state(self, button):
        return False


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


@pytest.fixture
def device():
    dev = VrTeleopDevice(auto_connect=False, max_frozen_s=1.0)
    dev._reader_cls = FakeReader
    dev._clock = Clock()
    dev.connect()
    yield dev
    dev.disconnect()


def test_valid_pose_is_reported(device):
    state = device.get_raw_state()
    assert state[26] == 1.0
    np.testing.assert_allclose(state[0:3], [0.1, 0.2, 1.0])
    np.testing.assert_allclose(state[3:6], [0.0, 0.0, 0.3], atol=1e-12)
    assert not np.any(state[6:12])  # the left hand has no pose


@pytest.mark.parametrize(
    "garbage",
    [
        "nan_rotation",
        "nan_translation",
        "inf",
        "reflection",
        "scaled",
        "zero",
        "identity",  # zero translation: what the app streams for an untracked controller
    ],
)
def test_garbage_matrices_are_an_invalid_hand_not_an_exception(device, garbage):
    matrix = pose()
    if garbage == "nan_rotation":
        matrix[0, 0] = np.nan
    elif garbage == "nan_translation":
        matrix[1, 3] = np.nan
    elif garbage == "inf":
        matrix[2, 2] = np.inf
    elif garbage == "reflection":
        matrix[:3, :3] = matrix[:3, :3] @ np.diag([1.0, 1.0, -1.0])
    elif garbage == "scaled":
        matrix[:3, :3] *= 1.5
    elif garbage == "zero":
        matrix = np.zeros((4, 4))
    elif garbage == "identity":
        matrix = np.eye(4)
    assert not _pose_is_valid(matrix)
    device._reader.transforms["right"] = matrix
    state = device.get_raw_state()  # must not raise
    assert state[26] == 0.0
    assert not np.any(state[0:6])
    # the other hand still counts
    device._reader.transforms["left"] = pose(position=(0.3, 0.0, 1.0))
    state = device.get_raw_state()
    assert state[26] == 1.0 and not np.any(state[0:6])
    np.testing.assert_allclose(state[6:9], [0.3, 0.0, 1.0])


def test_reader_side_validation_rejects_nan_and_reflections():
    assert is_proper_transform(pose())
    reflected = pose()
    reflected[:3, :3] = -reflected[:3, :3]  # det -1: abs(abs(det) - 1) == 0 passed the old check
    nan = pose()
    nan[0, 1] = np.nan
    for matrix in (reflected, nan, np.zeros((4, 4)), np.full((4, 4), np.nan)):
        assert not is_proper_transform(matrix)


def test_frozen_pose_invalidates_the_hand(device):
    clock = device._clock
    assert device.get_raw_state()[26] == 1.0
    clock.t = 0.9
    assert device.get_raw_state()[26] == 1.0  # unchanged, but not for long enough yet
    clock.t = 1.1
    state = device.get_raw_state()
    assert state[26] == 0.0 and not np.any(state[0:6])
    assert device.frozen_hands == ("right",)
    # real tracking jitters in the last digits: any change makes it live again
    device._reader.transforms["right"][0, 3] += 1e-9
    clock.t = 1.2
    assert device.get_raw_state()[26] == 1.0
    assert device.frozen_hands == ()
    # an analog value of the same hand changing also counts as alive
    clock.t = 2.1
    device._reader.trigger["right"] = 0.5
    assert device.get_raw_state()[26] == 1.0


def test_frozen_check_is_per_hand_and_can_be_disabled(device):
    clock = device._clock
    device._reader.transforms["left"] = pose(position=(0.3, 0.0, 1.0))
    device.get_raw_state()
    clock.t = 2.0
    device._reader.transforms["left"][0, 3] += 1e-6  # the left hand moves, the right one is frozen
    state = device.get_raw_state()
    assert state[26] == 1.0 and not np.any(state[0:6]) and np.any(state[6:12])
    assert device.frozen_hands == ("right",)

    device.max_frozen_s = None
    clock.t = 100.0
    state = device.get_raw_state()
    assert state[26] == 1.0 and np.any(state[0:6]) and np.any(state[6:12])


def test_dead_reader_thread_disables(device):
    assert device.get_raw_state()[26] == 1.0
    device._reader.thread_alive = False  # e.g. adb exited: the caches still hold a fresh-looking pose
    assert not device.reader_alive and not device.stream_alive
    assert device.get_raw_state()[26] == 0.0


def test_stale_stream_disables(device):
    device._reader.age = 1.0
    assert device.get_raw_state()[26] == 0.0


# --- the reader itself, with a local subprocess standing in for `adb logcat` ---


class _FakeAdbDevice:
    serial = "fake"

    def __init__(self, installed=True, am_start_output=""):
        self.installed = installed
        self.am_start_output = am_start_output

    def is_installed(self, name):
        return self.installed

    def shell(self, command):
        return self.am_start_output


def make_reader(monkeypatch, adb_device=None):
    adb_device = adb_device or _FakeAdbDevice()
    monkeypatch.setattr(MetaQuestReader, "get_device", lambda self: adb_device)
    return MetaQuestReader(run=False)


def matrix_tokens(matrix):
    return " ".join(f"{v:.6f}" for v in matrix.flatten())


def test_reader_thread_survives_malformed_lines(monkeypatch, capsys):
    reader = make_reader(monkeypatch)
    good = pose(position=(0.5, 0.0, 0.0))
    lines = [
        f"I {TAG}: r:{matrix_tokens(np.eye(4))[:-1]}x|&R,rightGrip 0.5",  # non-numeric token: ValueError
        f"I {TAG}:",  # ends right after the tag (stripped): was an IndexError
        f"I {TAG}: r:{matrix_tokens(np.eye(4))} 7 8 9|&R",  # too many values: IndexError
        f"I {TAG}: r:{matrix_tokens(np.eye(4))}|&R,rightGrip abc",  # bad button value: ValueError
        f"I {TAG}: r:{matrix_tokens(good)}|&R,rightGrip 0.25",  # finally a good one
    ]
    script = "import sys\nfor line in sys.argv[1:]:\n    print(line, flush=True)\n"
    reader.running = True
    reader.thread = threading.Thread(target=reader._read_logcat_subprocess, args=([sys.executable, "-c", script, *lines],))
    reader.thread.start()
    reader.thread.join(timeout=10)
    assert not reader.thread.is_alive()  # it ended at EOF, not on the first malformed line
    assert reader.parse_errors == 3  # the bare-tag line just carries no data
    assert reader.get_grip_value("right") == pytest.approx(0.25)
    np.testing.assert_allclose(reader._latest_transforms["r"], good, atol=1e-6)
    assert "Skipped malformed logcat line" in capsys.readouterr().err  # reported (once: rate-limited)
    # the logcat process exited: the thread is gone, and says so
    assert not reader.thread_alive
    reader.running = False


def test_reader_skips_nan_and_reflected_transforms(monkeypatch):
    reader = make_reader(monkeypatch)
    reflected = pose()
    reflected[:3, :3] = -reflected[:3, :3]
    nan = pose()
    nan[0, 0] = np.nan
    for matrix in (reflected, nan):
        reader._handle_line(f"I {TAG}: r:{matrix_tokens(matrix)}|&R".encode())
        assert "r" not in reader._latest_transforms
    reader._handle_line(f"I {TAG}: r:{matrix_tokens(pose())}|&R".encode())
    assert "r" in reader._latest_transforms


def test_thread_alive_before_run(monkeypatch):
    reader = make_reader(monkeypatch)
    assert not reader.thread_alive


def test_missing_app_message_is_not_rewritten_as_a_permission_error(monkeypatch, tmp_path):
    reader = make_reader(monkeypatch)
    reader.device.installed = False
    with pytest.raises(TeleopAppNotInstalledError, match="is not installed on the headset"):
        reader.install(APK_path=str(tmp_path / "missing.apk"))


def test_permission_errors_still_get_the_permission_hint(monkeypatch, capsys):
    reader = make_reader(monkeypatch)

    def no_access(name):
        raise RuntimeError("device unauthorized")

    reader.device.is_installed = no_access
    with pytest.raises(SystemExit):
        reader.install()
    assert "could not be accessed" in capsys.readouterr().err


@pytest.mark.parametrize(
    "output",
    [
        "Starting: Intent { cmp=com.wrong/com.rail.oculus.teleop.MainActivity }\n"
        "Error type 3\nError: Activity class {com.wrong/com.rail.oculus.teleop.MainActivity} does not exist.",
        "Error: Activity not started, unable to resolve Intent",
    ],
)
def test_run_raises_when_am_start_fails(monkeypatch, output):
    reader = make_reader(monkeypatch, _FakeAdbDevice(am_start_output=output))
    with pytest.raises(RuntimeError, match=r"com\.rail\.oculus\.teleop\.v2"):
        reader.run()
    assert not reader.running and not hasattr(reader, "thread")  # no logcat started


def test_run_check_accepts_a_normal_am_start_output(monkeypatch):
    reader = make_reader(monkeypatch)
    reader._check_am_start_output(
        "Starting: Intent { act=android.intent.action.MAIN cmp=com.rail.oculus.teleop.v2/"
        "com.rail.oculus.teleop.MainActivity }\nWarning: Activity not started, its current task has been brought to "
        "the front",
        "com.rail.oculus.teleop.v2/com.rail.oculus.teleop.MainActivity",
    )
