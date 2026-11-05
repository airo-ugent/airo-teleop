import time
from typing import Any, Literal

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation

from airo_teleop_devices.teleop_device import TeleopDevice

# the only button keys the .v2 log format emits (see airo_teleop_vr.buttons_parser)
BUTTONS = ("A", "B", "X", "Y", "LJ", "RJ")

# (array offset, hand) pairs for the pose and joystick blocks of the raw state
_POSE_OFFSETS: tuple[tuple[int, Literal["left", "right"]], ...] = ((0, "right"), (6, "left"))
_JOYSTICK_OFFSETS: tuple[tuple[int, Literal["left", "right"]], ...] = ((16, "right"), (18, "left"))
# the buttons on each controller (for the per-hand frozen-pose check)
_HAND_BUTTONS = {"right": ("A", "B", "RJ"), "left": ("X", "Y", "LJ")}
# tolerance on |det(R) - 1| and on the entries of R^T R - I (the app logs a few decimals)
_ROTATION_TOLERANCE = 0.05


def _pose_is_valid(transform: npt.NDArray[np.float64] | None) -> bool:
    """Guard against the untracked-pose matrices the Quest app streams.

    While the headset sits un-worn (or a controller is lost), the app keeps
    streaming a live but meaningless identity/zero pose — the stream looks
    alive while the values are garbage. Require the proper homogeneous bottom
    row AND a nonzero translation (same guard as vr-teleop's pose_is_valid;
    there, a zero matrix once drove an arm through the table). Also require
    finite entries and a proper rotation block (det +1, orthonormal): NaN slips
    through tolerance comparisons, and scipy's Rotation.from_matrix raises on
    reflections and NaN instead of returning a pose.
    """
    if transform is None:
        return False
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        return False
    if not np.allclose(transform[3, :], (0.0, 0.0, 0.0, 1.0), atol=1e-6):
        return False
    rotation = transform[:3, :3]
    if abs(np.linalg.det(rotation) - 1.0) > _ROTATION_TOLERANCE:
        return False
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=_ROTATION_TOLERANCE):
        return False
    return bool(np.linalg.norm(transform[:3, 3]) > 1e-3)


class VrTeleopDevice(TeleopDevice):
    """
    Wraps the Meta Quest reader (airo_teleop_vr.MetaQuestReader) and exposes a
    fixed-size raw state array for both hand controllers.

    Poses are the controller transforms in ROS coordinates (x forward, y left,
    z up), relative to the headset's tracking origin, converted to axis-angle.

    Raw state format (27 dims):
        [0:3]    right hand position   (x, y, z, m)
        [3:6]    right hand rotation   (axis-angle rx, ry, rz, rad)
        [6:9]    left hand position
        [9:12]   left hand rotation
        [12]     right trigger  (0..1)
        [13]     right grip     (0..1)
        [14]     left trigger
        [15]     left grip
        [16:18]  right joystick (x, y, -1..1)
        [18:20]  left joystick  (x, y, -1..1)
        [20:26]  buttons A, B, X, Y, LJ, RJ (1.0 = pressed)
        [26]     enabled: 1.0 while the reader thread runs, the pose stream is
                 fresh (< max_pose_age) AND at least one controller pose is
                 valid (a finite proper rigid transform, not the identity/zero
                 matrix the app streams while the headset sits un-worn or the
                 controllers are untracked, and not frozen, see below). 0.0
                 otherwise. NOTE: the Quest reader caches the last pose forever
                 when the headset sleeps or the app dies, so consumers MUST
                 gate on this flag before using the pose entries.

    Frozen poses: an un-worn headset keeps streaming its last pose with fresh
    timestamps. Real tracking jitters in the last digits, so a hand whose pose,
    analog values and buttons have stayed bit-identical for longer than
    `max_frozen_s` is treated as invalid (`frozen_hands` lists those hands).

    Invalid hand poses appear as their zero block even when enabled=1 (the
    other hand may be the valid one).
    Returns an all-zero array while no valid pose data has been received yet.
    """

    RAW_STATE_SIZE = 27

    def __init__(
        self,
        ip_address: str | None = None,
        port: int = 5555,
        apk_name: str = "com.rail.oculus.teleop.v2",
        axis_mask: list[int] | None = None,
        max_pose_age: float = 0.5,
        auto_connect: bool = True,
        max_frozen_s: float | None = 1.0,
    ) -> None:
        """
        Args:
            ip_address: IP of the headset for network adb; None (default) uses USB.
            port: adb port used for the network connection.
            apk_name: package name of the teleop app to launch/read. The .v2 app (default) streams controller poses
                fixed in the room; the original com.rail.oculus.teleop streams them relative to the headset, so
                turning your head moves the "controller" (and a teleoperated robot) even when the hand is still.
            axis_mask: optional 6-element mask [x, y, z, roll, pitch, yaw] (1 =
                enabled) that zeroes out unwanted pose axes.
            max_pose_age: seconds after which a cached (no longer streaming)
                pose flips the `enabled` flag to 0.
            auto_connect: start the reader immediately instead of on connect().
            max_frozen_s: seconds after which a hand whose pose, analog values and
                buttons have not changed bit for bit is treated as invalid (the
                un-worn headset replays its last pose); None or <= 0 disables it.
        """
        try:
            from airo_teleop_vr import MetaQuestReader
        except ImportError as exc:
            raise ImportError(
                "VrTeleopDevice requires airo_teleop_vr (and its deps numpy, "
                "scipy, pure-python-adb). Install with: "
                "pip install -e <path-to>/airo-teleop-devices"
            ) from exc

        self._reader_cls = MetaQuestReader
        self._reader_args: dict[str, Any] = dict(
            ip_address=ip_address, port=port, APK_name=apk_name,
            axis_mask=axis_mask, run=False,
        )
        self.max_pose_age = max_pose_age
        self.max_frozen_s = max_frozen_s
        self._reader = None
        self._clock = time.monotonic
        # per hand: (last signature, time it was first seen) for the frozen-pose check
        self._unchanged_since: dict[str, tuple[bytes, float]] = {}
        self.frozen_hands: tuple[str, ...] = ()

        if auto_connect:
            self.connect()

    def connect(self) -> None:
        if self._reader is not None:
            return
        # run=False above: MetaQuestReader.__init__ auto-launches the app on the
        # headset and starts the logcat reader thread; do it here explicitly.
        self._reader = self._reader_cls(**self._reader_args)
        self._reader.run()

    def disconnect(self) -> None:
        if self._reader is not None:
            self._reader.stop()
            self._reader = None

    def close(self) -> None:
        self.disconnect()

    @property
    def reader_alive(self) -> bool:
        """True while the reader's logcat thread runs (False once adb died: no pose will arrive any more)."""
        if self._reader is None:
            return False
        return bool(getattr(self._reader, "thread_alive", True))

    @property
    def stream_alive(self) -> bool:
        """True while pose lines are arriving (fresh stream, live reader thread), ignores validity."""
        if not self.reader_alive:
            return False
        return self._reader.pose_stream_age() < self.max_pose_age

    def _is_frozen(self, hand: str, signature: bytes) -> bool:
        """True once `signature` has been unchanged for longer than max_frozen_s."""
        if self.max_frozen_s is None or self.max_frozen_s <= 0:
            return False
        now = self._clock()
        previous = self._unchanged_since.get(hand)
        if previous is None or previous[0] != signature:
            self._unchanged_since[hand] = (signature, now)
            return False
        return now - previous[1] > self.max_frozen_s

    def get_raw_state(self) -> npt.NDArray[np.float64]:
        """
        :return: array of shape (27,), see class docstring for the layout.
        """
        if self._reader is None:
            return np.zeros(self.RAW_STATE_SIZE, dtype=np.float64)

        state = np.zeros(self.RAW_STATE_SIZE, dtype=np.float64)

        for idx, value in enumerate(
            (
                self._reader.get_trigger_value("right"),
                self._reader.get_grip_value("right"),
                self._reader.get_trigger_value("left"),
                self._reader.get_grip_value("left"),
            )
        ):
            state[12 + idx] = value or 0.0

        for base, hand in _JOYSTICK_OFFSETS:
            joystick = self._reader.get_joystick_value(hand)
            if joystick is not None:
                state[base : base + 2] = joystick

        for idx, button in enumerate(BUTTONS):
            state[20 + idx] = 1.0 if self._reader.get_button_state(button) else 0.0

        any_valid = False
        frozen_hands = []
        for base, hand in _POSE_OFFSETS:
            hand_pose = self._read_hand_pose(hand, state)
            if isinstance(hand_pose, str):  # "frozen"
                frozen_hands.append(hand)
            elif hand_pose is not None:
                any_valid = True
                state[base : base + 6] = hand_pose
        self.frozen_hands = tuple(frozen_hands)

        # gate: live reader, fresh stream AND at least one valid, non-frozen pose (see docstring)
        stream_fresh = self.stream_alive
        state[26] = 1.0 if (stream_fresh and any_valid) else 0.0

        return state

    def _read_hand_pose(
        self, hand: Literal["left", "right"], state: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64] | Literal["frozen"] | None:
        """[position, rotvec] of a hand, None if its pose is invalid, "frozen" if it is unchanged for max_frozen_s.

        `state` must already hold the analog values and buttons (part of the frozen check)."""
        transform = self._reader.get_hand_controller_transform_ros(hand=hand)
        if not _pose_is_valid(transform):
            self._unchanged_since.pop(hand, None)
            return None
        transform = np.asarray(transform, dtype=np.float64)
        try:
            rotvec = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
        except (ValueError, np.linalg.LinAlgError):  # a matrix _pose_is_valid let through: an invalid hand
            self._unchanged_since.pop(hand, None)
            return None
        if self._is_frozen(hand, self._hand_signature(hand, transform, state)):
            return "frozen"
        return np.concatenate([transform[:3, 3], rotvec])

    @staticmethod
    def _hand_signature(hand: str, transform: npt.NDArray[np.float64], state: npt.NDArray[np.float64]) -> bytes:
        """The exact bytes of everything one controller reports: pose, trigger, grip, joystick, buttons."""
        analog = state[12:14] if hand == "right" else state[14:16]  # trigger, grip
        joystick = state[16:18] if hand == "right" else state[18:20]
        buttons = state[[20 + BUTTONS.index(button) for button in _HAND_BUTTONS[hand]]]
        return transform.tobytes() + analog.tobytes() + joystick.tobytes() + buttons.tobytes()


if __name__ == "__main__":
    # Live I/O readout, the airo-teleop equivalent of vr-teleop's read_quest.py.
    # Run from airo-teleop-devices/ (or with the package installed):
    #   python -m airo_teleop_devices.vr_teleop_device [--hz 30] [--absolute]
    import argparse

    ap = argparse.ArgumentParser(description="Read Meta Quest controller I/O via VrTeleopDevice")
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--apk", default="com.rail.oculus.teleop.v2", help="teleop app package")
    ap.add_argument(
        "--absolute",
        action="store_true",
        help="show poses in the headset frame instead of the offset from the first live sample",
    )
    args = ap.parse_args()

    device = VrTeleopDevice(apk_name=args.apk)
    print("VrTeleopDevice connected. Put on the headset; lines stream when poses go live.")
    dt = 1.0 / args.hz
    ref: npt.NDArray[np.float64] | None = None
    try:
        while True:
            t0 = time.time()
            s = device.get_raw_state()
            if s[26] == 0.0:
                # distinguish "nothing arriving" from "arriving but garbage"
                # (identity poses while the headset sits un-worn / controllers
                # untracked or asleep)
                if int(time.time()) % 2 == 0:
                    if not device.reader_alive:
                        print("... the logcat reader thread died (adb disconnected?) — restart this script")
                    elif device.stream_alive and device.frozen_hands:
                        # the un-worn headset replays its last pose forever, with fresh timestamps
                        print(f"... poses unchanged for {device.max_frozen_s} s — headset likely un-worn "
                              "(it replays its last pose). Move a controller or press a button.")
                    elif device.stream_alive:
                        print("... stream alive but poses invalid — put the headset on / wake the controllers")
                    else:
                        print("... waiting for pose data (headset asleep or app not running?)")
                time.sleep(0.5)
                continue
            if ref is None:
                ref = s.copy()  # zero the readouts at the first live sample
            reference: npt.NDArray[np.float64] = ref

            def pose(base: int) -> str:
                pos = s[base : base + 3]
                rot = s[base + 3 : base + 6]
                if not args.absolute:
                    pos = pos - reference[base : base + 3]
                    R_rel = Rotation.from_rotvec(reference[base + 3 : base + 6]).inv() * Rotation.from_rotvec(rot)
                    rot = R_rel.as_rotvec()
                p = ",".join(f"{v:+.3f}" for v in pos)
                r = ",".join(f"{v:+.2f}" for v in rot)
                return f"[{p}] rot[{r}]"

            pressed = [b for b, v in zip(BUTTONS, s[20:26]) if v > 0.5]
            # a frozen hand (see max_frozen_s) is zeroed like an invalid one; say so instead of printing zeros
            frozen = f" | frozen: {','.join(device.frozen_hands)}" if device.frozen_hands else ""
            print(
                f"R {pose(0)}  T{s[12]:4.2f} G{s[13]:4.2f} js[{s[16]:+.2f},{s[17]:+.2f}] | "
                f"L {pose(6)}  T{s[14]:4.2f} G{s[15]:4.2f} js[{s[18]:+.2f},{s[19]:+.2f}] | "
                f"btn:{','.join(pressed) or '-'}{frozen}"
            )
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        device.disconnect()
