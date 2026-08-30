#!/usr/bin/env python
"""
Phone-based teleoperator (iOS via HEBI Mobile I/O, Android via WebXR/teleop package).

Standalone phone teleoperator (iOS via HEBI, Android via WebXR/teleop).
"""

import logging
import pathlib
import threading
import time
from typing import TYPE_CHECKING

import numpy as np

from .config_phone import PhoneConfig, PhoneOS
from .decorators import check_if_already_connected, check_if_not_connected
from .rotation import Rotation

if TYPE_CHECKING or True:
    try:
        import hebi
    except ImportError:
        hebi = None

try:
    from teleop import Teleop
except ImportError:
    Teleop = None

logger = logging.getLogger(__name__)


# ── helpers ─────────────────────────────────────────────────────────────────

def _require(pkg_name: str, import_name: str | None = None) -> None:
    name = import_name or pkg_name
    import importlib

    if importlib.util.find_spec(name) is None:
        raise ImportError(
            f"'{pkg_name}' is required but not installed. "
            f"Install with: pip install hebi-py teleop (or pip install {pkg_name})"
        )


# ── abstract base ───────────────────────────────────────────────────────────

class _PhoneBase:
    """Shared state and calibration logic for iOS / Android phone teleop."""

    _enabled: bool = False
    _calib_pos: np.ndarray | None = None
    _calib_rot_inv: Rotation | None = None

    def _reapply_position_calibration(self, pos: np.ndarray) -> None:
        self._calib_pos = pos.copy()

    @property
    def is_calibrated(self) -> bool:
        return self._calib_pos is not None and self._calib_rot_inv is not None

    @property
    def action_features(self) -> dict[str, type]:
        return {
            "phone.pos": np.ndarray,
            "phone.rot": Rotation,
            "phone.raw_inputs": dict,
            "phone.enabled": bool,
        }

    @property
    def feedback_features(self) -> dict[str, type] | None:
        return None

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError("Haptic feedback not yet implemented.")


# ── iOS (HEBI Mobile I/O) ───────────────────────────────────────────────────

class IOSPhone(_PhoneBase):
    name = "ios_phone"

    def __init__(self, config: PhoneConfig):
        _require("hebi-py", import_name="hebi")
        _require("teleop")
        self.config = config
        self._group = None

    @property
    def is_connected(self) -> bool:
        return self._group is not None

    @check_if_already_connected
    def connect(self) -> None:
        logger.info("Connecting to iPhone — open the HEBI Mobile I/O app.")
        lookup = hebi.Lookup()
        time.sleep(2.0)
        group = lookup.get_group_from_names(["HEBI"], ["mobileIO"])
        if group is None:
            raise RuntimeError(
                "Mobile I/O not found — check name/family settings in the app."
            )
        self._group = group
        logger.info("Connected to HEBI group with %d module(s).", group.size)
        self.calibrate()

    def calibrate(self) -> None:
        print("Press and hold B1 in the HEBI Mobile I/O app to capture this pose...\n")
        position, rotation = self._wait_for_capture()
        self._calib_pos = position.copy()
        self._calib_rot_inv = rotation.inv()
        self._enabled = False
        print("Calibration done\n")

    def _wait_for_capture(self) -> tuple[np.ndarray, Rotation]:
        while True:
            has_pose, pos, rot, fb = self._read_pose()
            if has_pose:
                io = getattr(fb, "io", None)
                if io is not None:
                    b = getattr(io, "b", None)
                    if b is not None and bool(b.get_int(1)):
                        return pos, rot
            time.sleep(0.01)

    def _read_pose(self) -> tuple[bool, np.ndarray | None, Rotation | None, object | None]:
        fbk = self._group.get_next_feedback()
        pose = fbk[0]
        ar_pos = getattr(pose, "ar_position", None)
        ar_quat = getattr(pose, "ar_orientation", None)
        if ar_pos is None or ar_quat is None:
            return False, None, None, None
        quat_xyzw = np.concatenate((ar_quat[1:], [ar_quat[0]]))
        rot = Rotation.from_quat(quat_xyzw)
        pos = ar_pos - rot.apply(self.config.camera_offset)
        return True, pos, rot, pose

    @check_if_not_connected
    def get_action(self) -> dict:
        has_pose, raw_pos, raw_rot, fb = self._read_pose()
        if not has_pose or not self.is_calibrated:
            return {}

        raw_inputs: dict[str, float | int | bool] = {}
        io = getattr(fb, "io", None)
        if io is not None:
            a, b = io.a, io.b
            if a:
                for ch in range(1, 9):
                    if a.has_float(ch):
                        raw_inputs[f"a{ch}"] = float(a.get_float(ch))
            if b:
                for ch in range(1, 9):
                    if b.has_int(ch):
                        raw_inputs[f"b{ch}"] = int(b.get_int(ch))
                    elif hasattr(b, "has_bool") and b.has_bool(ch):
                        raw_inputs[f"b{ch}"] = int(b.get_bool(ch))

        enable = bool(raw_inputs.get("b1", 0))
        if enable and not self._enabled:
            self._reapply_position_calibration(raw_pos)
            self._calib_rot_inv = raw_rot.inv()

        pos_cal = self._calib_rot_inv.apply(raw_pos - self._calib_pos)
        rot_cal = self._calib_rot_inv * raw_rot
        self._enabled = enable

        return {
            "phone.pos": pos_cal,
            "phone.rot": rot_cal,
            "phone.raw_inputs": raw_inputs,
            "phone.enabled": self._enabled,
        }

    @check_if_not_connected
    def disconnect(self) -> None:
        self._group = None


# ── Android (WebXR / teleop package) ────────────────────────────────────────

class AndroidPhone(_PhoneBase):
    name = "android_phone"

    def __init__(self, config: PhoneConfig):
        _require("hebi-py", import_name="hebi")
        _require("teleop")
        self.config = config
        self._teleop: Teleop | None = None
        self._teleop_thread: threading.Thread | None = None
        self._latest_pose: np.ndarray | None = None
        self._latest_message: dict | None = None
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._teleop is not None

    @check_if_already_connected
    def connect(self) -> None:
        logger.info("Starting teleop stream for Android...")
        # Use bundled custom webui if available, else fall back to teleop default
        _webui_dir = pathlib.Path(__file__).parent.parent / "teleop-webui"
        frontend_dir = str(_webui_dir) if _webui_dir.exists() else None
        self._teleop = Teleop(frontend_dir=frontend_dir)
        self._teleop.subscribe(self._callback)
        self._teleop_thread = threading.Thread(target=self._teleop.run, daemon=True)
        self._teleop_thread.start()
        logger.info("Android teleop stream started.")
        self.calibrate()

    def calibrate(self) -> None:
        print("Touch and move on the WebXR page to capture this pose...\n")
        pos, rot = self._wait_for_capture()
        self._calib_pos = pos.copy()
        self._calib_rot_inv = rot.inv()
        self._enabled = False
        print("Calibration done\n")

    def _wait_for_capture(self) -> tuple[np.ndarray, Rotation]:
        while True:
            with self._lock:
                msg = self._latest_message or {}
            if msg.get("move"):
                ok, pos, rot, _ = self._read_pose()
                if ok:
                    return pos, rot
            time.sleep(0.01)

    def _read_pose(self) -> tuple[bool, np.ndarray | None, Rotation | None, np.ndarray | None]:
        with self._lock:
            if self._latest_pose is None:
                return False, None, None, None
            p = self._latest_pose.copy()
        rot = Rotation.from_matrix(p[:3, :3])
        pos = p[:3, 3] - rot.apply(self.config.camera_offset)
        return True, pos, rot, p

    def _callback(self, pose: np.ndarray, message: dict) -> None:
        with self._lock:
            self._latest_pose = pose
            self._latest_message = message

    @check_if_not_connected
    def get_action(self) -> dict:
        ok, raw_pos, raw_rot, _ = self._read_pose()
        if not ok or not self.is_calibrated:
            return {}

        msg = self._latest_message or {}
        raw_inputs: dict[str, float | int | bool] = {
            "move": bool(msg.get("move", False)),
            "scale": float(msg.get("scale", 1.0)),
            "reservedButtonA": bool(msg.get("reservedButtonA", False)),
            "reservedButtonB": bool(msg.get("reservedButtonB", False)),
            "startRecording": bool(msg.get("startRecording", False)),
            "cancelRecording": bool(msg.get("cancelRecording", False)),
            "stopRecording": bool(msg.get("stopRecording", False)),
        }

        enable = bool(raw_inputs.get("move", False))
        if enable and not self._enabled:
            self._reapply_position_calibration(raw_pos)
            self._calib_rot_inv = raw_rot.inv()

        pos_cal = self._calib_rot_inv.apply(raw_pos - self._calib_pos)
        rot_cal = self._calib_rot_inv * raw_rot
        self._enabled = enable

        return {
            "phone.pos": pos_cal,
            "phone.rot": rot_cal,
            "phone.raw_inputs": raw_inputs,
            "phone.enabled": self._enabled,
        }

    @check_if_not_connected
    def disconnect(self) -> None:
        self._teleop = None
        if self._teleop_thread and self._teleop_thread.is_alive():
            self._teleop_thread.join(timeout=1.0)
        self._teleop_thread = None
        self._latest_pose = None


# ── unified Phone factory ───────────────────────────────────────────────────

class Phone:
    """
    Phone-based teleoperator.

    Routes to ``IOSPhone`` (HEBI Mobile I/O) or ``AndroidPhone`` (WebXR) based on
    ``config.phone_os``.  Press and hold **B1** (iOS) or touch-and-move (Android) to enable.
    """

    name = "phone"

    def __init__(self, config: PhoneConfig):
        self.config = config
        if config.phone_os == PhoneOS.IOS:
            self._impl = IOSPhone(config)
        elif config.phone_os == PhoneOS.ANDROID:
            self._impl = AndroidPhone(config)
        else:
            raise ValueError(f"Invalid phone_os: {config.phone_os}")

    # ── pass-throughs ─────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._impl.is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._impl.is_calibrated

    @property
    def action_features(self) -> dict[str, type]:
        return self._impl.action_features

    @property
    def feedback_features(self) -> dict[str, type] | None:
        return self._impl.feedback_features

    def connect(self) -> None:
        return self._impl.connect()

    def calibrate(self) -> None:
        return self._impl.calibrate()

    def configure(self) -> None:
        return self._impl.configure()

    def get_action(self) -> dict:
        return self._impl.get_action()

    def send_feedback(self, feedback: dict[str, float]) -> None:
        return self._impl.send_feedback(feedback)

    def disconnect(self) -> None:
        return self._impl.disconnect()

    # ── context-manager support ───────────────────────────────────

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()
