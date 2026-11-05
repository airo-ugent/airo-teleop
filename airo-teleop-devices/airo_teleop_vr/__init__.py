"""Meta Quest VR teleop reader (vendored from vr-teleop/meta_quest_teleop).

Reads controller poses + button/trigger/grip/joystick state from the
com.rail.oculus.teleop(.v2) Android app over `adb logcat` (USB or network adb).
Requires the `adb` binary on PATH, USB debugging enabled on the headset, and
the teleop APK installed on it.
"""

from airo_teleop_vr.buttons_parser import parse_buttons
from airo_teleop_vr.reader import MetaQuestReader, TeleopAppNotInstalledError, is_proper_transform

__all__ = ["MetaQuestReader", "TeleopAppNotInstalledError", "is_proper_transform", "parse_buttons"]
