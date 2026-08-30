from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import numpy as np


class PhoneOS(Enum):
    ANDROID = "android"
    IOS = "ios"


@dataclass(kw_only=True)
class PhoneConfig:
    """Configuration for phone-based teleoperation (iOS via HEBI, Android via WebXR)."""

    # Which OS / protocol to use
    phone_os: PhoneOS = PhoneOS.IOS

    # Offset from phone center to camera center (metres).
    # iPhone 14 Pro: camera is 2 cm off-centre (−y) and 4 cm above centre (+z).
    camera_offset: np.ndarray = field(default_factory=lambda: np.array([0.0, -0.02, 0.04]))

    # Optional identifier (for future calibration-file support)
    id: str | None = None

    # Optional directory for calibration files
    calibration_dir: Path | None = None
