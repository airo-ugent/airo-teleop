import numpy as np
import numpy.typing as npt

from airo_teleop_devices.teleop_device import TeleopDevice


class PhoneTeleopDevice(TeleopDevice):
    """
    Wraps LeRobot's Phone teleoperator and exposes a fixed-size raw state array.

    Raw state format:
        [pos_x, pos_y, pos_z, rotvec_x, rotvec_y, rotvec_z, enabled]
    """

    def __init__(self, phone_config, auto_connect: bool = True) -> None:
        try:
            from airo_teleop_phone import Phone
        except ImportError as exc:
            raise ImportError(
                "PhoneTeleopDevice requires airo_teleop_phone. "
                "Install with: pip install -e airo-teleop-devices/"
            ) from exc

        self._phone = Phone(phone_config)
        self.last_raw_inputs: dict[str, float | int | bool] = {}

        if auto_connect:
            self._phone.connect()

    def connect(self) -> None:
        self._phone.connect()

    def disconnect(self) -> None:
        self._phone.disconnect()

    def get_raw_state(self) -> npt.NDArray[np.float64]:
        action = self._phone.get_action()
        if not action:
            return np.zeros(7, dtype=np.float64)

        pos = action.get("phone.pos")
        rot = action.get("phone.rot")
        enabled = bool(action.get("phone.enabled", False))
        self.last_raw_inputs = action.get("phone.raw_inputs", {}) or {}

        if pos is None or rot is None:
            return np.zeros(7, dtype=np.float64)

        rotvec = rot.as_rotvec()
        return np.array(
            [pos[0], pos[1], pos[2], rotvec[0], rotvec[1], rotvec[2], 1.0 if enabled else 0.0],
            dtype=np.float64,
        )
