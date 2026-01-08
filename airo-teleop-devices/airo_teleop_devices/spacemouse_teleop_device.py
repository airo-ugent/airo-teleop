from typing import Tuple
from airo_teleop_devices.teleop_device import TeleopDevice
import threading
import time
from collections import deque
from loguru import logger
import pyspacemouse
import numpy as np
import numpy.typing as npt


class SpaceMouseTeleopDevice(TeleopDevice):

    ACTION_SPEC = None

    def __init__(self, deadzone: Tuple[float, float]=(0.1, 0.1)):
        """
        Args:
            deadzone: (translation, rotation) deadzone values. If output of spacemouse is below this value, 
            it is set to 0.
        """
        self.state_buffer = deque(maxlen=10)  # Use deque as a rolling buffer
        self.running = True
        self.deadzone = deadzone

        # separate thread needed for continuous reading of SpaceMouse
        # cf. https://github.com/wuphilipp/gello_software/blob/main/gello/agents/spacemouse_agent.py
        #
        self.thread = threading.Thread(target=self._spacemouse_thread)
        self.thread.daemon = True
        self.thread.start()

    def _spacemouse_thread(self):
        try:
            pyspacemouse.open()
            print("SpaceMouse connected successfully!")
        except Exception as e:
            raise ValueError(f"Could not open SpaceMouse: {e}")

        while self.running:
            try:
                state = pyspacemouse.read()
                if state is not None:
                    self.state_buffer.append(state)
            except Exception as e:
                ValueError(f"Error reading SpaceMouse: {e}")
                break



    def get_raw_state(self) -> npt.NDArray[np.float64]:
        """
        :return: [x, y, z, rx, ry, rz, button0, button1]. A positive rx/ry/rz is a positive rotation around the x/y/z-axis (right-hand rule).
        """
        
        if not self.state_buffer:  # check if buffer is empty.
            logger.warning("SpaceMouse buffer is empty.")
            return np.array([0.0]*8)

        state = self.state_buffer.pop()

        # Conversion from the spacemouse coordinate frame to a different coordinate frame that makes teleop more intuitive
        # for the orientations: e.g. a positive rx is a positive rotation around the x-axis (right-hand rule).
        rx, ry, rz = -state.pitch, state.roll, -state.yaw
        rot = [rx, ry, rz]

        pos = [state.x, state.y, state.z]

        for i in range(3):
            if abs(pos[i]) < self.deadzone[0]:
                pos[i] = 0
            if abs(rot[i]) < self.deadzone[1]:
                rot[i] = 0

        return np.array([*pos, *rot, *state.buttons], dtype=np.float64)

    def close(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
