"""Meta Quest Reader."""

import atexit
import math
import os
import select
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Literal

import numpy as np
from ppadb.client import Client as AdbClient
from scipy.spatial.transform import Rotation

from airo_teleop_vr.buttons_parser import parse_buttons


def eprint(*args: Any, **kwargs: Any) -> None:
    """Print error messages to stderr."""
    RED = "\033[1;31m"
    sys.stderr.write(RED)
    print(*args, file=sys.stderr, **kwargs)
    RESET = "\033[0;0m"
    sys.stderr.write(RESET)


def is_proper_transform(matrix: np.ndarray, tolerance: float = 0.05) -> bool:
    """True for a finite 4x4 matrix whose rotation block is a proper rotation.

    Rejects NaN/inf entries (comparisons with NaN are False, so they slip
    through tolerance checks), the all-zero matrix, reflections (det -1) and
    scaled/sheared blocks: scipy's Rotation.from_matrix raises on several of
    these, and none of them is a pose. `tolerance` bounds |det - 1| and the
    entries of R^T R - I (the app logs a few decimals, hence not 1e-6).
    """
    matrix = np.asarray(matrix)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        return False
    rotation = matrix[:3, :3]
    if abs(np.linalg.det(rotation) - 1.0) > tolerance:
        return False
    return bool(np.allclose(rotation.T @ rotation, np.eye(3), atol=tolerance))


class TeleopAppNotInstalledError(RuntimeError):
    """The teleop app is not on the headset and there is no APK to install."""


class MetaQuestReader:
    """Meta Quest Reader class with high-level APIs for transforms and button callbacks.

    This class handles the Meta Quest device connection, data reading, and provides
    clean APIs to access hand controller transforms in OpenXR and ROS coordinate
    systems, button event callbacks, and analog input values.
    """

    PARSE_ERROR_REPORT_INTERVAL_S = 5.0
    # the .v2 app keeps the original app's activity class
    ACTIVITY_CLASS = "com.rail.oculus.teleop.MainActivity"

    def __init__(
        self,
        ip_address: str | None = None,
        port: int = 5555,
        APK_name: str = "com.rail.oculus.teleop.v2",
        run: bool = True,
        axis_mask: list[int] | None = None,
    ) -> None:
        """Initialize the MetaQuestReader.

        Args:
            ip_address: IP address to the device. If None, USB connection is used.
            port: Port number for connection. Defaults to 5555.
            APK_name: Android package name. Defaults to "com.rail.oculus.teleop.v2" (room-fixed controller poses;
                the original "com.rail.oculus.teleop" streams them relative to the headset).
            run: Whether to start reader immediately. Defaults to True.
            axis_mask: Mask for axes [x, y, z, roll, pitch, yaw]. 1 = enabled, 0 = disabled.
                       Masked axes (x, y, z, roll, pitch, yaw) will be zeroed.
        """
        self.running = False
        self.last_transforms: dict[str, Any] | None = {}
        self.last_buttons: dict[str, Any] | None = {}
        self._lock = threading.Lock()
        self.tag = "wE9ryARX"

        self.ip_address = ip_address
        self.port = port
        self.APK_name = APK_name

        # Validate axis mask
        if axis_mask is not None:
            assert (
                len(axis_mask) == 6
            ), "axis_mask must have 6 elements [x, y, z, roll, pitch, yaw]"
            assert np.all(np.isin(axis_mask, [0, 1])), "axis_mask values must be 0 or 1"
            # NOTE: Because we are reading in openxr coordinates, we need to resort the mask for ROS coordinates
            # x -> z, y -> -x, z -> -y , roll -> -pitch, pitch -> -roll, yaw -> yaw
            self.axis_mask = np.array(
                [
                    axis_mask[1],
                    axis_mask[2],
                    axis_mask[0],
                    axis_mask[4],
                    axis_mask[5],
                    axis_mask[3],
                ],
                dtype=int,
            )
        else:
            self.axis_mask = None

        # Button state tracking for edge detection
        self._prev_button_states: dict[str, bool] = {}

        # Callback system
        # TODO: add more button event callbacks.
        self._callbacks: dict[str, list[Callable]] = {
            "button_b_pressed": [],
            "button_a_pressed": [],
            "button_x_pressed": [],
            "button_y_pressed": [],
            "button_rj_pressed": [],
            "button_lj_pressed": [],
        }

        self._callbacks_locks: dict[str, threading.Lock] = {
            "button_b_pressed": threading.Lock(),
            "button_a_pressed": threading.Lock(),
            "button_x_pressed": threading.Lock(),
            "button_y_pressed": threading.Lock(),
            "button_rj_pressed": threading.Lock(),
            "button_lj_pressed": threading.Lock(),
        }

        # Cache latest transforms and button values (validated)
        self._latest_transforms: dict[str, np.ndarray] = {}
        self._latest_buttons: dict[str, Any] = {}
        # Local patch: monotonic stamp of the last parsed POSE line. The
        # caches above never expire: while the headset sleeps, logcat goes
        # quiet and get_* keeps returning the last pose forever. Consumers
        # must gate on pose_stream_age() instead of trusting a non-None
        # pose as "live" (distributed teleop put-on lurch, 2026-11-04).
        self._last_data_ts: float | None = None
        # malformed lines skipped by the reader thread (see _report_parse_error)
        self.parse_errors = 0
        self._last_parse_error_report = -math.inf

        self.device = self.get_device()
        self.install(verbose=False)
        if run:
            self.run()

    def __del__(self) -> None:
        """Destructor."""
        self.stop()

    def run(self) -> None:
        """Start reading data from the Meta Quest device."""
        # launch the configured package (e.g. the .v2 APK), whose activity class keeps the original name
        component = f"{self.APK_name}/{self.ACTIVITY_CLASS}"
        output = self.device.shell(
            f'am start -n "{component}" '
            "-a android.intent.action.MAIN -c android.intent.category.LAUNCHER"
        )
        self._check_am_start_output(output, component)
        self.running = True
        serial = self.device.serial
        # Two different "-s" flags here, for two different programs:
        #   adb -s <serial>   selects the target device (adb global option).
        #   logcat -s <tag>   silences everything except the teleop app's pose
        #                     lines, so the reader needn't drain the full device
        #                     log (this source-side filter is half the latency fix).
        # We shell out to the adb binary on purpose: that separate OS process
        # drains the device socket into the kernel pipe independently of this
        # process's GIL, so under heavy thread contention (the dual-teleop case)
        # the Python reader only has to keep up with a local pipe. A ppadb
        # in-process socket reader was benchmarked and degraded ~40-60% worse
        # under that contention. See Misc/logcat-reader-latency.md.
        cmd = [
            "adb",
            "-s",
            serial,
            "shell",
            "logcat",
            "-T",
            "0",
            "-s",
            self.tag,
        ]
        self.thread = threading.Thread(
            target=self._read_logcat_subprocess,
            args=(cmd,),
            daemon=True,
        )
        self.thread.start()

    def _check_am_start_output(self, output: Any, component: str) -> None:
        """Raise if `am start` could not launch the teleop app (it reports errors on stdout, exit code 0)."""
        text = output if isinstance(output, str) else ""
        if "Error" in text or "does not exist" in text:
            raise RuntimeError(
                f"Could not start the teleop app {component} on the headset (package '{self.APK_name}'). "
                "Check that this package is installed (`adb shell pm list packages | grep oculus`) and that "
                f"apk_name/APK_name is right; its activity class must be {self.ACTIVITY_CLASS}. "
                f"am start said: {text.strip()}"
            )

    def _read_logcat_subprocess(self, cmd: list[str]) -> None:
        """Read logcat output from adb shell subprocess.

        Reads the pipe in large chunks (one read() per select() wake-up, not
        one per byte as readline() on an unbuffered pipe would do) and
        processes every complete line in the chunk.  This keeps the reader
        ahead of the logcat stream even when other busy threads hold the GIL;
        with byte-wise reads the thread falls behind, the pipe backs up, and
        consumers see controller poses that are seconds stale.
        """
        try:
            with subprocess.Popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
                close_fds=True,
                start_new_session=True,
            ) as proc:
                # start_new_session=True means Ctrl+C of the parent would
                # otherwise orphan this child (reparented to systemd, still
                # holding a dup of the terminal's stdin!). Register cleanup.
                self._logcat_proc = proc
                atexit.register(self._kill_logcat_proc)
                assert proc.stdout is not None
                fd = proc.stdout.fileno()
                buffer = b""
                while self.running:
                    # Timeout so self.running is re-checked while logcat is quiet
                    readable, _, _ = select.select([fd], [], [], 1.0)
                    if not readable:
                        continue
                    chunk = os.read(fd, 65536)
                    if not chunk:
                        break
                    buffer += chunk
                    if b"\n" not in buffer:
                        continue
                    complete, buffer = buffer.rsplit(b"\n", 1)
                    for raw_line in complete.split(b"\n"):
                        # one malformed line (truncated write, non-numeric
                        # token, a raising button callback) must not kill the
                        # reader thread: count it, report rate-limited, skip it
                        try:
                            self._handle_line(raw_line)
                        except Exception as e:  # noqa: BLE001
                            self._report_parse_error(raw_line, e)
                proc.terminate()
                proc.wait(timeout=5)
        except FileNotFoundError:
            eprint(
                "⚠️ adb binary not found. Install android-tools-adb in the container."
            )
        except OSError as e:
            eprint(f"⚠️ Failed to start adb logcat subprocess: {e}")
        except subprocess.SubprocessError as e:
            eprint(f"⚠️ adb subprocess error: {e}")

    def _handle_line(self, raw_line: bytes) -> None:
        """Parse one logcat line and update the caches (raises on malformed data)."""
        data = self.extract_data(raw_line.decode("utf-8", errors="replace").strip())
        if not data:
            return
        transforms, buttons = MetaQuestReader.process_data(data)
        with self._lock:
            self.last_transforms, self.last_buttons = (
                transforms,
                buttons,
            )

        if transforms is not None:
            # data-flow freshness stamp (see _last_data_ts in
            # __init__): pose lines ONLY — [cam] diagnostics
            # parse to transforms=None and must not keep a
            # dead pose looking alive.
            self._last_data_ts = time.monotonic()
            for key, matrix in transforms.items():
                validated = self._validate_transform(matrix)
                if validated is not None:
                    self._latest_transforms[key] = validated

        if buttons is not None:
            self._latest_buttons = buttons
            self._handle_button_events(buttons)

    def _report_parse_error(self, raw_line: bytes, error: Exception) -> None:
        """Count a skipped line; print at most one message per PARSE_ERROR_REPORT_INTERVAL_S."""
        self.parse_errors += 1
        now = time.monotonic()
        if now - self._last_parse_error_report >= self.PARSE_ERROR_REPORT_INTERVAL_S:
            self._last_parse_error_report = now
            eprint(
                f"⚠️ Skipped malformed logcat line ({self.parse_errors} so far): "
                f"{type(error).__name__}: {error}; line: {raw_line[:120]!r}"
            )

    @property
    def thread_alive(self) -> bool:
        """True while the logcat reader thread is running.

        False before run(), after stop(), and when the thread has died (adb
        exited, the subprocess could not be started): then no new poses will
        ever arrive, whatever the caches still hold.
        """
        thread = getattr(self, "thread", None)
        return bool(self.running and thread is not None and thread.is_alive())

    def stop(self) -> None:
        """Stop reading data from the Meta Quest device."""
        self.running = False
        self._kill_logcat_proc()
        if hasattr(self, "thread"):
            self.thread.join()

    def _kill_logcat_proc(self) -> None:
        """Terminate the adb logcat child (whole process group)."""
        proc = getattr(self, "_logcat_proc", None)
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except OSError:
            try:
                proc.terminate()
            except OSError:
                pass

    def get_network_device(self, client: AdbClient, retry: int = 0) -> Any:
        """Get the Meta Quest device over the network.

        Args:
            client: ADB client.
            retry: Retry count.

        Returns:
            The Meta Quest device.
        """
        try:
            client.remote_connect(self.ip_address, self.port)
        except RuntimeError as e:
            eprint(f"⚠️ Failed to connect to device over network: {e}")
            os.system("adb devices")
            client.remote_connect(self.ip_address, self.port)
        assert self.ip_address is not None
        device = client.device(self.ip_address + ":" + str(self.port))

        if device is None:
            if retry == 1:
                os.system("adb tcpip " + str(self.port))
            if retry == 2:
                eprint(
                    "Make sure that device is running and is available at the "
                    "IP address specified as the OculusReader argument `ip_address`."
                )
                eprint("Currently provided IP address:", self.ip_address)
                eprint("Run `adb shell ip route` to verify the IP address.")
                exit(1)
            else:
                self.get_device()
                raise RuntimeError("Could not connect to device.")
        return device

    def get_usb_device(self, client: AdbClient) -> Any:
        """Get the Meta Quest device over USB.

        Args:
            client: ADB client.

        Returns:
            The Meta Quest device.
        """
        try:
            devices = client.devices()
        except RuntimeError as e:
            eprint(f"⚠️ Failed to get USB devices: {e}")
            os.system("adb devices")
            devices = client.devices()
        for device in devices:
            if device.serial.count(".") < 3:
                return device
        eprint(
            "Device not found. Make sure that device is running "
            "and is connected over USB"
        )
        eprint("Run `adb devices` to verify that the device is visible.")
        exit(1)

    def get_device(self) -> Any:
        """Get the Meta Quest device.

        Returns:
            The Meta Quest device.
        """
        # Default is "127.0.0.1" and 5037
        client = AdbClient(host="127.0.0.1", port=5037)
        if self.ip_address is not None:
            return self.get_network_device(client)
        else:
            return self.get_usb_device(client)

    def install(
        self, APK_path: str | None = None, verbose: bool = True, reinstall: bool = False
    ) -> None:
        """Install the APK on the Meta Quest device.

        Args:
            APK_path: Path to the APK file. If None, the default path is used.
            verbose: Whether to print messages. Defaults to True.
            reinstall: Whether to reinstall the APK if it is already installed.
                Defaults to False.
        """
        try:
            installed = self.device.is_installed(self.APK_name)
            if not installed or reinstall:
                if APK_path is None:
                    APK_path = os.path.join(
                        os.path.dirname(os.path.realpath(__file__)),
                        "APK",
                        "teleop-debug.apk",
                    )
                # airo-teleop does not bundle the APK: fail with instructions
                # instead of a bare FileNotFoundError from ppadb.
                if not os.path.exists(APK_path):
                    raise TeleopAppNotInstalledError(
                        f"Teleop app '{self.APK_name}' is not installed on the headset "
                        f"and no APK was found at '{APK_path}'. Install it manually "
                        "(e.g. `adb install teleop-pointer-frame-relative.apk` from the "
                        "vr-teleop repo) or pass APK_path= to MetaQuestReader.install()."
                    )
                success = self.device.install(APK_path, test=True, reinstall=reinstall)
                installed = self.device.is_installed(self.APK_name)
                if installed and success:
                    print("APK installed successfully.")
                else:
                    eprint("APK install failed.")
            elif verbose:
                print("APK is already installed.")
        except TeleopAppNotInstalledError:
            raise  # a RuntimeError too, but not an access problem: keep its message
        except RuntimeError:
            eprint("Device is visible but could not be accessed.")
            eprint(
                "Run `adb devices` to verify that the device is visible and accessible."
            )
            eprint(
                'If you see "no permissions" next to the device serial, '
                "please put on the Meta Quest and allow the access."
            )
            exit(1)

    def uninstall(self, verbose: bool = True) -> None:
        """Uninstall the APK from the Meta Quest device.

        Args:
            verbose: Whether to print messages. Defaults to True.
        """
        try:
            installed = self.device.is_installed(self.APK_name)
            if installed:
                success = self.device.uninstall(self.APK_name)
                installed = self.device.is_installed(self.APK_name)
                if not installed and success:
                    print("APK uninstall finished.")
                    print(
                        "Please verify if the app disappeared from the "
                        'list as described in "UNINSTALL.md".'
                    )
                    print(
                        "For the resolution of this issue, please follow "
                        "https://github.com/Swind/pure-python-adb/issues/71."
                    )
                else:
                    eprint("APK uninstall failed")
            elif verbose:
                print("APK is not installed.")
        except RuntimeError:
            eprint("Device is visible but could not be accessed.")
            eprint(
                "Run `adb devices` to verify that the device is visible and accessible."
            )
            eprint(
                'If you see "no permissions" next to the device serial, '
                "please put on the Oculus Quest and allow the access."
            )
            exit(1)

    @staticmethod
    def process_data(
        string: str,
    ) -> tuple[dict[str, np.ndarray] | None, dict[str, Any] | None]:
        """Process data from the Meta Quest device.

        Args:
            string: String to process.

        Returns:
            Tuple of transformations and button states.
        """
        # Diagnostic lines (e.g. "[cam] frames=... fps=...") share the logcat
        # tag but are not pose data: skip them silently instead of tripping
        # the split below once per second.
        if "&" not in string and "|" not in string:
            return None, None
        try:
            transforms_string, buttons_string = string.split("&")
        except ValueError as e:
            eprint(f"⚠️ Failed to split data string by '&': {e}")
            return None, None
        split_transform_strings = transforms_string.split("|")
        transforms = {}
        for pair_string in split_transform_strings:
            transform = np.empty((4, 4))
            pair = pair_string.split(":")
            if len(pair) != 2:
                continue
            left_right_char = pair[0]  # is r or l
            transform_string = pair[1]
            values = transform_string.split(" ")
            c = 0
            r = 0
            count = 0
            for value in values:
                if not value:
                    continue
                transform[r][c] = float(value)
                c += 1
                if c >= 4:
                    c = 0
                    r += 1
                count += 1
            if count == 16:
                transforms[left_right_char] = transform
        # Compat: the modified .v2 APK streams three transforms per hand
        # with suffixed keys (g=grip, m=model, p=pointer). Upstream readers
        # expect plain "l"/"r" keys holding the POINTER matrix, so alias it.
        for hand, ptr in (("l", "lp"), ("r", "rp")):
            if hand not in transforms and ptr in transforms:
                transforms[hand] = transforms[ptr]
        buttons = parse_buttons(buttons_string)
        return transforms, buttons

    def extract_data(self, line: str) -> str:
        """Extract data from a line of logcat output.

        Args:
            line: Line of logcat output.

        Returns:
            Extracted data.
        """
        # partition, not split()[1]: a line that ends right after the tag (no
        # ": " separator) yields "" instead of raising IndexError
        if self.tag not in line:
            return ""
        _, separator, output = line.partition(self.tag + ": ")
        return output if separator else ""

    def get_transformations_and_buttons(
        self,
    ) -> tuple[dict[str, np.ndarray] | None, dict[str, Any] | None]:
        """Get the latest transformations and button states.

        Returns:
            Tuple of transformations and button states.
        """
        with self._lock:
            return self.last_transforms, self.last_buttons

    def _apply_axis_mask(self, transform: np.ndarray) -> np.ndarray:
        """Apply axis mask to transform, zeroing masked axes.

        Args:
            transform: Current 4x4 transformation matrix (OpenXR coordinates)

        Returns:
            Masked 4x4 transformation matrix (OpenXR coordinates)
        """
        # Start with current transform

        transform_translation = transform[:3, 3]
        transform_translation_masked = transform_translation * self.axis_mask[:3]
        transform_rotation = transform[:3, :3]
        transform_rotation_euler = Rotation.from_matrix(transform_rotation).as_euler(
            "xyz"
        )
        transform_rotation_euler_masked = transform_rotation_euler * self.axis_mask[3:]
        transform_rotation_masked = Rotation.from_euler(
            "xyz", transform_rotation_euler_masked
        ).as_matrix()

        transform_masked = np.eye(4)
        transform_masked[:3, 3] = transform_translation_masked
        transform_masked[:3, :3] = transform_rotation_masked

        return transform_masked

    def pose_stream_age(self) -> float:
        """Seconds since the last parsed pose line (inf if never seen one).

        THE POSE CACHES NEVER EXPIRE: a sleeping headset / closed app / dead
        adb pipe keeps get_*_transform_* returning the last pose FOREVER.
        Always gate pose consumers on this (or on your own receive-side
        freshness) before trusting a non-None pose as "live".
        """
        ts = self._last_data_ts
        return math.inf if ts is None else time.monotonic() - ts

    def get_hand_controller_transform_openxr(
        self,
        hand: Literal["left", "right", "l", "r"] = "right",
    ) -> np.ndarray | None:
        """Get the 4x4 transformation matrix for a hand controller.

        The transform is in the OpenXR coordinate system.
        See README.md "Coordinate Systems: ROS vs OpenXR" section for details.

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')

        Returns:
            4x4 numpy array transformation matrix, or None if not
            available
        """
        hand_key = self._normalize_hand_key(hand)

        # Use hand key directly as the pointer transform key, falling back to
        # grip/pointer/model keys ('lg'/'rg', 'lp'/'rp', 'lm'/'rm') emitted by
        # the .v2 APK.
        for key in (hand_key, f"{hand_key}g", f"{hand_key}p", f"{hand_key}m"):
            if key in self._latest_transforms:
                with self._lock:
                    transform_openxr = self._latest_transforms[key].copy()
                if self.axis_mask is not None:
                    transform_openxr = self._apply_axis_mask(transform_openxr)
                return transform_openxr
        return None

    def get_hand_controller_transform_ros(
        self,
        hand: Literal["left", "right", "l", "r"] = "right",
    ) -> np.ndarray | None:
        """Get the 4x4 transformation matrix for a hand controller.

        The transform is in the ROS coordinate system. This function applies
        a quaternion [0.5, -0.5, -0.5, 0.5] to the
        transform to convert from OpenXR coordinate system to ROS coordinate
        system.

        See README.md "Coordinate Systems: ROS vs OpenXR" section for details
        on the coordinate system differences and conversion.

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')

        Returns:
            4x4 transformation matrix in ROS coordinates, or None if not
            available
        """
        transform_openxr = self.get_hand_controller_transform_openxr(hand)

        if transform_openxr is None:
            return None

        # Apply static transform: quaternion [0.5, -0.5, -0.5, 0.5]
        Q = Rotation.from_quat([0.5, -0.5, -0.5, 0.5])
        T_static = np.eye(4)
        T_static[:3, :3] = Q.as_matrix()

        transform_ros = T_static @ transform_openxr
        return transform_ros

    def get_button_state(self, button_name: str) -> bool:
        """Get current state of a button.

        Args:
            button_name: Button name (e.g., 'A', 'B', 'X', 'Y', 'RJ',
                'LJ')

        Returns:
            True if button is pressed, False otherwise
        """
        with self._lock:
            return self._latest_buttons.get(button_name, False)

    def get_grip_value(
        self, hand: Literal["left", "right", "l", "r"] = "right"
    ) -> float:
        """Get the continuous grip value (analog trigger).

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')

        Returns:
            Float value in range [0.0, 1.0] where 0.0 is not pressed and
            1.0 is fully pressed
        """
        hand_key = self._normalize_hand_key(hand)
        button_name = "leftGrip" if hand_key == "l" else "rightGrip"
        with self._lock:
            value = self._latest_buttons.get(button_name, 0.0)

        # Handle case where value might be a tuple from parsing
        if isinstance(value, tuple):
            return float(value[0]) if len(value) > 0 else 0.0
        return float(value) if value else 0.0

    def get_trigger_value(
        self, hand: Literal["left", "right", "l", "r"] = "right"
    ) -> float:
        """Get the continuous trigger value (index finger trigger).

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')

        Returns:
            Float value in range [0.0, 1.0] where 0.0 is not pressed and
            1.0 is fully pressed
        """
        hand_key = self._normalize_hand_key(hand)
        button_name = "leftTrig" if hand_key == "l" else "rightTrig"
        with self._lock:
            value = self._latest_buttons.get(button_name, 0.0)

        # Handle case where value might be a tuple from parsing
        if isinstance(value, tuple):
            return float(value[0]) if len(value) > 0 else 0.0
        return float(value) if value else 0.0

    def get_joystick_value(
        self, hand: Literal["left", "right", "l", "r"] = "right"
    ) -> tuple[float, float]:
        """Get the joystick position.

        Args:
            hand: Which hand ('left', 'right', 'l', or 'r')

        Returns:
            Tuple (x, y) where both x and y are in range [-1.0, 1.0]
            Returns (0.0, 0.0) if not available
        """
        hand_key = self._normalize_hand_key(hand)
        button_name = "leftJS" if hand_key == "l" else "rightJS"
        with self._lock:
            value = self._latest_buttons.get(button_name, (0.0, 0.0))

        if isinstance(value, tuple) and len(value) >= 2:
            return (float(value[0]), float(value[1]))
        return (0.0, 0.0)

    def on(self, event: str, callback: Callable) -> None:
        """Register a callback for an event.

        Available events:
        - 'button_b_pressed': Called when Button B is pressed
        - 'button_a_pressed': Called when Button A is pressed
        - 'button_x_pressed': Called when Button X is pressed
        - 'button_y_pressed': Called when Button Y is pressed
        - 'button_rj_pressed': Called when Right Joystick is pressed
        - 'button_lj_pressed': Called when Left Joystick is pressed

        Args:
            event: Event name
            callback: Function to call when event occurs
        """
        # make sure the event is a valid event
        if event not in self._callbacks:
            raise ValueError(
                f"Invalid event: {event}. Must be one of: "
                f"{list(self._callbacks.keys())}"
            )

        self._callbacks[event].append(callback)

    def _validate_transform(self, matrix: np.ndarray) -> np.ndarray | None:
        """Validate transformation matrix.

        Args:
            matrix: 4x4 transformation matrix

        Returns:
            The same matrix if valid, None if invalid
        """
        return matrix if is_proper_transform(matrix) else None

    def _normalize_hand_key(self, hand: Literal["left", "right", "l", "r"]) -> str:
        """Normalize hand identifier to 'l' or 'r'.

        Args:
            hand: Hand identifier ('left', 'right', 'l', or 'r')

        Returns:
            'l' or 'r'
        """
        if hand in ("left", "l"):
            return "l"
        elif hand in ("right", "r"):
            return "r"
        else:
            raise ValueError(
                f"Invalid hand: {hand}. Must be 'left', 'right', " f"'l', or 'r'"
            )

    def _handle_button_events(self, buttons: dict) -> None:
        """Handle button press events and trigger callbacks.

        Args:
            buttons: Dictionary of button states
        """
        # Use lock to prevent race conditions when called from multiple threads
        callbacks_to_trigger = []
        with self._lock:
            # Check for button presses (rising edge detection)
            button_map = {
                "B": "button_b_pressed",
                "A": "button_a_pressed",
                "X": "button_x_pressed",
                "Y": "button_y_pressed",
                "RJ": "button_rj_pressed",
                "LJ": "button_lj_pressed",
            }

            for button_key, event_name in button_map.items():
                current_state = buttons.get(button_key, False)
                prev_state = self._prev_button_states.get(button_key, False)

                # Rising edge detected
                if current_state and not prev_state:
                    if not self._callbacks_locks[event_name].locked():
                        self._callbacks_locks[event_name].acquire()
                    else:
                        continue
                    # Update state BEFORE triggering callbacks to prevent double-trigger
                    self._prev_button_states[button_key] = current_state
                    # Collect callbacks to trigger (release lock before calling to avoid blocking)
                    callbacks_to_trigger.extend(
                        [(event_name, cb) for cb in self._callbacks[event_name]]
                    )
                else:
                    self._prev_button_states[button_key] = current_state

        # Trigger callbacks outside the lock to avoid blocking other threads
        for event_name, callback in callbacks_to_trigger:
            try:
                callback()
            finally:
                self._callbacks_locks[event_name].release()


def main() -> None:
    """Main function to test the MetaQuestReader."""
    oculus_reader = MetaQuestReader()

    while True:
        time.sleep(0.3)
        print(oculus_reader.get_transformations_and_buttons())


if __name__ == "__main__":
    main()
