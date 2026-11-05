# IDlab-AIRO teleop devices driver package

This package provides readout of multiple teleop systems. The philisophy of this package is to provide readout of the raw teleop dimensions, abstracting away the driver code. This way, the teleop systems can also be deployed for purposes beyond robot manipulators, without including the dependency on e.g. airo-mono. The package airo-teleop-agents uses airo-teleop-devices to define specialised, ready to use teleop agents, that additionally map the raw teleop readouts to usable robot actions.

## Gello
[Gello teleoperation arms](https://wuphilipp.github.io/gello_site/) are kinematic copies of the robot arms they are intended to control. This allows to copy over joint positions and hence control directly in joint space.

## SpaceMouse
The [SpaceMouse](https://3dconnexion.com/br/spacemouse/) is a desktop device that captures 3D forces and torques, featuring also a set of buttons. This device is particularly suited for control in tool space.

Installation is described on [this webpage](https://spacemouse.kubaandrysek.cz/#dependencies-required).
For Ubuntu:
```
sudo apt-get install libhidapi-dev`
echo 'KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-hidraw-permissions.rules`
sudo usermod -aG plugdev $USER`
newgrp plugdev`
```
and restart your computer.

`ValueError: Could not open SpaceMouse: Failed to open device` is known to be solved by unplugging and replugging the spacemouse in the USB port, or by trying a different USB port and going back to the first.

## Meta Quest (VR)
A [Meta Quest](https://www.meta.com/quest/) headset streams both hand controllers' poses and button/trigger/grip/joystick state over `adb logcat` via USB; `VrTeleopDevice` exposes them as a fixed 27-dim raw state (layout in its docstring).

Setup for Ubuntu:
```
sudo apt install adb
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/51-quest.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG plugdev $USER    # re-login, then replug the headset
```
and restart `adb` (kill-server/start-server) if the device stays invisible after replugging.

Use the `com.rail.oculus.teleop.v2` app (the default): it streams controller poses fixed in the room. The original `com.rail.oculus.teleop` streams them relative to the headset, so turning your head moves a teleoperated robot even when your hand is still. Only the app in front on the headset streams; `adb shell am force-stop com.rail.oculus.teleop` stops the old one.

Verify with `python -m airo_teleop_devices.vr_teleop_device`: put the headset on and the I/O readout streams; while it is not ready, the printed message says which stage is missing.

The raw state's `enabled` flag (index 26) is 0 unless the reader thread runs, the pose stream is fresh (`max_pose_age`) and at least one controller pose is valid: a finite proper rigid transform (NaN, reflections and the identity/zero pose of an untracked controller are rejected) that has not stayed bit-identical, together with that controller's buttons and analog values, for longer than `max_frozen_s` (default 1 s; an un-worn headset replays its last pose with fresh timestamps, while real tracking jitters). Invalid or frozen hands read as a zero block. Malformed logcat lines are skipped and counted (`MetaQuestReader.parse_errors`) instead of stopping the reader.
