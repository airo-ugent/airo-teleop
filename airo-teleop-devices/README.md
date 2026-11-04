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

Verify with `python -m airo_teleop_devices.vr_teleop_device`: put the headset on and the I/O readout streams; while it is not ready, the printed message says which stage is missing.
