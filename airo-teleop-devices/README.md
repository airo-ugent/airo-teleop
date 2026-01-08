# IDlab-AIRO teleop devices driver package

This package provides readout of multiple teleop systems. The philisophy of this package is to provide readout of the raw teleop dimensions, abstracting away the driver code. This way, the teleop systems can also be deployed for purposes beyond robot manipulators, without including the dependency on e.g. airo-mono. The package airo-teleop-agents uses airo-teleop-devices to define specialised, ready to use teleop agents, that additionally map the raw teleop readouts to usable robot actions.

## SpaceMouse
Installation is described on [this webpage](https://spacemouse.kubaandrysek.cz/#dependencies-required).
For Ubuntu:
`sudo apt-get install libhidapi-dev`
`echo 'KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-hidraw-permissions.rules`
`sudo usermod -aG plugdev $USER`
`newgrp plugdev`
and restart your computer.

`ValueError: Could not open SpaceMouse: Failed to open device` is known to be solved by unplugging and replugging the spacemouse in the USB port, or by trying a different USB port and going back to the first.
