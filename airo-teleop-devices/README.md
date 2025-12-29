### airo-teleop-devices
# IDlab-AIRO teleop devices driver package

This package provides readout of multiple teleop systems. The philisophy of this package is to provide readout of the raw teleop dimensions, abstracting away the driver code. This way, the teleop systems can also be deployed for purposes beyond robot manipulators, without including the dependency on e.g. airo-mono. The package airo-teleop-agents uses airo-teleop-devices to define specialised, ready to use teleop agents, that additionally map the raw teleop readouts to usable robot actions.