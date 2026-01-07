### airo-teleop-agents
# IDlab-AIRO teleop agents package

This package provides common bindings between teleop devices (airo-teleop-devices) and manipulators (airo-robots). These are intended to be plug-and-play, e.g. for a teleoperation demo. For example: if you want to control any UR arm and parallel position gripper with a Gello teleoperation arm, you'd instantiate the Gello4UR_ParallelGripper class from `gello_teleop_agents.py`. See also the `examples` folder.

Executing the action that a TeleopAgent returns is done outside of the scope of the TeleopAgent class. This is because the user must be free to use different movement commands (e.g. high frequency servoing vs discrete, low frequency moves), to change the order of the UR and gripper move command, to decide whether to .wait() for a gripper and/or UR command, ... .