### airo-teleop-agents
# IDlab-AIRO teleop agents package

This package provides common bindings between teleop devices (airo-teleop-devices) and manipulators (airo-robots). These are intended to be plug-and-play, e.g. for a teleoperation demo. For example: if you want to control a UR5 arm and Schunk gripper with a Gello teleoperation arm, you'd instantiate the Gello4UR5_Schunk class from `gello_teleop_agents.py`. 