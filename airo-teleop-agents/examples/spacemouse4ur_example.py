from airo_teleop_agents.spacemouse_teleop_agents import SpaceMouse4PositionManipulator
import time
from loguru import logger
from airo_robots.manipulators.hardware.ur_rtde import URrtde
import numpy as np


#=============Example configuration==================#
ur = URrtde(ip_address="10.42.0.162")

ENABLE_TRANSLATIONS = True
ENABLE_ROTATIONS = True
CONTROL_ROBOT = True  # If True, will command the UR robot; if False, will only print the teleop actions
#=================================================#


loop_delay = 0.05  # seconds
teleop_agent = SpaceMouse4PositionManipulator(position_manipulator=ur, enabled_axes=[ENABLE_TRANSLATIONS]*3 + [ENABLE_ROTATIONS]*3 + [False]*2)

if CONTROL_ROBOT:  # Slowly move to start position
    ee_pose = teleop_agent.get_action()
    print(f"initial action={ee_pose}")
    ur.move_to_tcp_pose(ee_pose, joint_speed=0.1).wait()
while True:
    ee_pose = teleop_agent.get_action()
    if CONTROL_ROBOT:
        ur.servo_to_tcp_pose(ee_pose, duration=loop_delay).wait()
    logger.info(f"action={ee_pose}")
    time.sleep(loop_delay)
