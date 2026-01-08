from airo_teleop_agents.spacemouse_teleop_agents import SpaceMouse4PositionManipulator_ParallelGripper
import time
from loguru import logger
from airo_robots.manipulators.hardware.ur_rtde import URrtde
from airo_robots.grippers.hardware.schunk_process import SchunkGripperProcess
import numpy as np


#=============Example configuration==================#
ur = URrtde(ip_address="10.42.0.162")
schunk = SchunkGripperProcess(usb_interface="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0,11,115200,8E1")

ENABLE_TRANSLATIONS = True
ENABLE_ROTATIONS = True
ENABLE_GRIPPER = True
CONTROL_ROBOT = True  # If True, will command the UR robot; if False, will only print the teleop actions
#=================================================#


loop_delay = 0.05  # seconds
teleop_agent = SpaceMouse4PositionManipulator_ParallelGripper(position_manipulator=ur, 
                                                              gripper=schunk,
                                                              enabled_axes=[ENABLE_TRANSLATIONS]*3 + [ENABLE_ROTATIONS]*3 + [ENABLE_GRIPPER]*2)

if CONTROL_ROBOT:  # Slowly move to start position
    ee_pose, _ = teleop_agent.get_action()
    print(f"initial action={ee_pose}")
    ur.move_to_tcp_pose(ee_pose, joint_speed=0.1).wait()
while True:
    ee_pose, gripper_action = teleop_agent.get_action()
    if CONTROL_ROBOT:
        awaitable = ur.servo_to_tcp_pose(ee_pose, duration=loop_delay)
        schunk.move(width=gripper_action)
        awaitable.wait()
    logger.info(f"ee_pose={ee_pose}, gripper action={gripper_action}")
    time.sleep(loop_delay)
