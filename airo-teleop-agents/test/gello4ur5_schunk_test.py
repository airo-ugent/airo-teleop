from airo_teleop_agents.gello_teleop_agents import Gello4UR5_Schunk
from airo_teleop_agents.teleop_agent import TeleopAgentConfig
import time
from loguru import logger
from airo_robots.manipulators.hardware.ur_rtde import URrtde
import numpy as np


#=============Test configuration==================#
ur5 = URrtde(ip_address="10.42.0.162", manipulator_specs=URrtde.UR3E_CONFIG)
gello_trigger_range=(3.4560587151063498, 2.5327249992622245)  # Set to None if unknown, will prompt for calibration
USE_JOINT_SPACE = False
USE_DELTAS = False
CONTROL_ROBOT = True  # If True, will command the UR5 robot & Schunk; if False, will only print the teleop actions
#=================================================#


loop_delay = 0.05  # seconds
input("Hold gello in similar pose as robot and press Enter to continue...")
ur_start_joints = ur5.get_joint_configuration()
teleop_agent = Gello4UR5_Schunk(config=TeleopAgentConfig(use_joint_space=USE_JOINT_SPACE, use_deltas=USE_DELTAS),
                gello_usb_port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792DZ5-if00-port0",
                gello_trigger_range=gello_trigger_range,
                ur_start_joints=ur_start_joints)
if USE_JOINT_SPACE:
    if CONTROL_ROBOT:  # Slowly move to start position
        action = teleop_agent.get_action()
        ur5.move_to_joint_configuration(action[:6], joint_speed=0.1).wait()
    while True:
        action = teleop_agent.get_action()
        if CONTROL_ROBOT:
            ur5.servo_to_joint_configuration(action[:6], duration=loop_delay).wait()
        # convert joints to degrees
        action[:6] = action[:6] * 180 / np.pi
        logger.info(f"action={np.array2string(action, precision=3, suppress_small=True, floatmode='fixed')}")
        time.sleep(loop_delay)
else:  # Tool space
    if CONTROL_ROBOT:  # Slowly move to start position
        action = teleop_agent.get_action()
        print(f"initial action={action}")
        ur5.move_to_tcp_pose(action[0], joint_speed=0.1).wait()
    while True:
        action = teleop_agent.get_action()
        if CONTROL_ROBOT:
            ur5.servo_to_tcp_pose(action[0], duration=loop_delay).wait()
        logger.info(f"action={action}")
        time.sleep(loop_delay)
