from airo_teleop_agents.gello_teleop_agents import Gello4UR_ParallelGripper
import time
from loguru import logger
from airo_robots.manipulators.hardware.ur_rtde import URrtde
from airo_robots.grippers.hardware.schunk_process import SchunkGripperProcess
from airo_teleop_devices.gello_teleop_device import GelloTeleopDevice
import numpy as np


#=============Test configuration==================#
ur = URrtde(ip_address="10.42.0.162")
schunk = SchunkGripperProcess(usb_interface="/dev/serial/by-path/pci-0000:00:14.0-usb-0:13.3:1.0-port0,12,115200,8E1")

# Your physical Gello teleop device is labeled with either "Gello1" or "Gello2", 
# choose the according default config here or provide your own GelloConfig. For (re)calibrating
# a gello: see airo_teleop_devices/test/gello_calibration.py
gello_config = GelloTeleopDevice.GELLO1_DEFAULT_CONFIG
USE_JOINT_SPACE = True
USE_DELTAS = False  # TODO
CONTROL_ROBOT = True  # If True, will command the UR5 robot & Schunk; if False, will only print the teleop actions
#=================================================#


loop_delay = 0.05  # seconds
teleop_agent = Gello4UR_ParallelGripper(
                gello_usb_port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792DZ5-if00-port0",
                gello_config=gello_config,
                ur_robot=ur,
                gripper=schunk,
                use_joint_space=USE_JOINT_SPACE,)
if USE_JOINT_SPACE:
    if CONTROL_ROBOT:  # Slowly move to start position
        action = teleop_agent.get_action()
        ur.move_to_joint_configuration(action[:6], joint_speed=0.1).wait()
    while True:
        action = teleop_agent.get_action()
        if CONTROL_ROBOT:
            awaitable = ur.servo_to_joint_configuration(action[:6], duration=loop_delay)
            schunk.move(width=action[6])
            awaitable.wait()
        logger.info(f"action={np.array2string(action, precision=3, suppress_small=True, floatmode='fixed')}")
        #time.sleep(loop_delay)
else:  # Tool space
    if CONTROL_ROBOT:  # Slowly move to start position
        ee_pose, gripper_action = teleop_agent.get_action()
        print(f"initial action={ee_pose}")
        ur.move_to_tcp_pose(ee_pose, joint_speed=0.1).wait()
    while True:
        ee_pose, gripper_action = teleop_agent.get_action()
        if CONTROL_ROBOT:
            awaitable = ur.servo_to_tcp_pose(ee_pose, duration=loop_delay)
            schunk.move(width=gripper_action)
            awaitable.wait()
        logger.info(f"action={ee_pose}, gripper={gripper_action}")
        #time.sleep(loop_delay)
