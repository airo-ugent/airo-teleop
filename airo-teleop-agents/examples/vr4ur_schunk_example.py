"""Teleoperate a UR arm + Schunk gripper with a Meta Quest (right controller).

Hold GRIP to move the arm (release to reposition your hand), and also the TRIGGER for slow, precise motion; hold B to
open the gripper, A to close it. Recenter the headset (hold the Meta button) while facing the robot base axis set in OPERATOR_FORWARD_AXIS.

Start with CONTROL_ROBOT = False: the targets are only printed, so you can check the directions first.
"""

import time

import numpy as np
from airo_robots.grippers.hardware.schunk_process import SchunkGripperProcess
from airo_robots.manipulators.hardware.ur_rtde import URrtde
from loguru import logger

from airo_teleop_agents.vr_teleop_agents import Vr4PositionManipulator_ParallelGripper

# =============Example configuration==================#
ROBOT_IP = "10.42.0.163"
GRIPPER_PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:7.2:1.0-port0,11,115200,8E1"
HEADSET_IP = None  # None: headset on USB; else its IP for network adb (`adb tcpip 5555` once over USB first)
HAND = "right"
OPERATOR_FORWARD_AXIS = "+y"  # robot base axis the headset faces after recentering: "+x", "-x", "+y", "-y"
TRANSLATION_SCALE = 1.0
ROTATION_SCALE = 1.0
PRECISION_SCALE = 0.25  # while the TRIGGER is held, motion is scaled by this too (slow, precise); None disables it
GRIPPER_MODE = "buttons"  # hold B to open, A to close (from the width it holds); "analog"/"binary": the trigger
CONTROL_ROBOT = False  # If True, will command the UR robot; if False, will only print the teleop actions
LOOP_DELAY = 0.05  # seconds
# =================================================#

ur = URrtde(ip_address=ROBOT_IP)
schunk = SchunkGripperProcess(usb_interface=GRIPPER_PORT)
teleop_agent = Vr4PositionManipulator_ParallelGripper(
    position_manipulator=ur,
    gripper=schunk,
    hand=HAND,
    gripper_mode=GRIPPER_MODE,
    operator_forward_axis=OPERATOR_FORWARD_AXIS,
    translation_scale=TRANSLATION_SCALE,
    rotation_scale=ROTATION_SCALE,
    precision_scale=PRECISION_SCALE,
    ip_address=HEADSET_IP,
)
logger.info("Put on the headset; hold GRIP to move (+ TRIGGER: slow), B opens and A closes the gripper. Ctrl+C stops.")

start = ur.get_tcp_pose()
last_gripper = None
try:
    while True:
        ee_pose, gripper_width = teleop_agent.get_action()
        if CONTROL_ROBOT:
            awaitable = ur.servo_to_tcp_pose(ee_pose, duration=LOOP_DELAY)
            if last_gripper is None or abs(gripper_width - last_gripper) > 0.0005:
                schunk.move(width=gripper_width)
                last_gripper = gripper_width
            awaitable.wait()
        offset_mm = np.round((ee_pose[:3, 3] - start[:3, 3]) * 1000, 1)
        logger.info(
            f"engaged={teleop_agent.mapper.engaged} precision={teleop_agent.mapper.precision} target offset from start {offset_mm} mm, "
            f"gripper {gripper_width * 1000:.0f} mm"
        )
        time.sleep(LOOP_DELAY)
except KeyboardInterrupt:
    pass
finally:
    if CONTROL_ROBOT:
        ur.rtde_control.servoStop()
    teleop_agent.close()
