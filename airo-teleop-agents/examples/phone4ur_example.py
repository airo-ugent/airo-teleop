import time
from loguru import logger
from airo_robots.manipulators.hardware.ur_rtde import URrtde
from airo_teleop_phone import PhoneConfig, PhoneOS

from airo_teleop_agents.phone_teleop_agents import Phone4PositionManipulator


#=============Example configuration==================#
ur = URrtde(ip_address="10.42.0.162")

PHONE_OS = PhoneOS.ANDROID  # or PhoneOS.IOS
ENABLE_TRANSLATIONS = True
ENABLE_ROTATIONS = True
CONTROL_ROBOT = True  # If True, will command the UR robot; if False, will only print the teleop actions
#=================================================#

loop_delay = 0.1  # seconds (10 Hz)
phone_config = PhoneConfig(phone_os=PHONE_OS)
teleop_agent = Phone4PositionManipulator(
    position_manipulator=ur,
    phone_config=phone_config,
    translation_scale=0.5,
    rotation_scale=1.0,
    phone_forward_axis = "-x",
    enable_settle_time_s= 0.25,
    max_translation_step_m= 0.03,
    max_rotation_step_rad= 0.35,
    enabled_axes=[ENABLE_TRANSLATIONS] * 3 + [ENABLE_ROTATIONS] * 3,
    auto_connect=True,
)

if CONTROL_ROBOT:  # Slowly move to start position
    ee_pose = teleop_agent.get_action()
    print(f"initial action={ee_pose}")
    # ur.move_to_tcp_pose(ee_pose, joint_speed=1).wait()
    ur.servo_to_tcp_pose(ee_pose, duration=loop_delay)
while True:
    ee_pose = teleop_agent.get_action()
    if CONTROL_ROBOT:
        # ur.move_to_tcp_pose(ee_pose, joint_speed=1).wait()
        ur.servo_to_tcp_pose(ee_pose, duration=loop_delay)

    logger.info(f"action={ee_pose}")
    time.sleep(loop_delay)
