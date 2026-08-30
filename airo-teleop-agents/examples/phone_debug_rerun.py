#!/usr/bin/env python3
import argparse
import pathlib
import time
from typing import Any

import numpy as np

import rerun as rr

from airo_teleop_phone import PhoneConfig, PhoneOS
from airo_teleop_phone.rotation import Rotation

from airo_teleop_agents.phone_teleop_agents import Phone4PositionManipulator


class StaticPoseProvider:
    """Minimal pose provider to satisfy Phone4PositionManipulator without a robot."""

    def __init__(self, pose: np.ndarray | None = None) -> None:
        self._pose = np.eye(4) if pose is None else pose

    def get_tcp_pose(self) -> np.ndarray:
        return self._pose.copy()

    def set_tcp_pose(self, pose: np.ndarray) -> None:
        self._pose = pose.copy()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Phone teleop debug visualization with Rerun.")
    parser.add_argument("--phone-os", choices=["android", "ios"], default="android")
    parser.add_argument("--ur-ip", default=None, help="Optional UR IP for live reference pose.")
    parser.add_argument("--loop-hz", type=float, default=10.0)
    parser.add_argument("--phone-forward-axis", default="-x", choices=["-x", "+x", "+y", "-y"])
    parser.add_argument("--translation-scale", type=float, default=0.5)
    parser.add_argument("--rotation-scale", type=float, default=1.0)
    parser.add_argument("--enable-settle-time", type=float, default=0.25)
    parser.add_argument("--max-translation-step", type=float, default=0.03)
    parser.add_argument("--max-rotation-step", type=float, default=0.35)
    parser.add_argument("--enable-translations", action="store_true", default=True)
    parser.add_argument("--disable-translations", action="store_true", default=False)
    parser.add_argument("--enable-rotations", action="store_true", default=True)
    parser.add_argument("--disable-rotations", action="store_true", default=False)
    parser.add_argument("--axes-length", type=float, default=0.05)
    parser.add_argument("--no-spawn", action="store_true", help="Do not spawn the Rerun viewer.")
    parser.add_argument(
        "--urdf",
        help="Path to a URDF file to visualize the robot.",
    )
    return parser.parse_args()


def _get_phone_os(phone_os: str) -> PhoneOS:
    if phone_os.lower() == "android":
        return PhoneOS.ANDROID
    if phone_os.lower() == "ios":
        return PhoneOS.IOS
    raise ValueError(f"Unsupported phone OS: {phone_os}")


def _build_pose_provider(ur_ip: str | None) -> Any:
    if ur_ip:
        from airo_robots.manipulators.hardware.ur_rtde import URrtde

        return URrtde(ip_address=ur_ip)
    return StaticPoseProvider()


def _log_axes(path: str, position: np.ndarray, rotation: np.ndarray, axes_length: float, alpha: int) -> None:
    position = position.reshape(3)
    rotation = rotation.reshape(3, 3)
    origins = np.repeat(position[None, :], 3, axis=0)
    vectors = np.stack([rotation[:, 0], rotation[:, 1], rotation[:, 2]], axis=0) * axes_length
    axis_colors = np.array(
        [
            [255, 0, 0, alpha],
            [0, 255, 0, alpha],
            [0, 0, 255, alpha],
        ],
        dtype=np.uint8,
    )
    rr.log(f"{path}/origin", rr.Points3D([position], colors=[[255, 255, 255, alpha]], radii=axes_length * 0.08))
    rr.log(
        f"{path}/axes",
        rr.Arrows3D(
            origins=origins,
            vectors=vectors,
            colors=axis_colors,
            radii=max(axes_length * 0.02, 0.001),
        ),
    )


def _log_pose(path: str, pose: np.ndarray, axes_length: float, alpha: int = 255) -> None:
    if pose is None:
        return
    _log_axes(path, pose[:3, 3], pose[:3, :3], axes_length, alpha)


def _log_raw_pose(path: str, position: np.ndarray, rotvec: np.ndarray, axes_length: float) -> None:
    rotation = Rotation.from_rotvec(rotvec).as_matrix()
    _log_axes(path, position, rotation, axes_length, 255)


def _log_inputs(raw_inputs: dict[str, Any]) -> None:
    if not raw_inputs:
        rr.log("phone/raw_inputs/text", rr.TextLog(""))
        return
    parts = []
    for key in sorted(raw_inputs.keys()):
        value = raw_inputs[key]
        parts.append(f"{key}={value}")
        if isinstance(value, (bool, int, float, np.floating)):
            rr.log(f"phone/raw_inputs/{key}", rr.Scalars(float(value)))
        else:
            rr.log(f"phone/raw_inputs/{key}", rr.TextLog(str(value)))
    rr.log("phone/raw_inputs/text", rr.TextLog(", ".join(parts)))


def _set_time(frame_idx: int, loop_hz: float) -> None:
    if hasattr(rr, "set_time_sequence"):
        rr.set_time_sequence("frame", frame_idx)
        return
    if hasattr(rr, "set_time_seconds"):
        rr.set_time_seconds("frame", frame_idx / max(loop_hz, 1e-6))
        return
    if hasattr(rr, "set_time_nanos"):
        rr.set_time_nanos("frame", int(frame_idx * 1e9 / max(loop_hz, 1e-6)))
        return


def _get_phone_impl(teleop_agent: Phone4PositionManipulator) -> Any:
    teleop_device = getattr(teleop_agent, "teleop_device", None)
    if teleop_device is None:
        return None
    phone = getattr(teleop_device, "_phone", None)
    if phone is None:
        return None
    return getattr(phone, "_phone_impl", None)


# ── placo-based IK from TCP pose ─────────────────────────────────────────

_ik_solver: tuple | None = None  # (robot, solver, tip, joint_names)


def _init_ik_solver(urdf_path: str, ee_frame: str | None = None) -> None:
    global _ik_solver
    import placo

    robot = placo.RobotWrapper(urdf_path)
    solver = placo.KinematicsSolver(robot)
    solver.mask_fbase(True)
    joint_names = list(robot.joint_names())
    # Auto-detect EE: use last frame in the kinematic chain
    if ee_frame is None:
        ee_frame = list(robot.frame_names())[-1]
    tip = solver.add_frame_task(ee_frame, np.eye(4))
    _ik_solver = (robot, solver, tip, joint_names)
    print(f"  IK target frame: {ee_frame}")


def _ik_from_pose(target_pose: np.ndarray) -> list:
    """Solve IK with placo, returning joint angles in radians."""
    if _ik_solver is None:
        return []
    robot, solver, tip, joint_names = _ik_solver

    tip.T_world_frame = target_pose
    tip.configure("tip", "soft", 1.0, 0.01)
    solver.solve(True)
    robot.update_kinematics()

    return [robot.get_joint(name) for name in joint_names]


def main() -> None:
    args = _parse_args()

    enable_translations = args.enable_translations and not args.disable_translations
    enable_rotations = args.enable_rotations and not args.disable_rotations

    phone_os = _get_phone_os(args.phone_os)
    phone_config = PhoneConfig(phone_os=phone_os)
    pose_provider = _build_pose_provider(args.ur_ip)

    teleop_agent = Phone4PositionManipulator(
        position_manipulator=pose_provider,
        phone_config=phone_config,
        translation_scale=args.translation_scale,
        rotation_scale=args.rotation_scale,
        phone_forward_axis=args.phone_forward_axis,
        enable_settle_time_s=args.enable_settle_time,
        max_translation_step_m=args.max_translation_step,
        max_rotation_step_rad=args.max_rotation_step,
        enabled_axes=[enable_translations] * 3 + [enable_rotations] * 3,
        auto_connect=True,
    )

    rr.init("phone_teleop_debug", spawn=not args.no_spawn)

    # ── Load URDF (optional) ─────────────────────────────────────
    urdf_tree = None
    urdf_path_str = None
    if args.urdf:
        urdf_path = pathlib.Path(args.urdf).resolve()
        if not urdf_path.exists():
            print(f"WARNING: URDF not found: {urdf_path}")
        else:
            try:
                urdf_path_str = str(urdf_path)
                urdf_tree = rr.urdf.UrdfTree.from_file_path(urdf_path_str)
                urdf_tree.log_urdf_to_recording()
                revolute = [j for j in urdf_tree.joints() if j.joint_type == "revolute"]
                print(f"URDF loaded: {urdf_tree.name} — {len(revolute)} revolute joints")
                # Initialise placo IK solver (for visualisation without real robot)
                try:
                    _init_ik_solver(urdf_path_str)
                    print("placo IK solver ready (warm-start from home pose)")
                except Exception as e:
                    print(f"WARNING: placo IK not available: {e}")
            except Exception as e:
                print(f"WARNING: Failed to load URDF: {e}")
                urdf_tree = None

    rr.log("config/phone_os", rr.TextLog(phone_os.value))
    rr.log("config/translation_scale", rr.Scalars(args.translation_scale))
    rr.log("config/rotation_scale", rr.Scalars(args.rotation_scale))
    rr.log("config/phone_forward_axis", rr.TextLog(args.phone_forward_axis))
    rr.log("config/enable_settle_time", rr.Scalars(args.enable_settle_time))
    rr.log("config/max_translation_step", rr.Scalars(args.max_translation_step))
    rr.log("config/max_rotation_step", rr.Scalars(args.max_rotation_step))
    rr.log("config/enable_translations", rr.Scalars(float(enable_translations)))
    rr.log("config/enable_rotations", rr.Scalars(float(enable_rotations)))
    rr.log("config/camera_offset", rr.TextLog(str(phone_config.camera_offset)))

    phone_device = teleop_agent.teleop_device
    # phone_device.connect()

    prev_calib_pos = None
    prev_calib_rotvec = None
    prev_enabled = None

    loop_period_s = 1.0 / max(args.loop_hz, 0.1)
    next_time = time.monotonic()
    frame_idx = 0

    try:
        while True:
            frame_idx += 1
            _set_time(frame_idx, args.loop_hz)
            rr.log("time/monotonic_s", rr.Scalars(time.monotonic()))

            raw_state = phone_device.get_raw_state()
            raw_pos = raw_state[:3]
            raw_rotvec = raw_state[3:6]
            enabled = bool(raw_state[6] > 0.5)
            rr.log("phone/raw_data/rotvec_x", rr.Scalars(float(raw_rotvec[0])))
            rr.log("phone/raw_data/rotvec_y", rr.Scalars(float(raw_rotvec[1])))
            rr.log("phone/raw_data/rotvec_z", rr.Scalars(float(raw_rotvec[2])))
            phone_impl = _get_phone_impl(teleop_agent)
            is_calibrated = bool(getattr(phone_impl, "is_calibrated", False)) if phone_impl else False
            rr.log("phone/status/is_calibrated", rr.Scalars(float(is_calibrated)))
            rr.log("phone/status/enabled", rr.Scalars(float(enabled)))

            calib_pos = getattr(phone_impl, "_calib_pos", None) if phone_impl else None
            calib_rot_inv = getattr(phone_impl, "_calib_rot_inv", None) if phone_impl else None
            if calib_pos is not None and calib_rot_inv is not None:
                calib_pos = np.array(calib_pos, dtype=float)
                calib_rotvec = calib_rot_inv.as_rotvec()
                rr.log("phone/calibration/pos_x", rr.Scalars(float(calib_pos[0])))
                rr.log("phone/calibration/pos_y", rr.Scalars(float(calib_pos[1])))
                rr.log("phone/calibration/pos_z", rr.Scalars(float(calib_pos[2])))
                rr.log("phone/calibration/rotvec_x", rr.Scalars(float(calib_rotvec[0])))
                rr.log("phone/calibration/rotvec_y", rr.Scalars(float(calib_rotvec[1])))
                rr.log("phone/calibration/rotvec_z", rr.Scalars(float(calib_rotvec[2])))

                calib_changed = False
                if prev_calib_pos is None or prev_calib_rotvec is None:
                    calib_changed = True
                else:
                    if not np.allclose(calib_pos, prev_calib_pos):
                        calib_changed = True
                    if not np.allclose(calib_rotvec, prev_calib_rotvec):
                        calib_changed = True

                if calib_changed:
                    rr.log(
                        "events/calibration",
                        rr.TextLog(f"Calibration updated: pos={calib_pos.tolist()} rotvec={calib_rotvec.tolist()}"),
                    )
                    prev_calib_pos = calib_pos.copy()
                    prev_calib_rotvec = calib_rotvec.copy()

            if prev_enabled is None:
                prev_enabled = enabled
            elif enabled and not prev_enabled:
                rr.log("events/enable", rr.TextLog("Teleop enabled"))
                prev_enabled = enabled
            elif (not enabled) and prev_enabled:
                rr.log("events/enable", rr.TextLog("Teleop disabled"))
                prev_enabled = enabled

            prev_command_pose = getattr(teleop_agent, "_last_command_pose", None)
            desired_pose = teleop_agent.transform_func(raw_state)
            if isinstance(pose_provider, StaticPoseProvider):
                pose_provider.set_tcp_pose(desired_pose)
            else:
                pose_provider.servo_to_tcp_pose(desired_pose, duration=0.1)

            _log_raw_pose("phone/raw", raw_pos, raw_rotvec, args.axes_length)
            _log_pose("robot/mapped", desired_pose, args.axes_length)

            _log_pose("robot/reference", getattr(teleop_agent, "_reference_pose", None), args.axes_length, 120)
            _log_pose(
                "robot/last_command",
                getattr(teleop_agent, "_last_command_pose", None),
                args.axes_length,
                180,
            )
            _log_pose(
                "robot/command_when_disabled",
                getattr(teleop_agent, "_command_when_disabled", None),
                args.axes_length,
                80,
            )

            if prev_command_pose is not None:
                translation_delta = desired_pose[:3, 3] - prev_command_pose[:3, 3]
                delta_norm = float(np.linalg.norm(translation_delta))
                rr.log("robot/translation_delta_norm", rr.Scalars(delta_norm))
                rotation_delta = desired_pose[:3, :3] @ prev_command_pose[:3, :3].T
                rotation_delta_rotvec = Rotation.from_matrix(rotation_delta).as_rotvec()
                rotation_delta_norm = float(np.linalg.norm(rotation_delta_rotvec))
                rr.log("robot/rotation_delta_norm", rr.Scalars(rotation_delta_norm))

            rr.log("phone/enable_settle_active", rr.Scalars(float(getattr(teleop_agent, "_enable_time_s", None) is not None)))

            # ── URDF joint animation ──────────────────────────────
            if urdf_tree is not None:
                try:
                    revolute_joints = [j for j in urdf_tree.joints() if j.joint_type == "revolute"]
                    # Try to get joint angles from the robot first
                    if hasattr(pose_provider, "get_joint_configuration"):
                        joint_vals = list(pose_provider.get_joint_configuration())
                    elif _ik_solver is not None:
                        # No real robot — solve IK from the TCP pose
                        joint_vals = _ik_from_pose(desired_pose)
                    else:
                        joint_vals = [0.0] * len(revolute_joints)

                    for joint, val in zip(revolute_joints, joint_vals[: len(revolute_joints)]):
                        transform = joint.compute_transform(float(val), clamp=True)
                        rr.log("robot/urdf/transforms", transform)
                except Exception:
                    pass

            _log_inputs(phone_device.last_raw_inputs)

            next_time += loop_period_s
            sleep_s = next_time - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_time = time.monotonic()
    except KeyboardInterrupt:
        rr.log("events/shutdown", rr.TextLog("Shutting down"))
    finally:
        phone_device.disconnect()


if __name__ == "__main__":
    main()
