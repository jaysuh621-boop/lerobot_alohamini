"""Shared setup for the AM-ARM200 follower scripts (record_poses.py, pick_and_place.py)."""

import json
import math
from pathlib import Path

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

PORT = "/dev/ttyACM0"
ROBOT_ID = "my_follower"
ARM_PROFILE = "am-follower-6dof"

POSES_FILE = Path(__file__).parent / "poses.json"

# Poses that pick_and_place.py needs. Arm poses use all joints except the gripper.
# grip_open / grip_closed are used only for their gripper value.
ARM_POSES = ["home", "above_pick", "pick", "above_place", "place"]
GRIP_POSES = ["grip_open", "grip_closed"]
REQUIRED_POSES = ARM_POSES + GRIP_POSES


# Official model (134.9 mm upper arm, matches the printed OB_F_J3_Shoulder.stl). The only URDF in urdf/.
URDF_PATH = Path(__file__).parent / "urdf" / "am-arm200.urdf"

# Map from LeRobot joint values (degrees, gripper 0-100) to URDF joint angles (radians):
#   urdf_deg = SIGN * lerobot_deg + OFFSET_DEG
# Directions checked with view_3d.py --live on 2026-10-16. Offsets: see below.
URDF_JOINT_SIGN = {
    "shoulder_pan": 1,
    "shoulder_lift": -1,
    "elbow_flex": -1,
    "wrist_flex": 1,
    "wrist_yaw": 1,  # 2026-10-19: reversed; real wrist right (+reading) moved the model left
    "wrist_roll": -1,
    "gripper": -1,
}
URDF_JOINT_OFFSET_DEG = {
    "shoulder_pan": 0.0,
    # lift/elbow/wrist_flex/wrist_yaw: solved on 2026-10-19 from a "straight up" reference pose
    # (100 samples from view_3d.py --live, arm still): upper-arm BODY vertical (URDF lift -90.0),
    # servo 3-4 axis line vertical (URDF elbow -179.09), wrist_roll axis along the forearm (wrist_flex 0.91, yaw 0.0).
    # Readings in that pose: shoulder_lift=-3.2, elbow_flex=-89.4, wrist_flex=8.8, wrist_yaw=3.8.
    # The upper arm is curved: its servo 2-3 axis line is 17.7 deg from the body. A first solve assumed the
    # axis line was vertical (lift -75.5, elbow 73.8); then the model upper arm sank into the base at rest.
    # Check: rest pose on the rear pillar (lift -103.3) gives a 1.0 mm mesh gap; photo
    # a side photo of the rest pose; ruler: servo 2 at 148 mm, servo 3 at 180 mm (model 166).
    # TODO: wrist_flex is not reliable (the wrist drooped after the sample); read again with the wrist held straight.
    # elbow_flex: the straight pose gave 91.5, but then the forearm sank 35 mm into the upper arm in the folded
    # pose of reference/30-follower-arm-complete.jpg. Now from that physical stop: elbow reading 100.0 = URDF +0.5
    # (first angle where the forearm mesh clears the upper arm). The straight pose then shows the forearm ~9 deg off.
    "shoulder_lift": -93.2,
    "elbow_flex": 100.5,
    "wrist_flex": -7.9,
    "wrist_yaw": -3.8,  # straight pose reading 3.8 = URDF 0.0 (sign +1 since 2026-10-19)
    "wrist_roll": 0.0,
    # gripper: URDF angle where the moving-jaw tip touches the fixed jaw (closed), from the meshes.
    "gripper": 67.0,
}
# Gripper: LeRobot 0-100 is mapped to 0..GRIPPER_FULL_OPEN_DEG before SIGN/OFFSET.
# 133 deg = calibrated gripper range (3559 - 2045 steps) * 360 / 4096.
GRIPPER_FULL_OPEN_DEG = 133.0


def to_urdf_rad(joints: dict[str, float]) -> dict[str, float]:
    out = {}
    for name, val in joints.items():
        deg = val * GRIPPER_FULL_OPEN_DEG / 100.0 if name == "gripper" else val
        out[name] = math.radians(URDF_JOINT_SIGN[name] * deg + URDF_JOINT_OFFSET_DEG[name])
    return out


def make_robot(max_relative_target: float | None = None) -> SOFollower:
    cfg = SOFollowerRobotConfig(
        port=PORT,
        id=ROBOT_ID,
        arm_profile=ARM_PROFILE,
        max_relative_target=max_relative_target,
    )
    return SOFollower(cfg)


def connect_bus_only(robot: SOFollower) -> None:
    """Open the serial bus without touching torque, and check the calibration.

    robot.connect() is not used here, because it turns torque ON at the end.
    """
    robot.bus.connect()
    if not robot.calibration:
        raise RuntimeError("No calibration file found. Run lerobot-calibrate first.")
    if not robot.is_calibrated:
        raise RuntimeError(
            "The calibration in the motors does not match the calibration file. Run lerobot-calibrate again."
        )


def read_joints(robot: SOFollower) -> dict[str, float]:
    """Joint positions: degrees for arm joints, 0-100 for the gripper."""
    return dict(robot.bus.sync_read("Present_Position"))


def load_poses() -> dict[str, dict[str, float]]:
    if POSES_FILE.exists():
        return json.loads(POSES_FILE.read_text())
    return {}


def save_poses(poses: dict[str, dict[str, float]]) -> None:
    POSES_FILE.write_text(json.dumps(poses, indent=2) + "\n")


def format_joints(joints: dict[str, float]) -> str:
    return "  ".join(f"{name}={val:7.1f}" for name, val in joints.items())
