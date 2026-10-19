"""Show the AM-ARM200 URDF in a 3D viewer (rerun).

This script never moves the arm and never changes torque.

Usage:
  python view_3d.py                  # zero pose (all joints at 0)
  python view_3d.py --poses          # each pose in poses.json, one per timeline step
  python view_3d.py --live           # read the real arm and update the 3D view (Ctrl+C to stop)
  python view_3d.py --web ...        # open the viewer in a web browser instead of a window

In live mode, move the arm by hand only if torque is OFF (for example after record_poses.py).
"""

import argparse
import os
import sys
import time
from pathlib import Path

import rerun as rr

from common import URDF_PATH, connect_bus_only, format_joints, load_poses, make_robot, read_joints, to_urdf_rad

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"]


def log_joints(tree: rr.urdf.UrdfTree, joints: dict[str, float]) -> None:
    for name, rad in to_urdf_rad(joints).items():
        joint = tree.get_joint_by_name(name)
        rr.log("transforms", joint.compute_transform(rad, clamp=False))


def main() -> None:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--poses", action="store_true", help="Show the poses saved in poses.json.")
    mode.add_argument("--live", action="store_true", help="Read the real arm (read-only) and follow it.")
    parser.add_argument("--web", action="store_true", help="Serve a web viewer at http://localhost:9090.")
    parser.add_argument("--hz", type=float, default=20.0, help="Update rate in live mode.")
    args = parser.parse_args()

    if not args.web and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("No screen found (DISPLAY is not set). Using the web viewer.")
        args.web = True

    rr.init("am_arm200_view")
    if args.web:
        server_uri = rr.serve_grpc()
        rr.serve_web_viewer(connect_to=server_uri, open_browser=False)
        print(f"Open this link in a browser: http://localhost:9090/?url={server_uri.replace('+', '%2B')}")
        print("(VS Code Remote: forward ports 9090 and 9876 in the PORTS tab if they are not forwarded.)")
    else:
        # Use the viewer from this Python env; another `rerun` on PATH may be a different version.
        rr.spawn(executable_path=str(Path(sys.executable).parent / "rerun"))

    tree = rr.urdf.UrdfTree.from_file_path(URDF_PATH)
    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)  # URDF uses Z up
    tree.log_urdf_to_recording()
    rr.log("world_axes", rr.Arrows3D(vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
                                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]), static=True)

    if args.poses:
        poses = load_poses()
        for i, (name, joints) in enumerate(poses.items()):
            rr.set_time("pose", sequence=i)
            rr.log("pose_name", rr.TextLog(name))
            log_joints(tree, joints)
            print(f"{i}: {name:<12} {format_joints(joints)}")
        print("Use the timeline at the bottom of the viewer to step through the poses.")
    elif args.live:
        robot = make_robot()
        connect_bus_only(robot)
        print("Reading the arm. Torque is not changed. Press Ctrl+C to stop.")
        try:
            t0 = time.time()
            fails = 0
            while True:
                try:
                    joints = read_joints(robot)
                    fails = 0
                except ConnectionError:
                    # A single missed reply should not stop the viewer. Stop only if the bus stays silent.
                    fails += 1
                    print(f"\nRead failed ({fails} in a row). Check the cables and the 12 V power.")
                    if fails >= 20:
                        raise
                    if fails % 5 == 0:
                        # After a USB reset (kernel log: "disabled by hub (EMI?)") the old port handle is dead.
                        print("Opening the serial port again...")
                        try:
                            robot.bus.port_handler.closePort()
                            robot.bus.port_handler.openPort()
                            robot.bus.port_handler.setBaudRate(robot.bus.default_baudrate)
                        except Exception as exc:
                            print(f"Reopen failed: {exc}")
                    time.sleep(0.2)
                    continue
                rr.set_time("time", duration=time.time() - t0)
                log_joints(tree, joints)
                print("\r" + format_joints(joints), end="", flush=True)
                time.sleep(1 / args.hz)
        except KeyboardInterrupt:
            print()
        finally:
            robot.bus.disconnect(disable_torque=False)
    else:
        log_joints(tree, dict.fromkeys(JOINT_NAMES, 0.0))
        print("Showing the zero pose (all joints at 0).")

    if args.web:
        print("Press Ctrl+C to stop the web viewer.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
