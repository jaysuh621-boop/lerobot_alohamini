"""Record named poses of the AM-ARM200 follower by hand and save them to poses.json.

Torque is OFF while this script runs, so you can move the arm by hand.
The arm cannot hold itself up. Support it at all times.

Commands at the prompt:
  <name>   save the current position under that name (for example: home)
  p        print the current position
  l        list saved poses
  d <name> delete a saved pose
  q        quit
"""

from common import (
    POSES_FILE,
    REQUIRED_POSES,
    connect_bus_only,
    format_joints,
    load_poses,
    make_robot,
    read_joints,
    save_poses,
)


def print_status(poses: dict) -> None:
    missing = [name for name in REQUIRED_POSES if name not in poses]
    print(f"Saved: {', '.join(poses) or '(none)'}")
    print(f"Still needed for pick_and_place.py: {', '.join(missing) or '(none - all done)'}")


def main() -> None:
    robot = make_robot()
    connect_bus_only(robot)

    poses = load_poses()
    print(f"Poses file: {POSES_FILE}")
    print_status(poses)

    print("\nWARNING: The next step turns torque OFF. The arm will go limp.")
    input("Hold the arm, then press ENTER to turn torque OFF...")
    robot.bus.disable_torque()
    print("Torque is OFF. Move the arm by hand.\n")

    try:
        while True:
            cmd = input("Pose name to save (p=print, l=list, d <name>=delete, q=quit): ").strip()
            if not cmd:
                continue
            if cmd == "q":
                break
            if cmd == "p":
                print("  " + format_joints(read_joints(robot)))
            elif cmd == "l":
                for name, joints in poses.items():
                    print(f"  {name:<12} {format_joints(joints)}")
                print_status(poses)
            elif cmd.startswith("d "):
                name = cmd[2:].strip()
                if poses.pop(name, None) is None:
                    print(f"  No pose named '{name}'.")
                else:
                    save_poses(poses)
                    print(f"  Deleted '{name}'.")
            elif " " in cmd:
                print("  Pose names cannot contain spaces.")
            else:
                joints = read_joints(robot)
                if cmd in poses:
                    print(f"  Replacing existing pose '{cmd}'.")
                poses[cmd] = joints
                save_poses(poses)
                print(f"  Saved '{cmd}': {format_joints(joints)}")
                print_status(poses)
    finally:
        # Torque is already off, so disconnecting does not drop the arm.
        robot.bus.disconnect(disable_torque=True)
        print(f"\nDisconnected. Poses are in {POSES_FILE}")


if __name__ == "__main__":
    main()
