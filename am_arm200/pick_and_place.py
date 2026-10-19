"""Run a hard-coded pick-and-place sequence on the AM-ARM200 follower, using poses.json.

The arm MOVES. Clear the area and keep one hand near the 12 V power switch.

Usage:
  python pick_and_place.py --dry-run         # print the plan only, no connection
  python pick_and_place.py                   # step mode: press ENTER before each move
  python pick_and_place.py --no-step         # run the whole sequence without stopping
  python pick_and_place.py --speed 30        # joint speed in degrees per second (default 20)
"""

import argparse
import time

from common import ARM_POSES, GRIP_POSES, connect_bus_only, format_joints, load_poses, make_robot, read_joints

FPS = 30
MAX_SPEED = 60.0  # deg/s; hard upper limit for --speed
MAX_STEP_DEG = 10.0  # max_relative_target: largest change per command, as a second safety layer


def build_sequence() -> list[tuple[str, str, str]]:
    """Each step: (description, arm pose name, gripper pose name)."""
    return [
        ("Go home, open gripper", "home", "grip_open"),
        ("Move above the object", "above_pick", "grip_open"),
        ("Move down to the object", "pick", "grip_open"),
        ("Close gripper", "pick", "grip_closed"),
        ("Lift the object", "above_pick", "grip_closed"),
        ("Move above the place spot", "above_place", "grip_closed"),
        ("Move down to the place spot", "place", "grip_closed"),
        ("Open gripper", "place", "grip_open"),
        ("Move up", "above_place", "grip_open"),
        ("Go home", "home", "grip_open"),
    ]


def target_for(poses: dict, arm_pose: str, grip_pose: str) -> dict[str, float]:
    target = {k: v for k, v in poses[arm_pose].items() if k != "gripper"}
    target["gripper"] = poses[grip_pose]["gripper"]
    return target


def move_to(robot, target: dict[str, float], speed: float) -> None:
    """Move smoothly from the present position to the target, at about `speed` deg/s."""
    start = read_joints(robot)
    max_delta = max(abs(target[k] - start[k]) for k in target)
    duration = max(max_delta / speed, 0.5)
    n_steps = max(int(duration * FPS), 1)

    for i in range(1, n_steps + 1):
        t = i / n_steps
        s = t * t * (3 - 2 * t)  # smoothstep: slow start, slow stop
        cmd = {k: start[k] + s * (target[k] - start[k]) for k in target}
        robot.send_action({f"{k}.pos": v for k, v in cmd.items()})
        time.sleep(1 / FPS)

    time.sleep(0.3)  # let the motors settle


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true", help="Print the plan only. Do not connect.")
    parser.add_argument("--no-step", action="store_true", help="Do not wait for ENTER before each move.")
    parser.add_argument("--speed", type=float, default=20.0, help="Joint speed in deg/s (default 20).")
    args = parser.parse_args()
    if not 0 < args.speed <= MAX_SPEED:
        parser.error(f"--speed must be between 0 and {MAX_SPEED}")

    poses = load_poses()
    missing = [n for n in ARM_POSES + GRIP_POSES if n not in poses]
    if missing:
        raise SystemExit(f"Missing poses in poses.json: {', '.join(missing)}. Run record_poses.py first.")

    sequence = build_sequence()
    print("Plan:")
    for i, (desc, arm_pose, grip_pose) in enumerate(sequence, 1):
        print(f"  {i:2}. {desc:<28} arm={arm_pose:<12} gripper={grip_pose}")
    if args.dry_run:
        return

    robot = make_robot(max_relative_target=MAX_STEP_DEG)
    connect_bus_only(robot)

    print("\nWARNING: The arm will turn torque ON and then MOVE.")
    print("Clear the area. Keep one hand near the 12 V power switch.")
    input("Put the arm near the 'home' pose and press ENTER to turn torque ON...")

    # Set the goal to the present position BEFORE torque turns on, so the arm does not jump.
    with robot.bus.torque_disabled():
        robot.bus.sync_write("Goal_Position", robot.bus.sync_read("Present_Position"))
    robot.configure()  # standard lerobot motor setup; turns torque ON at the end
    print("Torque is ON. The arm holds its position.")
    print("Present: " + format_joints(read_joints(robot)))

    try:
        for i, (desc, arm_pose, grip_pose) in enumerate(sequence, 1):
            if not args.no_step:
                input(f"\n[{i}/{len(sequence)}] {desc}. Press ENTER to move (Ctrl+C to stop)...")
            else:
                print(f"\n[{i}/{len(sequence)}] {desc}")
            move_to(robot, target_for(poses, arm_pose, grip_pose), args.speed)
        print("\nSequence finished.")
    except KeyboardInterrupt:
        print("\nStopped by user. The arm holds its present position.")
    finally:
        try:
            input("\nHold the arm, then press ENTER to turn torque OFF and disconnect...")
        except KeyboardInterrupt:
            pass
        robot.bus.disconnect(disable_torque=True)
        print("Torque is OFF. Disconnected.")


if __name__ == "__main__":
    main()
