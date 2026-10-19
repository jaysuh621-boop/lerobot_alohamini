"""Read the follower joint positions once and print one joint per line.

Read-only: does not move the arm and does not change torque.
"""

from common import connect_bus_only, make_robot, read_joints

robot = make_robot()
connect_bus_only(robot)
try:
    for name, val in read_joints(robot).items():
        print(f"{name:<14} {val:8.1f}")
finally:
    robot.bus.disconnect(disable_torque=False)
