"""Find which URDF meshes touch or overlap for a set of motor readings. Never connects to the arm.

Joint interfaces (pairs that also touch in the straight-up pose) are listed separately.

Examples (run from anywhere):
  PY=~/miniforge3/envs/lerobot_alohamini/bin/python
  # check a pose (paste a line from view_3d.py --live)
  $PY tools/find_contacts.py "shoulder_pan=-12.8 shoulder_lift=-103.3 elbow_flex=100.0 wrist_flex=-9.1 wrist_yaw=0.7 wrist_roll=1.5 gripper=1.8"
  # sweep one joint reading and report the first contact of each new pair
  $PY tools/find_contacts.py "<reading>" --sweep shoulder_lift -60 -100 0.5
  # sweep in URDF degrees (to find the URDF angle of a physical stop)
  $PY tools/find_contacts.py "<reading>" --sweep elbow_flex -1.5 2 0.25 --urdf
  # print the overlap (+) or gap (-) of one pair at each step; use this for neighbour links such as
  # Lower_Arm/Upper_Arm, which always touch a little at the joint (about 0.6 mm in the straight pose)
  $PY tools/find_contacts.py "<reading>" --sweep elbow_flex -1.5 2 0.25 --urdf --pair left_Lower_Arm left_Upper_Arm
"""

import argparse
import math

import numpy as np

import urdf_model as um

common = um.common

# Straight-up reference pose of 2026-10-19 (see common.py). Pairs that touch here are joint interfaces.
STRAIGHT = {"shoulder_pan": -12.8, "shoulder_lift": -3.2, "elbow_flex": -89.4, "wrist_flex": 8.8,
            "wrist_yaw": 3.8, "wrist_roll": -8.5, "gripper": 2.0}


def short(n: str) -> str:
    return n.replace("left_", "")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("reading", help="motor readings, e.g. a line copied from view_3d.py --live")
    ap.add_argument("--sweep", nargs=4, metavar=("JOINT", "FROM", "TO", "STEP"),
                    help="sweep one joint and report the first contact of each new pair")
    ap.add_argument("--urdf", action="store_true", help="sweep values are URDF degrees, not motor readings")
    ap.add_argument("--pair", nargs=2, metavar=("LINK_A", "LINK_B"),
                    help="with --sweep: print overlap (+mm) or gap (-mm) of this pair at each step")
    args = ap.parse_args()

    reading = um.parse_reading(args.reading)
    print("URDF:", um.URDF.name)
    print("offsets:", common.URDF_JOINT_OFFSET_DEG)
    interfaces = um.colliding_pairs(um.fk(common.to_urdf_rad(STRAIGHT)))

    q = common.to_urdf_rad(reading)
    T = um.fk(q)
    o = um.objects(T)
    print("\nreading:", reading)
    print("URDF deg:", {k: round(math.degrees(v), 1) for k, v in q.items()})
    new = sorted(um.colliding_pairs(T) - interfaces)
    print("\nNew overlaps (not joint interfaces):" if new else "\nNew overlaps: none")
    for a, b in new:
        print(f"  {short(a)} <-> {short(b)}: overlap {um.depth_mm(o, a, b):.1f} mm{'  (adjacent links)' if um.adjacent(a, b) else ''}")
    print("Joint interfaces (ignored):", ", ".join(f"{short(a)}-{short(b)}" for a, b in sorted(interfaces)))

    if args.sweep:
        joint, lo, hi, step = args.sweep[0], *map(float, args.sweep[1:])
        step = abs(step) if hi >= lo else -abs(step)
        unit = "URDF deg" if args.urdf else "reading"
        print(f"\nSweep {joint} ({unit}) {lo} -> {hi}: first contact of each new pair")
        seen = set()
        for v in np.arange(lo, hi + step / 2, step):
            if args.urdf:
                qq = dict(q)
                qq[joint] = math.radians(v)
            else:
                qq = common.to_urdf_rad({**reading, joint: float(v)})
            Tq = um.fk(qq)
            if args.pair:
                print(f"  {unit} {v:7.2f} (URDF {math.degrees(qq[joint]):6.1f}): {short(args.pair[0])} <-> "
                      f"{short(args.pair[1])} {um.depth_mm(um.objects(Tq), *args.pair):+6.1f} mm")
                continue
            for a, b in sorted(um.colliding_pairs(Tq) - interfaces - seen):
                print(f"  {unit} {v:7.2f} (URDF {math.degrees(qq[joint]):6.1f}): {short(a)} <-> {short(b)}"
                      f"{'  (adjacent links)' if um.adjacent(a, b) else ''}")
                seen.add((a, b))
        if not seen and not args.pair:
            print("  no new contacts in this range")


if __name__ == "__main__":
    main()
