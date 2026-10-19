"""Solve URDF target angles for the "straight up" reference pose, and the offsets from motor readings.

Pure FK on common.URDF_PATH. Never connects to the arm.

Targets:
  shoulder_lift  upper-arm BODY vertical (mesh +Y up). The upper arm is curved: its servo 2-3 shaft line
                 is 17.7 deg from the body. People align the body by eye, so the body is the reference.
                 (--lift-shaft-line uses the shaft line instead; that was the first, wrong solve of 2026-10-19.)
  elbow_flex     servo 3 -> servo 4 line vertical
  wrist_flex/yaw wrist_roll axis vertical, wrist above the forearm

  OFFSET = target - SIGN * reading     (common.py: urdf_deg = SIGN * lerobot_deg + OFFSET_DEG)

Example:
  ~/miniforge3/envs/lerobot_alohamini/bin/python tools/solve_straight.py \\
      "shoulder_pan=-12.8 shoulder_lift=-3.2 elbow_flex=-89.4 wrist_flex=8.8 wrist_yaw=3.8 wrist_roll=-8.5 gripper=2.0"
"""

import argparse
import math

import numpy as np

import urdf_model as um

common = um.common


def tilt(v: np.ndarray) -> float:
    """Angle in degrees between v and +Z."""
    v = v / np.linalg.norm(v)
    return math.degrees(math.acos(max(-1.0, min(1.0, v[2]))))


def search(f, lo: float, hi: float, step: float) -> float:
    """Coarse grid search, then three finer passes."""
    best = min(np.arange(lo, hi, step), key=f)
    for s in (step / 10, step / 100, step / 1000):
        best = min(np.arange(best - 10 * s, best + 10 * s, s), key=f)
    return float(best)


def pos(T, link: str) -> np.ndarray:
    return T[link][:3, 3]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("reading", nargs="?", help="motor readings in the straight pose (view_3d.py --live line)")
    ap.add_argument("--lift-shaft-line", action="store_true", help="use the servo 2-3 shaft line, not the body")
    args = ap.parse_args()

    reading = um.parse_reading(args.reading) if args.reading else {}
    q = {"shoulder_pan": math.radians(reading.get("shoulder_pan", 0.0) * common.URDF_JOINT_SIGN["shoulder_pan"]
                                      + common.URDF_JOINT_OFFSET_DEG["shoulder_pan"])}
    deg = {}

    def with_(**kw):
        return um.fk({**q, **{k: math.radians(v) for k, v in kw.items()}})

    # 1) upper arm
    if args.lift_shaft_line:
        f = lambda a: tilt(pos(with_(shoulder_lift=a), "left_Lower_Arm") - pos(with_(shoulder_lift=a), "left_Upper_Arm"))
    else:
        f = lambda a: tilt(with_(shoulder_lift=a)["left_Upper_Arm"][:3, 1])
    deg["shoulder_lift"] = search(f, -180, 180, 1)
    q["shoulder_lift"] = math.radians(deg["shoulder_lift"])
    T = um.fk(q)
    print(f"upper arm: body tilt {tilt(T['left_Upper_Arm'][:3, 1]):.1f} deg, shaft line tilt "
          f"{tilt(pos(T, 'left_Lower_Arm') - pos(T, 'left_Upper_Arm')):.1f} deg from vertical")

    # 2) forearm: servo 3 -> servo 4 line vertical
    f = lambda a: tilt(pos(with_(elbow_flex=a), "left_Wrist_Pitch_Roll") - pos(with_(elbow_flex=a), "left_Lower_Arm"))
    deg["elbow_flex"] = search(f, -180, 180, 1)
    q["elbow_flex"] = math.radians(deg["elbow_flex"])

    # 3) wrist: wrist_roll axis vertical (either sign), wrist above the forearm; flex and yaw together
    roll_child = next(c for n, _, _, c, _, _ in um.joints if n == "wrist_roll")
    roll_parent = um.child_to_parent[roll_child]
    roll_axis = next(ax for n, _, _, _, _, ax in um.joints if n == "wrist_roll")

    def f_wr(fy):
        Tw = with_(wrist_flex=fy[0], wrist_yaw=fy[1])
        axis_world = Tw[roll_child][:3, :3] @ roll_axis
        t = tilt(axis_world)
        above = pos(Tw, roll_child)[2] > pos(Tw, "left_Wrist_Pitch_Roll")[2]
        return min(t, 180 - t) + (0 if above else 1000)

    best = min(((a, b) for a in range(-90, 90) for b in range(-90, 90)), key=f_wr)
    for s in (0.2, 0.02, 0.002):
        best = min(((best[0] + i * s, best[1] + k * s) for i in range(-12, 13) for k in range(-12, 13)), key=f_wr)
    deg["wrist_flex"], deg["wrist_yaw"] = best
    _ = roll_parent

    print("\nURDF targets (deg):", {k: round(v, 2) for k, v in deg.items()})
    if reading:
        print("\njoint          reading   target    new offset   current offset")
        for k, t in deg.items():
            new = t - common.URDF_JOINT_SIGN[k] * reading[k]
            new = (new + 180) % 360 - 180  # same angle, shortest form
            print(f"{k:14s} {reading[k]:7.1f}  {t:8.2f}  {new:10.1f}  {common.URDF_JOINT_OFFSET_DEG[k]:12.1f}")
        print("\nNote: a physical stop (see find_contacts.py --pair) can be a better reference than a pose by eye.")


if __name__ == "__main__":
    main()
