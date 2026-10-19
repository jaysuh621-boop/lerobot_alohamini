"""Read-only cable test for the AM-ARM200 follower.

Pings each motor in a loop and counts failed replies per motor.
It does NOT write any register: torque, IDs, baud and EEPROM stay unchanged.
Press Ctrl+C to stop.
"""

import sys
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

PORT = "/dev/ttyACM0"
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 0  # seconds; 0 = run until Ctrl+C
QUIET = DURATION > 0  # timed mode: log events only, no screen redraw

MOTORS = {
    "shoulder_pan": Motor(1, "sts3095", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "sts3095", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "sts3095", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    "wrist_yaw": Motor(5, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(6, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100),
}

bus = FeetechMotorsBus(port=PORT, motors=MOTORS)
bus.connect(handshake=False)

fails = dict.fromkeys(MOTORS, 0)
first_fail_loop = dict.fromkeys(MOTORS, None)
loops = 0
t0 = time.time()

print("Bend one joint at a time. Watch the FAIL column. Press Ctrl+C to stop.\n")
try:
    while True:
        loops += 1
        lines = []
        for name, m in MOTORS.items():
            model = bus.ping(m.id)
            if model is None and QUIET:
                print(f"t={time.time() - t0:6.1f}s  LOST {m.id} {name}", flush=True)
            if model is None:
                fails[name] += 1
                if first_fail_loop[name] is None:
                    first_fail_loop[name] = loops
            status = " ok " if model is not None else "LOST"
            lines.append(f"  {m.id} {name:<14} {status}   FAIL={fails[name]}")
        if DURATION and time.time() - t0 > DURATION:
            break
        if loops % 100 == 0 and QUIET:
            print(f"t={time.time() - t0:6.1f}s  loop {loops} (alive)", flush=True)
        if not QUIET:
            sys.stdout.write("\033[H\033[J" + f"loop {loops}\n" + "\n".join(lines) + "\n\nCtrl+C to stop\n")
            sys.stdout.flush()
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    bus.port_handler.closePort()

print(f"\n=== Summary after {loops} loops ===")
for name, m in MOTORS.items():
    print(f"  {m.id} {name:<14} FAIL={fails[name]}")
