# AM-ARM200 robot arm on DGX Spark

Scripts, URDF model and status for an AM-ARM200 follower arm, controlled with this LeRobot fork.
Read this file first when you resume work.

Last update: 2026-10-19.

## Folder layout

| Path | Contents | Moves the arm? |
|---|---|---|
| `common.py` | Shared settings: serial port, robot config, URDF joint mapping (SIGN / OFFSET) | - |
| `read_once.py` | Read all joint positions once | No (read-only) |
| `cable_test.py` | Ping each motor in a loop and count failed replies | No (read-only) |
| `view_3d.py` | 3D viewer (rerun): `--live` follows the real arm, `--poses` shows `poses.json` | No (read-only) |
| `record_poses.py` | Save named poses by hand to `poses.json` | **Torque OFF**: support the arm |
| `pick_and_place.py` | Hard-coded pick and place from `poses.json` | **Yes** |
| `poses.json` | Saved poses | - |
| `tools/` | URDF analysis: forward kinematics and mesh collision checks | No (does not connect) |
| `urdf/` | `am-arm200.urdf` (the only URDF) and its meshes | - |
| `reference/` | Photos and the official print files (STL) | - |

## Hardware

- Robot: AM-ARM200 (6+1 DoF, LeRobot compatible). Repo: https://github.com/liyiteng/AM-ARM
- Follower arm: 12 V supply, Feetech STS3095 (joints 1-3) + STS3215 (joints 4-7).
- Leader arm: 5 V supply, Feetech STS3215 x7 (for teleop / ACT later).
- Computer: NVIDIA DGX Spark, DGX OS (Ubuntu 24.04, ARM64 / aarch64).
- Follower serial port: set in `common.py` (`/dev/ttyACM0` now). The port can change. Check with `ls /dev/ttyACM*` before use.

## Software

- Repo: this fork of huggingface/lerobot (`lerobot_alohamini`, shared by AM-ARM200 and AlohaMini).
- Conda env: `lerobot_alohamini` (Python 3.12). Do NOT use the `base` env (Python 3.14; its argparse breaks draccus). Do NOT use `uv` here: it creates `.venv`, which hides the conda Python.
- Install: `pip install -e ".[core_scripts,training,feetech]"`. Do NOT use `.[all]`: pyrealsense2 has no aarch64 wheel.
- Extra packages for `tools/`: `trimesh`, `rtree`, `python-fcl`.
- Robot config: `--robot.type=so101_follower --robot.id=my_follower --robot.arm_profile=am-follower-6dof`
- Leader config: `--teleop.type=so101_leader --teleop.id=my_leader --teleop.arm_profile=am-leader-6dof`
- Python API: `from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig`
  Joint keys: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper (+ ".pos").
  Joints in degrees; gripper 0-100.
- Docs: https://github.com/liyiteng/lerobot_alohamini/blob/main/docs/alohamini/am-arm200.md

## Commands

Use the conda env Python directly. Another venv (for example `env_isaaclab`) can hide the conda `python`.

```bash
cd ~/lerobot_alohamini/am_arm200
PY=~/miniforge3/envs/lerobot_alohamini/bin/python
$PY view_3d.py --live                     # 3D viewer, read-only
$PY read_once.py                          # one reading, read-only
$PY cable_test.py                         # ping test, read-only
$PY tools/find_contacts.py "<reading>"    # which meshes touch for a reading (no arm needed)
$PY tools/solve_straight.py "<reading>"   # offsets from a straight-up reading (no arm needed)
```

`<reading>` is one line of `view_3d.py --live` output, for example
`"shoulder_pan=-12.8 shoulder_lift=-103.3 elbow_flex=100.0 wrist_flex=-9.1 wrist_yaw=0.7 wrist_roll=1.5 gripper=1.8"`.

Viewer link (no screen on the DGX, so it uses the web viewer):
http://localhost:9090/?url=rerun%2Bhttp://127.0.0.1:9876/proxy

## Safety rules

- Before any command that moves the arm or turns motor torque on/off (lerobot-calibrate, lerobot-teleoperate,
  lerobot-record, lerobot-replay, lerobot-rollout, `record_poses.py`, `pick_and_place.py`, or any script that
  calls send_action / disable_torque): hold the arm and clear the area.
- With torque off, the arm cannot hold itself up. Support it.
- Interactive commands (calibration, recording) need a person at the keyboard.
- Do not change motor IDs, baud rates or EEPROM settings.
- Explain any `sudo` command before you run it.

## Status

### Done

1. Follower arm calibrated. File: `~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json`.
2. Method 1 (hard-coded pick and place) works: `record_poses.py`, `pick_and_place.py`, `poses.json`.
3. URDF: `urdf/am-arm200.urdf` is the official model (joint names changed to LeRobot names, mesh paths fixed).
4. 3D viewer works: `view_3d.py` (`--live`, `--poses`).
5. URDF joint mapping fixed on 2026-10-19 (`URDF_JOINT_SIGN` and `URDF_JOINT_OFFSET_DEG` in `common.py`).
   The model matches the real arm in the rest pose and in the folded pose.

   | Joint | SIGN | OFFSET | Reference |
   |---|---|---|---|
   | shoulder_lift | -1 | -93.2 | Upper-arm body vertical; confirmed by the rest pose on the rear pillar (0.6 mm mesh gap) |
   | elbow_flex | -1 | 100.5 | Folded-pose stop (`reference/30-follower-arm-complete.jpg`): forearm against the upper arm = URDF +0.5 deg |
   | wrist_flex | 1 | -7.9 | Straight-up pose (not verified) |
   | wrist_yaw | 1 | -3.8 | Straight-up pose; SIGN reversed (real right moved the model left) |

   Cause of the old errors: the offsets came from poses aligned by eye. The curved upper arm made the lift
   offset wrong by about 17.7 deg, so the model upper arm went into the base. The comments in `common.py`
   record each reading and check.
6. Gripper mapping is correct: closed = URDF +67 deg, range 133 deg.

### Open items

1. wrist_flex offset (-7.9) is not verified. Hold the wrist straight in line with the forearm and read it again.
2. Elbow: the offset comes from the folded-pose stop. In the straight-up pose, the model forearm can tilt about 9 deg.
3. Next goal: Method 2 (ACT): `lerobot-record` with the leader arm and cameras, `lerobot-train --policy.type=act`, `lerobot-rollout`.

### Facts found

- The upper arm is curved: the servo 2 - servo 3 shaft line is 17.7 deg from the part body. Reference poses use
  the body, because people align the body by eye.
- Upper arm, axis to axis: 134.9 mm. Total length: 172.6 mm (same as the print file `reference/OB_F_J3_Shoulder.stl`).
- Forearm, axis to axis: 19.8 cm (URDF and real arm agree).
- USB drops with "disabled by hub (EMI?)" in `journalctl -k`. Cause: electrical noise. Keep the USB cable away
  from the 12 V and servo cables.

## Notes for AI assistants

Parts of this folder (`tools/`, the URDF offset work) were written with an AI assistant (Claude) and checked
on the real arm.

- Follow the safety rules above. Ask the user before any command that moves the arm or changes torque.
  Give interactive commands to the user to run.
- The user is a beginner. Write ALL replies in ASD-STE100 Simplified Technical English:
  - Procedural sentences: max 20 words. Descriptive sentences: max 25 words.
  - Instructions: imperative verb first, one instruction per step, numbered steps.
  - Use active voice. Use simple present, simple past, or future tense.
  - Use one word for one meaning. Do not use slang, idioms, or "-ing" phrases as nouns.
  - Put a warning or caution before the step it applies to.
  - Commands, code, file names, and technical names stay unchanged.
