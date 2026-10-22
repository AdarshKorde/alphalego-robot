"""Read-only Piper check over CAN: never enables or commands the arm.

Reports arm mode/status, firmware version, joint + gripper positions, and the timing of joint feedback frames
(rate and inter-arrival jitter), which tells us whether the CAN link (e.g. NoMachine USB forwarding) is good
enough for control.

Run:  .venv-robot/bin/python robot/read_arm.py [can0] [--seconds 10]
"""
import argparse
import time

import can
import numpy as np
from piper_sdk import C_PiperInterface_V2

parser = argparse.ArgumentParser()
parser.add_argument("can", nargs="?", default="can0")
parser.add_argument("--seconds", type=float, default=10.0)
args = parser.parse_args()

# --- raw frame timing (listen only) ---------------------------------------------------------------------------
JOINT_FB_IDS = (0x2A5, 0x2A6, 0x2A7)  # joint feedback J1/J2, J3/J4, J5/J6
bus = can.interface.Bus(channel=args.can, interface="socketcan")
counts, stamps = {}, []
t_end = time.time() + args.seconds
while time.time() < t_end:
    msg = bus.recv(timeout=0.5)
    if msg is None:
        continue
    counts[msg.arbitration_id] = counts.get(msg.arbitration_id, 0) + 1
    if msg.arbitration_id == JOINT_FB_IDS[0]:
        stamps.append(time.perf_counter())
bus.shutdown()
print(f"--- {args.can}: frames seen in {args.seconds:.0f} s ---")
for cid in sorted(counts):
    print(f"  0x{cid:03X}: {counts[cid] / args.seconds:7.1f} Hz")
if len(stamps) > 10:
    dt = np.diff(stamps) * 1e3
    print(f"joint feedback 0x2A5: {1e3 / dt.mean():.1f} Hz, inter-arrival mean {dt.mean():.2f} ms, "
          f"p50 {np.percentile(dt, 50):.2f}, p99 {np.percentile(dt, 99):.2f}, max {dt.max():.2f} ms")
else:
    print("no joint feedback frames: is the arm powered and the bus up at 1 Mbit/s?")

# --- decoded state via piper_sdk ---------------------------------------------------------------------------------
arm = C_PiperInterface_V2(args.can, judge_flag=False)  # its bitrate check shells out to `ip`, absent in the container
arm.ConnectPort()
arm.SetSDKGripperRangeParam(0.0, 0.1)  # SDK clips gripper feedback to 0-70 mm by default; this is the 100 mm gripper
arm.EnableFkCal()
time.sleep(1.0)
st = arm.GetArmStatus().arm_status
js = arm.GetArmJointMsgs().joint_state
g = arm.GetArmGripperMsgs().gripper_state
print("--- decoded ---")
print(f"  ctrl_mode {st.ctrl_mode}   arm_status {st.arm_status}   mode_feed {st.mode_feed}   teach {st.teach_status}")
print("  joints (deg):", [round(getattr(js, f'joint_{i}') / 1000, 2) for i in range(1, 7)])
print(f"  gripper opening {g.grippers_angle / 1000:.1f} mm   effort {g.grippers_effort / 1000:.2f} N·m")
try:
    print("  firmware:", arm.GetPiperFirmwareVersion())
except Exception as e:  # older SDKs / firmwares
    print("  firmware: n/a", e)
print("  SDK FK flange (mm, deg):", [round(v, 1) for v in arm.GetFK("feedback")[-1]])
arm.DisconnectPort()
