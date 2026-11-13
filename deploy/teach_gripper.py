"""Gripper control while the arm is moved by hand in teach (drag) mode, for teach_plate.py.

Sends GripperCtrl only (no MotionCtrl, JointCtrl, enable or disable), so the arm's joints are never commanded.
Reads one-line commands appended to a control file:
  goto <mm> [effort]   gripper to an opening (0..100 mm), effort 0.001 N*m (default 1000, max 4000)
  gripoff              disable the gripper motor: the fingers move freely by hand (in teach mode the firmware
                       ignores gripper position commands, so fingers are closed on the brick by hand)
  gripon               re-enable the gripper, holding its current measured opening
  status               joints, fingertip point (TCP), tilt from vertical, gripper opening/effort/status, arm mode
  quit
It also re-sends the current gripper command at 10 Hz and prints a status line every 2 s.

Run:  python deploy/teach_gripper.py --ctl deploy/calib/gripper.ctl
      echo "goto 0 1000" >> deploy/calib/gripper.ctl
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

import core
from piper_io import PiperArm

HERE = Path(__file__).resolve().parent
EFFORT_MAX = 4000
CTRL_MODES = {0x00: "STANDBY", 0x01: "CAN_CTRL", 0x02: "TEACH", 0x03: "ETHERNET", 0x04: "WIFI", 0x07: "OFFLINE_TRAJ"}


def status_line(arm, tcp_z, grip_um, eff, code):
    st = arm.state()
    p, R = core.tcp_pose(st["q"], tcp_z)
    tilt = math.degrees(math.acos(min(1.0, -R[2, 2])))
    s = st["status"]
    g = int(arm.arm.GetArmGripperMsgs().gripper_state.status_code)
    return (f"q {np.round(np.degrees(st['q']), 2).tolist()}  tcp ({p[0] * 1e3:.1f}, {p[1] * 1e3:.1f}, "
            f"{p[2] * 1e3:.1f}) mm  tilt {tilt:.1f} deg  opening {st['grip_m'] * 1e3:.2f} mm  "
            f"grip effort {st['grip_effort']:.3f} N*m  grip status 0x{g:02X} (enabled {g >> 6 & 1})  "
            f"mode {CTRL_MODES.get(s['ctrl_mode'], hex(s['ctrl_mode']))}  arm_status 0x{s['arm_status']:02X}  "
            f"cmd {grip_um / 1000:.1f} mm @ {eff} code 0x{code:02X}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ctl", default=str(HERE / "calib" / "gripper.ctl"))
    args = ap.parse_args()
    cfg_r = json.loads((HERE / "robot_config.json").read_text())
    tcp_z = core.load_config()["geometry"]["tcp_z"]
    arm = PiperArm(cfg_r["can"])
    ctl = Path(args.ctl)
    ctl.parent.mkdir(exist_ok=True)
    ctl.touch()
    pos = ctl.stat().st_size                         # ignore commands written before this start
    grip_um = int(round(arm.state()["grip_m"] * 1e6))  # hold the current opening until told otherwise
    eff, code = 1000, 0x01
    print(status_line(arm, tcp_z, grip_um, eff, code), flush=True)
    t_status = time.monotonic() + 2.0
    try:
        while True:
            arm.arm.GripperCtrl(max(0, min(grip_um, 100_000)), eff, code, 0)
            code = 0x01 if code == 0x03 else code      # 0x03 = enable + clear errors, sent once
            if ctl.stat().st_size > pos:
                with ctl.open() as f:
                    f.seek(pos)
                    lines = [ln.split() for ln in f.read().splitlines() if ln.strip()]
                    pos = f.tell()
                for w in lines:
                    if w[0] == "quit":
                        return
                    if w[0] == "status":
                        print(status_line(arm, tcp_z, grip_um, eff, code), flush=True)
                    elif w[0] == "gripoff":
                        code = 0x00
                        print("gripper disabled: fingers are free to move by hand", flush=True)
                        t_status = time.monotonic() + 1.0
                    elif w[0] == "gripon":
                        grip_um, eff, code = int(round(arm.state()["grip_m"] * 1e6)), 1000, 0x03
                        print(f"gripper enabled, holding {grip_um / 1000:.2f} mm", flush=True)
                        t_status = time.monotonic() + 1.0
                    elif w[0] == "goto" and code == 0x00:
                        print("refused: gripper is disabled; send 'gripon' first", flush=True)
                    elif w[0] == "goto" and len(w) in (2, 3):
                        mm, e = float(w[1]), int(w[2]) if len(w) == 3 else 1000
                        if not (0 <= mm <= 100 and 0 < e <= EFFORT_MAX):
                            print(f"refused: need 0..100 mm and effort 1..{EFFORT_MAX}", flush=True)
                            continue
                        grip_um, eff = round(mm * 1000), e
                        print(f"gripper to {mm:.1f} mm at effort {e}", flush=True)
                        t_status = time.monotonic() + 1.5
                    else:
                        print(f"ignored: {' '.join(w)!r}", flush=True)
            if time.monotonic() >= t_status:
                print(status_line(arm, tcp_z, grip_um, eff, code), flush=True)
                t_status = time.monotonic() + 2.0
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        arm.close()
        print("quit: gripper holds its last target; the arm was never commanded", flush=True)


if __name__ == "__main__":
    main()
