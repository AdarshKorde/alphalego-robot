"""Read-only live display of the fingertip point (TCP) for placing the plate by hand. Never enables or commands the arm.

Prints, 10x per second on one line: TCP x, y, z (arm frame, mm), tilt from vertical, gripper opening, and the TCP's
offset from the sim's cell 0 0 (or from --ref X Y in mm). Ctrl+C to stop.

Run:  python deploy/watch_tcp.py
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref", type=float, nargs=2, help="reference point x y in mm (default: sim cell 0 0)")
    args = ap.parse_args()
    cfg = core.load_config()
    cfg_r = json.loads((HERE / "robot_config.json").read_text())
    ref = np.array(args.ref) / 1000 if args.ref else core.Plate(cfg).cell_centre(0, 0)[:2]
    tcp_z = cfg["geometry"]["tcp_z"]
    arm = PiperArm(cfg_r["can"])
    print(f"offset is from ({ref[0] * 1e3:.1f}, {ref[1] * 1e3:.1f}) mm; Ctrl+C to stop")
    try:
        while True:
            st = arm.state()
            p, R = core.tcp_pose(st["q"], tcp_z)
            tilt = math.degrees(math.acos(min(1.0, -R[2, 2])))
            d = (p[:2] - ref) * 1e3
            print(f"\rTCP x {p[0] * 1e3:7.1f}  y {p[1] * 1e3:7.1f}  z {p[2] * 1e3:7.1f} mm   tilt {tilt:4.1f} deg   "
                  f"opening {st['grip_m'] * 1e3:5.1f} mm   offset dx {d[0]:+7.1f}  dy {d[1]:+7.1f} mm ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        arm.close()


if __name__ == "__main__":
    main()
