"""Measure where the baseplate is, using the arm itself (read-only: never enables or commands the arm).

Idea: the arm always knows where its fingertips are (joint encoders -> forward kinematics). Put the fingertips at
known places on the plate and the arm has measured the plate.

  xy:  press a brick firmly onto a cell, close the gripper on it at grasp height, press Enter. Closing fingers centre
       the brick only along the CLOSING direction (along the finger width it can sit a few mm off), so each grip gives
       one exact coordinate. Grip each cell twice: fingers closing along the plate's x, then (wrist turned 90 deg)
       along y. At least 2 cells far apart (e.g. 0 0 and 5 5); 3-4 cells give an error check.
  z:   close the gripper fully and rest the fingertips lightly on the plate's stud tops (no brick), press Enter.

Moving the arm: use the Piper's teach (drag) mode, or jog it with your own tool; this script only reads.
Output: calib/plate.json with plate_pos (underside centre, sim convention), plate_yaw, per-grip residuals in mm.

Run:  python deploy/teach_plate.py
"""
from __future__ import annotations

import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np

import core

HERE = Path(__file__).resolve().parent


def average_state(arm, seconds=0.5):
    qs, gs = [], []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        st = arm.state()
        qs.append(st["q"])
        gs.append(st["grip_m"])
        time.sleep(0.01)
    return np.mean(qs, 0), float(np.mean(gs))


def fit(cfg, grips, z_touches):
    """Solve plate xy offset + yaw from 1-D constraints n_i . (c + R(yaw) l_i - p_i) = 0, z from the stud touches."""
    g = cfg["geometry"]
    plate0 = core.Plate(cfg)
    rows = []
    for gr in grips:
        local = plate0.cell_centre(*gr["cell"]) - plate0.pos[:2]         # cell centre relative to the plate centre
        rows.append((np.array(gr["closing_dir_xy"]), local, np.array(gr["tcp"][:2])))
    best = None
    for yaw in np.radians(np.linspace(-5, 5, 2001)):                      # yaw: small, search; c: linear least squares
        c, s = math.cos(yaw), math.sin(yaw)
        A = np.array([n for n, _, _ in rows])
        b = np.array([n @ (p - np.array([c * l[0] - s * l[1], s * l[0] + c * l[1]])) for n, l, p in rows])
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        res = A @ sol - b
        cost = float(res @ res)
        if best is None or cost < best[0]:
            best = (cost, yaw, sol, res)
    _, yaw, cxy, res = best
    out = {"plate_yaw": float(yaw), "residual_mm": [float(r * 1e3) for r in res]}
    if z_touches:
        stud_top = float(np.mean([t["tcp"][2] for t in z_touches]))
        slab_top = stud_top - g["stud_h"]
        z = slab_top - (g["slab_top_z"] - g["plate_pos"][2])
    else:
        z = g["plate_pos"][2]
        out["warning"] = "no z touch: plate height assumed from CAD"
    out["plate_pos"] = [float(cxy[0]), float(cxy[1]), float(z)]
    return out


def main():
    from piper_io import PiperArm
    cfg = core.load_config()
    cfg_r = json.loads((HERE / "robot_config.json").read_text())
    arm = PiperArm(cfg_r["can"])
    g = cfg["geometry"]
    grips, touches = [], []
    print(__doc__.split("Moving the arm")[0])
    print("Commands:  'a b' = grip on cell (a,b) now  |  z = fingertips on stud tops now  |  fit  |  quit")
    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd in ("quit", "q"):
                return
            if cmd == "fit":
                break
            q, grip = average_state(arm)
            p, R = core.tcp_pose(q, g["tcp_z"])
            if cmd == "z":
                touches.append({"tcp": p.tolist(), "q": q.tolist()})
                print(f"  stud-top touch at z {p[2] * 1e3:.1f} mm")
                continue
            try:
                a, b = (int(v) for v in cmd.split())
            except ValueError:
                print("  type 'a b' (cell), z, fit or quit")
                continue
            n = R[:, 1].copy()                                             # finger closing axis (fingertip y)
            n[2] = 0.0
            n /= np.linalg.norm(n)
            if abs(grip - g["brick_w"]) > 0.010:
                print(f"  WARNING: gripper reads {grip * 1e3:.1f} mm; is it closed on a 2x2 brick?")
            if abs(R[2, 2] + 1.0) > 0.02:
                print(f"  WARNING: gripper is {math.degrees(math.acos(min(1.0, -R[2, 2]))):.1f} deg from vertical")
            grips.append({"cell": [a, b], "tcp": p.tolist(), "closing_dir_xy": n[:2].tolist(), "q": q.tolist(),
                          "grip_m": grip})
            print(f"  cell ({a},{b}): fingertips at ({p[0] * 1e3:.1f}, {p[1] * 1e3:.1f}, {p[2] * 1e3:.1f}) mm, "
                  f"closing along ({n[0]:+.2f}, {n[1]:+.2f})")
    finally:
        arm.close()
    dirs = np.array([gr["closing_dir_xy"] for gr in grips])
    if len(grips) < 3 or np.linalg.matrix_rank(np.abs(dirs) > 0.5) < 2:
        raise SystemExit("need >= 3 grips with both closing directions (x and y) to fit position and rotation")
    out = fit(cfg, grips, touches)
    d = (np.array(out["plate_pos"]) - np.array(g["plate_pos"])) * 1e3
    print(f"plate vs sim: dx {d[0]:+.1f} dy {d[1]:+.1f} dz {d[2]:+.1f} mm, yaw {math.degrees(out['plate_yaw']):+.2f} "
          f"deg; residuals mm {np.round(out['residual_mm'], 2)}")
    if max(abs(r) for r in out["residual_mm"]) > 1.0:
        print("WARNING: a grip disagrees with the others by > 1 mm; redo that grip (brick fully pressed? gripper vertical?)")
    out.update({"created": datetime.now().isoformat(timespec="seconds"), "grips": grips, "z_touches": touches})
    f = HERE / cfg_r["plate_calib_file"]
    f.parent.mkdir(exist_ok=True)
    f.write_text(json.dumps(out, indent=1))
    print("saved", f)


if __name__ == "__main__":
    main()
