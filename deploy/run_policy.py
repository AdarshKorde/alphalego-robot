"""Run the phase-0 policy on the Piper at 25 Hz (exported bundle, numpy only; checked against the sim by
tasks/phase0/deploy_parity.py: inputs identical to 1e-6).

Modes, in the order to try them:
  dry     no robot: a simulated arm and a virtual brick. Tests the software and its timing anywhere.
  shadow  robot connected, NEVER enabled or commanded: the policy runs on the live joints, commands are only logged.
  air     robot moves, everything 10 cm above the plate, no brick: a virtual brick makes the gripper reading look
          like the real thing. Checks the motion end to end where nothing can be hit.
  real    the real task. A brick must sit unpressed on --cell, square to the plate (--yaw-deg multiple of 90).

Each tick (40 ms): read the newest state -> build the 120 inputs -> policy -> joint rules -> send -> checks -> log.
Safety: joint limits at start, feedback age, arm status/errors, teach mode, fingertip height, tracking error, joint
effort; on any fault (or Ctrl+C) the arm is commanded to hold where it is and the loop stops. It never disables.

Run:  python deploy/run_policy.py --mode dry --cell 3 2
      python deploy/run_policy.py --mode air --cell 3 2 --confirm
"""
from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np

import core
from piper_io import DryArm, PiperArm

HERE = Path(__file__).resolve().parent
AIR_LIFT = 0.10


class VirtualBrick:
    """Stand-in brick for dry/air runs: held when the closed gripper is on it, follows the fingertips, released on
    opening, pressed down by closed fingertips on its top. Only the gripper reading and the run outcome use it."""

    def __init__(self, cfg, plate, pos, R):
        g = cfg["geometry"]
        self.g, self.plate = g, plate
        self.pos, self.R = pos.copy(), R.copy()
        self.top = g["brick_h"] + g["stud_h"]
        self.held, self.off = False, None
        self.outcome = None
        self.press_t = 0.0

    def between_fingers(self, tcp_p):
        rel = tcp_p - self.pos
        return np.linalg.norm(rel[:2]) < 0.012 and 0.002 < rel[2] < self.top + 0.008

    def update(self, tcp_p, tcp_R, opening, dt):
        g = self.g
        if not self.held and opening < g["brick_w"] + 0.002 and opening > g["brick_w"] - 0.002 and \
                self.between_fingers(tcp_p):
            self.held, self.off = True, tcp_R.T @ (self.pos - tcp_p)
        if self.held:
            if opening > g["brick_w"] + 0.004:
                self.held = False
                h = self.pos[2] - self.plate.slab_top_z
                if h > g["h_pre"] + 0.0025:
                    self.outcome = "dropped"
                self.pos[2] = self.plate.slab_top_z + min(h, g["h_pre"])
            else:
                self.pos = tcp_p + tcp_R @ self.off
                return
        over = np.linalg.norm((tcp_p - self.pos)[:2]) < g["brick_w"] / 2
        if opening < g["press_max_opening"] and over and tcp_p[2] < self.pos[2] + self.top + 0.0005:
            self.press_t += dt
            if self.press_t > 0.3:
                txy = self.plate.target_xy
                on_target = np.linalg.norm(self.pos[:2] - txy) < 0.002
                self.outcome = "seated_on_target" if on_target else "seated_elsewhere"
        else:
            self.press_t = 0.0


def check(st, cfg_r, rules, tcp_p, plate, mode, now):
    if now - st["t"] > cfg_r["feedback_max_age_s"]:
        return f"feedback {1000 * (now - st['t']):.0f} ms old"
    s = st["status"]
    if s["arm_status"] != 0:
        return f"arm status 0x{s['arm_status']:02X}"
    if s["ctrl_mode"] == 0x02:
        return "arm switched to teach mode"
    if s["comm_err"]:
        return "joint communication error"
    if tcp_p[2] < plate.slab_top_z + cfg_r["fingertip_min_above_slab_m"]:
        return f"fingertip {1000 * (tcp_p[2] - plate.slab_top_z):.1f} mm above the slab"
    if mode != "shadow":
        err = np.abs(st["q"] - rules.target)
        if err.max() > cfg_r["tracking_error_max_rad"]:
            return f"J{err.argmax() + 1} tracking error {math.degrees(err.max()):.1f} deg"
    eff = np.abs(st["effort"]) > np.array(cfg_r["effort_abort_nm"])
    if eff.any():
        j = int(np.argmax(eff))
        return f"J{j + 1} effort {st['effort'][j]:.2f} N*m over {cfg_r['effort_abort_nm'][j]}"
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["dry", "shadow", "air", "real"], required=True)
    ap.add_argument("--cell", type=int, nargs=2, required=True, help="start cell (a b) of the brick, plate studs")
    ap.add_argument("--yaw-deg", type=float, default=0.0, help="brick yaw on the plate, multiple of 90")
    ap.add_argument("--confirm", action="store_true", help="required for air/real: the arm will move")
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()

    cfg = core.load_config()
    cfg_r = json.loads((HERE / "robot_config.json").read_text())
    calib_path = HERE / cfg_r["plate_calib_file"]
    calib = json.loads(calib_path.read_text()) if calib_path.exists() else None
    if args.mode == "real" and calib is None:
        raise SystemExit(f"mode real needs a taught plate: run teach_plate.py first ({calib_path})")
    if args.mode == "air":                       # the whole task 10 cm up: nothing near the plate can be hit
        base = calib or {"plate_pos": cfg["geometry"]["plate_pos"], "plate_yaw": 0.0}
        calib = {**base, "plate_pos": [base["plate_pos"][0], base["plate_pos"][1], base["plate_pos"][2] + AIR_LIFT]}
    plate = core.Plate(cfg, calib)
    if calib:
        d = (np.array(plate.pos) - np.array(cfg["geometry"]["plate_pos"])) * 1e3
        print(f"plate vs sim: dx {d[0]:+.1f} dy {d[1]:+.1f} dz {d[2]:+.1f} mm, yaw {math.degrees(plate.yaw):+.2f} deg"
              + ("  (air: +100 mm on purpose)" if args.mode == "air" else ""))
        if args.mode == "real" and (np.linalg.norm(d[:2]) > 5 or abs(d[2]) > 5):
            print("WARNING: the plate is > 5 mm from where the policy was trained; robustness was only tested to a few mm")

    policy, rules, obs_b = core.Policy(), core.ActionRules(cfg), core.ObsBuilder(cfg, plate)
    g = cfg["geometry"]
    bxy = plate.cell_centre(*args.cell)
    brick_p = np.array([bxy[0], bxy[1], plate.slab_top_z + g["h_pre"]])
    brick_R = core.rot_z(math.radians(args.yaw_deg) + plate.yaw)

    arm = DryArm(cfg["geometry"]["q_home"]) if args.mode == "dry" else PiperArm(cfg_r["can"])
    st = arm.state()
    qd = np.degrees(st["q"])
    lo, hi = np.array(cfg_r["joint_min_deg"]), np.array(cfg_r["joint_max_deg"])
    bad = (qd < lo - cfg_r["start_margin_deg"]) | (qd > hi + cfg_r["start_margin_deg"])
    if bad.any():
        raise SystemExit(f"start pose outside joint limits: {np.round(qd, 1)} (J6 has been seen at 171 deg)")
    vb = VirtualBrick(cfg, plate, brick_p, brick_R) if args.mode in ("dry", "air") else None
    if isinstance(arm, DryArm):
        arm.brick_between = lambda: vb.between_fingers(core.tcp_pose(arm.q, g["tcp_z"])[0])
    if args.mode in ("air", "real"):
        if not args.confirm:
            raise SystemExit("air/real move the arm: add --confirm (e-stop in reach, workspace clear)")
        input(f"{args.mode.upper()}: the arm will move. E-stop in reach? Press Enter to start, Ctrl+C to abort ")
        arm.enable(cfg_r["gripper_effort"])

    rate = cfg_r["rate_hz"]
    dt = 1.0 / rate
    n_ticks = int(round((args.seconds or cfg_r["episode_s"]) * rate))
    rules.reset(st["q"])
    obs_b.reset()
    last_a = np.zeros(7)
    log = {k: [] for k in ("t", "q", "target", "action", "opening", "opening_raw", "effort", "lat_ms", "obs")}
    reason, t0 = None, time.perf_counter()
    try:
        for k in range(n_ticks):
            t_tick = t0 + k * dt
            while (rem := t_tick - time.perf_counter()) > 0:
                time.sleep(rem - 0.0005 if rem > 0.001 else 0)
            t_read = time.perf_counter()
            st = arm.state()
            q = st["q"]
            tcp_p, tcp_R = core.tcp_pose(q, g["tcp_z"])
            reason = check(st, cfg_r, rules, tcp_p, plate, args.mode, time.monotonic())
            if reason:
                break
            raw = st["grip_m"]
            if args.mode == "real" or args.mode == "shadow":
                opening = core.gripper_real_to_sim(raw, cfg_r["gripper_real_to_sim"])
            elif args.mode == "air":    # nothing in the real gripper: report the virtual brick's width while held
                opening = g["brick_w"] if (vb.held or (raw < g["brick_w"] and vb.between_fingers(tcp_p)))\
                    else core.gripper_real_to_sim(raw, cfg_r["gripper_real_to_sim"])
            else:
                opening = raw
            obs = obs_b(q=q, target=rules.target, opening=opening, last_action=last_a, tcp_p=tcp_p,
                        brick_p=brick_p, brick_R=brick_R)
            a = policy(obs)
            target, close = rules.step(a, q)
            grip_um = cfg_r["gripper_close_um"] if close else cfg_r["gripper_open_um"]
            if args.mode != "shadow":
                arm.command(target, grip_um, cfg_r["gripper_effort"])
            lat = (time.perf_counter() - t_read) * 1e3
            for key, v in (("t", t_read - t0), ("q", q), ("target", target), ("action", a), ("opening", opening),
                           ("opening_raw", raw), ("effort", st["effort"]), ("lat_ms", lat), ("obs", obs)):
                log[key].append(v)
            last_a = a
            if vb is not None:
                vb.update(tcp_p, tcp_R, opening, dt)
                if vb.outcome:
                    reason = f"virtual brick: {vb.outcome}"
                    break
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    finally:
        if args.mode in ("air", "real"):
            st = arm.state()
            arm.command(st["q"], grip_um if log["t"] else cfg_r["gripper_open_um"], cfg_r["gripper_effort"])  # hold
        arm.close()

    t = np.array(log["t"])
    lat = np.array(log["lat_ms"])
    if len(t) > 2:
        per = np.diff(t) * 1e3
        print(f"ticks {len(t)}  period ms: median {np.median(per):.1f}  p99 {np.percentile(per, 99):.1f}  "
              f"max {per.max():.1f}   read->send ms: median {np.median(lat):.2f}  p99 {np.percentile(lat, 99):.2f}  "
              f"max {lat.max():.2f}")
        if np.percentile(lat, 99) > cfg_r["timing_budget_ms"] or np.percentile(np.abs(per - 1e3 * dt), 99) > 2.0:
            print("WARNING: timing outside budget (read->send < 5 ms, period 40 +- 2 ms)")
    print("stopped:", reason or "episode time up")
    out = HERE / "logs"
    out.mkdir(exist_ok=True)
    f = out / f"{datetime.now():%Y%m%d_%H%M%S}_{args.mode}_cell{args.cell[0]}{args.cell[1]}.npz"
    np.savez(f, **{k: np.array(v) for k, v in log.items()}, reason=np.array(reason or ""),
             cell=np.array(args.cell), yaw_deg=np.array(args.yaw_deg), mode=np.array(args.mode))
    print("log:", f)


if __name__ == "__main__":
    main()
