"""Grip-slip test for the Piper 100 mm gripper (GRIP_TEST_PROMPT.md): the arm holds q_home, only the gripper moves.

  serve      enable, move to q_home at speed 20 (checked offline first), open to 60 mm, then hold the pose at 25 Hz
             with the sysID command path while reading one-line commands appended to <out>/ctl:
               close <effort> <try>   close to 20 mm (below the 31.8 mm brick) at effort (0.001 N·m, <= 5000),
                                      report steady opening/effort over 1.5..3.5 s after the close
               push / pushend         mark the operator's plate push (logging never stops)
               open                   open to 60 mm; 1.5 s later the trial is saved as grip_eff<e>_try<k>.npz
               status                 print measured joints, opening, effort, gripper status
               goto <mm> [effort]     move the gripper to an opening (0..100 mm), status printed 2 s later
               gripoff                disable the gripper (fingers move freely by hand)
               setzero                while disabled: set the current finger position as the gripper zero (0xAE)
               gripon                 re-enable the gripper, holding its current measured opening
               quit                   stop commanding (the arm holds q_home, it is never disabled)
  summarize  write <out>/SUMMARY.md from the .npz logs plus <out>/observations.json (operator's slip/click/tilt notes)

Safety: same live checks as sysid_record.py (feedback age, fingertip z, tracking / held-joint drift, arm status) plus
the gripper status byte (bits 0-5 are faults). On any fault or Ctrl+C the script stops commanding and the arm holds.

Run:  ~/alphalego-venv/bin/python grip_test.py serve --confirm
      echo "close 1000 1" >> results/<YYYYMMDD>_grip/ctl
      ~/alphalego-venv/bin/python grip_test.py summarize
"""
import argparse
import hashlib
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from sysid_record import (APPROACH_SPEED_PCT, BASE_RATE, PHASES, Q_HOME, J_MAX, J_MIN, Schedule, check_schedule,
                          connect, eased_move, execute, live_fault, print_stats, tip_pos)

OPEN_UM = 60_000
CLOSE_UM = 20_000  # below the 31.8 mm brick width, so the gripper squeezes at its effort limit
EFFORT_MAX = 5000  # 0.001 N·m
EFFORT_OPEN = 1000
HOLD_SPEED_PCT = 60  # as in sysID segment D
SETTLE_S, STEADY_S = 1.5, 2.0
SAVE_AFTER_OPEN_S = 1.5
SAVE_GRACE_S = 0.5  # converting a ~200 s trial (40k rows) holds the GIL ~110 ms and stalls the feedback thread
GRIP_FAULT_BITS = 0x3F  # low voltage, motor/driver over-temp, over-current, sensor, driver error
HOLD = PHASES.index("hold")
OUT_DEFAULT = Path(__file__).parent / "results" / f"{datetime.now():%Y%m%d}_grip"


def grip_status(arm):
    g = arm.GetArmGripperMsgs().gripper_state
    return int(g.status_code), int(g.grippers_angle), int(g.grippers_effort)


def steady_stats(fb, t0, t1):
    m = (fb["fb_t"] >= t0) & (fb["fb_t"] <= t1)
    o, e = fb["fb_grip_um"][m] / 1000.0, fb["fb_grip_effort"][m] / 1000.0
    return {"open_mm_mean": float(o.mean()), "open_mm_std": float(o.std()), "eff_nm_mean": float(e.mean()),
            "eff_nm_std": float(e.std()), "n": int(m.sum())}


def serve(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    ctl = out / "ctl"
    ctl.touch()
    ctl_pos = ctl.stat().st_size  # ignore commands written before this start

    arm, log = connect(args.can)
    q_meas = log.last_q.copy()
    code, open_um, _ = grip_status(arm)
    print(f"measured q (deg): {np.round(q_meas, 2).tolist()}   tip {np.round(tip_pos(q_meas)[0], 1).tolist()} mm   "
          f"gripper {open_um / 1000:.1f} mm status 0x{code:02X}")
    over = np.maximum(J_MIN - q_meas, q_meas - J_MAX)
    if over.max() > 3.0:  # the rest pose is ~2 deg past the J2/J3 limits; anything more would make clipping a jump
        j = int(over.argmax())
        raise SystemExit(f"J{j + 1} measured {q_meas[j]:.1f} deg is {over[j]:.1f} deg outside its limit "
                         f"[{J_MIN[j]:.0f}, {J_MAX[j]:.0f}]; refusing to plan (the first command would jump)")
    q_start = np.clip(q_meas, J_MIN, J_MAX)
    s = Schedule()
    if np.max(np.abs(q_start - Q_HOME)) > 1e-6:
        s.add(eased_move(q_start, Q_HOME, BASE_RATE), open_um, EFFORT_OPEN, APPROACH_SPEED_PCT, BASE_RATE, "approach")
    s.add(Q_HOME, open_um, EFFORT_OPEN, APPROACH_SPEED_PCT, BASE_RATE, "settle")
    s.add(np.tile(Q_HOME, (25, 1)), open_um, EFFORT_OPEN, APPROACH_SPEED_PCT, BASE_RATE, "hold")
    bad, st = check_schedule(s, q_start)
    print_stats("approach to q_home", bad, st, False)
    if bad:
        raise SystemExit("refusing to run: the approach failed the safety checks")
    if not args.confirm:
        raise SystemExit("plan only; rerun with --confirm once the operator approves (e-stop in reach, workspace clear)")

    meta_path = out / "meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {"trials": {}}
    meta.update({"started": datetime.now().isoformat(timespec="seconds"), "firmware": arm.GetPiperFirmwareVersion(),
                 "sdk_version": arm.GetCurrentSDKVersion().value, "can": args.can,
                 "script_sha256": {p: hashlib.sha256((Path(__file__).parent / p).read_bytes()).hexdigest()
                                   for p in ("grip_test.py", "sysid_record.py")},
                 "q_home_deg": Q_HOME.tolist(), "rate_hz": BASE_RATE, "hold_speed_pct": HOLD_SPEED_PCT,
                 "open_um": OPEN_UM, "close_um": CLOSE_UM, "settle_s": SETTLE_S, "steady_s": STEADY_S,
                 "command_path": "MotionCtrl_2(0x01, 0x01, speed_pct, 0x00); JointCtrl(0.001 deg); "
                                 "GripperCtrl(um, effort, 0x01, 0)",
                 "units": {"fb_grip_um": "0.001 mm", "fb_grip_effort": "0.001 N·m", "fb_effort": "0.001 N·m",
                           "fb_q_mdeg": "0.001 deg", "fb_t / cmd_t / events_t": "host time.monotonic() s",
                           "grip_status": "0x2A8 byte 6; bits 0-5 faults, bit 6 enabled, bit 7 zeroed"},
                 "wall_minus_monotonic_s": time.time() - time.monotonic()})
    meta_path.write_text(json.dumps(meta, indent=2))

    # enable like sysid_record.enable(), but keep the gripper at its current opening instead of closing it
    t0 = time.monotonic()
    while not arm.EnablePiper():
        if time.monotonic() - t0 > 5.0:
            raise SystemExit("arm did not report all six joints enabled within 5 s")
        time.sleep(0.01)
    arm.GripperCtrl(0, EFFORT_OPEN, 0x02, 0)  # clear gripper errors
    time.sleep(0.02)
    arm.GripperCtrl(open_um, EFFORT_OPEN, 0x01, 0)
    print("enabled; moving to q_home at speed 20", flush=True)
    _, reason = execute(arm, log, s)
    if reason:
        log.stop()
        raise SystemExit(f"STOPPED ({reason}): no longer commanding, the arm holds its last target")
    print(f"at q_home, measured q {np.round(log.last_q, 2).tolist()}; opening to {OPEN_UM / 1000:.0f} mm", flush=True)

    grip, eff = OPEN_UM, EFFORT_OPEN
    cmd = {"t": [], "q": [], "grip": [], "eff": [], "spd": [], "phase": []}
    gst = {"t": [], "code": []}
    trial = None  # dict while a brick is gripped
    timers = []  # (due time, action)
    t_changed = np.full(6, time.monotonic())
    qi = [int(v) for v in np.round(Q_HOME * 1000)]
    t_next = time.perf_counter()
    reason = None
    grace_until = 0.0
    grip_code, zero_pending = 0x01, False

    def save(tr):
        i_fb, i_cmd, i_g = tr["i0"]
        fb = log.arrays(i_fb, len(log.rows))
        steady = steady_stats(fb, tr["t_close"] + SETTLE_S, tr["t_close"] + SETTLE_S + STEADY_S)
        stem = f"grip_eff{tr['effort']}_try{tr['try']}"
        ev_n, ev_t = zip(*tr["events"])
        np.savez(out / f"{stem}.npz", **fb,
                 cmd_t=np.array(cmd["t"][i_cmd:]), cmd_q_mdeg=np.array(cmd["q"][i_cmd:], dtype=np.int64).reshape(-1, 6),
                 cmd_grip_um=np.array(cmd["grip"][i_cmd:], dtype=np.int64),
                 cmd_grip_effort=np.array(cmd["eff"][i_cmd:], dtype=np.int64),
                 cmd_speed_pct=np.array(cmd["spd"][i_cmd:], dtype=np.int64),
                 cmd_phase=np.array(cmd["phase"][i_cmd:], dtype=np.int64),
                 grip_status_t=np.array(gst["t"][i_g:]), grip_status=np.array(gst["code"][i_g:], dtype=np.int64),
                 events=np.array(ev_n), events_t=np.array(ev_t), effort=np.array(tr["effort"]),
                 try_=np.array(tr["try"]))
        m = json.loads(meta_path.read_text())
        m["trials"][stem] = {"started": tr["wall"], "effort": tr["effort"], "try": tr["try"], "steady": steady,
                             "events": dict(zip(ev_n, [t - tr["t_close"] for t in ev_t])), "n_fb": len(fb["fb_t"])}
        meta_path.write_text(json.dumps(m, indent=2))
        print(f"saved {stem}.npz ({len(fb['fb_t'])} feedback samples)", flush=True)

    def print_status():
        c, o, e = grip_status(arm)
        print(f"q {np.round(log.last_q, 2).tolist()}  opening {o / 1000:.2f} mm  effort {e / 1000:.3f} N·m  "
              f"gripper status 0x{c:02X} (enabled {c >> 6 & 1}, zeroed {c >> 7 & 1})  "
              f"cmd {grip / 1000:.1f} mm @ {eff} code 0x{grip_code:02X}", flush=True)

    def handle(line):
        nonlocal grip, eff, trial, grip_code, zero_pending
        w = line.split()
        now = time.monotonic()
        if w[0] == "close" and len(w) == 3:
            e, k = int(w[1]), int(w[2])
            if trial:
                return print("refused: a trial is active; send 'open' first")
            if not 0 < e <= EFFORT_MAX:
                return print(f"refused: effort must be 1..{EFFORT_MAX}")
            grip, eff = CLOSE_UM, e
            trial = {"effort": e, "try": k, "t_close": now, "events": [("close", now)],
                     "i0": (len(log.rows), len(cmd["t"]), len(gst["t"])), "wall": datetime.now().isoformat("T", "seconds")}
            trial["i_close"] = len(log.rows)
            timers.append((now + SETTLE_S + STEADY_S, "steady"))
            print(f"closing to {CLOSE_UM / 1000:.0f} mm at effort {e} (try {k})")
        elif w[0] in ("push", "pushend") and trial:
            trial["events"].append(("push_start" if w[0] == "push" else "push_end", now))
            print(f"marked {w[0]}")
            if w[0] == "push":
                trial["i_push"] = len(log.rows)
            else:
                ts = dict(trial["events"]).get("push_start", trial["t_close"])
                fb = log.arrays(trial.get("i_push", trial["i_close"]), len(log.rows))  # only the push rows: cheap
                m = fb["fb_t"] >= ts
                e = fb["fb_grip_effort"][m] / 1000.0
                o = fb["fb_grip_um"][m] / 1000.0
                if len(e):
                    print(f"during push: fb_grip_effort min {e.min():.3f} max {e.max():.3f} N·m, "
                          f"opening {o.min():.2f}..{o.max():.2f} mm")
        elif w[0] == "open":
            grip = OPEN_UM
            if trial:
                trial["events"].append(("open", now))
                timers.append((now + SAVE_AFTER_OPEN_S, "save"))
            print(f"opening to {OPEN_UM / 1000:.0f} mm")
        elif w[0] == "status":
            print_status()
        elif w[0] in ("goto", "gripoff", "setzero", "gripon") and trial:
            print(f"refused: a trial is active; send 'open' first")
        elif w[0] == "goto" and len(w) in (2, 3):
            mm, e = float(w[1]), int(w[2]) if len(w) == 3 else EFFORT_OPEN
            if not (0 <= mm <= 100 and 0 < e <= EFFORT_MAX) or grip_code != 0x01:
                return print(f"refused: need 0..100 mm, effort 1..{EFFORT_MAX}, gripper enabled")
            grip, eff = round(mm * 1000), e
            timers.append((now + 2.0, "status"))
            print(f"gripper to {mm:.1f} mm at effort {e}")
        elif w[0] == "gripoff":
            grip_code = 0x00
            print("gripper disabled: fingers are free to move by hand")
        elif w[0] == "setzero":
            if grip_code != 0x00:
                return print("refused: send 'gripoff' first")
            zero_pending = True
        elif w[0] == "gripon":
            _, o, _ = grip_status(arm)
            grip, eff, grip_code = min(max(o, 0), 100_000), EFFORT_OPEN, 0x03  # enable + clear errors, then 0x01
            print(f"gripper enabled, holding {grip / 1000:.2f} mm")
        elif w[0] == "quit":
            return "quit"
        else:
            print(f"ignored: {line!r}")

    try:
        while True:
            now = time.monotonic()
            reason = live_fault(log, Q_HOME, HOLD, now, t_changed)
            if reason and reason.startswith("joint feedback") and now < grace_until:
                reason = None  # host stall from save(), not the bus; the arm holds a fixed target meanwhile
            code, _, _ = grip_status(arm)
            if code & GRIP_FAULT_BITS:
                reason = f"gripper status 0x{code:02X}"
            if reason:
                break
            arm.MotionCtrl_2(0x01, 0x01, HOLD_SPEED_PCT, 0x00)
            arm.JointCtrl(*qi)
            if zero_pending:  # as piper_sdk demo/V2/piper_set_gripper_zero.py: disabled, then 0xAE
                arm.GripperCtrl(0, EFFORT_OPEN, 0x00, 0xAE)
                zero_pending = False
                timers.append((now + 1.0, "status"))
                print("sent set-zero (0xAE)", flush=True)
            else:
                arm.GripperCtrl(grip, eff, grip_code, 0)
                grip_code = 0x01 if grip_code == 0x03 else grip_code
            for key, v in zip(cmd, (now, qi, grip, eff, HOLD_SPEED_PCT, HOLD)):
                cmd[key].append(v)
            gst["t"].append(now)
            gst["code"].append(code)

            for due, action in [x for x in timers if x[0] <= now]:
                timers.remove((due, action))
                if action == "steady" and trial:
                    fb = log.arrays(trial["i_close"], len(log.rows))
                    st = steady_stats(fb, trial["t_close"] + SETTLE_S, trial["t_close"] + SETTLE_S + STEADY_S)
                    print(f"steady (effort {trial['effort']}, try {trial['try']}): opening {st['open_mm_mean']:.2f} "
                          f"± {st['open_mm_std']:.3f} mm, fb_grip_effort {st['eff_nm_mean']:.3f} ± "
                          f"{st['eff_nm_std']:.3f} N·m (n={st['n']}); gripper status 0x{code:02X}", flush=True)
                elif action == "status":
                    print_status()
                elif action == "save" and trial:
                    save(trial)
                    trial, eff = None, EFFORT_OPEN
                    grace_until = time.monotonic() + SAVE_GRACE_S
                    t_next = time.perf_counter()
            if ctl.stat().st_size > ctl_pos:
                with ctl.open() as f:
                    f.seek(ctl_pos)
                    lines = f.read().splitlines()
                    ctl_pos = f.tell()
                if any(handle(ln.strip()) == "quit" for ln in lines if ln.strip()):
                    break
                print(end="", flush=True)

            t_next += 1.0 / BASE_RATE
            while (rem := t_next - time.perf_counter()) > 0:
                time.sleep(rem - 0.0005 if rem > 0.001 else 0)
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    if trial:
        save(trial)
    log.stop()
    print(f"{'STOPPED (' + reason + ')' if reason else 'quit'}: no longer commanding; the arm holds q_home and the "
          f"gripper its last target (never disabled)", flush=True)


def summarize(args):
    out = Path(args.out)
    meta = json.loads((out / "meta.json").read_text())
    obs_path = out / "observations.json"
    obs = json.loads(obs_path.read_text()) if obs_path.exists() else {}
    rows, by_eff = [], {}
    for p in sorted(out.glob("grip_eff*_try*.npz"), key=lambda p: (int(p.stem.split("_")[1][3:]), p.stem)):
        d = np.load(p)
        ev = dict(zip(d["events"].tolist(), d["events_t"].tolist()))
        st = steady_stats(d, ev["close"] + SETTLE_S, ev["close"] + SETTLE_S + STEADY_S)
        t0, t1 = ev.get("push_start"), ev.get("push_end", ev.get("open"))
        peak = "n/a"
        if t0 is not None and t1 is not None:
            m = (d["fb_t"] >= t0) & (d["fb_t"] <= t1)
            if m.any():
                e = d["fb_grip_effort"][m] / 1000.0
                peak = f"{e[np.argmax(np.abs(e))]:.3f}"
        flags = sorted({f"0x{c:02X}" for c in d["grip_status"] if c & GRIP_FAULT_BITS})
        o = obs.get(p.stem, {})
        eff, k = int(d["effort"]), int(d["try_"])
        by_eff.setdefault(eff, []).append(o.get("slip"))
        notes = "; ".join(x for x in [o.get("notes", ""), f"peak fb effort during push {peak} N·m",
                                      "gripper faults " + ",".join(flags) if flags else ""] if x)
        rows.append(f"| {eff} | {k} | {st['eff_nm_mean']:.3f} ± {st['eff_nm_std']:.3f} | "
                    f"{st['open_mm_mean']:.2f} ± {st['open_mm_std']:.3f} | {o.get('clicked', '?')} | "
                    f"{o.get('slip', '?')} | {o.get('tilt', '?')} | {notes} |")
    no_slip = [e for e, s in sorted(by_eff.items()) if len(s) >= 3 and all(str(x) == "0" for x in s)]
    if no_slip:
        rec = min(round(no_slip[0] * 1.5, -2), 4000)
        concl = (f"Lowest effort with zero slip in all tries: **{no_slip[0]}** (0.001 N·m); "
                 f"recommended effort {rec:.0f} (x1.5, capped at 4000).")
    else:
        concl = "No effort setting had zero slip in 3 tries."
    md = [f"# Grip-slip test {out.name}", "",
          f"Firmware {meta.get('firmware')}, piper_sdk {meta.get('sdk_version')}, q_home {meta.get('q_home_deg')} deg, "
          f"close target {CLOSE_UM / 1000:.0f} mm, steady window {SETTLE_S}..{SETTLE_S + STEADY_S} s after close.", "",
          "| effort (0.001 N·m) | try | steady fb_grip_effort (N·m) | steady opening (mm) | clicked on? | slip (mm) | "
          "tilt? | notes |", "|---|---|---|---|---|---|---|---|", *rows, "", concl, ""]
    if args.extra:
        md += [Path(args.extra).read_text()]
    (out / "SUMMARY.md").write_text("\n".join(md))
    print("\n".join(md))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("serve", "summarize"))
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    ap.add_argument("--can", default="can0")
    ap.add_argument("--confirm", action="store_true", help="serve: the operator approved the move to q_home")
    ap.add_argument("--extra", help="summarize: markdown file appended to SUMMARY.md (observations, oddities)")
    args = ap.parse_args()
    serve(args) if args.mode == "serve" else summarize(args)


if __name__ == "__main__":
    main()
