# AlphaLego: real-robot side (run this on the laptop the Piper is plugged into)

You are helping with **AlphaLego**: training an RL policy in Isaac Lab (on a separate workstation) to place
LEGO **Duplo** bricks with an **AgileX Piper** arm, then transferring it to the real arm. The workstation does
sim, training and model fitting. **This laptop does only the real-robot work**, because the USB-CAN adapter is
plugged in here. Forwarding CAN over NoMachine was measured at 132 Hz instead of ~200 Hz, with p99 61 ms and
max 292 ms gaps. That is useless for control.

Your job, in order:
1. Set up the environment and verify the CAN link (read-only).
2. Write and run a **system-identification (sysID) recording script**.
3. Package the logs for the workstation.

Ask the user before **every** step that makes the arm move.

---

## Hardware and facts (verified; don't re-derive)

- **Arm and gripper:** AgileX **Piper** (not Piper-X), firmware **S-V1.8-8**, with the **100 mm AgileX gripper**.
  - Gripper travel is 0–100 mm. `piper_sdk` clips the gripper to 0–70 mm by default, so call
    `SetSDKGripperRangeParam(0.0, 0.1)`.
- **Adapter:** candleLight / gs_usb USB-CAN adapter (`1d50:606f`). Bring it up with
  `sudo ip link set can0 up type can bitrate 1000000`.
- **Mounting:** the arm is bolted on top of 40 mm aluminium rails, so **the tabletop is at z = −40 mm in the
  arm base frame**. The arm faces +x.
  - Rails run along y at x ∈ [−55, −15] and [15, 55] mm, and along x at y ∈ [−55, −15] mm. All are 40 mm tall
    (z −40…0 in the arm frame).
  - The Duplo baseplate lies on the tabletop at x 150…534, y 25…409 mm.
- **TCP:** the fingertip is **142.5 mm** along the link6 z-axis from the link6 (flange) frame.
- **Firmware kinematics:** `piper_sdk` FK (`C_PiperForwardKinematics(dh_is_offset=1)`) matches our sim model
  exactly. Use it for all safety checks: flange pose, then add 142.5 mm along its z-axis.
- **Joint limits (SDK):**

  | Joint | J1 | J2 | J3 | J4 | J5 | J6 |
  |---|---|---|---|---|---|---|
  | Range (°) | ±150 | 0…180 | −170…0 | ±100 | ±70 | ±120 |

  - Max speed is 3 rad/s per joint.
- **SDK units:** joints are 0.001°; gripper is 0.001 mm.
  - `GetArmHighSpdInfoMsgs()`: motor_speed is 0.001 rad/s, current is 0.001 A, effort is 0.001 N·m.
- **Reference control code:** the lab's teleop repo, `piper_teleop/arms.py` class `Follower`, at
  https://github.com/priyabiswas-cmd/agile_x_piper_setup. Copy its enable sequence and command path:
  - enable: `EnablePiper()` in a loop; then `GripperCtrl(0, effort, 0x02, 0)` (clear errors), then
    `GripperCtrl(pos, effort, 0x01, 0)` (enable);
  - every command tick: `MotionCtrl_2(0x01, 0x01, speed_pct, 0x00)` (CAN control, MOVE J), then
    `JointCtrl(j1..j6 in 0.001 deg)`, then `GripperCtrl(...)`;
  - Ctrl+C: stop commanding. The arm **holds** its last target. **Never** call `DisablePiper()` with the arm in
    the air, because it goes limp and falls.
- **Container quirk:** the workstation container lacks `ip`, so it uses `judge_flag=False`. On the laptop,
  the default is fine.

## Safety rules (non-negotiable)

1. **Ask the user before every motion.** Each time, confirm: e-stop in reach, workspace clear, nothing on the
   baseplate.
2. **Check every planned trajectory offline before sending it.**
   - Run SDK FK on every target.
   - Reject the trajectory if the **fingertip z < +40 mm (arm frame)**, i.e. less than 80 mm above the tabletop.
   - Reject it if any joint comes within **5° of its limit**.
   - Reject it if the fingertip comes within 60 mm of a rail in x/y while below z = +60 mm.
3. **Watch the arm live during execution:**
   - stop commanding (the arm holds) if joint feedback is older than 100 ms;
   - stop if the measured fingertip z drops below +30 mm;
   - stop if |measured − commanded| exceeds 15° on any joint;
   - stop if the arm reports any error status.
4. **Moving to the start pose:** use `speed_pct = 20` and interpolate in joint space. Only start excitation once
   the arm is within 1° of the start pose.
5. Start each new kind of motion **small and slow**, show the user, then scale up.

## Task 1: environment and read-only check

```bash
python3 -m venv ~/alphalego-robot && source ~/alphalego-robot/bin/activate
pip install piper_sdk python-can numpy
sudo ip link set can0 up type can bitrate 1000000
python3 read_arm.py can0 --seconds 30      # provided next to this file; read-only, never moves the arm
```

**Expected:**
- joint feedback 0x2A5 at ~200 Hz (anything from ~180 to 210 Hz is fine), p99 inter-arrival under ~10 ms;
- firmware S-V1.8-8; arm status NORMAL.

Show the user the output. If the rate is low or jittery on the laptop too, stop and report it.

## Task 2: sysID recording script (`sysid_record.py`)

**Goal:** log how the real arm responds to the **exact command path the RL policy will use**. On the
workstation we then fit the sim's joint drive (stiffness, damping, friction, armature), the command latency,
and the firmware speed scaling, so that sim and real respond the same way. In the sim, the policy outputs
**relative joint-position targets at 25 Hz**: each tick it sends `q_target = q_target_prev + Δq`.

**Command path:** use exactly the Follower's path, `MotionCtrl_2(0x01, 0x01, speed_pct, 0x00)` +
`JointCtrl`, sent once per tick.

**Rates:**
- command ticks at **25 Hz** (40 ms) for the main segments;
- one extra segment at **200 Hz**, for comparison.

**Logging thread (independent of commands):**
- Poll the SDK at ≥ 500 Hz.
- Record a sample **whenever a joint feedback timestamp changes**:
  - host `time.monotonic()`;
  - the SDK message `time_stamp`;
  - joint positions (0x2A5–0x2A7, `GetArmJointMsgs`);
  - high-speed motor info per joint: speed, current, effort (`GetArmHighSpdInfoMsgs`);
  - gripper position and effort;
  - arm status.
- Also log every command sent: host time, `speed_pct`, the 6 joint targets, and the gripper target.

**Start pose:** `q_home = [0, 60, -50, 0, 50, 0]` deg. Its fingertip is ≈ (278, 0, 196) mm in the arm frame,
i.e. 236 mm above the tabletop. Verify this with SDK FK before using it.

**Segments.** Each segment starts from `q_home` and returns there at speed 20.
- **A. Steps, one joint at a time:** +5°, back, −5°, back; then the same with ±10°. Hold each for 1.5 s.
  Run for J1…J6.
- **B. Chirps, one joint at a time:** sine sweep from 0.1 to 4 Hz over 20 s, sent as 25 Hz targets.
  - Amplitude 4° below 1 Hz, tapering to 1° at 4 Hz.
  - Keep peak commanded joint velocity ≤ 1.5 rad/s.
- **C. Policy-like random walk, all 6 joints together, 60 s:**
  - each tick Δq ~ U(−1.5°, +1.5°) per joint;
  - add a pull back toward `q_home` so no joint drifts more than ±12°;
  - this is closest to what the policy will do.
- **D. Gripper:** open 0→100 mm, close 100→0 mm, then steps 0→30→60→30→0 mm.
  - Run at gripper effort 1.0 N·m, then 2.0 N·m.
  - The arm holds `q_home` throughout.
- **E. Same as C but at 200 Hz command rate,** with Δq scaled by 25/200, for 30 s.

**Speed settings:** run segments A, B and C at **speed_pct = 30, 60 and 100**. We will pick the policy's
speed from these. D and E at 60 only. Total motion time is roughly 20 minutes. Split the runs so the user can
pause between them.

**Output:**
- one `.npz` per segment run in `~/alphalego_sysid/<YYYYmmdd_HHMM>/`, named like
  `A_steps_J3_spd60.npz`, containing:
  - `fb_t`: host time;
  - `fb_sdk_t`: SDK timestamp;
  - `fb_q_mdeg` (N×6);
  - `fb_motor_speed`, `fb_current`, `fb_effort` (N×6);
  - `fb_grip_um`, `fb_grip_effort`;
  - `cmd_t`, `cmd_q_mdeg` (M×6), `cmd_grip_um`, `cmd_speed_pct`;
- a `meta.json` holding the segment parameters, firmware version, SDK version, `q_home`, rates, and the date.

**Before running anything:**
- Dry-run each segment offline: print its duration, joint ranges, minimum fingertip z, and peak velocity.
- Show these to the user for approval, then execute.

## Task 3: hand back

Send `~/alphalego_sysid/` to the workstation into the host folder that is mounted in the Isaac container as
`/workspace/my_isaac_project` (ask the user for its host path, e.g. via Tailscale):

```bash
scp -r ~/alphalego_sysid adarsh@tandoor:<HOST_PATH_OF_my_isaac_project>/alphalego_sysid
```

Also send `sysid_record.py` itself, so the workstation can reuse the exact command code for policy deployment.
Then tell the user to go back to the workstation session and say "sysID logs are in my_isaac_project".
