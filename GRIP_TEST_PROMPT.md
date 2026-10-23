You are helping run a short grip-slip test on an AgileX Piper arm for the AlphaLego project (sim-to-real LEGO Duplo
assembly). Work in the `alphalego-robot` repo on this laptop. The arm is on CAN `can0` (1 Mbit/s), firmware S-V1.8-8,
piper_sdk 0.6.2, 100 mm parallel gripper with rubber pads. A human operator is at the robot for the whole test.

## Goal
Find the lowest gripper effort at which a gripped Duplo 2x2 brick does NOT slide in the fingers while it is pressed onto
a Duplo baseplate hard enough to click into place. The sim assumes ~30 N grip at effort ~3000 (0.001 N·m units), based
on the spec sheet (40 N rated / 50 N max); this test checks that guess. Also record the gripper's own effort feedback
at each setting so the effort -> force mapping can be calibrated later.

## Safety
- The arm must NOT move during the test. Only the gripper moves. Enable the arm, move it once to a comfortable pose
  (e.g. home q = [0, 60, -50, 0, 50, 0] deg, at low speed) with the gripper pointing down, then hold that pose.
- Use the same command path as `results/sysid_record.py`: `MotionCtrl_2(0x01, 0x01, speed_pct, 0x00)` + `JointCtrl`
  (0.001 deg) + `GripperCtrl(opening_um, effort, 0x01, 0)`, sent every tick at 25 Hz, so the controller state matches
  the sysID runs.
- Never exceed effort 5000. Start low and work up. Keep fingers away from the jaws while they close.
- Ask the operator before every gripper close, and stop if anything looks or sounds wrong
  (e.g. motor noise, brick cracking, error flags in the feedback status).

## Procedure (operator does the pressing by hand; the arm only holds)
1. Operator marks the brick: a thin pen line on two opposite side faces, then the brick is placed between the open
   jaws (opening 60 mm) so the fingertips sit ~6.5 mm above the brick's bottom edge. Draw a second short line on the
   brick exactly at the bottom edge of each fingertip after the brick is gripped (this is the slip reference).
2. Close the gripper on the brick: command opening 20 mm (below the 31.8 mm brick width, so the gripper squeezes at its
   effort limit) at the test effort. Wait 1.5 s for it to settle.
3. Log for 2 s while holding still: `fb_grip_um`, `fb_grip_effort` (0.001 N·m), status flags. Report the mean and std
   of the steady effort reading and the opening.
4. Operator holds a loose Duplo baseplate (or a second Duplo brick, studs up) underneath and pushes it UP onto the
   gripped brick, increasing force until the brick clicks fully onto the studs. Keep logging during the push.
5. Operator checks the reference line against the fingertip: did the brick move up in the fingers? Estimate the shift
   in mm (0 / <0.5 / ~1 / >1 mm). Also: did the brick click fully on? Did it tilt in the grip?
6. Pull the plate off, open the gripper (60 mm), and wipe the pen line off before re-marking.
7. Repeat at effort 1000, 2000, 3000, then 4000 only if 3000 slipped. Do each setting 3 times.

## Output
Save raw logs to `results/<YYYYMMDD>_grip/` as `grip_eff<effort>_try<k>.npz` (same field names and units as the sysID
logs) plus `meta.json` (firmware, SDK version, timestamps, script hash). Write `SUMMARY.md` with one table:

| effort (0.001 N·m) | try | steady fb_grip_effort (N·m) | steady opening (mm) | clicked on? | slip (mm) | tilt? | notes |

and a one-line conclusion: lowest effort with zero slip in all 3 tries, and the effort you would recommend
(lowest no-slip effort x ~1.5, capped at 4000). Also report fb_grip_effort while the plate was being pushed (peak), and
anything odd (status flags, brick deformation, pads marked). Commit the script and results; do not push without asking.
