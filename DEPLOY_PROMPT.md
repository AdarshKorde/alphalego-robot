You are helping bring the AlphaLego phase-0 policy (pick one Duplo 2x2 brick from a known cell, place it on the
target cell, press it in) from simulation to a real AgileX Piper. Work in the `alphalego-robot` repo on this laptop
(CAN `can0`, firmware S-V1.8-8, piper_sdk 0.6.2, 100 mm gripper with ~1 mm rubber pads). A human operator is at the
robot the whole time with the e-stop in reach. This prompt covers setup, measurements and the first safe tests only;
it stops BEFORE any real pick-and-place. Do the steps in order; report after each step before starting the next.

## What is in `deploy/`
- `bundle/`: the trained policy (numpy weights + config). Do not edit.
- `core.py`: builds the policy's 120 inputs and applies the joint rules exactly like the simulator (verified in sim:
  inputs identical to 1e-6, 92% success when the sim is driven by this code).
- `run_policy.py`: the 25 Hz loop. Modes: `dry` (no robot), `shadow` (robot connected, never enabled or moved),
  `air` (whole task 10 cm above the plate, no brick), `real` (NOT in this prompt).
- `teach_plate.py`: measures where the plate is with the arm (read-only).
- `robot_config.json`: settings; fields marked TODO are filled in below.
- `piper_io.py`: Piper feedback thread + command path (same as `results/sysid_record.py`).

## Safety rules (always)
- Never run `air` or `real` without the operator saying "go" for that exact run. Ctrl+C stops the loop and commands
  the arm to hold where it is; the script never disables the arm.
- Before enabling: no joint more than 3 deg outside its limits (J6 has been seen at 171 deg; ask the operator to
  turn it back by hand). `run_policy.py` refuses to start otherwise.
- If anything looks wrong (noise, shaking, unexpected motion), stop and report. Do not tune limits upward without
  the operator's OK.

## Step 1: software check (no robot)
`git pull`; the deploy code needs only numpy (+ piper_sdk for the robot steps).
Run `python deploy/run_policy.py --mode dry --cell 3 2` three times with different cells.
Report the printed timing line. Pass: period median 40.0 ms, p99 within 40 +- 2 ms; read->send p99 under 5 ms.
(The dry arm has no physics, so the task itself won't complete in dry mode; only the software and timing matter.)

## Step 2: shadow run (robot connected, arm does not move)
Arm powered, NOT enabled by this step. Put the arm at a comfortable pose above the plate.
Run `python deploy/run_policy.py --mode shadow --cell 3 2 --seconds 5`.
Report: the timing line, any warnings, and the log path (`deploy/logs/*_shadow_*.npz`). Pass: timing as in step 1
with the real CAN bus (if p99 read->send > 5 ms, report the numbers, do not continue).

## Step 3: gripper calibration (fills `gripper_real_to_sim`)
The real gripper reading differs from the sim's opening (it read 37.7 mm on a 31.8 mm brick at effort 1000).
At effort 3000 (`gripper_effort` in the config), record the steady gripper reading (`GetArmGripperMsgs`, average
1 s) in three states: (a) fully closed, nothing between the fingers; (b) closed on a 2x2 brick; (c) after
commanding 50000 um (open). Also do one pen-mark slip check at effort 3000 (grip a brick, push a plate up onto it by
hand until it clicks, check the mark). Write `gripper_real_to_sim` as
`[[a, 0.0], [b, 0.0318], [c, 0.050]]` (metres). Report the three readings and the slip result.

## Step 4: teach the plate (fills `deploy/calib/plate.json`)
Read the docstring of `deploy/teach_plate.py`. Find out from the Piper docs how to put the arm in teach (drag) mode,
or jog it in small steps with your own read-and-command tool (max 1 deg per step, operator approving).
1. Operator presses a brick FULLY onto cell `0 0` (the target, near corner of the plate).
2. Bring the open gripper straight down around the brick, fingertips ~6.5 mm above the brick's bottom edge, gripper
   vertical. Close gently. In `teach_plate.py` type `0 0` and Enter.
3. Open, turn the wrist (J6) 90 deg, grip the same brick again, type `0 0` again.
4. Repeat on cells `5 5`, `5 0`, `0 5` (both wrist angles each).
5. Remove the brick, close the gripper fully, rest the fingertips lightly on plate stud tops, type `z`. Do 4 spots
   spread over the cells the task uses (near 0 0, 5 0, 0 5, 5 5); `fit` warns if they differ by > 0.7 mm (bowed plate).
6. Type `fit`. Report the "plate vs sim" line and the residuals. Pass: all residuals within 1 mm (redo outliers).
7. The plate must be where the sim had it: the policy was trained with a fixed plate and places the brick at that
   trained spot, whatever the calibration says. If |dx| or |dy| > 1 mm or |yaw| > 0.5 deg, slide/turn the plate by
   the opposite amount and check again: each grip prints "+x.x mm from the sim cell along that axis", so gripping
   cell `0 0` at both wrist angles shows the target's offset directly (aim for both within +-1 mm), then `5 5` at
   both angles for the rotation. When it's in, do one full teach (1-6) and `fit` for the final `plate.json`.
The plate must stay where it is from now on (tape or clamp it).

## Step 5: air run (arm moves, 10 cm above the plate, no brick)
Operator: workspace clear, e-stop in hand, nothing on the plate within reach.
Run `python deploy/run_policy.py --mode air --cell 3 2 --confirm` (press Enter when asked).
The arm should go from its current pose toward the brick's (virtual) position 10 cm above the real one, close the
gripper, carry, lower, release and close again over the target. A virtual brick stands in for the real one.
Report: what the motion looked like, the stop reason, the timing line, the max |joint effort| per joint during the
run (from the log, field `effort`), and the log path. Then run cells `5 0` and `0 4` the same way.
Use the max efforts + ~50% margin to fill `effort_abort_nm` (only if the operator agrees).

## Output
Commit `deploy/calib/plate.json`, the updated `robot_config.json`, all `deploy/logs/*.npz` and a short
`deploy/STEP_REPORT.md` with the numbers above. Do not push without asking. Stop after step 5 and wait.
