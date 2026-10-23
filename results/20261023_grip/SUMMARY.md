# Grip-slip test 20261023_grip

Firmware S-V1.8-8, piper_sdk 0.6.2, q_home [0.0, 60.0, -50.0, 0.0, 50.0, 0.0] deg, close target 20 mm, steady window 1.5..3.5 s after close.

| effort (0.001 N·m) | try | steady fb_grip_effort (N·m) | steady opening (mm) | clicked on? | slip (mm) | tilt? | notes |
|---|---|---|---|---|---|---|---|
| 1000 | 1 | -1.043 ± 0.003 | 37.70 ± 0.000 | yes | 0 | no | operator: no slip, no rotation; peak fb effort during push -1.074 N·m |
| 1000 | 2 | -1.043 ± 0.003 | 37.80 ± 0.000 | yes | ~1 | slight | operator: just a little rotation, slip very little ~1 mm; peak fb effort during push -1.064 N·m |
| 1000 | 3 | -1.043 ± 0.003 | 37.80 ± 0.000 | yes | 0 | no | operator: no slip, full click, no tilt; peak fb effort during push -1.066 N·m |
| 2000 | 1 | -2.086 ± 0.003 | 35.90 ± 0.000 | yes | 0 | no | operator: no slip, everything good; peak fb effort during push -2.101 N·m |

No effort setting had zero slip in 3 tries.

## Conclusion

- **Effort 1000 slipped in 1 of 3 tries** (~1 mm, with a slight rotation in try 2), so 1000 is not a safe grip.
- **Effort 2000 held with no slip, click or tilt problems, but in only 1 try** (the operator stopped after one; the
  protocol asks for 3). Taking 2000 as the lowest no-slip effort, the recommended effort is **3000** (2000 x 1.5),
  which matches the sim's ~30 N at effort 3000 assumption. 3000 itself was not tested.

## Effort feedback and opening

- `fb_grip_effort` follows the command: -1.043 N·m at 1000 and -2.086 N·m at 2000 (sign = closing), std 0.003 N·m.
  It reads the motor's own torque, so it does not give the fingertip force; a force gauge is still needed for the
  effort -> N mapping.
- During the plate push, `fb_grip_effort` moved only ±0.03 N·m (peaks -1.074 at 1000, -2.101 at 2000) and the
  opening did not change, so the push does not show up in the gripper feedback.
- Opening on the 31.8 mm brick reads 37.7-37.8 mm at effort 1000 and 35.9 mm at 2000. The ~1.9 mm drop is probably
  rubber-pad compression. The ~4-6 mm offset from the brick width is not explained; the gripper status never showed
  the "zeroed" bit (0x40, bit 7 unset), so the opening calibration is suspect. Not measured with calipers.

## Other observations

- Arm deflection during the push was at most 0.14 deg (J5) at either effort; no gripper fault bits, no brick damage
  reported.
- Before the test J6 read 171.6 deg (outside its ±120 limit, wrist rotated about half a turn); the operator turned it
  back by hand before enabling. `sysid_record.py` clips the measured pose to the limits and would have planned from
  a pose 50 deg away; `grip_test.py` now refuses to start if any joint is more than 3 deg outside its limits.
- The first server run stopped after try 1 on a 150 ms feedback gap that its own statistics caused (converting the
  whole trial held the Python GIL). The arm held q_home. Fixed (stats over only the needed rows, 0.5 s grace
  after a save) and restarted at the same pose; tries 2 onward ran without stops. Host jitter also gave gaps of
  74 ms and 94 ms in try 1.
- A first effort 2000 attempt closed on nothing (the operator could not place the brick); it is kept in
  `discarded/` and was redone.
