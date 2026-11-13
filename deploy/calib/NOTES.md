# Plate teaching notes, 2026-11-13

- `plate.json` was taught after the Step 4.0 TCP check FAILED (table z error +0.9 to -2.7 mm, growing with reach;
  see `table_check_analysis.md`). The operator chose to continue.
- Arm moved by hand in teach (drag) mode. In teach mode the firmware ignores GripperCtrl position commands, so the
  gripper motor was disabled (`teach_gripper.py gripoff`) and the fingers were closed on the brick by hand
  (readings 39.3-40.3 mm on a 2x2 brick).
- The cell taught as `0 0` (the target) is NOT the sim's cell 0 0: it is the plate's **6th stud along x and 7th
  along y** (operator's count; assumed counted from 1 at the plate corner nearest the arm base on its right, i.e.
  smallest arm-frame x and y). The other taught cells are relative to it (5 0 = 5 studs further along +x, etc.).
- Fit: plate vs sim dx +34.9, dy +54.5, dz -1.0 mm, yaw -0.05 deg; grip residuals within 0.9 mm; stud-top spread
  1.1 mm (lower further out, matching the reach-dependent error of the table check rather than a bowed plate).
- The operator will NOT move the plate to the sim position (Step 4.7); the sim will be retrained for this plate
  position instead.
