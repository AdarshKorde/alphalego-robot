# Step 4.0 TCP / table check, 2026-11-13 (drag mode, gripper closed by hand, motor off)

Expected fingertip z on the bare tabletop: -40.0 mm (arm frame). Spots: the earlier spot at (250, 39) plus the
5-spot series in `table_series.txt` (q in deg, tcp from deploy/core.tcp_pose with tcp_z 0.1425).

| spot | tcp x, y (mm) | reach r (mm) | z error (mm) | tilt from vertical (FK, deg) |
|---|---|---|---|---|
| 0 | 246.3, 38.8 | 249 | -1.8 | 2.0 |
| 1 | 117.3, 104.2 | 157 | +0.9 | 2.5 |
| 2 | 211.8, 257.1 | 333 | -2.3 | 3.4 |
| 3 | 154.0, 262.1 | 304 | -2.3 | 1.8 |
| 4 | 351.7, 61.6 | 357 | -2.7 | 1.1 |
| 5 | 110.8, 112.6 | 158 | +0.5 | 1.4 |

Fits (6 points; none can be told apart with this data):
- error vs reach: +3.26 - 0.0174 * r(mm), rms residual 0.39 mm (1.7 mm lower per 100 mm of reach)
- tilted plane (table or base tilt): 1.10 deg, falling toward (+0.85, +0.52), rms residual 0.48 mm
- J2 + J3 zero offsets: J2 -1.58 deg, J3 +2.04 deg, rms residual 0.23 mm (max 0.34)
- J2 alone -0.34 deg (rms 0.99 mm) or J3 alone -0.49 deg (rms 1.27 mm): poor

Over the task cells (x 166-246, y 41-121 mm) the error runs from about +0.4 to -1.8 mm, so the arm sees the plate
area ~2 mm lower at the far side than the near side. Operator judged every pose vertical and flush; FK read 1.1-3.4
deg from vertical. Result: FAIL (> 1.5 mm), step 4 stopped here as the prompt asks.
