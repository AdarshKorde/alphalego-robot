# DEPLOY_PROMPT step report, 2026-11-13

Piper S-V1.8-8, piper_sdk 0.6.2, `can0` 1 Mbit/s, venv `~/alphalego-venv`. All runs with
`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1` unless noted (see step 1).

## Step 1: software check (dry) — PASS (with BLAS fix)
| log | period median / p99 (ms) | read->send p99 (ms) |
|---|---|---|
| `174500_dry_cell32` (default BLAS) | 40.00 / 40.00 | 5.06 |
| `174512_dry_cell50` (default BLAS) | 40.00 / 40.00 | 4.73 |
| `174516_dry_cell04` (default BLAS) | 40.00 / 40.00 | 4.32 |
| `174526`, `174530_dry_cell32` (default BLAS) | 40.00 / 40.00-40.06 | 5.52, 5.20 |
| `174546`, `174549`, `174552_dry_cell32` (BLAS 1 thread) | 40.00 / 40.00 | 1.89, 3.66, 2.04 |

With numpy/OpenBLAS on 8 threads, read->send p99 went above 5 ms. With 1 BLAS thread it stays at 1.9-3.7 ms.
**Always set `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1` for `run_policy.py`.**

## Step 2: shadow — PASS
`175028`, `175033`, `175038_shadow_cell32`: period median 40.00 ms, p99 41.32 / 40.07 / 41.64 ms; read->send p99
3.08 / 2.82 / 3.63 ms. No warnings. The arm was not enabled.

## Step 3: gripper calibration (effort 3000)
| state | reading |
|---|---|
| (a) closed, empty | 0.30 mm |
| (b) closed on a 2x2 brick | 36.00 mm (std 0.0 over 1 s) |
| (c) commanded 50000 um | 50.20 mm |

`gripper_real_to_sim = [[0.0003, 0.0], [0.0360, 0.0318], [0.0502, 0.050]]`.

- **Slip:** the plate clicked fully onto the gripped brick. The operator did not measure the pen mark and judged the
  force good.
- **Grip force:** effort 3000 read back −2.087 N·m, the same as effort 2000 (−2.086 N·m), so the grip force may be
  capped near 2 N·m. The 1500/4000 check was not run.
- **Data:** `results/20261113_gripcal/`.

## Step 4: plate teaching — TCP check FAILED, continued at the operator's request
**4.0 table check** (6 spots; expected −40.0 mm):
- Errors: +0.9 and +0.5 mm near the base (reach about 157 mm); −1.8 to −2.7 mm far out (reach 249-357 mm).
- Fits, none distinguishable with this data:
  - error vs reach: −1.7 mm per 100 mm (rms 0.39 mm);
  - table or base tilt: 1.1° (rms 0.48 mm);
  - J2/J3 zero offsets: −1.58° / +2.04° (rms 0.23 mm).
- A spirit level on the base and table would separate the tilt explanation from the joint-offset one.
- Details: `calib/table_check_analysis.md`.

**Teaching**, done in drag mode:
- In teach mode the firmware ignores gripper position commands. The gripper motor was turned off
  (`teach_gripper.py gripoff`) and the fingers were closed on the brick by hand.
- All 4 cells were gripped at both wrist angles, and the stud tops were touched at 5 spots.
- The cell taught as `0 0` is **the plate's 6th stud along x and 7th along y**. I assumed the count runs from the
  plate corner nearest the base on its right; **the operator has not confirmed this.** It is not the sim's cell 0 0.

**Fit**, in `calib/plate.json`:
- Plate vs sim: dx +34.9, dy +54.5, dz −1.0 mm; yaw −0.05°.
- Grip residuals are within 0.88 mm (PASS against 1 mm).
- Stud-top z spread is 1.1 mm (warning against 0.7 mm). The far side reads lower, which matches the reach error
  above rather than a bowed plate.

**4.7: the plate was not moved to the sim position.** The sim will be retrained for this plate position instead.

## Step 5: air runs
The operator said "go" before each run. The operator also asked that runs not be stopped for going out of bounds,
so some runs used `--sim-plate` and `--air-floor-mm`.

| log | setup | stop reason | max \|effort\| J1..J6 (N·m) | read->send p99 |
|---|---|---|---|---|
| `192419_air_cell32` | taught plate | fingertip 0.8 mm above the (raised) slab, 1.4 s | 0.33 0.63 1.47 0.43 1.01 0.38 | 1.26 ms |
| `192716_air_cell50` | `--sim-plate` | fingertip 1.4 mm above the slab | 0.34 0.85 1.56 0.33 0.89 0.37 | 1.79 ms |
| `193048_air_cell04` | sim plate, floor +10 mm | fingertip z 8.5 mm < floor | 0.33 0.79 1.16 0.41 0.82 0.33 | 1.65 ms |
| `193528_air_cell04` | **started from a low pose by mistake** (see below), floor −25 | time up (12 s) | 0.41 1.21 1.14 0.61 0.66 0.36 | 2.50 ms |
| `193830_air_cell04` | from q_home, sim plate, floor −25 | time up (12 s) | 0.33 1.15 1.25 0.55 0.89 0.39 | 2.40 ms |

Every run had a period median of 40.00 ms and a p99 of 40.01-40.11 ms.

**What the motion looked like:**
- **Cell 3 2, taught plate:** the arm reached (249, 102) mm. The virtual brick was at (248.8, 127.6), so the arm
  stopped 25.6 mm short in y. It then went straight down.
- **Cell 5 0, sim plate:** the gripper closed at z about 137 mm, 70 mm above the virtual brick, and 11 mm off in x.
- **Cell 0 4, final run from q_home:** the arm went through the whole pick-and-place shape, at the wrong height and
  offset:
  1. It opened and descended to about (175, 80, −3) mm. The virtual brick was at (166, 105) and 100 mm higher.
  2. At 2.1 s it closed on nothing at z −5.
  3. It carried to about (170, 33, −13), about 9 mm from the sim target (166, 41).
  4. It hovered there with the gripper closed until time ran out. It never lowered or released.
  - Lowest point: −13.3 mm, about 22 mm above the real stud tops.
  - It ended with **J6 at its 120° limit**.

**Why air mode puts the policy out of distribution:** in the sim the policy starts about 234 mm above the slab with
the gripper open. In air mode:
- it starts about 134 mm above the raised slab;
- the H move to q_home closes the gripper first;
- its inputs include absolute joint angles and TCP position, so it aims for about the **real** plate height,
  not the virtual one 10 cm up.

Air mode is therefore not a faithful dry rehearsal for this policy. The step-4 plate offset (+35, +55 mm) also means
real mode would place at the sim's spot, not the taught cell, until the sim is retrained.

**`effort_abort_nm` is not changed.** The runs were short or out of distribution, so they are not a good basis for
the limit. The highest effort seen was 1.56 N·m (J3).

**Operator mistake (mine), corrected:**
- I chained `sysid_record H ... | grep ... && run_policy ...`. The H move was rejected (start z 8.4 mm < 40), but
  grep succeeded, so run `193528` started from the low pose instead of q_home.
- Exit statuses are now checked separately before any further run.

## New tools and options
- **`deploy/teach_gripper.py`:** gripper-only control during drag mode (goto, gripoff, gripon, status). It never
  sends joint commands.
- **`deploy/watch_tcp.py`:** read-only live readout of TCP x/y/z, tilt and opening, plus the offset from sim cell
  0 0.
- **`run_policy.py --mode air --sim-plate`:** puts the virtual task over the sim plate instead of `calib/plate.json`.
- **`run_policy.py --air-floor-mm Z`:** in air mode, aborts when the fingertip goes below Z.
- **`grip_test.py`:** gained goto, gripoff, setzero and gripon. This is the version used for the gripcal run; its
  sha matches `meta.json`.
- **Root `sysid_record.py`, not committed here:** `--allow-low-start` lets H and P moves start below the rail if the
  lead-in only rises.

## Open items
- Confirm which plate corner the 6th/7th-stud count starts from.
- Decide whether to retrain the sim with this plate position, or move the plate.
- The TCP/table error (about 2 mm over the task area) is unresolved. Check it with a spirit level.
- Check the grip effort cap (grip at 1500 and at 4000).
