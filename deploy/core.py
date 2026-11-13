"""Robot-independent core of the phase-0 policy deployment (numpy only).

Everything here must reproduce the simulator exactly; tasks/phase0/deploy_parity.py checks it against the Isaac Lab
env. Frames: the arm base frame (= the sim env frame), metres and radians. Brick pose = bottom-face centre + rotation.

  Policy        actor MLP + observation normaliser from bundle/policy_weights.npz
  ObsBuilder    the policy observation: terms x 3 ticks, term-major, oldest tick first, first tick repeated
  ActionRules   running joint target (+-scale per tick, max_lead from the measured joints, joint limits), gripper bit
  BrickTracker  brick pose without a camera: known start cell, follows the fingers while grasped, follows the
                fingertips down while they press on its top
  Plate         plate geometry, from the sim config or a taught calibration (teach_plate.py)
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

BUNDLE = Path(__file__).resolve().parent / "bundle"

# firmware DH (piper_sdk C_PiperForwardKinematics, dh_is_offset=1); matches the sim model to 0.005 mm
DH_A = (0.0, 0.0, 0.28503, -0.02198, 0.0, 0.0)
DH_ALPHA = (0.0, -math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, math.pi / 2)
DH_THETA = (0.0, -math.radians(172.22), -math.radians(102.78), 0.0, 0.0, 0.0)
DH_D = (0.123, 0.0, 0.0, 0.25075, 0.0, 0.091)


def load_config(bundle: Path = BUNDLE) -> dict:
    return json.loads((bundle / "policy_config.json").read_text())


def tcp_pose(q, tcp_z: float):
    """Fingertip centre position (3,) and rotation (3, 3) in the base frame for joint angles q (6,) rad."""
    M = np.eye(4)
    for i in range(6):
        th = q[i] + DH_THETA[i]
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(DH_ALPHA[i]), math.sin(DH_ALPHA[i])
        M = M @ np.array([[ct, -st, 0, DH_A[i]],
                          [st * ca, ct * ca, -sa, -sa * DH_D[i]],
                          [st * sa, ct * sa, ca, ca * DH_D[i]],
                          [0, 0, 0, 1]])
    R = M[:3, :3]
    return M[:3, 3] + tcp_z * R[:, 2], R


def rot_z(yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


class Policy:
    def __init__(self, bundle: Path = BUNDLE):
        w = np.load(bundle / "policy_weights.npz")
        self.cfg = load_config(bundle)
        self.mean, self.std = w["obs_mean"], w["obs_std"]
        self.layers = [(w[f"W{i}"], w[f"b{i}"]) for i in range(self.cfg["n_layers"])]
        self.eps = self.cfg["obs_norm_eps"]

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        x = (obs - self.mean) / (self.std + self.eps)
        for i, (W, b) in enumerate(self.layers):
            x = W @ x + b
            if i < len(self.layers) - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))  # ELU
        return x


class Plate:
    """Plate geometry in the base frame. `pos` = plate underside centre (sim PLATE_POS); optional yaw about z."""

    def __init__(self, cfg: dict, calib: dict | None = None):
        g = cfg["geometry"]
        self.g = g
        self.pos = np.array(g["plate_pos"], dtype=float)
        self.yaw = 0.0
        if calib:
            self.pos = np.array(calib["plate_pos"], dtype=float)
            self.yaw = float(calib.get("plate_yaw", 0.0))
        self.slab_top_z = self.pos[2] + (g["slab_top_z"] - g["plate_pos"][2])

    def cell_centre(self, a: float, b: float):
        """xy of the 2x2 cell whose lower-left stud is (a, b) (mdp.cell_centre, plus the plate yaw)."""
        g = self.g
        local = np.array([-g["plate_size"] / 2 + g["pitch"] * (a + 1), -g["plate_size"] / 2 + g["pitch"] * (b + 1)])
        return self.pos[:2] + rot_z(self.yaw)[:2, :2] @ local

    @property
    def target_xy(self):
        return self.cell_centre(*self.g["target_cell"])


class ActionRules:
    """PiperFirmwareJointAction.process_actions + BinaryJointPositionAction, exactly."""

    def __init__(self, cfg: dict):
        a = cfg["action"]
        self.scale, self.max_lead = a["scale"], a["max_lead"]
        self.lo, self.hi = np.array(a["q_lo"]), np.array(a["q_hi"])
        self.target = None

    def reset(self, q):
        self.target = np.array(q, dtype=float)

    def step(self, action, q):
        """action (7,) raw policy output, q (6,) measured joints -> (new joint target (6,), close_gripper bool)."""
        t = self.target + np.clip(action[:6], -1.0, 1.0) * self.scale
        t = np.clip(t, q - self.max_lead, q + self.max_lead)
        self.target = np.clip(t, self.lo, self.hi)
        return self.target.copy(), bool(action[6] < 0.0)


class BrickTracker:
    """Brick pose from the known start and the arm's own state (no camera).

    rest:     where it was put down (start cell at the pre-engagement height, or where it was released)
    grasped:  the closed gripper holds it at about the brick width near the grasp point -> rigidly attached to the
              fingertips with the offset at the moment of grasping
    pressed:  released, gripper closed (< press_max_opening), fingertips over it and at/below its stud tops -> the
              brick top follows the fingertips down, bottom never below seated
    Unseen: slip in the fingers. The caller checks `lost` (released in the air) and stops.
    """

    def __init__(self, cfg: dict, plate: Plate):
        g = cfg["geometry"]
        self.plate, self.g = plate, g
        self.top = g["brick_h"] + g["stud_h"]
        self.pos = None
        self.R = np.eye(3)
        self.attached = False
        self.offset = None
        self.lost = False

    def reset(self, cell, yaw: float):
        xy = self.plate.cell_centre(*cell)
        self.pos = np.array([xy[0], xy[1], self.plate.slab_top_z + self.g["h_pre"]])
        self.R = rot_z(yaw)
        self.attached, self.offset, self.lost = False, None, False

    @property
    def h(self):
        return self.pos[2] - self.plate.slab_top_z

    def update(self, tcp_p, tcp_R, opening: float, close_cmd: bool):
        g = self.g
        if not self.attached:
            rel = tcp_p - self.pos
            over = np.linalg.norm(rel[:2]) < 0.012
            beside = 0.002 < rel[2] < self.top + 0.008   # fingertips somewhere along the brick's height
            holding = close_cmd and abs(opening - g["brick_w"]) < 0.002   # fingers reach the brick sides
            if over and beside and holding:
                self.attached = True
                # the closing fingers centre the brick along the closing direction (fingertip y axis) and square
                # it to the finger faces; along the finger width (21 mm fingers, 31.8 mm brick) it stays where it
                # was. Yaw = gripper yaw modulo 90 deg, nearest the tracked yaw.
                c = tcp_R[:, 1].copy()
                c[2] = 0.0
                c /= np.linalg.norm(c) + 1e-9
                self.pos = self.pos + c * float(np.dot(tcp_p - self.pos, c))
                self.pos[2] = self.plate.slab_top_z + self.h if self.h < self.g["h_pre"] + 0.003 else self.pos[2]
                g_yaw = math.atan2(tcp_R[1, 0], tcp_R[0, 0])
                b_yaw = math.atan2(self.R[1, 0], self.R[0, 0])
                yaw = g_yaw + round((b_yaw - g_yaw) / (math.pi / 2)) * (math.pi / 2)
                self.R = rot_z(yaw)
                self.offset = (tcp_R.T @ (self.pos - tcp_p), tcp_R.T @ self.R)
        if self.attached:
            if (not close_cmd and opening > g["brick_w"] + 0.003) or opening < g["brick_w"] - 0.004:
                self.attached = False                       # released (or closed past it: it fell out)
                if self.h > g["h_pre"] + 0.025:
                    self.lost = True                        # let go well up in the air: pose unknown
                else:                                       # set down or dropped a little: lands on the studs
                    self.pos[2] = self.plate.slab_top_z + min(self.h, g["h_pre"])
                    yaw = math.atan2(self.R[1, 0], self.R[0, 0])
                    self.R = rot_z(yaw)
            else:
                dp, dR = self.offset
                self.pos = tcp_p + tcp_R @ dp
                self.R = tcp_R @ dR
                return
        # pressed from above with the closed fingers: top of the brick follows the fingertips down
        over = np.linalg.norm((tcp_p - self.pos)[:2]) < g["brick_w"] / 2
        if close_cmd and opening < g["press_max_opening"] and over and tcp_p[2] < self.pos[2] + self.top:
            self.pos[2] = max(tcp_p[2] - self.top, self.plate.slab_top_z)


class ObsBuilder:
    """Policy observation, identical to the env's 'policy' group without noise (see policy_config obs_terms)."""

    def __init__(self, cfg: dict, plate: Plate):
        self.cfg, self.plate = cfg, plate
        self.g = cfg["geometry"]
        self.H = cfg["history"]
        self.hist = None

    def reset(self):
        self.hist = None

    def terms(self, q, target, opening, last_action, tcp_p, brick_p, brick_R):
        g = self.g
        up = brick_R[:, 2]
        psi = math.atan2(brick_R[1, 0], brick_R[0, 0])
        h = brick_p[2] - self.plate.slab_top_z
        txy = self.plate.target_xy
        grip_pt = brick_p + up * g["grasp_tip_h"]
        return {
            "arm_q": np.asarray(q, float), "arm_target": np.asarray(target, float),
            "gripper": np.array([opening]), "last_action": np.asarray(last_action, float),
            "tcp": tcp_p, "brick_pos": brick_p,
            "brick_rot": np.concatenate([up, [math.cos(4 * psi), math.sin(4 * psi)]]),
            "brick_to_target": np.array([brick_p[0] - txy[0], brick_p[1] - txy[1], h]),
            "tcp_to_brick": grip_pt - tcp_p,
            # fingertips minus the holding pose over the target (mdp.tcp_to_target)
            "tcp_to_target": tcp_p - np.array([txy[0], txy[1], self.plate.slab_top_z + g["h_pre"] + g["grasp_tip_h"]]),
        }

    def __call__(self, **state) -> np.ndarray:
        t = self.terms(**state)
        names = [n for n, _ in self.cfg["obs_terms"]]
        if self.hist is None:                       # first tick after reset fills the whole history (CircularBuffer)
            self.hist = {n: [t[n]] * self.H for n in names}
        else:
            for n in names:
                self.hist[n] = self.hist[n][1:] + [t[n]]
        return np.concatenate([np.concatenate(self.hist[n]) for n in names])   # term-major, oldest first


def gripper_real_to_sim(opening_real_m: float, table) -> float:
    """Map the real gripper reading to the sim opening by piecewise-linear interpolation through calibration
    points [[real, sim], ...] (metres). The real reading is 6-7 mm wide on a brick (gripper test 2026-10-23)."""
    pts = np.array(sorted(table), dtype=float)
    if len(pts) < 2:
        return float(opening_real_m)
    x, y = pts[:, 0], pts[:, 1]
    if opening_real_m <= x[0]:                         # beyond the table: slope 1 (no calibration information)
        return float(y[0] + opening_real_m - x[0])
    if opening_real_m >= x[-1]:
        return float(y[-1] + opening_real_m - x[-1])
    return float(np.interp(opening_real_m, x, y))
