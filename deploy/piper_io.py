"""Piper I/O for the deploy loop: a feedback thread holding the newest state, and the command path used in sysID
(MotionCtrl_2 + JointCtrl + GripperCtrl once per tick). DryArm has the same interface with no hardware, for tests.

State snapshot (dict): t (host monotonic time of the newest joint frame), q (6,) rad, effort (6,) N*m,
grip_m (gripper opening, m), grip_effort (N*m), status dict (ctrl_mode, arm_status, comm_err, limit_err).
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

MDEG = math.pi / 180.0 / 1000.0          # 0.001 deg -> rad


class PiperArm:
    def __init__(self, can: str = "can0"):
        from piper_sdk import C_PiperInterface_V2
        self.arm = C_PiperInterface_V2(can)
        self.arm.ConnectPort()
        self.arm.SetSDKGripperRangeParam(0.0, 0.1)   # 100 mm gripper; SDK default range is 0-70 mm
        self._lock = threading.Lock()
        self._state = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._poll, daemon=True)
        self._thread.start()
        t0 = time.monotonic()
        while self.state() is None:
            if time.monotonic() - t0 > 3.0:
                raise SystemExit("no joint feedback: is the arm powered and the CAN bus up?")
            time.sleep(0.02)

    def _poll(self):
        last_ts = None
        while not self._stop.is_set():
            js = self.arm.GetArmJointMsgs()
            if js.time_stamp != last_ts and js.time_stamp != 0:
                time.sleep(0.0015)                   # let the rest of this 200 Hz burst arrive (see sysid_record.py)
                last_ts = js.time_stamp
                s = js.joint_state
                q = np.array([s.joint_1, s.joint_2, s.joint_3, s.joint_4, s.joint_5, s.joint_6], float) * MDEG
                hs = self.arm.GetArmHighSpdInfoMsgs()
                eff = np.array([getattr(hs, f"motor_{i}").effort for i in range(1, 7)], float) / 1000.0
                g = self.arm.GetArmGripperMsgs().gripper_state
                st = self.arm.GetArmStatus().arm_status
                err = st.err_status
                status = {"ctrl_mode": int(st.ctrl_mode), "arm_status": int(st.arm_status),
                          "comm_err": any(getattr(err, f"communication_status_joint_{i}") for i in range(1, 7)),
                          "limit_err": any(getattr(err, f"joint_{i}_angle_limit") for i in range(1, 7))}
                with self._lock:
                    self._state = {"t": time.monotonic(), "q": q, "effort": eff, "grip_m": g.grippers_angle / 1e6,
                                   "grip_effort": g.grippers_effort / 1000.0, "status": status}
            time.sleep(0.0005)

    def state(self):
        with self._lock:
            return None if self._state is None else dict(self._state)

    def enable(self, gripper_effort: int):
        t0 = time.monotonic()
        while not self.arm.EnablePiper():
            if time.monotonic() - t0 > 5.0:
                raise SystemExit("arm did not report all six joints enabled within 5 s")
            time.sleep(0.01)
        self.arm.GripperCtrl(0, gripper_effort, 0x02, 0)   # clear gripper errors
        time.sleep(0.02)

    def command(self, q_target, grip_um: int, grip_effort: int, speed_pct: int = 100):
        q_mdeg = [int(round(v / MDEG)) for v in q_target]
        self.arm.MotionCtrl_2(0x01, 0x01, speed_pct, 0x00)
        self.arm.JointCtrl(*q_mdeg)
        self.arm.GripperCtrl(int(grip_um), int(grip_effort), 0x01, 0)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)


class DryArm:
    """No hardware. Joints follow the Piper firmware model fitted in sysID (sysid/firmware_fit.json; the same model
    the sim's action term uses): per-joint transport delay 10-15 ms, then an acceleration-limited (5 rad/s^2,
    3 rad/s) reference that brakes onto the target, stepped at 1 ms. The gripper moves at 0.2 m/s and stops at the
    brick width when `brick_between()` says a (virtual) brick is between the fingers."""

    DELAY_S = (0.0125, 0.010, 0.0125, 0.0125, 0.015, 0.015)
    AMAX, VMAX, SUB_DT = 5.0, 3.0, 0.001

    def __init__(self, q0, rate_hz: float = 25.0):
        self.q = np.array(q0, float)
        self.v = np.zeros(6)
        self.grip = 0.05
        self.dt = 1.0 / rate_hz
        self.n_sub = int(round(self.dt / self.SUB_DT))
        self.delay = np.round(np.array(self.DELAY_S) / self.SUB_DT).astype(int)
        self.hist = [self.q.copy()] * (int(self.delay.max()) + 1)   # held target per 1 ms, newest last
        self.target = self.q.copy()
        self.brick_between = lambda: False
        self.brick_w = 0.0318

    def _advance(self):
        """Run the firmware model for one tick (the previous command is held during it)."""
        for _ in range(self.n_sub):
            self.hist = self.hist[1:] + [self.target.copy()]
            delayed = np.array([self.hist[-1 - d][j] for j, d in enumerate(self.delay)])
            e = delayed - self.q
            v_des = np.sign(e) * np.minimum(np.minimum(np.sqrt(2 * self.AMAX * np.abs(e)), self.VMAX),
                                            np.abs(e) / self.SUB_DT)
            self.v += np.clip(v_des - self.v, -self.AMAX * self.SUB_DT, self.AMAX * self.SUB_DT)
            self.q += self.v * self.SUB_DT

    def state(self):
        return {"t": time.monotonic(), "q": self.q.copy(), "effort": np.zeros(6), "grip_m": self.grip,
                "grip_effort": 0.0, "status": {"ctrl_mode": 1, "arm_status": 0, "comm_err": False, "limit_err": False}}

    def enable(self, gripper_effort: int):
        pass

    def command(self, q_target, grip_um: int, grip_effort: int, speed_pct: int = 100):
        self.target = np.array(q_target, float)
        self._advance()                               # the next state() sees one tick of firmware motion
        target = grip_um / 1e6
        new = self.grip + float(np.clip(target - self.grip, -0.2 * self.dt, 0.2 * self.dt))
        if target < self.grip and new < self.brick_w <= self.grip + 1e-6 and self.brick_between():
            new = self.brick_w                       # closing onto the (virtual) brick stops at its width
        self.grip = new

    def close(self):
        pass
