"""Simulace hardware pro suchý běh na PC (`python3 main.py --mock`).

Virtuální čas (nic se reálně nečeká), kinematika z enkodérů, gyro integrované
z rozdílu kol, barevný senzor vidí kostky rozmístěné po `spacing_mm`.
S `--flip` jsou kostky náhodně otočené o 180° (simulace 2. kola).
"""
import random

from common import TimeUp

DT_MAX = 0.05


class MockHardware:
    def __init__(self, cfg, log=print, flip=False, seed=1):
        self.log = log
        d = cfg["robot"]["drive"]
        self.deg_per_mm = d["deg_per_mm"]
        self.k_turn = d["wheel_deg_per_robot_deg"]
        self.use_gyro = d.get("use_gyro", True)
        self.mock_cfg = cfg.get("mock", {})
        self.t = 0.0
        self.deadline = None
        self.l = self.r = 0.0          # úhly kol od reset_wheels
        self.vl = self.vr = 0.0        # deg/s
        self.heading = 0.0             # CCW+
        self.dist = 0.0                # ujetá vzdálenost (mm, se znaménkem)
        self.flip = flip
        self.rng = random.Random(seed)
        self.blocks = []
        self.next_at = None
        self.spacing = 0
        self.arm_total = 0.0

    # --- čas ---
    def now(self):
        return self.t

    def set_deadline(self, t):
        self.deadline = t

    def check_time(self):
        if self.deadline is not None and self.t > self.deadline:
            raise TimeUp()

    def sleep(self, s):
        self.check_time()
        while s > 1e-9:
            dt = min(s, DT_MAX)
            self.l += self.vl * dt
            self.r += self.vr * dt
            self.dist += (self.vl + self.vr) / 2 * dt / self.deg_per_mm
            self.heading += (self.vr - self.vl) / 2 * dt / self.k_turn
            self.t += dt
            s -= dt

    # --- pojezd ---
    def drive_speed(self, l, r):
        self.vl, self.vr = float(l), float(r)

    def stop(self):
        self.vl = self.vr = 0.0

    def stop_all(self):
        self.stop()

    def reset_wheels(self):
        self.l = self.r = 0.0

    def wheel_angles(self):
        return (self.l, self.r)

    def gyro_angle(self):
        return self.heading if self.use_gyro else None

    # --- senzory ---
    def mock_blocks(self, pairs, spacing_mm):
        self.blocks = []
        for p in pairs:
            seen = p[1] if (self.flip and self.rng.random() < 0.5) else p[0]
            self.blocks.append(seen)
        self.spacing = spacing_mm
        self.next_at = self.dist + self.mock_cfg.get("first_block_mm", 120)
        self.log("    [mock] kostky (první viděná barva): {}".format(self.blocks))

    def colour(self):
        if self.blocks and self.next_at is not None and self.dist >= self.next_at:
            if self.dist < self.next_at + self.mock_cfg.get("block_visible_mm", 40):
                return self.blocks[0]
            self.blocks.pop(0)
            self.next_at += self.spacing
        return "WHITE"

    def reflected(self):
        return self.mock_cfg.get("reflected", 50)

    # --- motor C ---
    def arm_move(self, deg, speed, tol=15, stall_s=None, extra_wait_s=None):
        self.arm_total += abs(deg)
        self.sleep(abs(deg) / float(speed) + 0.1)

    def beep(self, hz=880, ms=150):
        self.log("    [mock] beep {} Hz {} ms".format(hz, ms))
