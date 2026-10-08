"""Adim 26: durus bacagi stratejileri -- ayak rocker'i (parmak ucu/topuk) ve kalca stratejisi."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14

KICK_PER_MS = 5.18 * 184.0 / 0.9 / 30.0
OFF = dict(rocker=False, hip_strategy_gain=0.0)


def min_height(push_ms, t=7.0, n=420, **kw):
    kw.setdefault("shock_mode", "servo")   # Adim 26 olcumu sok servosuyla (Adim 27: "force")
    kw.setdefault("hill", False)          # Adim 28 oncesi kas modeli
    sim = s14.ActiveBipedSim(gravity_mode="legacy", big_push_kick_px=push_ms * KICK_PER_MS, big_push_t=t, **kw)
    for _ in range(n):
        sim.step()
    return s14.GROUND_Y - max(sim.hip_y_log[210:]), sim


class StanceStrategyTest(unittest.TestCase):
    def test_rocker_inactive_in_normal_gait(self):
        sim = s14.ActiveBipedSim(gravity_mode="legacy", stumble_kick_px=0.0, big_push_kick_px=0.0)
        for f in range(900):
            n = len(sim.rocker_log)
            sim.step()
            if f >= 30 and len(sim.rocker_log) > n:
                self.assertEqual(sim.rocker_log[-1], 0.0, f)

    def test_rocker_shift_bounded_by_foot(self):
        _, sim = min_height(150.0 / KICK_PER_MS)
        self.assertLessEqual(max(sim.rocker_log), s14.ROCKER_TOE_PX + 1e-9)
        self.assertGreaterEqual(min(sim.rocker_log), -s14.ROCKER_HEEL_PX - 1e-9)

    def test_less_collapse_than_pivot_at_ankle(self):
        for dv in (2.0, 2.5, -1.0, 150.0 / KICK_PER_MS):
            on, a = min_height(dv)
            off, b = min_height(dv, **OFF)
            self.assertFalse(a.fell or a.nan, dv)
            self.assertGreater(on, off, dv)

    def test_hip_strategy_sign(self):
        """COM ondeyken govde one egilir (tepki kalcayi geri iter); ters isaret daha kotu."""
        good = sum(min_height(dv, t=7.0 + k * 5 / 30)[0] for dv in (2.5, -2.5) for k in range(3))
        bad = sum(min_height(dv, t=7.0 + k * 5 / 30, hip_strategy_gain=-0.3)[0] for dv in (2.5, -2.5) for k in range(3))
        self.assertGreater(good, bad)


if __name__ == "__main__":
    unittest.main()
