"""Adim 24: temas tabanli, ivme sinirli sok servosu + kapanma hizi (TTC) kapisi."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14

KICK_PER_MS = 5.18 * 184.0 / 0.9 / 30.0
PRE24 = dict(closing_ttc=0.0, shock_mode="rate_cap", shock_trigger="catch", preactivation=0,
             rocker=False, hip_strategy_gain=0.0)


def run(push_ms, t=7.0, n=420, **kw):
    kw.setdefault("shock_mode", "servo")   # Adim 24 sok servosu (Adim 27: "force")
    sim = s14.ActiveBipedSim(big_push_kick_px=push_ms * KICK_PER_MS, big_push_t=t, **kw)
    for _ in range(n):
        sim.step()
    return sim


def max_rise(sim, bp=210):
    return float(np.max(-np.diff(np.array(sim.hip_y_log))[bp:]))


class ContactShockTest(unittest.TestCase):
    def test_no_spring_snap_after_landing(self):
        """Eski sok emici 4.2 m/s'de kalcayi tek karede 50-60 px firlatiyordu."""
        for dv in (150.0 / KICK_PER_MS, -150.0 / KICK_PER_MS, 2.0, -2.0):
            new = run(dv)
            old = run(dv, **PRE24)
            self.assertFalse(new.fell or new.nan, dv)
            self.assertLess(max_rise(new), 12.0, dv)
            self.assertLess(max_rise(new), max_rise(old), dv)

    def test_extension_rate_is_limited(self):
        sim = run(-2.0)
        self.assertTrue(sim.shock_events)
        self.assertLessEqual(s14.SHOCK_EXT_VMAX, 3.0 + 1e-9)

    def test_undisturbed_walk_identical_to_step23(self):
        # Adim 26'nin rocker/kalca stratejisi baslangic sarsintisinda devreye giriyor
        a = s14.ActiveBipedSim(stumble_kick_px=0.0, big_push_kick_px=0.0, rocker=False, hip_strategy_gain=0.0)
        b = s14.ActiveBipedSim(stumble_kick_px=0.0, big_push_kick_px=0.0, **PRE24)
        for _ in range(900):
            a.step()
            b.step()
        self.assertTrue(np.array_equal(a.body.points, b.body.points))
        self.assertEqual(a.shock_events, [])
        self.assertLessEqual(max(c[2] for c in a.contact_log), s14.SHOCK_CONTACT_MIN_PX)


class ClosingGateTest(unittest.TestCase):
    def test_gate_reduces_catches_without_falls(self):
        on = off = 0
        for dv in (2.0, -2.0, 2.5, -2.5):
            for k in range(3):
                a = run(dv, t=7.0 + k * 5 / 30)
                b = run(dv, t=7.0 + k * 5 / 30, closing_ttc=0.0)
                self.assertFalse(a.fell, dv)
                on += sum(1 for c in a.catch_frames_log if c[0] >= 210)
                off += sum(1 for c in b.catch_frames_log if c[0] >= 210)
        self.assertLess(on, off)

    def test_no_zero_frame_catches(self):
        for dv in (2.0, -2.0, 150.0 / KICK_PER_MS):
            sim = run(dv)
            self.assertTrue(all(c[2] >= 1 for c in sim.catch_frames_log), dv)


if __name__ == "__main__":
    unittest.main()
