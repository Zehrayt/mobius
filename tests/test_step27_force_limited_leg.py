"""Adim 27: kuvvet sinirli bacak -- diz torku (200 Nm) gercek agirliga karsi, cokme."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14

KICK_PER_MS = 5.18 * 184.0 / 0.9 / 30.0


def run(push_ms, t=7.0, n=420, **kw):
    sim = s14.ActiveBipedSim(gravity_mode="legacy", big_push_kick_px=push_ms * KICK_PER_MS, big_push_t=t, **kw)
    for _ in range(n):
        sim.step()
    return sim


class ForceCapacityTest(unittest.TestCase):
    def test_capacity_matches_two_bone_formula(self):
        sim = s14.ActiveBipedSim(gravity_mode="legacy")
        W = s14.BODY_MASS_TOTAL * s14.G_REAL_PX
        # 200 Nm tek bacak, 70 kg: agirligi tam tasiyabildigi kalca-ayak mesafesi ~140 px
        self.assertAlmostEqual(sim.leg_force_capacity(140.0) / W, 1.0, delta=0.03)
        self.assertLess(sim.leg_force_capacity(100.0), W)
        self.assertGreater(sim.leg_force_capacity(170.0), W)
        caps = [sim.leg_force_capacity(d) for d in (60, 100, 140, 170)]
        self.assertEqual(caps, sorted(caps))


class ForceLimitedLegTest(unittest.TestCase):
    def test_undisturbed_walk_unchanged(self):
        a = s14.ActiveBipedSim(gravity_mode="legacy", stumble_kick_px=0.0, big_push_kick_px=0.0)
        b = s14.ActiveBipedSim(gravity_mode="legacy", stumble_kick_px=0.0, big_push_kick_px=0.0, shock_mode="servo")
        for _ in range(900):
            a.step()
            b.step()
        self.assertTrue(np.array_equal(a.body.points, b.body.points))

    def test_small_pushes_never_collapse(self):
        for dv in (1.0, -1.0, 1.5):
            for k in range(3):
                sim = run(dv, t=7.0 + k * 5 / 30)
                self.assertFalse(sim.collapsed or sim.fell, (dv, k))

    def test_big_push_collapses_and_stays_above_ground(self):
        sim = run(150.0 / KICK_PER_MS, n=360)
        self.assertTrue(sim.collapsed)
        self.assertLessEqual(max(sim.hip_y_log[sim.collapse_frame:]), s14.GROUND_Y + 1e-6)

    def test_preactivation_prevents_collapses(self):
        on = off = 0
        for dv in (2.0, -2.0, 2.5, -2.5):
            for k in range(3):
                on += run(dv, t=7.0 + k * 5 / 30).collapsed
                off += run(dv, t=7.0 + k * 5 / 30, preactivation=0).collapsed
        self.assertLess(on, off)


if __name__ == "__main__":
    unittest.main()
