"""Adim 28: Hill kas modeli -- kuvvet-hiz iliskisi (diz ekstansoru + kalca servosu)."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from physics.hill import force_velocity, JOINT_VMAX
from physics import active_gait as ag
from demo import step14_active_biped as s14

KICK_PER_MS = 5.18 * 184.0 / 0.9 / 30.0


def run(push_ms, t=7.0, n=420, **kw):
    sim = s14.ActiveBipedSim(gravity_mode="legacy", big_push_kick_px=push_ms * KICK_PER_MS, big_push_t=t, **kw)
    for _ in range(n):
        sim.step()
    return sim


class HillCurveTest(unittest.TestCase):
    def test_curve_shape(self):
        self.assertAlmostEqual(force_velocity(0.0), 1.0)
        self.assertEqual(force_velocity(JOINT_VMAX), 0.0)
        con = [force_velocity(s) for s in np.linspace(0, JOINT_VMAX, 9)]
        self.assertEqual(con, sorted(con, reverse=True))
        ecc = [force_velocity(-s) for s in np.linspace(0, 3 * JOINT_VMAX, 9)]
        self.assertEqual(ecc, sorted(ecc))
        self.assertLessEqual(max(ecc), 1.5 + 1e-9)


class HillSimTest(unittest.TestCase):
    def test_undisturbed_walk_unchanged(self):
        a = s14.ActiveBipedSim(gravity_mode="legacy", stumble_kick_px=0.0, big_push_kick_px=0.0)
        b = s14.ActiveBipedSim(gravity_mode="legacy", stumble_kick_px=0.0, big_push_kick_px=0.0, hill=False)
        for _ in range(900):
            a.step()
            b.step()
        self.assertTrue(np.array_equal(a.body.points, b.body.points))

    def test_hip_torque_within_eccentric_limit(self):
        for dv in (1.0, -1.0, 2.0, -2.0):
            sim = run(dv, n=320)
            tau = max(abs(x[1]) for x in sim.leg_mass.log[211:])
            self.assertLessEqual(tau, ag.HIP_TORQUE_MAX * 1.5 * 1.02, dv)

    def test_recovery_rise_is_bounded(self):
        rises = []
        for dv in (2.0, -2.0, 2.5, -2.5):
            for k in range(3):
                sim = run(dv, t=7.0 + k * 5 / 30)
                if not sim.collapsed:
                    H = s14.GROUND_Y - np.array(sim.hip_y_log)
                    rises.append(float(np.diff(H)[210:].max()))
        self.assertTrue(rises)
        self.assertLess(max(rises), 8.0)

    def test_fewer_collapses_than_without_hill(self):
        on = off = 0
        for dv in (-1.5, 2.0, -2.0, -2.5):
            for k in range(3):
                on += run(dv, t=7.0 + k * 5 / 30).collapsed
                off += run(dv, t=7.0 + k * 5 / 30, hill=False).collapsed
        self.assertLessEqual(on, off)


if __name__ == "__main__":
    unittest.main()
