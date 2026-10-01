"""Adim 23: yakalama adiminin suresi kalca torkundan (tork sinirli ayak servosu)."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from physics import active_gait as ag
from demo import step14_active_biped as s14

# 1 m/s tum-govde COM hiz degisimi icin kalca durtusu (px): toplam kutle 5.18,
# 1 m/s = 184/0.9/30 px/kare
KICK_PER_MS = 5.18 * 184.0 / 0.9 / 30.0


def run(push_ms, n=420, **kw):
    kw.setdefault("hill", False)   # Adim 23 olcumu: sabit (hizdan bagimsiz) tork tavani
    sim = s14.ActiveBipedSim(big_push_kick_px=push_ms * KICK_PER_MS, big_push_t=7.0, **kw)
    double = 0
    for _ in range(n):
        sim.step()
        double += all(l.state == "swing" for l in sim.legs)
    return sim, double


class BrakeCurveTest(unittest.TestCase):
    def test_servo_reaches_target_without_overshoot(self):
        a = 5.0
        x, v, target = 0.0, 0.0, 70.0
        xs = []
        for _ in range(40):
            dv = ag.ActiveFootPlantingLeg._brake_velocity(target - x, a) - v
            dv = float(np.clip(dv, -a, a))
            v += dv
            x += v
            xs.append(x)
        # ayrik zaman: frenleme egrisi tam sayi adima denk gelmezse asim <= a/8
        self.assertLessEqual(max(xs), target + a / 8.0 + 1e-6)
        self.assertAlmostEqual(xs[-1], target, places=6)


class TorqueCatchTest(unittest.TestCase):
    def test_hip_torque_never_exceeds_limit(self):
        for dv in (1.0, -1.0, 2.0, -2.0):
            sim, _ = run(dv, n=320)
            tau = max(abs(x[1]) for x in sim.leg_mass.log[210:])
            self.assertLessEqual(tau, ag.HIP_TORQUE_MAX * 1.01, dv)

    def test_bigger_push_takes_longer_catch(self):
        # Adim 26'dan beri +1 m/s itkide cogu fazda yakalama gerekmiyor (ayak
        # rocker'i karsiliyor); servo zamanlamasi Adim 25 govdesinde olculur.
        small, _ = run(1.0, n=320, rocker=False, hip_strategy_gain=0.0)
        big, _ = run(2.0, n=320, rocker=False, hip_strategy_gain=0.0)
        first = lambda s: next(x[2] for x in s.catch_frames_log if x[0] >= 210)
        self.assertGreater(first(big), first(small))

    def test_realistic_pushes_recover(self):
        """0.5-2.5 m/s COM itkileri (insan tek/cok adimli toparlanma araligi)."""
        # Adim 27'den beri 2-2.5 m/s'de kuvvet sinirli bacak cokebiliyor; bu test
        # Adim 23-26 sok servosuyla tork sinirli yakalamayi olcer.
        for dv in (0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0, 2.5, -2.5):
            sim, double = run(dv, shock_mode="servo")
            self.assertFalse(sim.fell or sim.nan, dv)
            self.assertEqual(double, 0, dv)
            self.assertGreater(s14.GROUND_Y - max(sim.hip_y_log[210:]), 90.0, dv)

    def test_undisturbed_walk_unchanged_by_catch_timing(self):
        a = s14.ActiveBipedSim(stumble_kick_px=0.0, big_push_kick_px=0.0)
        b = s14.ActiveBipedSim(stumble_kick_px=0.0, big_push_kick_px=0.0, catch_timing="fixed")
        for _ in range(900):
            a.step()
            b.step()
        self.assertEqual(a.fazb_events, [])
        self.assertTrue(np.array_equal(a.body.points, b.body.points))


if __name__ == "__main__":
    unittest.main()
