"""Adim 22: anatomik bacak kutlesi, gercek momentum alisverisi, govde yaw iptali."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from physics.verlet import VerletSystem, apply_angular_couple
from physics.leg_mass import (LegMassModel, THIGH_MASS, SHANK_MASS, THIGH_AXIS_FRAC,
                              SHANK_AXIS_FRAC)
from demo import step14_active_biped as s14


class _Leg:
    pass


def _floating_body():
    b = VerletSystem(points=np.zeros((0, 2)), prev_points=np.zeros((0, 2)))
    b.gravity = np.zeros(2)
    b.friction = 0.0
    h = b.add_point([0.0, 0.0], mass=1.0)
    s = b.add_point([0.0, -55.0], mass=1.0)
    hd = b.add_point([0.0, -75.0], mass=1.0)
    b.add_stick(h, s, length=55.0)
    b.add_stick(s, hd, length=20.0)
    return b, h, s, hd


class LegMassConservationTest(unittest.TestCase):
    def test_linear_momentum_conserved_with_swinging_leg(self):
        """Yercekimsiz, zeminsiz serbest govde + programli salinim ayagi:
        govde + bacak dogrusal momentumu sabit kalmali (ortuk/acik ayrim)."""
        b, h, s, hd = _floating_body()
        leg = _Leg()
        lm = LegMassModel(np.zeros(2))
        P = []
        for t in range(80):
            leg.foot_target = np.array([40.0 * np.sin(0.3 * t), 180.0 + 10.0 * np.cos(0.5 * t)])
            leg.state = "swing"
            b.step(dt=1.0)
            lm.observe([leg])
            F, tau, n = lm.hip_load([leg], b.points[h])
            b.masses[h] = 1.0 + n * LegMassModel.implicit_hip_mass()
            LegMassModel.apply_reaction(b.points, b.prev_points, b.masses, h, s, F, tau)
            v = b.points - b.prev_points
            mom = 1.0 * v[h] + v[s] + v[hd]
            H = lm.hist[id(leg)]
            if len(H) >= 2:
                vf = H[-1][0] - H[-2][0]
                for fr, m in ((THIGH_AXIS_FRAC, THIGH_MASS), (SHANK_AXIS_FRAC, SHANK_MASS)):
                    mom = mom + m * ((1.0 - fr) * v[h] + fr * vf)
            P.append(mom)
        P = np.array(P[5:])
        self.assertLess(np.ptp(P[:, 0]), 1e-9)
        self.assertLess(np.ptp(P[:, 1]), 1e-9)

    def test_posture_couple_has_zero_net_force(self):
        b, h, s, _ = _floating_body()
        b.points[s] += [10.0, 0.0]   # govde egik
        b.prev_points[:] = b.points
        before = (b.points - b.prev_points) * b.masses[:, None]
        apply_angular_couple(b.points, b.prev_points, b.masses, h, s, s14.UP, 0.0, 0.2, 0.8)
        after = (b.points - b.prev_points) * b.masses[:, None]
        self.assertTrue(np.allclose(before.sum(axis=0), after.sum(axis=0)))
        self.assertFalse(np.allclose(before, after))


class LegMassWalkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = {}
        for mode in ("off", "cancel"):
            sim = s14.ActiveBipedSim(stumble_kick_px=0.0, big_push_kick_px=0.0, arms_mode=mode)
            pitch = []
            for _ in range(900):
                sim.step()
                d = sim.body.points[sim.idx["shoulder"]] - sim.body.points[sim.idx["hip"]]
                pitch.append(np.degrees(np.arctan2(d[0], -d[1])))
            cls.runs[mode] = (sim, np.array(pitch[60:]))

    def test_walk_is_stable_and_upright(self):
        for mode, (sim, pitch) in self.runs.items():
            self.assertFalse(sim.fell or sim.nan, mode)
            self.assertAlmostEqual(float(np.mean(sim.hip_vx_log[60:])), s14.TARGET_VX, delta=0.2)
            # durus kontrolu: govde -12 derece duvarina yaslanmiyor
            self.assertLess(np.mean(np.abs(pitch) > s14.TORSO_MAX_LEAN_DEG - 0.1), 0.01, mode)
            self.assertLess(pitch.std(), 2.0, mode)

    def test_slips_only_on_ice(self):
        for mode, (sim, _) in self.runs.items():
            xs = np.array(sim.hip_x_log)
            (x0, x1, _), = s14.ICE_ZONES
            off_ice = [e for e in sim.slip_events if not (x0 - 30 < xs[e[0]] < x1 + 30)]
            self.assertEqual(off_ice, [], mode)

    def test_hip_load_free_of_fabrik_jitter(self):
        sim, _ = self.runs["cancel"]
        F = np.array([np.linalg.norm(x[0]) for x in sim.leg_mass.log[60:]])
        self.assertLess(np.percentile(F, 99), 3.0)

    def test_arms_cancel_trunk_yaw(self):
        def rms(sim):
            th = np.array(sim.trunk_yaw.log[60:])[:, 0]
            return float(np.sqrt(np.mean(th ** 2)))
        self.assertLess(rms(self.runs["cancel"][0]), 0.6 * rms(self.runs["off"][0]))
        log = np.array(self.runs["cancel"][0].trunk_yaw.log[60:])
        self.assertLess(np.corrcoef(log[:, 2], log[:, 3])[0, 1], -0.7)


class LegMassPushTest(unittest.TestCase):
    def test_pushes_recover_single_support(self):
        for push in (150.0, -150.0, 300.0, -300.0, 500.0, -500.0):
            # sayisal stres itkileri (COM 4-14 m/s): Adim 22'nin sabit 3 karelik yakalamasiyla
            sim = s14.ActiveBipedSim(big_push_kick_px=push, big_push_t=7.0, catch_timing="fixed",
                                     closing_ttc=0.0, shock_mode="rate_cap", shock_trigger="catch",
                                     preactivation=0, rocker=False, hip_strategy_gain=0.0,
                                     hill=False)
            double = 0
            for _ in range(480):
                sim.step()
                double += all(l.state == "swing" for l in sim.legs)
            self.assertFalse(sim.fell or sim.nan, push)
            self.assertEqual(double, 0, push)
            self.assertGreater(s14.GROUND_Y - max(sim.hip_y_log[210:]), 50.0, push)
            self.assertAlmostEqual(float(np.mean(sim.hip_vx_log[-60:])), s14.TARGET_VX, delta=0.6)


if __name__ == "__main__":
    unittest.main()
