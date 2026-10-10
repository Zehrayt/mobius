"""Durust fizik modu (bayrakla): ic eklem torklari ve kare butceli Coulomb surtunmesi.

Varsayilan kapali (kalkis zinciri bu modda henuz ayaga kalkamiyor, README
"Durust fizik modu"). Bu testler modun kendi garantilerini sabitler.
"""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14
import physics.ground_recovery as gr

TRANSFER = dict(ground_recovery=True, recovery_reposition=True, recovery_transfer=True)


class HonestMode:
    def __init__(self, internal=True, coulomb=False):
        self.internal, self.coulomb = internal, coulomb

    def __enter__(self):
        self.saved = gr.INTERNAL_TORQUES, s14.COULOMB_RECOVERY
        gr.INTERNAL_TORQUES, s14.COULOMB_RECOVERY = self.internal, self.coulomb

    def __exit__(self, *exc):
        gr.INTERNAL_TORQUES, s14.COULOMB_RECOVERY = self.saved


class InternalTorqueTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with HonestMode(internal=True, coulomb=False):
            sim = s14.ActiveBipedSim(big_push_kick_px=150.0, big_push_t=7.0, **TRANSFER)
        cls.states, cls.clearance = [], []
        for _ in range(1200):
            sim.step()
            st = sim.recovery.state
            if not cls.states or cls.states[-1] != st:
                cls.states.append(st)
            if st == "foot_placing":
                k, f = sim.fallen_legs[sim.recovery.front_side]
                cls.clearance.append(s14.GROUND_Y - 6.0 - float(sim.body.points[f, 1]))
        cls.sim = sim

    def test_defaults_are_legacy(self):
        self.assertFalse(gr.INTERNAL_TORQUES)
        self.assertFalse(s14.COULOMB_RECOVERY)

    def test_motor_torques_sum_to_zero_every_frame(self):
        r = self.sim.recovery
        self.assertTrue(r.internal_torques)
        net = {}
        for row in r.motor_log:
            net[row[0]] = net.get(row[0], 0.0) + row[4]
        for row in r.reaction_log:
            net[row[0]] = net.get(row[0], 0.0) + row[4]
        self.assertGreater(len(net), 100)
        self.assertLess(max(abs(v) for v in net.values()), 1e-9)

    def test_foot_is_lifted_not_dragged(self):
        # yeni yerlestirme: on ayak havaya kalkar (eski hedefler ayagi zeminde surukluyordu)
        self.assertIn("transferring", self.states, self.states)
        self.assertGreater(max(self.clearance), 30.0)


class CoulombBudgetTest(unittest.TestCase):
    def test_tangential_correction_never_exceeds_mu_times_normal(self):
        with HonestMode(internal=True, coulomb=True):
            sim = s14.ActiveBipedSim(big_push_kick_px=150.0, big_push_t=7.0, **TRANSFER)
        self.assertTrue(sim.coulomb_friction)
        worst, checked = 0.0, 0
        for _ in range(900):
            sim.step()
            log = getattr(sim, "friction_budget_log", None)
            if log is None:
                continue
            dn, tcorr = log
            excess = np.abs(tcorr) - s14.FALLEN_STATIC_MU * dn
            worst = max(worst, float(excess.max()))
            checked += 1
        self.assertGreater(checked, 100)
        self.assertLessEqual(worst, 1e-9)


class NeckXPBDTest(unittest.TestCase):
    def test_soft_hinge_conserves_linear_momentum(self):
        from physics.spine import SoftHinge
        rng = np.random.default_rng(3)
        p = rng.normal(size=(3, 2)) * 30.0
        prev = p - rng.normal(size=(3, 2))
        m = np.array([0.6, 0.4, 0.3])
        before = (m[:, None] * p).sum(axis=0)
        hinge = SoftHinge(0.01, 16.0)
        for _ in range(20):
            hinge.project(p, prev, m, [0, 1, 2])
        np.testing.assert_allclose((m[:, None] * p).sum(axis=0), before, atol=1e-9)

    def test_passive_falls_neck_mostly_off_the_end_stop(self):
        k = 5.18 * 184.0 / 0.9 / 30.0
        scenes = [(p, ph) for p in (150.0, -150.0, 2.5 * k, -2.5 * k) for ph in (0, 10)]

        def max_neck(enabled, push, phase):
            sim = s14.ActiveBipedSim(big_push_kick_px=push, big_push_t=7.0 + phase / 30.0, bracing=False)
            worst = 0.0
            for _ in range(420):
                sim.step()
                if sim.collapsed and "waist" in sim.idx:
                    i, p = sim.idx, sim.body.points
                    a, b = p[i["shoulder"]] - p[i["waist"]], p[i["head"]] - p[i["shoulder"]]
                    worst = max(worst, abs(np.degrees(np.arctan2(a[0] * b[1] - a[1] * b[0], a @ b))))
            return worst

        saved = s14.NECK_XPBD_ENABLED, s14.NECK_XPBD_COMPLIANCE, s14.NECK_XPBD_BETA
        try:
            res = {}
            for enabled in (False, True):
                s14.NECK_XPBD_ENABLED, s14.NECK_XPBD_COMPLIANCE, s14.NECK_XPBD_BETA = enabled, 0.01, 16.0
                res[enabled] = [max_neck(enabled, p, ph) for p, ph in scenes]
        finally:
            s14.NECK_XPBD_ENABLED, s14.NECK_XPBD_COMPLIANCE, s14.NECK_XPBD_BETA = saved
        # olculen: XPBD'siz medyan 60.1 der, 8/8 sahne sinirda; XPBD (0.01, 16) ile 50.1, 3/8
        self.assertEqual(sum(v > 59.0 for v in res[False]), len(scenes))
        self.assertLess(np.median(res[True]), np.median(res[False]) - 5.0)
        self.assertLess(sum(v > 59.0 for v in res[True]), len(scenes) // 2)


if __name__ == "__main__":
    unittest.main()
