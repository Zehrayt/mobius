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


if __name__ == "__main__":
    unittest.main()
