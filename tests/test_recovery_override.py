"""Kalkma sirasinda dis darbe: kalkma hata vermeden kesilir, koruyucu refleks yeniden
kurulur, karakter durulunca kalkma bastan baslar (durum gecisi, exception degil)."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14

STAND = dict(ground_recovery=True, recovery_reposition=True, recovery_transfer=True,
             recovery_rise=True, recovery_stand=True)


def run(bracing, kick=0.0, push=150.0, frames=4100):
    sim = s14.ActiveBipedSim(big_push_kick_px=push, big_push_t=7.0, bracing=bracing, **STAND)
    states, kicked, below = [], None, 0.0
    for f in range(frames):
        sim.step()
        st = sim.recovery.state
        if not states or states[-1][1] != st:
            states.append((f, st))
        if kick and kicked is None and st == "standing":
            kicked = f + 40
        if kicked is not None and f == kicked:
            sim.push(kick)
        if sim.collapsed:
            anchor = sim.idx["anchor"]
            below = max(below, float(np.delete(sim.body.points[:, 1], anchor).max()) - s14.GROUND_Y)
        if sim.nan:
            break
    return sim, states, kicked, below


class RecoveryOverrideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plain = run(True)
        cls.column = run(True, kick=-120.0)
        cls.impulse = run("impulse", kick=-120.0)

    def test_column_reflex_and_recovery_chain_coexist(self):
        sim, states, _, below = self.plain
        self.assertEqual(sim.bracing_mode, "reflex")
        self.assertEqual(sim.recovery.state, "standing", states)
        self.assertEqual(sim.recovery_aborts, [], "normal kalkis kesilmemeli")
        self.assertLessEqual(below, 1e-6)

    def test_push_while_standing_aborts_rebraces_and_rises_again(self):
        for name, (sim, states, kicked, below) in (("column", self.column), ("impulse", self.impulse)):
            with self.subTest(name):
                self.assertFalse(sim.nan)
                self.assertIsNotNone(kicked)
                self.assertTrue(sim.recovery_aborts, states)
                frame, state, reason = sim.recovery_aborts[0]
                self.assertIn(state, ("standing", "standing_rising"))
                self.assertIn(reason, ("xcom", "ttc"))
                self.assertLessEqual(frame - kicked, 15)
                # kesildikten sonra zincir bastan: once 'waiting', sonra yeniden ayakta
                after = [s for f, s in states if f >= frame]
                self.assertEqual(after[0], "waiting", states)
                self.assertEqual(sim.recovery.state, "standing", states)
                self.assertLessEqual(below, 1e-6)

    def test_reflex_is_rearmed_on_override(self):
        sim = self.column[0]
        self.assertEqual(sim.brace_trigger, "override")
        self.assertGreater(sim.brace_start, sim.recovery_aborts[0][0] - 1)
        imp = self.impulse[0]
        # impuls refleksi yeni bir FallBracing ile yeniden kuruldu ve kullanildi
        self.assertGreaterEqual(imp.fall_bracing.start_frame, imp.recovery_aborts[0][0])


if __name__ == "__main__":
    unittest.main()
