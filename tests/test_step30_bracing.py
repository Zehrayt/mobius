"""Adim 30: koruyucu kol refleksi (bracing) -- uzanma, kol kolonu, kontrollu indirme."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14

KICK_PER_MS = 5.18 * 184.0 / 0.9 / 30.0
# (itki px, itki zamani s): hepsi varsayilan fizikte bacak cokusu uretiyor
SCENES = ((150.0, 7.0), (-150.0, 7.0), (-200.0, 7.0), (130.0, 6.6))


def run(push, t, brace, n=560):
    sim = s14.ActiveBipedSim(big_push_kick_px=push, big_push_t=t, brace=brace)
    pts, cols = [], []
    for _ in range(n):
        sim.step()
        pts.append(sim.body.points[:8].copy())
        cols.append(dict(sim.arm_column) if sim.bracing and sim.brace_level > 0.5 else {})
    return sim, np.array(pts), cols


def head_near_ground_speed(sim, pts):
    """Bas zemine 4 px yakinken en buyuk asagi hiz (px/kare), cokusten sonra."""
    i = sim.idx["head"]
    vy = np.diff(pts[:, i, 1])
    y = pts[1:, i, 1]
    near = (y > s14.GROUND_Y - s14.FALLEN_RADIUS["head"] - 4.0) & (np.arange(1, len(pts)) > sim.collapse_frame)
    return float(vy[near].max()) if near.any() else 0.0


class BracingFallTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = {(p, t, b): run(p, t, b) for p, t in SCENES for b in (False, True)}

    def test_scenes_collapse(self):
        for (p, t, b), (sim, _, _) in self.runs.items():
            self.assertTrue(sim.collapsed, (p, t, b))
            self.assertEqual(sim.bracing or sim.brace_level == 0.0, True)

    def test_head_impact_is_cushioned(self):
        pas, br = [], []
        for p, t in SCENES:
            pas.append(head_near_ground_speed(*self.runs[(p, t, False)][:2]))
            br.append(head_near_ground_speed(*self.runs[(p, t, True)][:2]))
            # refleksli: bas yere ~0.15 m/s (1 px/kare) ile iner
            self.assertLess(br[-1], 2.0, (p, t))
        # pasif yigilmada bas carpmasi sahneye gore 1.6-18 px/kare (kaotik); ortalamada >= 3 kat azalma
        self.assertGreater(np.mean(pas), 3.0 * np.mean(br), (pas, br))

    def test_arms_hold_the_shoulder(self):
        for p, t in SCENES:
            sim, pts, cols = self.runs[(p, t, True)]
            sh = sim.idx["shoulder"]
            loaded = [f for f, c in enumerate(cols) if len(c) == 2]
            self.assertGreater(len(loaded), 40, (p, t))
            # kollar yuk aldigi ilk 15 karede omuz zeminden en az 40 px yukarida tutulur
            first = loaded[0]
            h = s14.GROUND_Y - pts[first + 2:first + 17, sh, 1]
            self.assertGreater(float(h.min()), 40.0, (p, t))

    def test_hands_reach_the_falling_side(self):
        for p, t in SCENES:
            sim, pts, cols = self.runs[(p, t, True)]
            f = next(f for f, c in enumerate(cols) if len(c) == 2)
            hip, sh = pts[f, sim.idx["hip"], 0], pts[f, sim.idx["shoulder"], 0]
            sh_y = pts[f, sim.idx["shoulder"], 1]
            for e_i, h_i in sim.arms.idx.values():
                self.assertGreater((pts[f, h_i, 0] - hip) * (sh - hip), 0.0, (p, t))
                # el, omzun dusey izdusumune yakin (surtunme konisi icinde destek)
                self.assertLess(abs(pts[f, h_i, 0] - sh), 0.8 * (s14.GROUND_Y - sh_y) + 5.0, (p, t))

    def test_controlled_lowering_then_rest(self):
        for p, t in SCENES:
            sim, pts, _ = self.runs[(p, t, True)]
            self.assertTrue(sim.brace_lowered, (p, t))
            tail = pts[-60:]
            self.assertLess(float(np.linalg.norm(np.diff(tail, axis=0), axis=2).max()), 0.2, (p, t))
            anchor = sim.idx["anchor"]
            for P in pts[sim.collapse_frame + 1:]:
                self.assertLessEqual(float(np.delete(P[:, 1], anchor).max()), s14.GROUND_Y + 1e-6)


class BracingTriggerTest(unittest.TestCase):
    def test_no_reflex_in_walk_or_recoverable_pushes(self):
        for push in (0.0, KICK_PER_MS, -KICK_PER_MS, 2.0 * KICK_PER_MS, -2.0 * KICK_PER_MS):
            sim = s14.ActiveBipedSim(big_push_kick_px=push, big_push_t=7.0)
            ever = False
            for _ in range(480):
                sim.step()
                ever |= sim.bracing
            self.assertFalse(ever, push)
            self.assertFalse(sim.collapsed, push)


class ExtremePushFloorTest(unittest.TestCase):
    def test_low_hip_enters_fallen_state_instead_of_sinking(self):
        # Adim 29'da >= ~5.6 m/s itkilerde kalca zeminin 184 px altina iniyordu
        for push, t in ((200.0, 7.0), (-260.0, 7.0)):
            sim = s14.ActiveBipedSim(big_push_kick_px=push, big_push_t=t)
            ys = []
            for _ in range(420):
                sim.step()
                ys.append(sim.body.points[sim.idx["hip"]][1])
            self.assertTrue(sim.collapsed, push)
            self.assertLessEqual(max(ys), s14.GROUND_Y + 1e-6, push)
            self.assertFalse(sim.nan, push)


class ArmCapacityTest(unittest.TestCase):
    def test_straight_arm_is_a_column_and_eccentric_is_stronger(self):
        sim = s14.ActiveBipedSim()
        straight, bent = sim.arm_column_capacity(65.0, 0.0), sim.arm_column_capacity(40.0, 0.0)
        self.assertGreater(straight, 3.0 * bent)
        self.assertGreater(sim.arm_column_capacity(40.0, 3.0), 1.3 * bent)   # Hill eksantrik


if __name__ == "__main__":
    unittest.main()
