"""Adim 29: cokus sonrasi yere yigilma -- ragdoll bacaklar, zemin, diz menteşesi, durgunluk."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14
from demo.step17_bilge_physics_skin import simulate, PhysicsBilgeRig, SCREEN_GROUND_Y

KICK_PER_MS = 5.18 * 184.0 / 0.9 / 30.0


def run_collapse(push_px, n=420):
    sim = s14.ActiveBipedSim(gravity_mode="legacy", big_push_kick_px=push_px, big_push_t=7.0)
    pts = []
    for _ in range(n):
        sim.step()
        pts.append(sim.body.points.copy())
    return sim, pts


class FallenStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = {p: run_collapse(p) for p in (150.0, -150.0, 2.5 * KICK_PER_MS, -2.5 * KICK_PER_MS)}

    def collapsed_runs(self):
        out = [(p, r) for p, r in self.runs.items() if r[0].collapsed]
        self.assertGreaterEqual(len(out), 2)
        return out

    def test_no_point_below_ground(self):
        for push, (sim, pts) in self.collapsed_runs():
            anchor = sim.idx["anchor"]
            for P in pts[sim.collapse_frame + 1:]:
                ys = np.delete(P[:, 1], anchor)
                self.assertLessEqual(float(ys.max()), s14.GROUND_Y + 1e-6, push)

    def test_comes_to_rest_without_sliding(self):
        for push, (sim, pts) in self.collapsed_runs():
            tail = np.array(pts[-60:])
            speed = np.linalg.norm(np.diff(tail, axis=0), axis=2).max()
            self.assertLess(speed, 0.2, push)
            self.assertLess(abs(tail[-1][sim.idx["hip"]][0] - tail[0][sim.idx["hip"]][0]), 1.0, push)
            self.assertFalse(sim.nan, push)

    def test_knee_hinge_and_bone_lengths(self):
        for push, (sim, pts) in self.collapsed_runs():
            hip = sim.idx["hip"]
            # carpma karelerinde (cokus +0..+3) diz siniri 5 px'e kadar asilabiliyor
            for P in pts[sim.collapse_frame + 5:]:
                for k_i, f_i in sim.fallen_legs.values():
                    d, v = P[f_i] - P[hip], P[k_i] - P[hip]
                    n = np.linalg.norm(d)
                    offset = -(d[0] * v[1] - d[1] * v[0]) / max(n, 1e-6)
                    self.assertGreaterEqual(offset, -1.0, push)          # geri bukulme yok
                    self.assertGreaterEqual(n, s14.KNEE_FLEX_MIN_DIST - 2.0, push)
                    self.assertAlmostEqual(np.linalg.norm(P[k_i] - P[hip]), s14.LEG_SEGMENT_LEN, delta=2.0)
                    self.assertAlmostEqual(np.linalg.norm(P[f_i] - P[k_i]), s14.LEG_SEGMENT_LEN, delta=2.0)

    def test_render_torso_and_head_above_ground(self):
        frames, sim = simulate(320, gravity_mode="legacy", big_push_kick_px=150.0, big_push_t=7.0)
        rig = PhysicsBilgeRig()
        for snap in frames[sim.collapse_frame:]:
            pts = rig.pose(snap)["points"]
            self.assertLessEqual(pts["chest"][1], SCREEN_GROUND_Y + 1e-6)
            self.assertLessEqual(pts["head"][1], SCREEN_GROUND_Y + 1e-6)


if __name__ == "__main__":
    unittest.main()
