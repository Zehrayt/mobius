"""Adim 25: inise hazirlik (pre-activation) -- TTC ile temastan once ekstansor kasilmasi."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14

KICK_PER_MS = 5.18 * 184.0 / 0.9 / 30.0


def run(push_ms, t=7.0, n=420, **kw):
    kw.setdefault("shock_mode", "servo")   # Adim 24-26 sok servosu (Adim 27: "force")
    sim = s14.ActiveBipedSim(big_push_kick_px=push_ms * KICK_PER_MS, big_push_t=t, **kw)
    for _ in range(n):
        sim.step()
    return sim


def sag_and_recovery(sim, bp=210):
    H = s14.GROUND_Y - np.array(sim.hip_y_log)
    c = sim.shock_events[0][0] - 1
    sag = H[c] - H[c:c + 6].min()
    i = bp + int(np.argmin(H[bp:]))
    j = next(k for k in range(i, len(H)) if H[k] >= 170)
    return sag, j - i


class PreactivationTest(unittest.TestCase):
    def test_undisturbed_walk_unchanged(self):
        a = s14.ActiveBipedSim(stumble_kick_px=0.0, big_push_kick_px=0.0)
        b = s14.ActiveBipedSim(stumble_kick_px=0.0, big_push_kick_px=0.0, preactivation=0)
        for _ in range(900):
            a.step()
            b.step()
        self.assertTrue(np.array_equal(a.body.points, b.body.points))

    def test_contact_starts_with_preactivated_velocity(self):
        sim = run(150.0 / KICK_PER_MS)
        v0 = [p[2] for p in sim.preact_log if p[0] >= 210]
        self.assertTrue(v0)
        self.assertGreater(max(v0), 0.0)
        self.assertLessEqual(max(v0), s14.SHOCK_EXT_VMAX + 1e-9)

    def test_less_sag_and_faster_recovery(self):
        for dv in (150.0 / KICK_PER_MS, -150.0 / KICK_PER_MS):
            on = sag_and_recovery(run(dv))
            off = sag_and_recovery(run(dv, preactivation=0))
            self.assertLessEqual(on[0], off[0], dv)
            self.assertLessEqual(on[1], off[1], dv)

    def test_time_to_contact_prediction(self):
        sim = s14.ActiveBipedSim(big_push_kick_px=150.0, big_push_t=7.0)
        preds = []
        for f in range(300):
            sim.step()
            for leg in sim.legs:
                if getattr(leg, "catch_servo", False):
                    t = leg.time_to_contact(sim.body.points[sim.idx["hip"]])
                    if t is not None and t <= 3:
                        preds.append((f, "l" if leg is sim.left_leg else "r", t))
        errs = []
        for f, side, t in preds:
            land = [c[0] for c in sim.catch_frames_log if c[1] == side and c[0] > f]
            if land:
                errs.append(min(land) - f - t)
        self.assertTrue(errs)
        self.assertLessEqual(float(np.median(np.abs(errs))), 1.0)


if __name__ == "__main__":
    unittest.main()
