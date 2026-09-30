"""Adim 17: Bilge derisi + step14 fizigi + Faz A kinematik kontak-faz sensoru."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from physics.active_gait import (classify_contact_phase, PHASE_HEEL_STRIKE, PHASE_FLAT_FOOT,
                                 PHASE_TOE_OFF, PHASE_SWING)
from demo import step14_active_biped as s14
from demo.step17_bilge_physics_skin import simulate, inspect, phase_report, stance_pitch_deg


class ContactPhaseSensorTest(unittest.TestCase):
    def test_thresholds(self):
        self.assertEqual(classify_contact_phase(-2.01), PHASE_HEEL_STRIKE)
        self.assertEqual(classify_contact_phase(-2.0), PHASE_FLAT_FOOT)
        self.assertEqual(classify_contact_phase(0.0), PHASE_FLAT_FOOT)
        self.assertEqual(classify_contact_phase(2.0), PHASE_FLAT_FOOT)
        self.assertEqual(classify_contact_phase(2.01), PHASE_TOE_OFF)

    def test_pitch_sign_and_caps(self):
        self.assertLess(stance_pitch_deg(PHASE_HEEL_STRIKE, -10.0), 0.0)
        self.assertGreater(stance_pitch_deg(PHASE_TOE_OFF, 10.0), 0.0)
        self.assertEqual(stance_pitch_deg(PHASE_FLAT_FOOT, 1.5), 0.0)
        self.assertGreaterEqual(stance_pitch_deg(PHASE_HEEL_STRIKE, -90.0), -20.0)
        self.assertLessEqual(stance_pitch_deg(PHASE_TOE_OFF, 90.0), 35.0)


class Step14RegressionTest(unittest.TestCase):
    """Refactor (ActiveBipedSim) + sensor + diz tohumu fizigi DEGISTIRMEMELI."""

    @classmethod
    def setUpClass(cls):
        cls.sim = s14.ActiveBipedSim()
        cls.knee_offsets = {"stance": [], "swing": []}
        cls.phases_seen = set()
        for _ in range(s14.N_FRAMES):
            cls.sim.step()
            for leg in cls.sim.legs:
                hip, knee, foot = leg.chain.points
                d, v = foot - hip, knee - hip
                cls.knee_offsets[leg.state].append(-(d[0] * v[1] - d[1] * v[0]) / np.linalg.norm(d))
                cls.phases_seen.add(leg.contact_phase)
                if leg.state == "swing":
                    assert leg.contact_phase == PHASE_SWING

    def test_documented_canaries(self):
        sim = self.sim
        self.assertFalse(sim.nan)
        self.assertFalse(sim.fell)
        self.assertEqual(len(sim.step_events), 27)
        self.assertEqual(len(sim.emergency_step_events), 36)
        self.assertEqual(len(sim.slip_events), 89)
        self.assertAlmostEqual(sim.hip_y_log[-1], 147.51, places=2)

    def test_knees_bend_forward(self):
        self.assertEqual(sum(o < -1.0 for o in self.knee_offsets["swing"]), 0)
        self.assertEqual(sum(o < -1.0 for o in self.knee_offsets["stance"]), 0)

    def test_all_phases_observed(self):
        self.assertTrue({PHASE_HEEL_STRIKE, PHASE_FLAT_FOOT, PHASE_TOE_OFF, PHASE_SWING} <= self.phases_seen)


class SkinOnPhysicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.frames, cls.sim = simulate(n_frames=120)

    def test_phase_sequences_are_monotonic(self):
        report = phase_report(self.frames)
        self.assertEqual(report["non_monotonic"], 0)

    def test_skin_contact_and_connections(self):
        report = inspect(self.frames)
        self.assertGreaterEqual(min(report["minimum_pair_overlap_pixels"].values()), 5)
        self.assertLessEqual(report["max_visible_sole_penetration_px"], 0.0)
        self.assertLess(report["max_stance_pivot_drift_px_excluding_physics_slip"], 2.0)


if __name__ == "__main__":
    unittest.main()
