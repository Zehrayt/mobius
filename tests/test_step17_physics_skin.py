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
        # Adim 19c: prediktif sensor dahil (19b: 26/0/5/1/146.42; 18: 26/0/14/1/146.62; 17: 27/36/89/147.51)
        self.assertEqual(len(sim.step_events), 26)
        self.assertEqual(len(sim.emergency_step_events), 0)
        self.assertEqual(len(sim.slip_events), 2)
        self.assertEqual(len(sim.fazb_events), 1)
        self.assertAlmostEqual(sim.hip_y_log[-1], 146.60, places=2)

    def test_knees_bend_forward(self):
        self.assertEqual(sum(o < -1.0 for o in self.knee_offsets["swing"]), 0)
        self.assertEqual(sum(o < -1.0 for o in self.knee_offsets["stance"]), 0)

    def test_all_phases_observed(self):
        self.assertTrue({PHASE_HEEL_STRIKE, PHASE_FLAT_FOOT, PHASE_TOE_OFF, PHASE_SWING} <= self.phases_seen)


class FazBTest(unittest.TestCase):
    """Adim 18: toe_off tetikli yakalama adimi."""

    @staticmethod
    def run_sim(n, **kw):
        sim = s14.ActiveBipedSim(**kw)
        double_swing, pts = 0, []
        for _ in range(n):
            sim.step()
            double_swing += all(l.state == "swing" for l in sim.legs)
            pts.append(sim.body.points.copy())
        return sim, double_swing, np.array(pts)

    def test_undisturbed_walk_never_triggers_and_is_unchanged(self):
        on, ds_on, p_on = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0)
        off, _, p_off = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0, faz_b=False)
        self.assertEqual(on.fazb_events, [])
        self.assertEqual(ds_on, 0)
        self.assertTrue(np.array_equal(p_on, p_off))

    def test_forward_pushes_single_support_and_no_collapse(self):
        for push in (100.0, 150.0, 300.0):
            sim, ds, _ = self.run_sim(420, big_push_kick_px=push)
            self.assertFalse(sim.fell, push)
            self.assertEqual(ds, 0, push)
            self.assertGreater(s14.GROUND_Y - max(sim.hip_y_log[210:]), 75.0, push)

    def test_backward_pushes_heel_side_catch(self):
        """Adim 19: heel_strike aynasi -- geri itkide cift-havada yok, eski acil yol devreye girmez."""
        for push in (-100.0, -150.0, -300.0):
            sim, ds, _ = self.run_sim(420, big_push_kick_px=push)
            self.assertFalse(sim.fell, push)
            self.assertEqual(ds, 0, push)
            self.assertEqual(len(sim.emergency_step_events), 0, push)
            self.assertTrue(any(e[3] == "heel" for e in sim.fazb_events), push)
            self.assertGreater(s14.GROUND_Y - max(sim.hip_y_log[210:]), 60.0, push)

    def test_catch_shock_absorption_limits_rise(self):
        """Adim 19b: yakalama sonrasi kalca yukselisi sinirli; itkisiz yuruyus degismez."""
        for push in (150.0, -150.0):
            # sok emilimini izole etmek icin prediktif sensor kapali (19c o
            # senaryoda cokusun kendisini kucultuyor)
            on, _, _ = self.run_sim(420, big_push_kick_px=push, predictive_sensor=False)
            off, _, _ = self.run_sim(420, big_push_kick_px=push, shock_absorb=False, predictive_sensor=False)
            rise_on = max(-np.diff(np.array(on.hip_y_log))[210:])
            rise_off = max(-np.diff(np.array(off.hip_y_log))[210:])
            self.assertLess(rise_on, 25.0, push)
            self.assertGreater(rise_off, 45.0, push)
            self.assertFalse(on.fell, push)
        a, _, pa = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0)
        b, _, pb = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0, shock_absorb=False)
        self.assertTrue(np.array_equal(pa, pb))

    def test_predictive_sensor_removes_lag(self):
        """Adim 19c: yakalama itkinin geldigi karede tetiklenir; itkisiz yuruyus degismez."""
        for push in (150.0, -150.0):
            on, _, _ = self.run_sim(300, big_push_kick_px=push)
            off, _, _ = self.run_sim(300, big_push_kick_px=push, predictive_sensor=False)
            bp = round(s14.BIG_PUSH_T * s14.FPS)
            self.assertEqual(min(e[0] for e in on.fazb_events if e[0] >= bp), bp, push)
            self.assertGreater(min(e[0] for e in off.fazb_events if e[0] >= bp), bp, push)
            self.assertGreater(s14.GROUND_Y - max(on.hip_y_log[bp:]), s14.GROUND_Y - max(off.hip_y_log[bp:]), push)
        a, _, pa = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0)
        b, _, pb = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0, predictive_sensor=False)
        self.assertEqual(a.fazb_events, [])
        self.assertTrue(np.array_equal(pa, pb))

    def test_compress_swing_keeps_position_continuous(self):
        leg = s14.make_leg(np.array([0.0, s14.HIP_Y]), 0.0)
        leg.state, leg.swing_t, leg._active_swing_duration = "swing", 0.4, 10.0
        leg.swing_start, leg.swing_target = np.array([-20.0, s14.GROUND_Y]), np.array([30.0, s14.GROUND_Y])
        self.assertTrue(leg.compress_swing(0.0, 2.0, frames=3.0))
        self.assertAlmostEqual(leg.swing_t, 0.4)
        self.assertAlmostEqual((1.0 - leg.swing_t) * leg._active_swing_duration, 3.0)
        self.assertTrue(leg.catch_active and leg.emergency_step_active)


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
