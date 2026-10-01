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


CANARY = (16, 0, 92, 1, 327.21)  # varsayilan senaryo, Adim 28 (Hill): 150 px (~4.2 m/s) itkide bacak COKUYOR (kare 222)
CANARY_26 = (25, 0, 89, 1, 145.95)  # Adim 26 (shock_mode="servo")
CANARY_23 = (22, 0, 165, 7, 145.99)  # Adim 23 + servo baslangic duzeltmesi (PRE24 bayraklariyla; duzeltme oncesi 22/0/157/6/145.97)
CANARY_22 = (25, 0, 122, 2, 146.20)  # Adim 22 (catch_timing="fixed")
CANARY_21 = (23, 0, 63, 9, 146.32)  # Adim 21 fizigi (LEGACY_21 bayraklariyla birebir)
# Adim 22 oncesi govde: kutlesiz bacak, kalcadan itki, durus kontrolu yok, PD kol
# Adim 24 oncesi sok/tetik davranisi (sabit hiz sinirli sok emici, kapanma kapisi yok)
PRE24 = dict(closing_ttc=0.0, shock_mode="rate_cap", shock_trigger="catch", preactivation=0,
             rocker=False, hip_strategy_gain=0.0, hill=False)
LEGACY_21 = dict(leg_mass=False, thrust_mode="hip", posture_k=0.0, posture_c=0.0, arms_mode="drive",
                 catch_timing="fixed", **PRE24)
LEGACY_PHYS = dict(leg_mass=False, thrust_mode="hip", posture_k=0.0, posture_c=0.0, catch_timing="fixed", **PRE24)
# Adim 23 oncesi yakalama: sabit 3 karelik zamanlayici (Faz B testleri bu davranisi olcer)
FIXED_CATCH = dict(catch_timing="fixed", **PRE24)


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
        self.assertTrue(sim.fell)              # Adim 27: kuvvet sinirli bacak cokuyor
        self.assertEqual(sim.collapse_frame, 222)
        # Adim 23: 150 px itki (COM ~4 m/s) insan torkuyla 4-12 karelik yakalamalarla karsilaniyor
        # Adim 22: kutleli bacak (buzda mikro kaymalar: 122 kayma karesi, hepsi buz bolgesinde)
        # Adim 21: esnek olmayan govde kelepcesi + kollar (20: 25/0/4/2/146.69;
        # 19c: 26/0/2/1/146.60; 18: 26/0/14/1/146.62; 17: 27/36/89/147.51)
        self.assertEqual(len(sim.step_events), CANARY[0])
        self.assertEqual(len(sim.emergency_step_events), CANARY[1])
        self.assertEqual(len(sim.slip_events), CANARY[2])
        self.assertEqual(len(sim.fazb_events), CANARY[3])
        self.assertAlmostEqual(sim.hip_y_log[-1], CANARY[4], places=2)

    def test_servo_shock_reproduces_step26(self):
        sim = s14.ActiveBipedSim(shock_mode="servo", hill=False)
        for _ in range(s14.N_FRAMES):
            sim.step()
        self.assertFalse(sim.fell)
        self.assertEqual((len(sim.step_events), len(sim.emergency_step_events), len(sim.slip_events),
                          len(sim.fazb_events)), CANARY_26[:4])
        self.assertAlmostEqual(sim.hip_y_log[-1], CANARY_26[4], places=2)

    def test_pre24_flags_reproduce_step23(self):
        sim = s14.ActiveBipedSim(**PRE24)
        for _ in range(s14.N_FRAMES):
            sim.step()
        self.assertEqual((len(sim.step_events), len(sim.emergency_step_events), len(sim.slip_events),
                          len(sim.fazb_events)), CANARY_23[:4])
        self.assertAlmostEqual(sim.hip_y_log[-1], CANARY_23[4], places=2)

    def test_fixed_catch_reproduces_step22(self):
        sim = s14.ActiveBipedSim(**FIXED_CATCH)
        for _ in range(s14.N_FRAMES):
            sim.step()
        self.assertEqual((len(sim.step_events), len(sim.emergency_step_events), len(sim.slip_events),
                          len(sim.fazb_events)), CANARY_22[:4])
        self.assertAlmostEqual(sim.hip_y_log[-1], CANARY_22[4], places=2)

    def test_legacy_flags_reproduce_step21(self):
        sim = s14.ActiveBipedSim(**LEGACY_21)
        for _ in range(s14.N_FRAMES):
            sim.step()
        self.assertEqual((len(sim.step_events), len(sim.emergency_step_events), len(sim.slip_events),
                          len(sim.fazb_events)), CANARY_21[:4])
        self.assertAlmostEqual(sim.hip_y_log[-1], CANARY_21[4], places=2)

    def test_arms_off_reproduces_step19(self):
        sim = s14.ActiveBipedSim(arms_mode="off", torso_clamp_mode=True, **LEGACY_PHYS)
        for _ in range(s14.N_FRAMES):
            sim.step()
        self.assertEqual((len(sim.step_events), len(sim.emergency_step_events), len(sim.slip_events),
                          len(sim.fazb_events)), (26, 0, 2, 1))
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
        # Adim 21: ilk 30 kare baslangic sarsintisi (govde ilk karede kelepceden
        # ters sinira savruluyor, kare 9'da bir yakalama) -- yuruyus degil
        self.assertEqual([e for e in on.fazb_events if e[0] >= 30], [])
        self.assertEqual(ds_on, 0)
        # Adim 21'den beri baslangic sarsintisindaki tek yakalama yorungeyi
        # ayirdigi icin bit-bit esitlik yerine yuruyus olcutleri karsilastirilir
        self.assertLessEqual(abs(len(on.step_events) - len(off.step_events)), 2)
        self.assertFalse(on.fell or off.fell)

    def test_forward_pushes_single_support_and_no_collapse(self):
        for push in (100.0, 150.0, 300.0):
            sim, ds, _ = self.run_sim(420, big_push_kick_px=push, **FIXED_CATCH)
            self.assertFalse(sim.fell, push)
            self.assertEqual(ds, 0, push)
            self.assertGreater(s14.GROUND_Y - max(sim.hip_y_log[210:]), 75.0, push)

    def test_backward_pushes_heel_side_catch(self):
        """Adim 19: heel_strike aynasi -- geri itkide cift-havada yok, eski acil yol devreye girmez."""
        for push in (-100.0, -150.0, -300.0):
            sim, ds, _ = self.run_sim(420, big_push_kick_px=push, **FIXED_CATCH)
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
            # (kollar da kapali: Adim 19b olcumu kolsuz govdede yapildi)
            on, _, _ = self.run_sim(420, big_push_kick_px=push, predictive_sensor=False, arms_mode="off",
                                    torso_clamp_mode=True, **LEGACY_PHYS)
            off, _, _ = self.run_sim(420, big_push_kick_px=push, shock_absorb=False, predictive_sensor=False,
                                     arms_mode="off", torso_clamp_mode=True, **LEGACY_PHYS)
            rise_on = max(-np.diff(np.array(on.hip_y_log))[210:])
            rise_off = max(-np.diff(np.array(off.hip_y_log))[210:])
            self.assertLess(rise_on, 25.0, push)
            self.assertGreater(rise_off, 45.0, push)
            self.assertFalse(on.fell, push)
        a, _, pa = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0)
        b, _, pb = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0, shock_absorb=False)
        self.assertTrue(np.array_equal(pa, pb))

    def test_predictive_sensor_removes_lag(self):
        """Adim 19c: yakalama itkinin geldigi karede tetiklenir; itkisiz yuruyus degismez.
        Cokme derinligi kiyasi Adim 21 govdesinde (LEGACY_21) -- Adim 22'nin kutleli
        bacaklarinda +150'de prediktif sensor cokmeyi KUCULTMUYOR (README Adim 22)."""
        for push in (150.0, -150.0):
            on, _, _ = self.run_sim(300, big_push_kick_px=push, **LEGACY_21)
            off, _, _ = self.run_sim(300, big_push_kick_px=push, predictive_sensor=False, **LEGACY_21)
            bp = round(s14.BIG_PUSH_T * s14.FPS)
            self.assertEqual(min(e[0] for e in on.fazb_events if e[0] >= bp), bp, push)
            self.assertGreater(min(e[0] for e in off.fazb_events if e[0] >= bp), bp, push)
            self.assertGreater(s14.GROUND_Y - max(on.hip_y_log[bp:]), s14.GROUND_Y - max(off.hip_y_log[bp:]), push)
        a, _, pa = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0)
        b, _, pb = self.run_sim(900, stumble_kick_px=0.0, big_push_kick_px=0.0, predictive_sensor=False)
        self.assertEqual([e for e in a.fazb_events if e[0] >= 30], [])
        self.assertTrue(np.array_equal(pa, pb))

    def test_compress_swing_keeps_position_continuous(self):
        leg = s14.make_leg(np.array([0.0, s14.HIP_Y]), 0.0)
        leg.catch_timing = "fixed"
        leg.state, leg.swing_t, leg._active_swing_duration = "swing", 0.4, 10.0
        leg.swing_start, leg.swing_target = np.array([-20.0, s14.GROUND_Y]), np.array([30.0, s14.GROUND_Y])
        self.assertTrue(leg.compress_swing(0.0, 2.0, frames=3.0))
        self.assertAlmostEqual(leg.swing_t, 0.4)
        self.assertAlmostEqual((1.0 - leg.swing_t) * leg._active_swing_duration, 3.0)
        self.assertTrue(leg.catch_active and leg.emergency_step_active)


class PhysicalArmsTest(unittest.TestCase):
    """Adim 20: kutleli Verlet kollar."""

    def test_cancel_mode_is_contralateral(self):
        """Adim 22: yaw momentum iptali capraz salinimi KENDILIGINDEN uretir."""
        sim = s14.ActiveBipedSim(stumble_kick_px=0.0, big_push_kick_px=0.0)
        al, ar, ll, lr = [], [], [], []
        for f in range(900):
            sim.step()
            hip = sim.body.points[sim.idx["hip"]]
            if f >= 60:
                al.append(sim.arms.arm_angle("l"))
                ar.append(sim.arms.arm_angle("r"))
                for out, leg in ((ll, sim.left_leg), (lr, sim.right_leg)):
                    ft = leg.chain.points[-1]
                    out.append(np.arctan2(ft[0] - hip[0], ft[1] - hip[1]))
        self.assertGreater(np.corrcoef(al, lr)[0, 1], 0.6)    # sol kol ~ sag bacak
        self.assertGreater(np.corrcoef(ar, ll)[0, 1], 0.6)
        self.assertLess(np.corrcoef(al, ar)[0, 1], -0.4)      # kollar birbirine ters

    def test_contralateral_swing_and_integrity(self):
        sim = s14.ActiveBipedSim(stumble_kick_px=0.0, big_push_kick_px=0.0, **LEGACY_21)
        arm_a, leg_a, stick_err, elbow = [], [], 0.0, []
        sh = sim.idx["shoulder"]
        for f in range(900):
            sim.step()
            p = sim.body.points
            hip = p[sim.idx["hip"]]
            foot = sim.left_leg.chain.points[-1]
            if f >= 60:
                arm_a.append(sim.arms.arm_angle("l"))
                leg_a.append(np.arctan2(foot[0] - hip[0], foot[1] - hip[1]))
            for i, j, L, _ in sim.body.sticks[3:]:
                stick_err = max(stick_err, abs(np.linalg.norm(p[j] - p[i]) - L))
            for e, h in sim.arms.idx.values():
                ua, fa = p[e] - p[sh], p[h] - p[e]
                elbow.append(np.degrees(np.arctan2(ua[0] * fa[1] - ua[1] * fa[0], ua @ fa)))
        self.assertFalse(sim.fell)
        self.assertLess(np.corrcoef(arm_a, leg_a)[0, 1], -0.7)
        self.assertLess(stick_err, 0.01)
        self.assertLessEqual(max(elbow), -1.0)   # tek yonlu dirsek: hiperekstansiyon yok

    def test_pushes_with_arms(self):
        for push in (150.0, -150.0, 500.0, -500.0):
            sim = s14.ActiveBipedSim(big_push_kick_px=push, **FIXED_CATCH)
            double = 0
            for _ in range(420):
                sim.step()
                double += all(l.state == "swing" for l in sim.legs)
            self.assertFalse(sim.fell, push)
            self.assertEqual(double, 0, push)


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
