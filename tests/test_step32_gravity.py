"""One gravity across integration, inverse dynamics, catch torque and collapse."""
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from demo import step14_active_biped as s14
from demo.step32_gravity import compare, walk_report
from physics.gravity import GravityPolicy, REAL_GRAVITY, LEGACY_WALK_GRAVITY


class GravityPolicyTest(unittest.TestCase):
    def test_all_consumers_share_the_physical_value(self):
        sim=s14.ActiveBipedSim()
        self.assertEqual(sim.gravity_policy.mode,'unified')
        self.assertAlmostEqual(REAL_GRAVITY,9.81*(184/.9)/30**2)
        self.assertEqual(sim.body.gravity[1],REAL_GRAVITY)
        self.assertEqual(sim.leg_mass.g[1],REAL_GRAVITY)
        self.assertEqual(sim.gravity_policy.muscle_load,REAL_GRAVITY)
        for leg in sim.legs:
            self.assertEqual(leg.gravity_y,REAL_GRAVITY)
            self.assertAlmostEqual(leg.omega0,np.sqrt(REAL_GRAVITY/s14.ARM_LENGTH))
        gravity=sim.body.gravity
        sim._collapse('test')
        self.assertIs(sim.body.gravity,gravity)
        self.assertEqual(sim.body.gravity[1],REAL_GRAVITY)

    def test_free_fall_acceleration_is_unchanged_after_transition(self):
        for collapsed in (False,True):
            sim=s14.ActiveBipedSim()
            if collapsed:sim._collapse('test')
            body=sim.body
            body.sticks=[]
            body.friction=0
            body.prev_points=body.points.copy()
            start=body.points.copy()
            free=[i for i in range(len(start)) if i not in body.pinned]
            body.step()
            np.testing.assert_allclose((body.points-start)[free,1],REAL_GRAVITY,atol=1e-12)
            first=body.points.copy()
            body.step()
            np.testing.assert_allclose((body.points-first)[free,1],2*REAL_GRAVITY,atol=1e-12)

    def test_catch_torque_accounts_for_greater_gravity_load(self):
        new=s14.ActiveBipedSim()
        old=s14.ActiveBipedSim(gravity_mode='legacy')
        for sim in (new,old):
            sim.left_leg.servo_pos=np.array([80.,300.])
        self.assertLess(new.left_leg.servo_accel_limit(np.array([0.,150.])),
                        old.left_leg.servo_accel_limit(np.array([0.,150.])))

    def test_legacy_mode_is_explicit_and_reproduces_the_switch(self):
        old=s14.ActiveBipedSim(gravity_mode='legacy')
        self.assertEqual(old.body.gravity[1],LEGACY_WALK_GRAVITY)
        old._collapse('test')
        self.assertEqual(old.body.gravity[1],REAL_GRAVITY)
        with self.assertRaises(ValueError):GravityPolicy('unknown')
        with self.assertRaises(ValueError):s14.ActiveBipedSim(fps=60)


class UnifiedBipedTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report,cls.frames=compare()

    def test_gravity_constant_in_entire_reference_fall(self):
        report=self.report['unified']
        self.assertIsNotNone(report['collapse_frame'])
        self.assertEqual(report['gravity_values'],[REAL_GRAVITY])
        self.assertEqual(self.report['legacy']['gravity_values'],[LEGACY_WALK_GRAVITY,REAL_GRAVITY])
        self.assertEqual(report['collapse_reason'],'support_height')
        self.assertTrue(report['finite'])
        self.assertEqual(report['max_ground_penetration_px'],0)
        self.assertLess(report['tail_max_speed_px_frame'],.2)
        self.assertLess(report['tail_hip_drift_px'],1.)

    def test_sixty_second_walk_on_dry_and_ice_ground(self):
        for ice in (False,True):
            r=walk_report(ice=ice)
            self.assertFalse(r['fell'] or r['collapsed'],r)
            self.assertTrue(r['finite'])
            self.assertEqual(r['gravity_values'],[REAL_GRAVITY])
            self.assertGreater(r['steps'],100)
            self.assertGreater(r['mean_speed'],1.7)
            self.assertLess(r['mean_speed'],2.2)
            self.assertGreater(r['min_hip_height'],150.)
            self.assertTrue({'heel_strike','flat_foot','toe_off'}<=set(r['phases']))

    def test_small_push_does_not_collapse(self):
        sim=s14.ActiveBipedSim(big_push_kick_px=15.)
        for _ in range(900):sim.step()
        self.assertFalse(sim.fell or sim.collapsed or sim.nan)
        self.assertGreater(np.mean(sim.hip_vx_log[-300:]),1.7)

    def test_hard_pushes_enter_ragdoll_before_body_penetrates_ground(self):
        for push in (75.,150.,-150.,300.,-300.):
            sim=s14.ActiveBipedSim(big_push_kick_px=push)
            for _ in range(420):
                sim.step()
                ids=[i for i in range(len(sim.body.points)) if i not in sim.body.pinned]
                self.assertLessEqual(sim.body.points[ids,1].max(),s14.GROUND_Y+1e-6,push)
                if sim.fell:self.assertTrue(sim.collapsed,push)
            self.assertTrue(sim.collapsed,push)
            self.assertFalse(sim.nan,push)

if __name__=='__main__':unittest.main()
