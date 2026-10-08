"""Recovery must wait for rest, use bounded impulses and verify real support."""
from pathlib import Path
import copy
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from demo.step14_active_biped import ActiveBipedSim,GROUND_Y
from demo.step34_ground_support import run_case
from physics.ground_recovery import GroundRecovery,ATTEMPT_TIMEOUT
from physics.gravity import REAL_GRAVITY
from physics.verlet import VerletSystem


class RecoveryMotorTest(unittest.TestCase):
    def test_motor_pairs_preserve_linear_momentum_and_never_move_positions(self):
        body=VerletSystem.empty()
        a=body.add_point([0.,0.],mass=2.)
        b=body.add_point([60.,10.],mass=.3)
        r=GroundRecovery();p=body.points.copy();q=body.prev_points.copy()
        r._motor(body,a,b,0.,10.,1.)
        r._extend(body,a,b,5000.,3.,1.)
        np.testing.assert_array_equal(body.points,p)
        impulse=np.sum(body.masses[:,None]*(q-body.prev_points),axis=0)
        np.testing.assert_allclose(impulse,0.,atol=1e-12)
        self.assertEqual(r.max_torque_ratio,1.)
        self.assertEqual(r.max_force_ratio,1.)

    def test_incompatible_configuration_is_rejected(self):
        for options in [dict(gravity_mode='legacy'),dict(arms_mode='off'),
                        dict(articulated_spine=False),dict(balance_recovery=False)]:
            with self.assertRaises(ValueError):ActiveBipedSim(ground_recovery=True,**options)


class RecoveryIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report,cls.frames,cls.sim=run_case(count=700)

    def test_no_physics_change_before_rest_gate_opens(self):
        old=ActiveBipedSim(bracing="impulse")
        end=self.report['start_frame']
        self.assertIsNotNone(end)
        self.assertGreaterEqual(end-self.report['collapse_frame'],60)
        for f in range(end+1):
            old.step()
            for key in ('hip','shoulder','head'):
                np.testing.assert_array_equal(old.body.points[old.idx[key]],self.frames[f][key])
        self.assertEqual(self.report['gravity_values'],[REAL_GRAVITY])

    def test_four_contacts_com_clearance_and_stable_hold(self):
        r=self.report
        self.assertEqual(r['state'],'supported',r)
        self.assertTrue(r['tail_all_supported'])
        self.assertEqual(r['min_supported_contacts'],4)
        self.assertGreater(r['com_margin_px'],0.)
        self.assertGreater(r['hip_height_px'],45.)
        self.assertGreater(r['chest_height_px'],40.)
        self.assertLess(r['tail_max_speed_px_frame'],.01)
        self.assertEqual(r['max_penetration_px'],0.)
        self.assertTrue(r['finite'])
        self.assertLessEqual(r['torque_limit_ratio'],1.)
        self.assertLessEqual(r['force_limit_ratio'],1.)
        self.assertEqual(self.sim.body.pinned,{self.sim.idx['anchor']})
        self.assertEqual(self.sim.body.sticks[0][3],1.)
        self.assertTrue(self.sim.collapsed) # no premature walk/standing transition

    def test_motion_interrupts_the_quiet_counter(self):
        sim=copy.deepcopy(self.sim)
        sim.recovery=GroundRecovery();r=sim.recovery
        r.last_positions=sim.body.points[[i for i in range(len(sim.body.points)) if i not in sim.body.pinned]].copy()
        r.calm_frames=29
        sim.body.points[sim.idx['head'],0]+=1.
        r.observe(sim,GROUND_Y)
        self.assertEqual(r.calm_frames,0)
        self.assertEqual(r.state,'waiting')

    def test_unsupported_leg_orientation_does_not_drive_or_claim_success(self):
        active=ActiveBipedSim(ground_recovery=True,big_push_kick_px=-150.)
        passive=ActiveBipedSim(big_push_kick_px=-150.,bracing="impulse")
        for _ in range(450):
            active.step();passive.step()
            np.testing.assert_array_equal(active.body.points,passive.body.points)
            np.testing.assert_array_equal(active.body.prev_points,passive.body.prev_points)
        self.assertEqual(active.recovery.state,'needs_roll')
        self.assertIsNone(active.recovery.start_frame)
        self.assertIsNone(active.recovery.support_frame)
        self.assertEqual(active.recovery.max_torque_ratio,0.)

    def test_missing_contact_revokes_support_and_attempt_is_bounded(self):
        sim=copy.deepcopy(self.sim);r=sim.recovery
        r.start_frame=sim.frame-ATTEMPT_TIMEOUT
        sim.ground_projection_impulses=[]
        r.observe(sim,GROUND_Y)
        self.assertEqual(r.state,'failed')
        self.assertEqual(r.failure_reason,'support_timeout')
        before=sim.body.prev_points.copy()
        r.drive(sim)
        np.testing.assert_array_equal(sim.body.prev_points,before)


if __name__=='__main__':unittest.main()
