"""Upright kneeling needs real leg support and completely unloaded arms."""
import copy
import unittest
import numpy as np
from demo.step14_active_biped import ActiveBipedSim,GROUND_Y
from demo.step37_kneel_rise import run_case
from physics.ground_recovery import ATTEMPT_TIMEOUT
from physics.gravity import REAL_GRAVITY


class KneelRiseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.forward=run_case(150.,count=1550)
        cls.backward=run_case(-150.,count=2050)

    def test_both_fall_directions_hold_upright_without_arm_support(self):
        for report,_,sim in (self.forward,self.backward):
            with self.subTest(push=report['push_px']):
                self.assertEqual(report['state'],'upright_kneeling',report)
                self.assertTrue(report['tail_all_upright'])
                self.assertEqual(report['min_lower_contacts'],2)
                self.assertEqual(report['max_arm_contacts'],0)
                self.assertGreater(report['min_hand_clearance_px'],20.)
                self.assertGreaterEqual(report['min_lower_margin_px'],5.)
                self.assertLess(report['max_abs_torso_tilt_deg'],20.)
                self.assertGreater(report['hip_height_px'],report['hip_height_before_raise']+5.)
                self.assertGreater(report['chest_height_px'],120.)
                self.assertLess(report['tail_max_speed_px_frame'],.02)
                self.assertLess(report['tail_foot_travel_px'],.1)
                self.assertLess(report['tail_knee_travel_px'],.1)
                self.assertEqual(report['max_penetration_px'],0.)
                self.assertEqual(report['gravity_values'],[REAL_GRAVITY])
                self.assertLessEqual(report['torque_limit_ratio'],1.)
                self.assertLessEqual(report['force_limit_ratio'],1.)
                self.assertEqual(sim.body.pinned,{sim.idx['anchor']})
                self.assertTrue(sim.collapsed)

    def test_previous_stage_is_unchanged_until_raise_starts(self):
        report,frames,_=self.forward
        old=ActiveBipedSim(ground_recovery=True,recovery_reposition=True,recovery_transfer=True)
        for f in range(report['raise_frame']+1):
            old.step()
            for name in ('hip','shoulder','head'):
                np.testing.assert_array_equal(old.body.points[old.idx[name]],frames[f][name])
            for side,leg in (('left',old.left_leg),('right',old.right_leg)):
                np.testing.assert_array_equal(leg.chain.points,frames[f]['legs'][side]['chain'])

    def test_pair_motors_preserve_positions_and_net_linear_impulse(self):
        sim=copy.deepcopy(self.forward[2]);p=sim.body.points.copy();q=sim.body.prev_points.copy()
        sim.recovery.drive(sim)
        np.testing.assert_array_equal(sim.body.points,p)
        np.testing.assert_allclose(np.sum(sim.body.masses[:,None]*(q-sim.body.prev_points),axis=0),0.,atol=1e-10)

    def test_losing_leg_contact_or_using_an_arm_revokes_success(self):
        for kind in ('missing_foot','arm_support'):
            with self.subTest(kind=kind):
                sim=copy.deepcopy(self.forward[2]);r=sim.recovery;sim.frame-=1
                if kind=='missing_foot':
                    foot=sim.fallen_legs[r.front_side][1]
                    sim.ground_projection_impulses=[v for v in sim.ground_projection_impulses if v[1]!=foot]
                else:
                    hand=sim.arms.idx['l'][1]
                    sim.ground_projection_impulses.append((sim.frame,hand,1.))
                r.observe(sim,GROUND_Y)
                self.assertEqual(r.state,'torso_raising')
                self.assertIn((sim.frame,'upright_support_lost'),r.events)

    def test_brief_motion_noise_is_tolerated_but_sustained_motion_revokes_support(self):
        sim=copy.deepcopy(self.forward[2]);r=sim.recovery;sim.frame-=1
        head=sim.idx['head']
        sim.body.points[head,0]+=.03
        r.observe(sim,GROUND_Y)
        self.assertEqual(r.state,'upright_kneeling')
        r.observe(sim,GROUND_Y)  # motion has stopped; reset the persistence counter
        self.assertEqual(r.motion_loss_frames,0)
        for _ in range(5):
            sim.body.points[head,0]+=.03
            r.observe(sim,GROUND_Y)
        self.assertEqual(r.state,'torso_raising')

    def test_timeout_stops_motors_without_claiming_upright_support(self):
        sim=copy.deepcopy(self.forward[2]);r=sim.recovery
        r.state='torso_raising';r.phase_frame=sim.frame-ATTEMPT_TIMEOUT
        sim.ground_projection_impulses=[]
        r.observe(sim,GROUND_Y)
        self.assertEqual(r.state,'failed')
        self.assertEqual(r.failure_reason,'torso_raise_timeout')
        q=sim.body.prev_points.copy();r.drive(sim)
        np.testing.assert_array_equal(q,sim.body.prev_points)

    def test_requires_all_previous_stages(self):
        for options in ({},{'ground_recovery':True,'recovery_reposition':True},
                        {'ground_recovery':True,'recovery_transfer':True}):
            with self.assertRaises(ValueError):ActiveBipedSim(recovery_rise=True,**options)


if __name__=='__main__':unittest.main()
