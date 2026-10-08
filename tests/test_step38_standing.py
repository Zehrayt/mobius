"""Full standing must be supported by two loaded feet, not knees or old pins."""
import copy
import unittest
import numpy as np
from demo.step14_active_biped import ActiveBipedSim,GROUND_Y
from demo.step38_standing import run_case
from physics.ground_recovery import ATTEMPT_TIMEOUT
from physics.gravity import REAL_GRAVITY
from physics.stand_recovery import LEG_FORCE_CAP


class StandingRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.forward=run_case(150.,count=2050)
        cls.backward=run_case(-150.,count=2550)

    def test_two_fall_directions_reach_full_standing_and_hold(self):
        for report,_,sim in (self.forward,self.backward):
            with self.subTest(push=report['push_px']):
                self.assertEqual(report['state'],'standing',report)
                self.assertTrue(report['tail_all_standing'])
                self.assertEqual(report['min_foot_contacts'],2)
                self.assertEqual(report['max_other_contacts'],0)
                self.assertGreaterEqual(report['min_standing_margin_px'],8.)
                self.assertGreaterEqual(report['min_foot_share'],.15)
                self.assertGreater(report['min_knee_clearance_px'],60.)
                self.assertGreater(report['hip_height_px'],170.)
                self.assertGreater(report['chest_height_px'],210.)
                self.assertLess(report['max_abs_torso_tilt_deg'],15.)
                self.assertLess(report['tail_max_speed_px_frame'],.02)
                self.assertLess(max(report['tail_foot_travel_px'].values()),.1)
                self.assertEqual(report['max_penetration_px'],0.)
                self.assertEqual(report['gravity_values'],[REAL_GRAVITY])
                self.assertLessEqual(report['max_stand_force_ratio'],1.)
                self.assertLessEqual(report['force_limit_ratio'],1.)
                self.assertLessEqual(report['torque_limit_ratio'],1.)
                self.assertEqual(sim.body.pinned,{sim.idx['anchor']})
                self.assertEqual(sim.body.sticks[0][3],1.)
                self.assertTrue(sim.collapsed)  # free-body recovery; no hidden walking-anchor reset

    def test_previous_stages_are_identical_until_standing_starts(self):
        report,frames,_=self.forward
        old=ActiveBipedSim(ground_recovery=True,recovery_reposition=True,recovery_transfer=True,recovery_rise=True)
        for f in range(report['stand_start_frame']+1):
            old.step()
            for key in ('hip','shoulder','head'):
                np.testing.assert_array_equal(old.body.points[old.idx[key]],frames[f][key])
            for side,leg in (('left',old.left_leg),('right',old.right_leg)):
                np.testing.assert_array_equal(leg.chain.points,frames[f]['legs'][side]['chain'])

    def test_axial_support_is_capped_and_conserves_pair_momentum(self):
        sim=copy.deepcopy(self.forward[2]);r=sim.recovery
        hip=sim.idx['hip'];foot=sim.fallen_legs['l'][1]
        p=sim.body.points.copy();axis=p[hip]-p[foot];axis/=np.linalg.norm(axis)
        sim.body.prev_points[hip]=p[hip]+axis*1000.
        q=sim.body.prev_points.copy();r.stand_force_peak=0.
        r._support_leg(sim,'l',1.)
        np.testing.assert_array_equal(sim.body.points,p)
        impulses=sim.body.masses[:,None]*(q-sim.body.prev_points)
        np.testing.assert_allclose(impulses.sum(axis=0),0.,atol=1e-10)
        self.assertAlmostEqual(float(np.sum(p[:,0]*impulses[:,1]-p[:,1]*impulses[:,0])),0.,places=8)
        self.assertAlmostEqual(r.stand_force_peak,LEG_FORCE_CAP)

    def test_missing_foot_or_knee_support_revokes_standing(self):
        for kind in ('foot_missing','knee_support'):
            with self.subTest(kind=kind):
                sim=copy.deepcopy(self.forward[2]);r=sim.recovery;sim.frame-=1
                knee,foot=sim.fallen_legs['r']
                if kind=='foot_missing':
                    sim.ground_projection_impulses=[v for v in sim.ground_projection_impulses if v[1]!=foot]
                else:
                    sim.ground_projection_impulses.append((sim.frame,knee,1.))
                r.observe(sim,GROUND_Y)
                self.assertEqual(r.state,'standing_rising')
                self.assertIn((sim.frame,'standing_support_lost'),r.events)
                self.assertTrue(r.continuing_stand)

    def test_brief_noise_is_tolerated_but_sustained_motion_revokes_standing(self):
        sim=copy.deepcopy(self.forward[2]);r=sim.recovery;sim.frame-=1
        head=sim.idx['head'];sim.body.points[head,0]+=.03
        r.observe(sim,GROUND_Y)
        self.assertEqual(r.state,'standing')
        r.observe(sim,GROUND_Y)
        self.assertEqual(r.stand_motion_frames,0)
        for _ in range(5):
            sim.body.points[head,0]+=.03;r.observe(sim,GROUND_Y)
        self.assertEqual(r.state,'standing_rising')

    def test_timeout_stops_the_standing_motors(self):
        sim=copy.deepcopy(self.forward[2]);r=sim.recovery
        r.state='standing_rising';r.phase_frame=sim.frame-ATTEMPT_TIMEOUT
        sim.ground_projection_impulses=[]
        r.observe(sim,GROUND_Y)
        self.assertEqual(r.state,'failed')
        self.assertEqual(r.failure_reason,'standing_timeout')
        q=sim.body.prev_points.copy();r.drive(sim)
        np.testing.assert_array_equal(q,sim.body.prev_points)

    def test_standing_requires_all_previous_stages(self):
        for options in ({},{'ground_recovery':True},
                {'ground_recovery':True,'recovery_reposition':True,'recovery_transfer':True}):
            with self.assertRaises(ValueError):ActiveBipedSim(recovery_stand=True,**options)


if __name__=='__main__':unittest.main()
