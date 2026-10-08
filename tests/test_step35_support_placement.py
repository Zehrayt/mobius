"""Repositioning uses bounded motors and retains real four-point support gates."""
import copy
import unittest
import numpy as np
from demo.step14_active_biped import ActiveBipedSim,GROUND_Y
from demo.step35_support_placement import run_case
from physics.ground_recovery import ATTEMPT_TIMEOUT
from physics.gravity import REAL_GRAVITY


class SupportPlacementTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.back=run_case(-150.,count=1250)
        cls.front=run_case(70.60148148148147,10,count=650)

    def test_backward_fall_places_both_legs_then_holds(self):
        report,frames,sim=self.back
        self.assertEqual(report['reposition_count'],2)
        self.assertEqual([e[1] for e in report['events']],['repositioning','repositioning','rising','supported'])
        self.assertEqual(report['state'],'supported')
        self.assertTrue(report['tail_all_supported'])
        self.assertEqual(report['min_supported_contacts'],4)
        self.assertGreater(report['com_margin_px'],0)
        self.assertLess(report['tail_max_speed_px_frame'],.01)
        self.assertEqual(report['gravity_values'],[REAL_GRAVITY])
        self.assertEqual(report['max_penetration_px'],0)
        self.assertLessEqual(report['torque_limit_ratio'],1)
        self.assertLessEqual(report['force_limit_ratio'],1)
        self.assertEqual(sim.body.pinned,{sim.idx['anchor']})
        self.assertTrue(sim.collapsed)
        for frame in frames[report['collapse_frame']+1:]:
            for leg in frame['legs'].values():
                hip,knee,foot=leg['chain']
                a,b=np.array(knee)-hip,np.array(foot)-knee
                angle=np.degrees(np.arctan2(a[0]*b[1]-a[1]*b[0],np.dot(a,b)))
                # PBD retains a small angular residual at initial ground impact.
                # 0.05 degrees rejects branch reversal without demanding exact convergence.
                self.assertGreaterEqual(angle,-.05)

    def test_previously_hovering_knee_reaches_real_contact(self):
        report,_,_=self.front
        self.assertEqual(report['reposition_count'],0)
        self.assertEqual(report['state'],'supported')
        self.assertEqual(report['min_supported_contacts'],4)
        self.assertTrue(report['tail_all_supported'])

    def test_reposition_needs_rearward_placement_and_times_out(self):
        sim=copy.deepcopy(self.back[2]);r=sim.recovery
        knee,foot=next(iter(sim.fallen_legs.values()))
        r.state='repositioning';r.moving_leg=knee;r.pending_legs=[]
        r.phase_frame=sim.frame-300
        sim.body.points[foot,0]=sim.body.points[knee,0]+20
        ids=[i for i in range(len(sim.body.points)) if i not in sim.body.pinned]
        r.last_positions=sim.body.points[ids].copy()
        r.observe(sim,GROUND_Y)
        self.assertEqual(r.state,'repositioning')
        r.phase_frame=sim.frame-ATTEMPT_TIMEOUT
        r.observe(sim,GROUND_Y)
        self.assertEqual(r.state,'failed')
        self.assertEqual(r.failure_reason,'leg_placement_timeout')
        q=sim.body.prev_points.copy();r.drive(sim)
        np.testing.assert_array_equal(q,sim.body.prev_points)

    def test_reposition_requires_recovery(self):
        with self.assertRaises(ValueError):ActiveBipedSim(recovery_reposition=True)


if __name__=='__main__':unittest.main()
