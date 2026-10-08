"""A planted foot must actually take support; a plausible pose is insufficient."""
import copy
import unittest
import numpy as np
from demo.step14_active_biped import ActiveBipedSim, GROUND_Y
from demo.step36_foot_transfer import run_case
from physics.ground_recovery import ATTEMPT_TIMEOUT
from physics.gravity import REAL_GRAVITY


class FootTransferTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.forward = run_case(150., count=1250)
        cls.backward = run_case(-150., count=1700)

    def test_both_fall_directions_reach_and_hold_loaded_foot_support(self):
        for report, frames, sim in (self.forward, self.backward):
            with self.subTest(push=report['push_px']):
                self.assertEqual(report['state'], 'half_kneeling', report)
                self.assertTrue(report['tail_all_half_kneeling'])
                self.assertEqual(report['min_transfer_contacts'], 4)
                self.assertGreater(report['front_knee_clearance_px'], 25)
                self.assertGreater(report['foot_share_final'], report['foot_share_placed']+.08)
                self.assertGreaterEqual(report['min_tail_foot_share'], .25)
                self.assertLess(report['max_tail_com_to_foot_px'], 20)
                self.assertLess(report['tail_max_speed_px_frame'], .01)
                self.assertLess(report['tail_foot_travel_px'], .05)
                self.assertEqual(report['max_penetration_px'], 0)
                self.assertTrue(report['finite'])
                self.assertEqual(report['gravity_values'], [REAL_GRAVITY])
                self.assertLessEqual(report['torque_limit_ratio'], 1)
                self.assertLessEqual(report['force_limit_ratio'], 1)
                self.assertEqual(sim.body.pinned, {sim.idx['anchor']})
                self.assertTrue(sim.collapsed)
                self.assertEqual([e[1] for e in report['events']][-3:],
                                 ['foot_placing', 'transferring', 'half_kneeling'])

    def test_previous_physics_is_identical_until_support_wait_finishes(self):
        report, frames, _ = self.forward
        self.assertGreaterEqual(report['placement_frame']-report['support_frame'], 60)
        old = ActiveBipedSim(ground_recovery=True, recovery_reposition=True)
        for f in range(report['placement_frame']+1):
            old.step()
            for key in ('hip', 'shoulder', 'head'):
                np.testing.assert_array_equal(old.body.points[old.idx[key]], frames[f][key])
            for side, leg in (('left', old.left_leg), ('right', old.right_leg)):
                np.testing.assert_array_equal(leg.chain.points, frames[f]['legs'][side]['chain'])

    def test_transfer_actuators_do_not_teleport_or_add_net_linear_impulse(self):
        sim = copy.deepcopy(self.forward[2])
        p, q = sim.body.points.copy(), sim.body.prev_points.copy()
        sim.recovery.drive(sim)
        np.testing.assert_array_equal(sim.body.points, p)
        np.testing.assert_allclose(np.sum(sim.body.masses[:, None]*(q-sim.body.prev_points), axis=0),
                                   0, atol=1e-10)

    def test_loss_of_front_foot_contact_revokes_success_and_times_out(self):
        sim = copy.deepcopy(self.forward[2]);r = sim.recovery
        sim.frame -= 1  # inspect the last solved contact frame
        foot = sim.fallen_legs[r.front_side][1]
        sim.ground_projection_impulses = [v for v in sim.ground_projection_impulses if v[1] != foot]
        r.observe(sim, GROUND_Y)
        self.assertFalse(r.foot_contact)
        self.assertEqual(r.state, 'transferring')
        self.assertIn((sim.frame, 'transfer_support_lost'), r.events)
        sim.frame += ATTEMPT_TIMEOUT
        r.observe(sim, GROUND_Y)
        self.assertEqual(r.state, 'failed')
        self.assertEqual(r.failure_reason, 'transferring_timeout')
        q = sim.body.prev_points.copy();r.drive(sim)
        np.testing.assert_array_equal(sim.body.prev_points, q)

    def test_contact_without_increased_load_does_not_complete_transfer(self):
        sim = copy.deepcopy(self.forward[2]);r = sim.recovery
        sim.frame -= 1
        r.state = 'transferring';r.phase_frame = sim.frame-200
        r.placed_share = r.foot_share
        r.transfer_quiet_frames = 29
        r.observe(sim, GROUND_Y)
        self.assertEqual(r.transfer_contacts, 4)
        self.assertEqual(r.state, 'transferring')
        self.assertEqual(r.transfer_quiet_frames, 0)

    def test_transfer_requires_previous_stages(self):
        for options in ({}, {'ground_recovery': True}, {'recovery_reposition': True}):
            with self.assertRaises(ValueError):
                ActiveBipedSim(recovery_transfer=True, **options)


if __name__ == '__main__':
    unittest.main()
