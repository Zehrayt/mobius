"""Walking must physically clear, land and alternate without changing topology."""
import copy
import unittest
import numpy as np
from demo.step14_active_biped import ActiveBipedSim, GROUND_Y
from demo.step39_walk_restart import run_case
from physics.walk_recovery import FORCE_MIN, FORCE_MAX, LAND_FRAMES
from physics.gravity import REAL_GRAVITY


class WalkingRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.forward = run_case(150.)
        cls.backward = run_case(-150.)

    def test_both_fall_directions_restart_alternating_supported_steps(self):
        for report, frames, sim in (self.forward, self.backward):
            with self.subTest(push=report['push_px']):
                self.assertTrue(report['passed'], report)
                self.assertGreaterEqual(report['completed_steps'], 3)
                self.assertEqual(report['min_foot_contacts'], 1)
                self.assertEqual(report['max_other_contacts'], 0)
                self.assertLess(report['max_vertical_hip_delta_px'], 2.)
                self.assertEqual(report['gravity_values'], [REAL_GRAVITY])
                self.assertEqual(sim.body.pinned, {sim.idx['anchor']})
                self.assertEqual(sim.body.sticks[0][3], 1.)
                self.assertEqual(len(sim.body.points), 13)
                self.assertTrue(sim.collapsed)
                self.assertLessEqual(report['force_limit_ratio'], 1.)
                self.assertLessEqual(report['torque_limit_ratio'], 1.)

    def test_standing_history_unchanged_until_walk_entry(self):
        report, frames, _ = self.forward
        old = ActiveBipedSim(ground_recovery=True, recovery_reposition=True,
            recovery_transfer=True, recovery_rise=True, recovery_stand=True)
        for f in range(report['walk_start_frame']+1):
            old.step()
            for key in ('hip','shoulder','head'):
                np.testing.assert_array_equal(old.body.points[old.idx[key]], frames[f][key])
            for side, leg in (('left', old.left_leg), ('right', old.right_leg)):
                np.testing.assert_array_equal(leg.chain.points, frames[f]['legs'][side]['chain'])

    def test_force_pair_is_bounded_and_preserves_linear_momentum_and_positions(self):
        sim = copy.deepcopy(self.forward[2]); r = sim.recovery
        p = sim.body.points.copy(); q = sim.body.prev_points.copy()
        r._pair_force(sim, 'l', np.array([1e6,-1e6]), 1.)
        np.testing.assert_array_equal(p, sim.body.points)
        impulses = sim.body.masses[:,None]*(q-sim.body.prev_points)
        np.testing.assert_allclose(impulses.sum(axis=0), 0., atol=1e-11)
        np.testing.assert_allclose(impulses[sim.idx['hip']], [FORCE_MAX[0],FORCE_MIN[1]], atol=1e-11)
        self.assertEqual(r.max_walk_force_ratio, 1.)

    def test_missing_landing_cannot_count_a_step_and_times_out(self):
        sim = copy.deepcopy(self.forward[2]); r = sim.recovery
        r.state = 'walk_land'; r.phase_frame = sim.frame-LAND_FRAMES-240
        before = len(r.completed_steps)
        stance = sim.fallen_legs[r.stance_side][1]
        sim.ground_projection_impulses = [(sim.frame, stance, 10.)]
        r.observe(sim, GROUND_Y)
        self.assertEqual(len(r.completed_steps), before)
        self.assertEqual(r.state, 'failed')
        self.assertEqual(r.failure_reason, 'walking_landing_timeout')
        q = sim.body.prev_points.copy(); r.drive(sim)
        np.testing.assert_array_equal(q, sim.body.prev_points)

    def test_new_nonfoot_support_revokes_walk(self):
        sim = copy.deepcopy(self.forward[2]); r = sim.recovery
        r.state = 'walk_swing'; r.phase_frame = sim.frame
        sim.ground_projection_impulses = [(sim.frame, sim.idx['hip'], 1.)]
        r.observe(sim, GROUND_Y)
        self.assertEqual(r.state, 'failed')
        self.assertEqual(r.failure_reason, 'walking_support_lost')

    def test_walk_requires_standing_stage(self):
        with self.assertRaisesRegex(ValueError, 'recovery_stand'):
            ActiveBipedSim(recovery_walk=True)


if __name__ == '__main__':
    unittest.main()
