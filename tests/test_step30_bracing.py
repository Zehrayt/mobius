"""Protective arms: isolation, internal momentum, impact and settled state."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14
from demo.step30_bracing import compare
from physics.arms import PhysicalArms
from physics.bracing import FallBracing, constrain_fallen_elbows
from physics.verlet import VerletSystem


class BracingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.frames = compare()

    def test_default_impact_reduction_and_hand_first(self):
        old, new = self.report['passive'], self.report['bracing']
        self.assertEqual(old['collapse_frame'], new['collapse_frame'])
        self.assertEqual(new['brace_start_frame'], new['collapse_frame'])
        for part, ratio in (('head', .8), ('chest', .6)):
            self.assertIsNotNone(new[part]['first_frame'])
            self.assertLess(new['first_hand_contact_frame'], new[part]['first_frame'])
            self.assertLess(new[part]['peak_input_speed_px_frame'],
                            ratio * old[part]['peak_input_speed_px_frame'])
        self.assertGreater(new['shoulder_actuator_upward_impulse'], 0)

    def test_before_collapse_is_identical(self):
        for a, b in zip(self.frames['passive'], self.frames['bracing']):
            if a['collapsed']:
                break
            for name in ('hip', 'shoulder', 'head'):
                np.testing.assert_array_equal(a[name], b[name])
            for side in ('l', 'r'):
                np.testing.assert_array_equal(a['arms'][side], b['arms'][side])

    def test_unperturbed_walk_is_identical_for_thirty_seconds(self):
        a = s14.ActiveBipedSim(gravity_mode="legacy", bracing=False, stumble_kick_px=0, big_push_kick_px=0)
        b = s14.ActiveBipedSim(gravity_mode="legacy", bracing=True, stumble_kick_px=0, big_push_kick_px=0)
        for _ in range(900):
            a.step()
            b.step()
            np.testing.assert_array_equal(a.body.points, b.body.points)
        self.assertEqual(b.bracing.state, 'idle')

    def test_releases_and_settles_without_ground_penetration(self):
        new = self.report['bracing']
        self.assertTrue(new['finite'])
        self.assertEqual(new['brace_final_state'], 'released')
        self.assertLess(new['tail_max_speed_px_frame'], .2)
        self.assertLess(new['tail_hip_drift_px'], 1)
        self.assertEqual(new['max_ground_penetration_px'], 0)

    def test_controller_preserves_positions_and_linear_momentum(self):
        body = VerletSystem.empty()
        shoulder = body.add_point([0., 100.])
        head = body.add_point([0., 70.])
        arms = PhysicalArms(body, shoulder)
        body.prev_points -= [4., 6.]
        before_p = body.points.copy()
        before_momentum = np.sum(body.masses[:, None] * (body.points - body.prev_points), axis=0)
        brace = FallBracing()
        brace.drive(arms, head, 200., 2.2, 0)
        self.assertTrue(brace.active)
        np.testing.assert_array_equal(body.points, before_p)
        after_momentum = np.sum(body.masses[:, None] * (body.points - body.prev_points), axis=0)
        np.testing.assert_allclose(after_momentum, before_momentum, atol=1e-12)

    def test_elbow_projection_preserves_center_of_mass(self):
        body = VerletSystem.empty()
        shoulder = body.add_point([0., 100.])
        arms = PhysicalArms(body, shoulder)
        for e, h in arms.idx.values():
            body.points[h] = body.points[e] + [-32., 0.]
        before = np.sum(body.points * body.masses[:, None], axis=0)
        for _ in range(30):
            constrain_fallen_elbows(arms)
        after = np.sum(body.points * body.masses[:, None], axis=0)
        np.testing.assert_allclose(before, after, atol=1e-12)
        for e, h in arms.idx.values():
            a, b = body.points[e] - body.points[shoulder], body.points[h] - body.points[e]
            angle = np.degrees(np.arctan2(a[0]*b[1] - a[1]*b[0], a @ b))
            self.assertGreaterEqual(angle, -140.01)
            self.assertLessEqual(angle, -1.99)

    def test_no_arms_mode_can_still_collapse(self):
        sim = s14.ActiveBipedSim(gravity_mode="legacy", arms_mode='off')
        for _ in range(420):
            sim.step()
        self.assertFalse(sim.nan)
        self.assertIsNone(sim.arms)
        self.assertEqual(sim.bracing.state, 'idle')


if __name__ == '__main__':
    unittest.main()
