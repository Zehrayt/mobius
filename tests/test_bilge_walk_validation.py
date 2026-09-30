"""Whole-sequence constraints on solved joints, independent of rendering."""
import copy
import unittest

import numpy as np

from demo.bilge_walk_validation import WalkConfig, measure, pelvis_at, simulate, validate


class BilgeWalkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = WalkConfig()
        cls.frames = simulate(cls.config)
        cls.report = measure(cls.frames, cls.config)

    def test_planted_feet_lengths_and_ground_throughout_sequence(self):
        validate(self.report)
        self.assertEqual(len(self.frames), 240)
        for side in ('left', 'right'):
            origin = None
            for frame in self.frames:
                p = frame['points']
                foot = p[f'{side}_foot']
                self.assertLessEqual(foot[1], self.config.ground_y + 1e-6)
                if frame['states'][side] == 'stance':
                    if origin is None:
                        origin = foot.copy()
                    np.testing.assert_allclose(foot, origin, atol=1e-6, rtol=0)
                else:
                    origin = None

    def test_alternating_lift_and_joint_motion_not_rigid_translation(self):
        launches = []
        for i, f in enumerate(self.frames[1:], start=1):
            self.assertIn('stance', f['states'].values())
            for side in ('left', 'right'):
                if f['states'][side] == 'swing' and self.frames[i-1]['states'][side] == 'stance':
                    launches.append(side)
        self.assertGreater(len(launches), 6)
        self.assertTrue(all(a != b for a, b in zip(launches, launches[1:])))
        walking = [f for f in self.frames if 2 < f['time'] < 5]
        for side in ('left', 'right'):
            relative = [f['points'][f'{side}_foot'] - f['points']['pelvis'] for f in walking]
            # This is travel RELATIVE to the moving pelvis, not stride length
            # in world space; a rigidly translated skeleton would yield zero.
            self.assertGreater(np.ptp(np.array(relative)[:, 0]),
                               .2 * (self.config.thigh + self.config.shin))
            self.assertGreater(self.report['max_foot_lift_px'][side], 20)
        self.assertGreater(self.report['knee_bend_deg'][0], 0)
        self.assertLess(self.report['knee_bend_deg'][1], 140)

    def test_arms_follow_opposite_legs(self):
        walking = [f['points'] for f in self.frames if 2 <= f['time'] < 5]
        for side, other in (('left', 'right'), ('right', 'left')):
            hand = [p[f'{side}_hand'][0]-p[f'{side}_shoulder'][0] for p in walking]
            opposite_leg = [p[f'{other}_foot'][0]-p[f'{side}_foot'][0] for p in walking]
            self.assertGreater(np.ptp(hand), 30)
            self.assertGreater(np.corrcoef(hand, opposite_leg)[0, 1], .85)

    def test_idle_and_smooth_start_stop(self):
        for start, end, tolerance in ((0, 1, 1e-8), (7.5, 8, .002)):
            frames = [f for f in self.frames if start <= f['time'] < end]
            joints = np.array([list(f['points'].values()) for f in frames])
            self.assertLess(np.max(np.ptp(joints, axis=0)), tolerance)
            for f in frames:
                self.assertTrue(all(s == 'stance' for s in f['states'].values()))
                # Standing pelvis is between the two support contacts.
                feet = sorted(f['points'][f'{s}_foot'][0] for s in ('left', 'right'))
                self.assertLessEqual(feet[0], f['points']['pelvis'][0])
                self.assertGreaterEqual(feet[1], f['points']['pelvis'][0])
        # The root's velocity approaches zero on BOTH sides of the boundaries.
        dt = 1e-4
        for t in (1.0, 6.0):
            center, _ = pelvis_at(t, self.config)
            for adjacent in (t-dt, t+dt):
                pos, _ = pelvis_at(adjacent, self.config)
                self.assertLess(np.linalg.norm(pos-center)/dt, .01)
        self.assertLess(self.report['max_joint_displacement_per_frame_px'], 18)

    def test_metrics_detect_corrupt_solved_foot_not_just_target(self):
        damaged = copy.deepcopy(self.frames)
        damaged[10]['points']['left_foot'] += [3, 2]
        report = measure(damaged, self.config)
        self.assertGreaterEqual(report['max_stance_horizontal_slip_px'], 2.99)
        self.assertGreaterEqual(report['max_ground_penetration_px'], 1.99)
        with self.assertRaises(RuntimeError):
            validate(report)


if __name__ == '__main__':
    unittest.main()
