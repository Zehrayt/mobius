"""Skinning regressions: pose preservation, attachment integrity and soles."""
import copy
import unittest

import cv2
import numpy as np

from demo.bilge_walk_validation import WalkConfig, simulate
from demo.bilge_walk_skinned import BilgeSkin, ASSETS, inspect_geometry, validate_skin, render_skin
from scene.skinning import transform_point


class BilgeSkinningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = WalkConfig()
        cls.frames = simulate(cls.config)
        cls.skin = BilgeSkin()
        cls.report = inspect_geometry(cls.frames, cls.skin, cls.config)

    def test_original_face_pixels_are_preserved(self):
        spec = self.skin.settings['head']
        source = cv2.imread(str(ASSETS/spec['source']), cv2.IMREAD_UNCHANGED)
        x0, y0, x1, y1 = spec['source_crop']
        np.testing.assert_array_equal(self.skin.parts['head'].image[..., :3], source[y0:y1, x0:x1, :3])
        # The render may rotate/resize the head, but must never shear its face.
        for frame in self.frames:
            matrices, _ = self.skin.transforms(frame)
            scales = np.linalg.svd(matrices['head'][:, :2], compute_uv=False)
            self.assertAlmostEqual(scales[0], scales[1], places=12)

    def test_bindings_hit_original_joints_without_mutating_pose(self):
        original = copy.deepcopy(self.frames)
        for frame in self.frames:
            matrices, ankles = self.skin.transforms(frame)
            p = frame['points']
            for side in ('left', 'right'):
                mappings = ((f'{side}_thigh', p[f'{side}_hip'], p[f'{side}_knee']),
                            (f'{side}_shin', p[f'{side}_knee'], ankles[side]),
                            (f'{side}_upper_arm', p[f'{side}_shoulder'], p[f'{side}_elbow']),
                            (f'{side}_forearm', p[f'{side}_elbow'], p[f'{side}_hand']))
                for name, a, b in mappings:
                    np.testing.assert_allclose(transform_point(matrices[name], self.skin.anchor(name)), a, atol=1e-9)
                    np.testing.assert_allclose(transform_point(matrices[name], self.skin.anchor(name, 'end')), b, atol=1e-9)
                self.assertLess(ankles[side][1], p[f'{side}_foot'][1]-10)
        for before, after in zip(original, self.frames):
            for name in before['points']:
                np.testing.assert_array_equal(before['points'][name], after['points'][name])
            self.assertEqual(before['states'], after['states'])
            self.assertEqual(before['time'], after['time'])

    def test_all_connections_overlap_and_visible_soles_respect_ground(self):
        validate_skin(self.report)
        self.assertEqual(self.report['max_visible_sole_penetration_px'], 0)
        self.assertLessEqual(self.report['max_visible_stance_sole_gap_px'], 1)
        self.assertLess(self.report['max_stance_shoe_translation_px'], 1e-5)
        self.assertGreater(min(self.report['minimum_pair_overlap_pixels'].values()), 20)

    def test_support_shoe_transform_is_fixed_but_limbs_rotate(self):
        planted = {}
        angles = {side: [] for side in ('left', 'right')}
        for frame in self.frames:
            matrices, _ = self.skin.transforms(frame)
            for side in ('left', 'right'):
                shoe = matrices[f'{side}_shoe']
                if frame['states'][side] == 'stance':
                    if side not in planted:
                        planted[side] = shoe.copy()
                    np.testing.assert_allclose(shoe, planted[side], atol=1e-7, rtol=0)
                else:
                    planted.pop(side, None)
                thigh = frame['points'][f'{side}_knee'] - frame['points'][f'{side}_hip']
                angles[side].append(np.arctan2(thigh[0], thigh[1]))
        for values in angles.values():
            self.assertGreater(np.ptp(values), np.radians(20))

    def test_debug_overlay_is_opt_in_and_rendering_does_not_mutate_pose(self):
        pose = copy.deepcopy(self.frames[70])
        normal = render_skin(pose, self.skin, self.config)
        debug = render_skin(pose, self.skin, self.config, debug=True)
        self.assertEqual(normal.shape, (720, 1280, 3))
        self.assertGreater(np.count_nonzero(normal != debug), 1000)
        np.testing.assert_array_equal(normal, render_skin(pose, self.skin, self.config))
        for key in pose['points']:
            np.testing.assert_array_equal(pose['points'][key], self.frames[70]['points'][key])

    def test_contact_validator_rejects_a_separated_limb(self):
        report = copy.deepcopy(self.report)
        report['minimum_pair_overlap_pixels']['left_thigh/left_shin'] = 0
        with self.assertRaises(RuntimeError):
            validate_skin(report)


if __name__ == '__main__':
    unittest.main()
