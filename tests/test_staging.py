"""Visible-alpha sizing and furniture contact/occlusion regressions."""
import tempfile
import unittest
import warnings
import numpy as np
from scene import Actor, Camera, composite_frame
from scene.staging import stage_sprite, place_at_contact, foreground_cutout


class StagingTests(unittest.TestCase):
    def test_transparent_canvas_margins_do_not_change_visible_size(self):
        figure = np.full((60, 20, 4), (50, 100, 150, 255), np.uint8)
        tight = np.pad(figure, ((2, 3), (3, 5), (0, 0)))
        loose = np.pad(figure, ((101, 7), (81, 120), (0, 0)))
        a = stage_sprite(tight, height=120)
        b = stage_sprite(loose, height=120)
        np.testing.assert_array_equal(a.image, b.image)
        self.assertEqual(a.image.shape, (120, 40, 4))
        self.assertEqual(a.source_scale, b.source_scale)
        with self.assertRaises(ValueError):
            stage_sprite(np.zeros((10, 10, 4), np.uint8), height=120)

    def test_contact_survives_actor_rotation_scale_and_camera_zoom(self):
        image = np.full((60, 40, 4), 255, np.uint8)
        sprite = stage_sprite(image, height=120)
        actor = Actor('seated', rotation=8, scale=1.2)
        surface = (610, 330)
        local = place_at_contact(actor, sprite, (.4, .65), surface)
        actual = actor.position + actor.rotate_vector(local*actor.scale, actor.rotation)
        np.testing.assert_allclose(actual, surface)
        camera = Camera(640, 360, 1.04)
        np.testing.assert_allclose(camera.world_to_screen(actual, (1280, 720)),
                                   camera.world_to_screen(surface, (1280, 720)))

    def test_foreground_mask_restores_furniture_but_preserves_holes(self):
        background = np.full((40, 40, 4), (20, 60, 100, 255), np.uint8)
        furniture = foreground_cutout(background, [[[0, 0], [39, 0], [39, 39], [0, 39]]],
                                     holes=[[[12, 12], [28, 12], [28, 28], [12, 28]]], feather=0)
        figure = np.full((40, 40, 4), (0, 255, 0, 255), np.uint8)
        frame = np.zeros((40, 40, 3), np.uint8)
        actors = [Actor('figure', position=(20, 20), sprite_name='figure', z_index=10),
                  Actor('furniture', position=(20, 20), sprite_name='furniture', z_index=15)]
        composite_frame(frame, actors, {'figure': figure, 'furniture': furniture})
        np.testing.assert_array_equal(frame[4, 4], background[4, 4, :3])
        np.testing.assert_array_equal(frame[20, 20], figure[20, 20, :3])

    def test_lighting_preserves_source_and_alpha(self):
        image = np.full((60, 40, 4), (90, 100, 110, 230), np.uint8)
        before = image.copy()
        sprite = stage_sprite(image, height=60, bgr_gain=(.95, 1, 1.02), side_light=.02)
        np.testing.assert_array_equal(image, before)
        np.testing.assert_array_equal(sprite.image[:, :, 3], image[:, :, 3])
        self.assertLess(sprite.image[30, 20, 0], image[30, 20, 0])

    def test_seated_contacts_fixed_and_child_shadows_follow_during_entry(self):
        from demo.step6_scenario_timeline import HospitalityScene
        with tempfile.TemporaryDirectory() as tmp, warnings.catch_warnings():
            warnings.simplefilter('ignore')
            scene = HospitalityScene(tmp)
            # A static preview must not start playback or hide the children.
            scene.render_static()
            self.assertIsNone(scene.timeline.current_time)
            self.assertTrue(scene.bilge.visible)
            for time in (0, .5, 1, 2, 3.9):
                scene.timeline.update(time)
                for guest in scene.guests:
                    contact = guest.position + guest.anchors['contact']
                    np.testing.assert_allclose(contact, scene.layout['characters'][guest.name]['contact_world'])
                shadow, _ = scene.bilge.attached_props['bilge_foot_0_shadow']
                world, _, _, _, visible = shadow.world_transform()
                np.testing.assert_allclose(world, scene.bilge.position+scene.bilge.anchors['foot_0'])
                self.assertEqual(visible, scene.bilge.visible)
            self.assertGreater(scene.layout['characters']['bilge']['visible_height'],
                               scene.layout['characters']['bilge_brother']['visible_height'])


if __name__ == '__main__':
    unittest.main()
