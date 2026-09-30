"""Behavioral regressions for seconds timing, transforms, assets and export."""
import tempfile
import unittest
import warnings
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from scene import Actor, ActorMode, Camera, Timeline, ActionType as A, SpriteManager
from scene.compositing import apply_sprite_transform, alpha_blend_onto_frame, composite_frame
from scene.export import mux_audio, render_video


class TimelineTests(unittest.TestCase):
    def test_off_grid_events_once_and_skipped_tweens(self):
        actor = Actor('a', visible=False)
        calls = []
        tl = Timeline()
        tl.at(.017, A.SHOW_ACTOR, actor=actor)
        tl.at(.019, A.CALLBACK, callback=lambda: calls.append('event'))
        tl.tween(.025, .925, A.MOVE_ACTOR, actor=actor, target=(90, 45))
        tl.update(2)
        tl.update(2)
        self.assertTrue(actor.visible)
        self.assertEqual(calls, ['event'])
        np.testing.assert_allclose(actor.position, (90, 45))

    def test_chained_overlapping_tweens_fps_independent(self):
        values = []
        for fps in (24, 30, 60):
            actor = Actor('a')
            tl = Timeline()
            tl.tween(.017, 1.017, A.MOVE_ACTOR, actor=actor, target=(100, 0))
            tl.tween(.517, 1.517, A.MOVE_ACTOR, actor=actor, target=(150, 0))
            tl.tween(1.517, 2.517, A.MOVE_ACTOR, actor=actor, target=(250, 0))
            for f in range(fps*2+1):
                tl.update(f/fps)
            values.append(actor.position.copy())
        np.testing.assert_allclose(values, [[198.3, 0]]*3, atol=1e-10)

    def test_discrete_order_fades_camera_and_expression(self):
        actor = Actor('a', alpha=0, visible=False, expressions={'happy': 'happy.png'})
        prop = Actor('tray')
        camera = Camera()
        tl = Timeline(camera)
        tl.at(0, A.ATTACH_PROP, actor=actor, prop=prop, prop_anchor='hands', prop_offset=(5, 2))
        tl.tween(0, 1, A.FADE_IN, actor=actor)
        tl.tween(0, 1, A.MOVE_CAMERA, camera_target=(100, 50))
        tl.tween(0, 1, A.CAMERA_ZOOM, zoom_target=2)
        tl.tween(0, 1, A.SCALE_ACTOR, actor=actor, scale=2)
        tl.tween(0, 1, A.ROTATE_ACTOR, actor=actor, angle=20)
        tl.at(.5, A.SET_EXPRESSION, actor=actor, expression='happy')
        tl.at(1, A.DETACH_PROP, actor=actor, prop=prop)
        tl.tween(1, 2, A.FADE_OUT, actor=actor)
        tl.at(2, A.HIDE_ACTOR, actor=actor)
        tl.update(.5)
        self.assertTrue(actor.visible)
        self.assertEqual(actor.alpha, .5)
        self.assertEqual(actor.sprite_key, 'happy.png')
        np.testing.assert_allclose(camera.position, (50, 25))
        self.assertEqual(camera.zoom, 1.5)
        self.assertEqual(actor.rotation, 10)
        tl.update(2)
        self.assertFalse(actor.visible)
        self.assertEqual(actor.alpha, 0)
        self.assertIsNone(prop._parent)
        self.assertEqual(actor.scale, 2)

    def test_validation_and_backward_clock(self):
        tl = Timeline()
        for time in (-1, float('nan')):
            with self.assertRaises(ValueError):
                tl.at(time, A.SHOW_ACTOR, actor=Actor('x'))
        with self.assertRaises(ValueError):
            tl.tween(1, 0, A.MOVE_ACTOR, actor=Actor('x'), target=(1, 1))
        with self.assertRaises(ValueError):
            tl.at(0, A.CAMERA_ZOOM, zoom_target=0)
        tl.update(1)
        with self.assertRaises(ValueError):
            tl.update(.5)
        with self.assertRaises(RuntimeError):
            tl.at(2, A.CALLBACK, callback=lambda: None)

    def test_look_at_and_sprite_walk(self):
        actor = Actor('a')
        tl = Timeline()
        actor.walk_to(50, 2, timeline=tl)
        tl.at(1, A.LOOK_AT, actor=actor, look_target=(100, -10))
        tl.update(1)
        np.testing.assert_allclose(actor.position, (25, 0))
        self.assertGreater(actor.rotation, 0)
        with self.assertRaises(NotImplementedError):
            actor.reach((10, 10))

    def test_easing_midpoints_and_exact_endpoints(self):
        expected = {'linear': .25, 'ease_in_out': .125, 'ease_out': .4375}
        for easing, value in expected.items():
            actor = Actor('a')
            tl = Timeline()
            tl.tween(0, 1, A.SCALE_ACTOR, actor=actor, scale=2, easing=easing)
            tl.update(.25)
            self.assertAlmostEqual(actor.scale, 1+value)
            tl.update(1)
            self.assertEqual(actor.scale, 2)



class RenderingTests(unittest.TestCase):
    def test_attachment_transform_detach_and_cycles(self):
        parent = Actor('p', position=(10, 20), scale=2, rotation=90)
        child = Actor('c', position=(1, 0))
        parent.attach_prop(child, anchor='hands', offset=(4, 0))
        np.testing.assert_allclose(child.world_transform()[0], (10, 10), atol=1e-9)
        with self.assertRaises(ValueError):
            child.attach_prop(parent)
        parent.detach_prop('c')
        np.testing.assert_allclose(child.position, (10, 10), atol=1e-9)
        self.assertEqual(child.scale, 2)
        self.assertEqual(child.rotation, 90)

    def test_rotation_bounds_scale_and_transparent_padding(self):
        sprite = np.full((20, 40, 4), 255, np.uint8)
        out, pos = apply_sprite_transform(sprite, (100, 100), 2, 90, .5)
        self.assertGreaterEqual(out.shape[0], 80)
        self.assertGreaterEqual(out.shape[1], 40)
        self.assertAlmostEqual(pos[0]+out.shape[1]/2, 100, delta=.5)
        diagonal, _ = apply_sprite_transform(sprite, (0, 0), 1, 45, 1)
        self.assertEqual(int(diagonal[0, 0, 3]), 0)
        self.assertEqual(int(out[:, :, 3].max()), 128)

    def test_clipping_and_alpha(self):
        frame = np.zeros((10, 10, 3), np.uint8)
        sprite = np.full((4, 4, 4), 255, np.uint8)
        sprite[:, :, 3] = 128
        alpha_blend_onto_frame(frame, sprite, (-2, -2))
        self.assertTrue(np.all(frame[:2, :2] == 128))
        self.assertEqual(int(frame[3, 3].sum()), 0)
        before = frame.copy()
        for pos in ((20, 0), (-20, 0), (0, 20), (0, -20)):
            alpha_blend_onto_frame(frame, sprite, pos)
        np.testing.assert_array_equal(before, frame)

    def test_global_z_dedup_visibility_and_zoom(self):
        red = np.full((4, 4, 4), (0, 0, 255, 128), np.uint8)
        green = np.full((4, 4, 4), (0, 255, 0, 255), np.uint8)
        parent = Actor('p', position=(10, 10), sprite_name='green', z_index=1)
        child = Actor('c', sprite_name='red', z_index=3)
        parent.attach_prop(child)
        other = Actor('o', position=(10, 10), sprite_name='green', z_index=2)
        frame = np.zeros((20, 20, 3), np.uint8)
        composite_frame(frame, [child, other, parent], {'red': red, 'green': green}, Camera(10, 10, 2))
        np.testing.assert_allclose(frame[10, 10], (0, 127, 128), atol=1)
        self.assertGreater(np.count_nonzero(frame[:, :, 1]), 16)
        parent.visible = False
        frame[:] = 0
        composite_frame(frame, [parent, child], {'red': red, 'green': green})
        self.assertFalse(frame.any())

    def test_camera_roundtrip(self):
        camera = Camera(30, 40, 1.2)
        for depth in (.3, 1, 1.5):
            screen = camera.world_to_screen((150, 70), (1280, 720), depth)
            np.testing.assert_allclose(camera.screen_to_world(screen, (1280, 720), depth), (150, 70))

    def test_png_loading_missing_corrupt_grayscale_and_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr = SpriteManager(tmp)
            rgba = np.full((4, 5, 4), (20, 40, 60, 128), np.uint8)
            cv2.imwrite(str(Path(tmp)/'rgba.png'), rgba)
            np.testing.assert_array_equal(mgr.get_sprite('rgba.png'), rgba)
            cv2.imwrite(str(Path(tmp)/'gray.png'), np.zeros((4, 5), np.uint8))
            self.assertEqual(mgr.get_sprite('gray.png').shape, (4, 5, 4))
            Path(tmp, 'broken.png').write_text('not a png')
            with warnings.catch_warnings(record=True) as recorded:
                warnings.simplefilter('always')
                for name in ('missing.png', 'missing.png', 'broken.png'):
                    self.assertEqual(mgr.get_sprite(name, (40, 30)).shape, (30, 40, 4))
                self.assertEqual(len(recorded), 2)

    def test_rig_adapter_parts_and_anchor(self):
        class Rig:
            def parts(self):
                return [Actor('hand', position=(3, 0), sprite_name='part')]
            def anchor(self, name):
                return np.array([3, 0])
            def reach(self, target):
                self.target = target
        rig = Rig()
        actor = Actor('rig', mode=ActorMode.PROCEDURAL_RIG, rig=rig, position=(10, 10))
        actor.reach((12, 10))
        self.assertEqual(rig.target, (12, 10))
        prop = Actor('prop', sprite_name='part')
        actor.attach_prop(prop, anchor='hand')
        np.testing.assert_allclose(prop.world_transform()[0], (13, 10))
        frame = np.zeros((20, 20, 3), np.uint8)
        composite_frame(frame, [actor], {'part': np.full((2, 2, 4), 255, np.uint8)})
        self.assertTrue(frame[10, 13].any())


class ExportTests(unittest.TestCase):
    def test_missing_audio_ffmpeg_and_failure_keep_silent(self):
        with tempfile.TemporaryDirectory() as tmp:
            silent, audio, output = [Path(tmp)/name for name in ('silent.mp4', 'audio.mp3', 'out.mp4')]
            silent.write_bytes(b'video')
            self.assertFalse(mux_audio(silent, audio, output, 8))
            audio.write_bytes(b'audio')
            with patch('scene.export.find_ffmpeg', return_value=None):
                self.assertFalse(mux_audio(silent, audio, output, 8))
            with patch('scene.export.find_ffmpeg', return_value='/ffmpeg'), patch('scene.export.subprocess.run', side_effect=OSError('failed')), warnings.catch_warnings():
                warnings.simplefilter('ignore')
                self.assertFalse(mux_audio(silent, audio, output, 8))
            self.assertEqual(output.read_bytes(), b'video')

    def test_real_video_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = render_video(lambda t: np.full((24, 32, 3), int(t*100), np.uint8),
                                Path(tmp)/'out.mp4', size=(32, 24), fps=10, duration=1)
            cap = cv2.VideoCapture(str(path))
            self.assertEqual(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), 10)
            self.assertEqual(cap.get(cv2.CAP_PROP_FPS), 10)
            cap.release()


class SceneIntegrationTests(unittest.TestCase):
    def test_same_scene_at_common_times_for_24_30_60fps(self):
        from demo.step6_scenario_timeline import HospitalityScene, DURATION_S
        results = []
        with tempfile.TemporaryDirectory() as tmp, warnings.catch_warnings():
            warnings.simplefilter('ignore')
            for fps in (24, 30, 60):
                scene = HospitalityScene(tmp)
                snapshots = []
                for f in range(DURATION_S*fps+1):
                    t = f/fps
                    scene.timeline.update(t)
                    if f % fps == 0:
                        # Render only common sample times; compare actual pixels,
                        # including nonaccumulating bob, expression and prop motion.
                        snapshots.append(scene.render(t))
                results.append(snapshots)
                self.assertEqual(scene.bilge.expression, 'happy')
                self.assertEqual(scene.brother.expression, 'smile')
                self.assertTrue({'bilge', 'bilge_brother', 'guest_1', 'guest_2', 'tea_tray', 'dessert_tray'}.issubset({a.name for a in scene.actors}))
                self.assertLessEqual(max(a.end_time() for a in scene.timeline.actions), DURATION_S)
                for actor in (scene.bilge, scene.brother):
                    point = actor.position + actor.anchors['contact']
                    np.testing.assert_allclose(point, scene.layout['characters'][actor.name]['contact_world'])
            for other in results[1:]:
                for a, b in zip(results[0], other):
                    np.testing.assert_array_equal(a, b)

    def test_real_png_replaces_room_without_code_changes(self):
        from demo.step6_scenario_timeline import HospitalityScene
        with tempfile.TemporaryDirectory() as tmp, warnings.catch_warnings():
            warnings.simplefilter('ignore')
            path = Path(tmp)/'backgrounds'
            path.mkdir()
            cv2.imwrite(str(path/'living_room.png'), np.full((72, 128, 4), (31, 61, 91, 255), np.uint8))
            scene = HospitalityScene(tmp)
            frame = scene.render(0)
            np.testing.assert_array_equal(frame[100, 100], (31, 61, 91))

    def test_mux_command_preserves_video_and_selects_audio_excerpt(self):
        with tempfile.TemporaryDirectory() as tmp:
            silent, audio, output = [Path(tmp)/name for name in ('silent.mp4', 'audio.mp3', 'out.mp4')]
            silent.touch()
            audio.touch()
            with patch('scene.export.find_ffmpeg', return_value='/ffmpeg'), patch('scene.export.subprocess.run') as run:
                self.assertTrue(mux_audio(silent, audio, output, 8, 12.5))
            cmd = run.call_args.args[0]
            self.assertEqual(cmd[cmd.index('-ss')+1], '12.5')
            self.assertEqual(cmd[cmd.index('-t')+1], '8')
            self.assertEqual(cmd[cmd.index('-c:v')+1], 'copy')
            self.assertEqual(cmd[cmd.index('-af')+1], 'apad')


if __name__ == '__main__':
    unittest.main()
