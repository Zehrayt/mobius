"""Türküz Biz / Misafiri Severiz: four-second scene for song seconds 10–14.

Run from any directory. Real assets override the drawn, labeled placeholders.
Physics remains untouched; the rig adapter boundary lives in scene.actor.
"""
from pathlib import Path
import argparse
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2
import numpy as np
from scene import Actor, Camera, Timeline, ActionType as A, SpriteManager, composite_frame
from scene.export import render_video
from scene.staging import stage_sprite, place_at_contact, foreground_cutout, contact_shadow

W, H, FPS = 1280, 720, 30
AUDIO_START_S, AUDIO_END_S = 10, 14
DURATION_S = AUDIO_END_S - AUDIO_START_S


def room_placeholder():
    """Simple geometric art, kept in the scene rather than the generic renderer."""
    img = np.full((H, W, 4), (208, 231, 245, 255), np.uint8)
    cv2.rectangle(img, (0, 465), (W, H), (154, 188, 210, 255), -1)
    cv2.rectangle(img, (0, 455), (W, 471), (175, 206, 225, 255), -1)
    # Window and a warm patch of sunlight.
    cv2.fillConvexPoly(img, np.array([[110, 110], [350, 110], [580, 455], [160, 455]]), (218, 242, 252, 255))
    cv2.rectangle(img, (80, 78), (340, 320), (244, 249, 250, 255), -1)
    cv2.rectangle(img, (96, 94), (324, 302), (229, 212, 168, 255), -1)
    cv2.circle(img, (266, 150), 35, (180, 237, 255, 255), -1, cv2.LINE_AA)
    cv2.line(img, (210, 94), (210, 302), (244, 249, 250, 255), 9)
    cv2.line(img, (96, 210), (324, 210), (244, 249, 250, 255), 9)
    for x in (62, 337):
        cv2.rectangle(img, (x, 63), (x+20, 335), (158, 173, 211, 255), -1)
    # Sofa with cushions, behind the two guests.
    cv2.rectangle(img, (688, 336), (1190, 525), (126, 158, 119, 255), -1)
    for x in (712, 951):
        cv2.rectangle(img, (x, 351), (x+210, 460), (157, 188, 146, 255), -1)
    cv2.rectangle(img, (670, 434), (1208, 510), (110, 143, 100, 255), -1)
    for x in (707, 1160):
        cv2.rectangle(img, (x, 511), (x+20, 542), (72, 98, 127, 255), -1)
    # Kilim and coffee table.
    cv2.ellipse(img, (805, 614), (370, 74), 0, 0, 360, (133, 150, 201, 255), -1, cv2.LINE_AA)
    cv2.ellipse(img, (805, 614), (329, 54), 0, 0, 360, (190, 207, 229, 255), 3, cv2.LINE_AA)
    for x in range(545, 1100, 65):
        cv2.polylines(img, [np.array([[x, 605], [x+15, 617], [x, 629], [x-15, 617]])], True, (160, 173, 209, 255), 2)
    for x in (808, 1080):
        cv2.line(img, (x, 548), (x-8, 598), (82, 114, 147, 255), 12)
    cv2.ellipse(img, (945, 543), (170, 30), 0, 0, 360, (130, 172, 206, 255), -1, cv2.LINE_AA)
    # Plant and framed geometric artwork.
    cv2.rectangle(img, (1160, 268), (1210, 330), (125, 165, 200, 255), -1)
    for p in ((1145, 220), (1210, 206), (1180, 176)):
        cv2.line(img, (1183, 278), p, (104, 145, 105, 255), 5)
        cv2.ellipse(img, p, (21, 36), -30, 0, 360, (132, 166, 108, 255), -1)
    cv2.rectangle(img, (766, 113), (962, 257), (144, 181, 208, 255), 8)
    cv2.circle(img, (864, 185), 43, (165, 190, 212, 255), 3)
    cv2.putText(img, 'MISAFIRI SEVERIZ', (470, 62), cv2.FONT_HERSHEY_SIMPLEX, .8, (99, 128, 158, 255), 2, cv2.LINE_AA)
    cv2.putText(img, 'living_room', (22, 698), cv2.FONT_HERSHEY_SIMPLEX, .4, (100, 130, 155, 255), 1, cv2.LINE_AA)
    return img


def person_placeholder(label, color, *, girl=False, seated=False, happy=False):
    img = np.zeros((260, 150, 4), np.uint8)
    skin, hair = (171, 207, 238, 255), (55, 73, 95, 255)
    cv2.ellipse(img, (75, 244), (47, 8), 0, 0, 360, (60, 65, 75, 45), -1, cv2.LINE_AA)
    if girl:
        cv2.ellipse(img, (75, 68), (44, 54), 0, 0, 360, hair, -1, cv2.LINE_AA)
    for x in (55, 94):
        cv2.line(img, (x, 173), (x+8 if seated else x, 228), (95, 110, 132, 255), 18, cv2.LINE_AA)
        cv2.ellipse(img, (x+8 if seated else x, 232), (17, 8), 0, 0, 360, hair, -1, cv2.LINE_AA)
    if girl:
        cv2.fillConvexPoly(img, np.array([[48, 104], [101, 104], [121, 194], [28, 194]]), (*color, 255), cv2.LINE_AA)
    else:
        cv2.rectangle(img, (43, 107), (106, 178), (*color, 255), -1)
    cv2.line(img, (45, 119), (29, 155), (*color, 255), 17, cv2.LINE_AA)
    cv2.line(img, (29, 155), (67, 155), skin, 12, cv2.LINE_AA)
    cv2.line(img, (105, 119), (120, 155), (*color, 255), 17, cv2.LINE_AA)
    cv2.line(img, (120, 155), (91, 155), skin, 12, cv2.LINE_AA)
    cv2.circle(img, (75, 62), 35, skin, -1, cv2.LINE_AA)
    cv2.ellipse(img, (75, 47), (36, 26), 0, 180, 360, hair, -1, cv2.LINE_AA)
    for x in (63, 87):
        cv2.circle(img, (x, 62), 3, hair, -1, cv2.LINE_AA)
    cv2.ellipse(img, (75, 73), (13 if happy else 8, 9 if happy else 5), 0, 0, 180, hair, 2, cv2.LINE_AA)
    if girl:
        cv2.circle(img, (108, 34), 9, (140, 151, 232, 255), -1, cv2.LINE_AA)
    cv2.putText(img, label, (8, 258), cv2.FONT_HERSHEY_SIMPLEX, .38, (55, 65, 83, 255), 1, cv2.LINE_AA)
    return img


def tray_placeholder(dessert=False):
    img = np.zeros((64, 142, 4), np.uint8)
    cv2.ellipse(img, (71, 39), (67, 12), 0, 0, 360, (120, 160, 191, 255), -1, cv2.LINE_AA)
    for x in (35, 70, 105):
        if dessert:
            cv2.rectangle(img, (x-10, 21), (x+10, 35), (99, 192, 227, 255), -1)
            cv2.line(img, (x-8, 24), (x+8, 30), (89, 151, 155, 255), 2)
        else:
            cv2.ellipse(img, (x, 36), (13, 4), 0, 0, 360, (229, 236, 239, 255), -1)
            cv2.fillConvexPoly(img, np.array([[x-9, 8], [x+9, 8], [x+6, 20], [x+9, 32], [x-9, 32], [x-6, 20]]), (65, 108, 170, 255))
    cv2.putText(img, 'dessert_tray' if dessert else 'tea_tray', (12, 61), cv2.FONT_HERSHEY_SIMPLEX, .36, (65, 75, 95, 255), 1, cv2.LINE_AA)
    return img


class HospitalityScene:
    """Contact-calibrated tableau; actors stay grounded during the camera move."""
    def __init__(self, assets_dir=ROOT/'assets', layout_path=None):
        layout_path = Path(layout_path or ROOT/'assets/layouts/step6_misafiri_severiz.json')
        self.layout = json.loads(layout_path.read_text())
        self.sprites, self.prepared, self.contact_report = {}, {}, {}
        mgr = SpriteManager(assets_dir)
        self.camera = Camera(*self.layout['camera']['center'])
        self.timeline = Timeline(self.camera)
        room_key = 'backgrounds/living_room.png'
        room = mgr.get_sprite(room_key, fallback_size=(W, H), fallback=room_placeholder)
        room = cv2.resize(room, (W, H), interpolation=cv2.INTER_AREA)
        self.sprites[room_key] = room
        background = Actor('living_room', position=(W/2, H/2), sprite_name=room_key)
        self.actors = [background]
        specs = [
            ('bilge_smile', 'bilge', 'Bilge', (152, 146, 221), True, False, False),
            ('bilge_happy', 'bilge', 'Bilge', (152, 146, 221), True, False, True),
            ('bilge_brother_happy', 'bilge_brother', 'Kardes', (193, 171, 92), False, False, True),
            ('guest_1', 'guest_1', 'Dede', (111, 167, 187), False, True, True),
            ('guest_2', 'guest_2', 'Nine', (173, 143, 175), True, True, True),
        ]
        for stem, name, label, color, girl, seated, happy in specs:
            key = f'characters/{stem}.png'
            original = mgr.get_sprite(key, fallback_size=(150, 260),
                fallback=lambda l=label, c=color, g=girl, s=seated, h=happy:
                person_placeholder(l, c, girl=g, seated=s, happy=h))
            prepared = stage_sprite(original,
                height=self.layout['characters'][name]['visible_height'],
                threshold=self.layout['alpha_threshold'], **self.layout['light'])
            self.sprites[key] = prepared.image
            self.prepared[key] = prepared

        def add_shadow(actor, name, local_point, radius, opacity, softness):
            # Local shadow sprites inherit the actor's visibility and translation.
            # This keeps foot shadows under moving children, not at their destination.
            padding = int(np.ceil(softness*4))+2
            size = tuple(int(np.ceil(2*(r+padding))) for r in radius)
            key = f'{actor.name}_{name}_shadow'
            self.sprites[key] = contact_shadow(size, np.array(size)/2, radius,
                                              opacity=opacity, softness=softness)
            actor.anchors[name] = local_point
            shadow = Actor(key, sprite_name=key, z_index=5)
            actor.attach_prop(shadow, anchor=name)

        def make_character(name, stem):
            config = self.layout['characters'][name]
            key = f'characters/{stem}.png'
            prepared = self.prepared[key]
            actor = Actor(name, sprite_name=key, z_index=config['z_index'])
            contact = place_at_contact(actor, prepared, config['contact_uv'], config['contact_world'])
            actor.anchors['contact'] = contact
            if 'hands_uv' in config:
                actor.anchors['hands'] = prepared.landmark(config['hands_uv'])
            feet = [(actor.position+prepared.landmark(uv)).tolist() for uv in config['feet_uv']]
            if config['contact_kind'] == 'seat':
                add_shadow(actor, 'seat_shadow', contact, config['seat_shadow_radius'], .30, 4)
            for index, foot in enumerate(feet):
                local = np.asarray(foot)-actor.position
                add_shadow(actor, f'foot_{index}', local, config['foot_shadow_radius'], .24, 3.5)
                add_shadow(actor, f'sole_{index}', local, [config['foot_shadow_radius'][0]*.65, 2], .20, 1.6)
            self.contact_report[name] = {
                'visible_height': config['visible_height'],
                'rendered_size': list(prepared.image.shape[1::-1]),
                'source_alpha_bounds': list(prepared.bounds),
                'source_scale': prepared.source_scale,
                'contact_kind': config['contact_kind'],
                'contact_world': config['contact_world'],
                'feet_world': feet,
            }
            self.actors.append(actor)
            return actor

        self.guests = [make_character(name, name) for name in ('guest_1', 'guest_2')]
        self.bilge = make_character('bilge', 'bilge_happy')
        self.brother = make_character('bilge_brother', 'bilge_brother_happy')
        self.bilge.expressions = {'smile': 'characters/bilge_smile.png', 'happy': 'characters/bilge_happy.png'}
        self.brother.expressions = {'happy': 'characters/bilge_brother_happy.png', 'smile': 'characters/bilge_brother_happy.png'}
        self.bilge.set_expression('happy')
        self.brother.set_expression('smile')
        self.masks = {}
        for name, config in self.layout['occluders'].items():
            cutout = foreground_cutout(room, config['polygons'], holes=config.get('holes', []))
            self.masks[name] = cutout[:, :, 3]
            self.sprites[name] = cutout
            self.actors.append(Actor(name, position=(W/2, H/2), z_index=config['z_index'], sprite_name=name))

        def make_prop(name, parent, dessert=False):
            config = self.layout['props'][name]
            key = f'props/{name}.png'
            original = mgr.get_sprite(key, fallback_size=(142, 64), fallback=lambda: tray_placeholder(dessert))
            prepared = stage_sprite(original, width=config['visible_width'],
                                    threshold=self.layout['alpha_threshold'], **self.layout['light'])
            self.sprites[key] = prepared.image
            prop = Actor(name, sprite_name=key, z_index=30)
            # Align the tray handle/support with the visible palm, not PNG center.
            parent.attach_prop(prop, anchor='hands', offset=-prepared.landmark(config['support_uv']))
            self.actors.append(prop)
            return prop

        self.tea = make_prop('tea_tray', self.bilge)
        self.dessert = make_prop('dessert_tray', self.brother, True)
        tl = self.timeline
        tl.tween(0, DURATION_S, A.CAMERA_ZOOM,
                 zoom_target=self.layout['camera']['end_zoom'], easing='ease_in_out')
        # No whole-body rotation/bob on seated people: hips and feet keep contact.
        # The children arrive at the same calibrated floor contacts as the preview.
        # Translation only: whole-image tilts would lift soles off the floor.
        self.final_positions = {a.name: a.position.copy() for a in (self.bilge, self.brother)}
        for actor, start, entry_x in ((self.bilge, .15, -120), (self.brother, .3, -230)):
            tl.at(0, A.HIDE_ACTOR, actor=actor)
            tl.at(0, A.MOVE_ACTOR, actor=actor, target=(entry_x, actor.position[1]))
            tl.at(start, A.SHOW_ACTOR, actor=actor)
            tl.tween(start, 1.85, A.MOVE_ACTOR, actor=actor,
                     target=self.final_positions[actor.name], easing='ease_in_out')
        for prop, delay in ((self.tea, 0), (self.dessert, .12)):
            tl.tween(1.9+delay, 2.8+delay, A.MOVE_ACTOR, actor=prop,
                     target=self.layout['props'][prop.name]['offer_offset'], easing='ease_in_out')

    def render_static(self):
        """Fresh scene preview with no timeline, camera or secondary motion."""
        frame = np.zeros((H, W, 3), np.uint8)
        composite_frame(frame, self.actors, self.sprites, Camera(*self.layout['camera']['center']))
        return frame

    def save_preview(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        preview = self.render_static()
        if not cv2.imwrite(str(path), preview):
            raise RuntimeError(f'Could not save preview: {path}')
        debug_dir = path.parent/'layout_debug'
        debug_dir.mkdir(exist_ok=True)
        debug = preview.copy()
        for name, report in self.contact_report.items():
            point = tuple(np.rint(report['contact_world']).astype(int))
            cv2.drawMarker(debug, point, (80, 255, 80), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(debug, name, (point[0]+7, point[1]-9), 0, .42, (80, 255, 80), 1, cv2.LINE_AA)
            for foot in report['feet_world']:
                cv2.circle(debug, tuple(np.rint(foot).astype(int)), 4, (255, 220, 60), 1, cv2.LINE_AA)
        cv2.imwrite(str(debug_dir/'contacts.png'), debug)
        for name, mask in self.masks.items():
            cv2.imwrite(str(debug_dir/f'{name}_mask.png'), mask)
        (debug_dir/'placement.json').write_text(json.dumps(self.contact_report, indent=2)+'\n')
        print(f'Statik onizleme: {path}')

    def render(self, seconds):
        self.timeline.update(seconds)
        frame = np.zeros((H, W, 3), np.uint8)
        composite_frame(frame, self.actors, self.sprites, self.camera)
        return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fps', type=float, default=FPS)
    parser.add_argument('--assets-dir', type=Path, default=ROOT/'assets')
    parser.add_argument('--layout', type=Path, default=ROOT/'assets/layouts/step6_misafiri_severiz.json')
    parser.add_argument('--preview', type=Path, default=ROOT/'outputs/step6_misafiri_severiz_preview.png')
    parser.add_argument('--preview-only', action='store_true', help='Render a static layout without running animation')
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/step6_misafiri_severiz.mp4')
    parser.add_argument('--audio-start', type=float, default=AUDIO_START_S,
                        help='Audio excerpt start in seconds (default: 10, Misafiri severiz)')
    args = parser.parse_args()
    scene = HospitalityScene(args.assets_dir, args.layout)
    scene.save_preview(args.preview)
    if not args.preview_only:
        render_video(scene.render, args.output, size=(W, H), fps=args.fps, duration=DURATION_S,
                     audio_path=args.assets_dir/'audio/turkuz_biz.mp3', audio_start=args.audio_start)


if __name__ == '__main__':
    main()
