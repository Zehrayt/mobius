"""[REFERANS -- ana motor DEGIL] Kalca kinematik bir egriyle suruluyor; fizik
tabanli karsiligi: demo/step17_bilge_physics_skin.py (bkz. README "Adim 17").

Dress the UNCHANGED bilge_walk_validation poses in articulated cutouts.

python3 demo/bilge_walk_skinned.py
python3 demo/bilge_walk_skinned.py --debug  # separate *_debug outputs
python3 demo/bilge_walk_skinned.py --preview-only

The existing solver owns all joint positions and timing. This module only
binds cloth/hair/skin textures to those joints, including a sole-to-ankle
visual offset. It never modifies a pose or invokes another gait solver.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo.bilge_walk_validation import WalkConfig, simulate, measure, validate, preview_indices
from scene.export import render_video
from scene.skinning import Cutout, bone_matrix, rigid_matrix, transform_point, composite_cutout

ASSETS = ROOT/'assets/characters/bilge_rig'
ORDER = ('right_hand', 'right_forearm', 'right_upper_arm',
         'right_shoe', 'right_shin', 'right_thigh',
         'left_shoe', 'left_shin', 'left_thigh', 'pelvis', 'torso',
         'left_hand', 'left_forearm', 'left_upper_arm', 'braid', 'head')


class BilgeSkin:
    def __init__(self, asset_dir=ASSETS):
        self.asset_dir = Path(asset_dir)
        self.config = json.loads((self.asset_dir/'rig.json').read_text())
        self.settings = self.config['parts']
        self.parts = {name: Cutout.load(name, self.asset_dir/entry['file'])
                      for name, entry in self.settings.items()}
        if set(ORDER) != set(self.parts):
            raise ValueError('Bilge requires all 16 named cutouts; no whole-PNG fallback')

    def anchor(self, name, key='anchor'):
        return self.parts[name].local(self.settings[name][key])

    def scale(self, name):
        return self.settings[name]['width_px']/self.parts[name].width

    def bind_bone(self, name, a, b):
        return bone_matrix(self.anchor(name), self.anchor(name, 'end'), a, b, self.scale(name))

    def transforms(self, frame):
        p = frame['points']
        matrices, ankles = {}, {}
        for side in ('right', 'left'):
            name = f'{side}_shoe'
            foot = p[f'{side}_foot']
            phase = frame['swing_phase'][side]
            # During support: fixed scale, zero rotation, fixed world contact.
            # During swing: a small toe lift, fading continuously at both ends.
            angle = 0. if phase is None else np.radians(-9.)*np.sin(np.pi*min(1., phase))**2
            matrix = rigid_matrix(self.anchor(name), foot, self.scale(name), angle)
            hull = self.parts[name].hull @ matrix[:, :2].T + matrix[:, 2]
            # Source alpha hull determines the actual sole, not canvas bounds.
            # Half-pixel inset accounts for antialiasing of the raster contact.
            matrix[1, 2] += foot[1] - .55 - hull[:, 1].max()
            matrices[name] = matrix
            ankles[side] = transform_point(matrix, self.anchor(name))
            matrices[f'{side}_shin'] = self.bind_bone(f'{side}_shin', p[f'{side}_knee'], ankles[side])
            matrices[f'{side}_thigh'] = self.bind_bone(f'{side}_thigh', p[f'{side}_hip'], p[f'{side}_knee'])
            matrices[f'{side}_upper_arm'] = self.bind_bone(f'{side}_upper_arm', p[f'{side}_shoulder'], p[f'{side}_elbow'])
            matrices[f'{side}_forearm'] = self.bind_bone(f'{side}_forearm', p[f'{side}_elbow'], p[f'{side}_hand'])
            direction = p[f'{side}_hand'] - p[f'{side}_elbow']
            angle = np.arctan2(-direction[0], direction[1])
            matrices[f'{side}_hand'] = rigid_matrix(self.anchor(f'{side}_hand'), p[f'{side}_hand'],
                                                    self.scale(f'{side}_hand'), angle)

        axis = p['chest'] - p['pelvis']
        body_angle = np.arctan2(axis[0], -axis[1])
        matrices['torso'] = self.bind_bone('torso', p['chest'], p['pelvis'] + [0., 6.])
        matrices['pelvis'] = rigid_matrix(self.anchor('pelvis'), p['pelvis'] + [0., 8.], self.scale('pelvis'), body_angle)
        matrices['head'] = rigid_matrix(self.anchor('head'), p['head'], self.scale('head'), body_angle)
        # Braid root is an actual local point on the head, not a world offset.
        braid_root = transform_point(matrices['head'], self.parts['head'].local([95/435, 341/438]))
        braid_end = p['pelvis'] + [-19., -12.]
        matrices['braid'] = self.bind_bone('braid', braid_root, braid_end)
        return matrices, ankles

    def layers(self, frame):
        matrices, ankles = self.transforms(frame)
        layers = {name: self.parts[name].warp(matrices[name]) for name in ORDER}
        return layers, matrices, ankles


def plain_background(c):
    image = np.full((c.height, c.width, 3), (237, 244, 249), np.uint8)
    floor = round(c.ground_y)
    image[floor+1:] = (226, 235, 241)
    cv2.line(image, (0, floor+1), (c.width-1, floor+1), (199, 212, 222), 1, cv2.LINE_AA)
    return image


def render_skin(frame, skin, c, debug=False):
    image = plain_background(c)
    # Contact shadows stay in world space and fade while a foot is lifted.
    for side in ('right', 'left'):
        foot = frame['points'][f'{side}_foot']
        opacity = .14 * np.exp(-max(0, c.ground_y-foot[1])/12.)
        shadow = image.copy()
        cv2.ellipse(shadow, (round(foot[0]+8), round(c.ground_y+2)), (24, 4), 0, 0, 360,
                    (117, 131, 145), -1, cv2.LINE_AA)
        cv2.addWeighted(shadow, opacity, image, 1-opacity, 0, image)
    layers, matrices, ankles = skin.layers(frame)
    for name in ORDER:
        warped, origin = layers[name]
        composite_cutout(image, warped, origin)
    if debug:
        p = frame['points']
        xy = lambda a: tuple(np.rint(a).astype(int))
        for side, color in (('left', (210, 170, 0)), ('right', (10, 130, 240))):
            for names in (('hip', 'knee', 'foot'), ('shoulder', 'elbow', 'hand')):
                points = [p[f'{side}_{name}'] for name in names]
                for a, b in zip(points, points[1:]):
                    cv2.line(image, xy(a), xy(b), color, 1, cv2.LINE_AA)
                for point in points:
                    cv2.circle(image, xy(point), 3, color, -1, cv2.LINE_AA)
            cv2.drawMarker(image, xy(ankles[side]), (180, 50, 180), cv2.MARKER_CROSS, 8, 1)
            sole = p[f'{side}_foot']
            cv2.line(image, xy(sole+[-12, 0]), xy(sole+[25, 0]), (60, 140, 40), 1)
        cv2.line(image, xy(p['pelvis']), xy(p['chest']), (30, 100, 30), 1)
        cv2.line(image, xy(p['chest']), xy(p['head']), (30, 100, 30), 1)
        cv2.putText(image, f'DEBUG / {frame["time"]:.2f}s / green: sole / purple: visual ankle',
                    (40, 45), cv2.FONT_HERSHEY_SIMPLEX, .65, (60, 70, 75), 1, cv2.LINE_AA)
    return image


def overlap_pixels(first, second):
    a, (ax, ay) = first
    b, (bx, by) = second
    x0, y0 = max(ax, bx), max(ay, by)
    x1, y1 = min(ax+a.shape[1], bx+b.shape[1]), min(ay+a.shape[0], by+b.shape[0])
    if x1 <= x0 or y1 <= y0:
        return 0
    aa = a[y0-ay:y1-ay, x0-ax:x1-ax, 3]
    ba = b[y0-by:y1-by, x0-bx:x1-bx, 3]
    return int(np.count_nonzero((aa > .5) & (ba > .5)))


def inspect_geometry(frames, skin, c):
    minimum_overlap = {}
    max_penetration = 0.
    max_stance_gap = 0.
    max_contact_slip = 0.
    contact_origins = {}
    for frame in frames:
        layers, matrices, ankles = skin.layers(frame)
        for side in ('left', 'right'):
            pairs = ((f'{side}_upper_arm', f'{side}_forearm'),
                     (f'{side}_forearm', f'{side}_hand'),
                     (f'{side}_thigh', f'{side}_shin'),
                     (f'{side}_shin', f'{side}_shoe'),
                     ('torso', f'{side}_upper_arm'), ('pelvis', f'{side}_thigh'))
            for a, b in pairs:
                key = a+'/'+b
                minimum_overlap[key] = min(minimum_overlap.get(key, 10**9), overlap_pixels(layers[a], layers[b]))
            name = f'{side}_shoe'
            warped, origin = layers[name]
            ys, xs = np.nonzero(warped[..., 3] > .5)
            bottom = float(ys.max()+origin[1])
            max_penetration = max(max_penetration, bottom-c.ground_y)
            if frame['states'][side] == 'stance':
                max_stance_gap = max(max_stance_gap, c.ground_y-bottom)
                contact = transform_point(matrices[name], skin.anchor(name))
                if side not in contact_origins:
                    contact_origins[side] = contact.copy()
                max_contact_slip = max(max_contact_slip, float(np.linalg.norm(contact-contact_origins[side])))
            else:
                contact_origins.pop(side, None)
        for a, b in (('torso', 'head'), ('torso', 'pelvis'), ('head', 'braid')):
            key = a+'/'+b
            minimum_overlap[key] = min(minimum_overlap.get(key, 10**9), overlap_pixels(layers[a], layers[b]))
    return dict(minimum_pair_overlap_pixels=minimum_overlap,
                max_visible_sole_penetration_px=max_penetration,
                max_visible_stance_sole_gap_px=max_stance_gap,
                max_stance_shoe_translation_px=max_contact_slip,
                alpha_measurement_threshold=.5,
                note='Raster sole measurement excludes contact shadows; shoes use an alpha-hull contact offset. '
                     'Original gait points are unchanged; visual ankles sit above the sole points.')


def validate_skin(report):
    if min(report['minimum_pair_overlap_pixels'].values()) < 5:
        raise RuntimeError('A joint connection has separated; inspect rig.json anchors')
    if report['max_visible_sole_penetration_px'] > 0 or report['max_visible_stance_sole_gap_px'] > 1:
        raise RuntimeError('Sole contact failed; inspect shoe alpha/ankle offsets')
    if report['max_stance_shoe_translation_px'] > 1e-5:
        raise RuntimeError('A support shoe moved in world space')


def montage(images, frames, indices, c):
    """Magnified crops of four frames; the VIDEO camera remains fixed."""
    canvas = np.full((720, 1280, 3), (237, 244, 249), np.uint8)
    for panel, (idx, label) in enumerate(zip(indices, ('BEKLEME', 'SOL ADIM', 'SAG ADIM', 'SON DURUS'))):
        x = round(frames[idx]['points']['pelvis'][0])
        crop = images[idx][175:610, x-112:x+112]
        tile = cv2.resize(crop, (300, 582), interpolation=cv2.INTER_CUBIC)
        canvas[87:669, panel*320+10:panel*320+310] = tile
        cv2.putText(canvas, label, (panel*320+23, 40), cv2.FONT_HERSHEY_SIMPLEX, .66, (65, 76, 89), 2, cv2.LINE_AA)
        cv2.putText(canvas, f'{frames[idx]["time"]:.2f} s', (panel*320+23, 68), cv2.FONT_HERSHEY_SIMPLEX, .5, (110, 120, 130), 1, cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--debug', action='store_true', help='Write separate outputs with skeleton and ankle/sole markers')
    parser.add_argument('--preview-only', action='store_true')
    args = parser.parse_args()
    c, skin = WalkConfig(), BilgeSkin()
    frames = simulate(c)
    gait_report = measure(frames, c)
    validate(gait_report)
    report = inspect_geometry(frames, skin, c)
    validate_skin(report)
    report['gait'] = gait_report
    report['missing_parts'] = skin.config['missing_parts']
    report['joint_source'] = 'demo.bilge_walk_validation.simulate(WalkConfig()) without changes'
    out = ROOT/'outputs'
    out.mkdir(exist_ok=True)
    stem = 'bilge_walk_skinned' + ('_debug' if args.debug else '')
    indices = preview_indices(frames, c)
    if args.preview_only:
        images = {i: render_skin(frames[i], skin, c, args.debug) for i in indices}
    else:
        path = out/f'{stem}.mp4'
        render_video(lambda t: render_skin(frames[round(t*c.fps)], skin, c, args.debug), path,
                     size=(c.width, c.height), fps=c.fps, duration=c.duration)
        cap = cv2.VideoCapture(str(path))
        count, images = 0, {}
        try:
            while True:
                ok, image = cap.read()
                if not ok:
                    break
                if image.shape != (c.height, c.width, 3):
                    raise RuntimeError('Wrong encoded frame size')
                if count in indices:
                    images[count] = image
                count += 1
        finally:
            cap.release()
        if count != len(frames):
            raise RuntimeError(f'Decoded {count}/{len(frames)} frames')
        report['decoded_video_frames'] = count
    if not cv2.imwrite(str(out/f'{stem}_preview.png'), montage(images, frames, indices, c)):
        raise RuntimeError('Cannot write preview')
    (out/f'{stem}_report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
