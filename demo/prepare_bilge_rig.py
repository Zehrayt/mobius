"""Package the reference-derived atlas as named transparent rig cutouts.

This only extracts atlas parts and attachment masks, not new artwork.
The immutable atlas was prepared with imagegen; see generation_prompt.txt.
Run again after adjusting MASKS / PARTS to reproduce the asset package.
"""
from pathlib import Path
import json
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT/'assets/characters/bilge_rig'

# Each search region selects the largest alpha component. The atlas generator
# did not adhere exactly to equal cells, so these boxes are explicitly saved.
PARTS = {
    'head': ([0, 0, 325, 343], [220/435, 205/438], None, 94),
    'torso': ([335, 0, 630, 343], [.50, .20], [.50, .91], 55),
    'pelvis': ([640, 0, 940, 330], [.5, .78], None, 49),
    'braid': ([1000, 0, 1200, 352], [.48, .08], [.48, .91], 22),
    'left_upper_arm': ([45, 344, 265, 615], [.65, .15], [.37, .93], 22),
    'left_forearm': ([390, 344, 575, 615], [.49, .17], [.49, .86], 19),
    'right_upper_arm': ([680, 344, 910, 615], [.35, .15], [.66, .93], 22),
    'right_forearm': ([990, 352, 1170, 615], [.50, .17], [.50, .86], 19),
    'left_thigh': ([45, 618, 260, 948], [.56, .17], [.44, .94], 32),
    'left_shin': ([375, 618, 580, 948], [.48, .10], [.55, .89], 30),
    'right_thigh': ([680, 618, 910, 948], [.42, .17], [.57, .94], 32),
    'right_shin': ([985, 618, 1180, 948], [.54, .10], [.49, .89], 30),
    'left_hand': ([45, 960, 255, 1230], [.52, .12], None, 18),
    'right_hand': ([375, 960, 570, 1230], [.49, .12], None, 18),
    'left_shoe': ([630, 980, 944, 1210], [.26, .22], None, 43),
    'right_shoe': ([947, 980, 1254, 1210], [.26, .22], None, 43),
}


def main():
    atlas = cv2.imread(str(ASSETS/'atlas.png'), cv2.IMREAD_UNCHANGED)
    reference = cv2.imread(str(ASSETS.parent/'bilge_happy.png'), cv2.IMREAD_UNCHANGED)
    if atlas is None or atlas.shape[2] != 4:
        raise RuntimeError('Missing RGBA atlas.png')
    if reference is None or reference.shape[2] != 4:
        raise RuntimeError('Missing original Bilge reference')
    manifest = {'reference': '../bilge_happy.png', 'atlas': 'atlas.png',
                'provenance': 'Original reference pixels for head/face; imagegen built-in for separated '
                              'body/limb parts and hidden surfaces; generation_prompt.txt',
                'parts': {}, 'missing_parts': [],
                'note': 'Hidden sleeve/hip surfaces are reconstructed from the reference. '
                        'This atlas supports this fixed camera, not rear views.'}
    for name, (box, a, b, width) in PARTS.items():
        x0, y0, x1, y1 = box
        cell = atlas[y0:y1, x0:x1].copy()
        n, labels, stats, _ = cv2.connectedComponentsWithStats((cell[..., 3] > 100).astype(np.uint8))
        if n < 2:
            raise RuntimeError(f'No part in {name}')
        component = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        region = (labels == component).astype(np.uint8)
        region = cv2.dilate(region, np.ones((3, 3), np.uint8))
        cell[..., 3] *= region
        ys, xs = np.nonzero(cell[..., 3] > 0)
        crop_box = [int(xs.min()), int(ys.min()), int(xs.max()+1), int(ys.max()+1)]
        l, t, r, bottom = crop_box
        part = cell[t:bottom, l:r].copy()
        if name == 'head':
            # Preserve the actual source face, not the atlas's reinterpreted
            # face. Only alpha below the chin is masked to isolate the neck.
            part = reference[45:483, 300:735].copy()
            mask = np.zeros(reference.shape[:2], np.uint8)
            mask[:400] = 255
            neck = np.array([[421, 391], [515, 390], [511, 427], [531, 456],
                             [532, 477], [471, 477], [447, 460], [430, 435], [421, 402]])
            cv2.fillPoly(mask, [neck], 255)
            part[..., 3] = np.minimum(part[..., 3], mask[45:483, 300:735])
        if name == 'torso':
            # Remove the atlas's incidental sleeve stubs; moving sleeves are
            # independent parts. Retain the hood and the jacket's front panels.
            polygon = np.array([[.25, 0], [.75, 0], [.86, .17], [.82, .32],
                                [.78, .48], [.89, .87], [.77, 1], [.18, 1],
                                [.11, .88], [.22, .45], [.18, .29], [.19, .15]])
            mask = np.zeros(part.shape[:2], np.uint8)
            cv2.fillPoly(mask, [np.rint(polygon*[part.shape[1]-1, part.shape[0]-1]).astype(int)], 255)
            part[..., 3] = np.minimum(part[..., 3], mask)
        if not cv2.imwrite(str(ASSETS/f'{name}.png'), part):
            raise RuntimeError(f'Cannot save {name}')
        manifest['parts'][name] = {'file': f'{name}.png', 'anchor': a, 'end': b,
                                   'width_px': width, 'atlas_region': box,
                                   'crop_in_region': crop_box}
        if name == 'head':
            manifest['parts'][name].update(source='../bilge_happy.png', source_crop=[300, 45, 735, 483],
                                           atlas_region=None, crop_in_region=None,
                                           preservation='Original RGB pixels; neck isolation changes alpha only')
    (ASSETS/'rig.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(f'Packaged {len(manifest["parts"])} cutouts: {ASSETS}')


if __name__ == '__main__':
    main()
