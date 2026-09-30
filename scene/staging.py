"""Deterministic sprite staging: visible bounds, contacts, light and occlusion.

All processing happens on render copies. Source PNG pixels/files are preserved.
Normalized landmark coordinates refer to the visible alpha bounding box, not
transparent canvas margins. World coordinates are pixels at camera zoom 1.
"""
from dataclasses import dataclass
import cv2
import numpy as np


def alpha_bounds(sprite, threshold=16):
    if sprite.ndim != 3 or sprite.shape[2] != 4:
        raise ValueError('Expected a BGRA sprite')
    ys, xs = np.nonzero(sprite[:, :, 3] > threshold)
    if not len(xs):
        raise ValueError('Sprite has no visible pixels')
    return int(xs.min()), int(ys.min()), int(xs.max()+1), int(ys.max()+1)


@dataclass
class StagedSprite:
    image: np.ndarray
    bounds: tuple
    source_scale: float

    def landmark(self, uv):
        """Visible-box normalized coordinates -> local centered actor coordinates."""
        return (np.asarray(uv, dtype=float)-.5)*np.array(self.image.shape[1::-1])


def stage_sprite(sprite, *, height=None, width=None, threshold=16,
                 bgr_gain=(1, 1, 1), side_light=0.0):
    if (height is None) == (width is None):
        raise ValueError('Specify exactly one visible dimension')
    bounds = alpha_bounds(sprite, threshold)
    x0, y0, x1, y1 = bounds
    crop = sprite[y0:y1, x0:x1].copy()
    scale = height/(y1-y0) if height is not None else width/(x1-x0)
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('Visible size must be positive and finite')
    size = (max(1, round((x1-x0)*scale)), max(1, round((y1-y0)*scale)))
    # Premultiplied resampling keeps transparent edges free of dark color bleed.
    premul = crop.astype(np.float32)/255
    premul[:, :, :3] *= premul[:, :, 3:4]
    image = cv2.resize(premul, size, interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
    alpha = image[:, :, 3:4]
    np.divide(image[:, :, :3], alpha, out=image[:, :, :3], where=alpha > 1e-8)
    light = np.linspace(1+side_light, 1-side_light, size[0])[None, :, None]
    image[:, :, :3] *= np.asarray(bgr_gain)*light
    image = np.clip(np.rint(image*255), 0, 255).astype(np.uint8)
    return StagedSprite(image, bounds, scale)


def place_at_contact(actor, sprite, landmark, world_point):
    """Place an actor's named/local contact exactly on a world-space surface."""
    local = sprite.landmark(landmark)
    actor.position = np.asarray(world_point, dtype=float)-actor.rotate_vector(local*actor.scale, actor.rotation)
    return local


def foreground_cutout(background, polygons, *, holes=(), feather=.55):
    """Extract actual furniture pixels from a flat background using calibrated masks.

    Transparent holes remain holes: never paste the whole furniture bounding box.
    The returned layer must use exactly the background's transform and camera.
    """
    mask = np.zeros(background.shape[:2], np.uint8)
    for polygon in polygons:
        cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 255, lineType=cv2.LINE_AA)
    for polygon in holes:
        cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 0, lineType=cv2.LINE_AA)
    if feather:
        mask = cv2.GaussianBlur(mask, (0, 0), feather)
    layer = cv2.cvtColor(background[:, :, :3], cv2.COLOR_BGR2BGRA)
    layer[:, :, 3] = mask
    return layer


def contact_shadow(size, center, radius, *, opacity=.25, softness=4, color=(26, 39, 55)):
    """Soft colored contact ellipse, in world space; rendered behind its actor."""
    w, h = size
    mask = np.zeros((h, w), np.uint8)
    cv2.ellipse(mask, tuple(np.rint(center).astype(int)), tuple(np.rint(radius).astype(int)),
                0, 0, 360, round(255*opacity), -1, cv2.LINE_AA)
    if softness:
        mask = cv2.GaussianBlur(mask, (0, 0), softness)
    image = np.empty((h, w, 4), np.uint8)
    image[:, :, :3] = color
    image[:, :, 3] = mask
    return image
