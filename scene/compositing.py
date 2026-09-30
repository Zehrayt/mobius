"""BGRA sprites -> BGR frames; centered transforms, shared global z-order."""
from __future__ import annotations
import cv2
import numpy as np
from .actor import ActorMode


def apply_sprite_transform(sprite, position, scale, rotation, alpha, screen_size=None):
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('Sprite scale must be finite and positive')
    h, w = sprite.shape[:2]
    mat = cv2.getRotationMatrix2D((w/2, h/2), rotation, scale)
    # Enclose all rotated corners. Transparent padding, never replicated edges.
    nw = max(1, int(np.ceil(abs(mat[0, 0])*w + abs(mat[0, 1])*h)))
    nh = max(1, int(np.ceil(abs(mat[0, 1])*w + abs(mat[0, 0])*h)))
    mat[:, 2] += np.array([nw/2-w/2, nh/2-h/2])
    # Interpolate premultiplied color to avoid dark fringes at transparent edges.
    premul = sprite.astype(np.float32)/255
    premul[:, :, :3] *= premul[:, :, 3:4]
    transformed = cv2.warpAffine(premul, mat, (nw, nh), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    a = transformed[:, :, 3:4]
    np.divide(transformed[:, :, :3], a, out=transformed[:, :, :3], where=a > 1e-8)
    transformed[:, :, 3] *= np.clip(alpha, 0, 1)
    transformed = np.clip(np.rint(transformed*255), 0, 255).astype(np.uint8)
    return transformed, (round(position[0]-nw/2), round(position[1]-nh/2))


def alpha_blend_onto_frame(frame, sprite, top_left, screen_size=None):
    h, w = frame.shape[:2]
    sh, sw = sprite.shape[:2]
    x, y = top_left
    x0, y0, x1, y1 = max(0, x), max(0, y), min(w, x+sw), min(h, y+sh)
    if x0 >= x1 or y0 >= y1:
        return
    src = sprite[y0-y:y1-y, x0-x:x1-x]
    dst = frame[y0:y1, x0:x1]
    alpha = src[:, :, 3:4].astype(np.float32)/255
    dst[:] = np.rint(src[:, :, :3]*alpha + dst*(1-alpha)).astype(np.uint8)


def composite_frame(frame, actors, sprites, camera=None, screen_size=None):
    """Render without mutating actors. Attached props use local coordinates.

    Props must remain in the scene registry after detach. Duplicate references
    in the registry and attachment graph are drawn only once. Rig parts use
    local transforms; stepping a rig is the scene controller's responsibility.
    """
    screen_size = screen_size or (frame.shape[1], frame.shape[0])
    entries, seen = [], set()

    def collect(actor):
        if id(actor) in seen:
            return
        seen.add(id(actor))
        p, s, r, a, visible = actor.world_transform()
        if visible:
            if actor.mode == ActorMode.PROCEDURAL_RIG:
                for part in actor._require_rig().parts():
                    if part.visible:
                        entries.append((actor.z_index+part.z_index, part.sprite_key,
                            p+actor.rotate_vector((part.position+part.render_offset)*s, r),
                            s*part.scale, r+part.rotation+part.render_rotation,
                            a*part.alpha, actor.parallax_depth))
            else:
                root = actor
                while root._parent is not None:
                    root = root._parent
                entries.append((actor.z_index, actor.sprite_key, p, s, r, a, root.parallax_depth))
        for prop, _ in actor.attached_props.values():
            collect(prop)

    for actor in actors:
        collect(actor)
    for _, key, p, s, r, alpha, depth in sorted(entries, key=lambda e: e[0]):
        if alpha <= 0:
            continue
        if key not in sprites:
            raise KeyError(f'Sprite {key!r} was not preloaded; use SpriteManager.get_sprite for fallback')
        if camera is not None:
            p = camera.world_to_screen(p, screen_size, depth)
            s *= camera.zoom
        transformed, top_left = apply_sprite_transform(sprites[key], p, s, r, alpha, screen_size)
        alpha_blend_onto_frame(frame, transformed, top_left)
