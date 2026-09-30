"""Affine cutout skinning: local attachment points -> world joints.

No simulation here. Matrices are computed from the supplied pose only.
Color interpolation is premultiplied to avoid dark transparent fringes.
"""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


def transform_point(matrix, point):
    return matrix[:, :2] @ np.asarray(point, float) + matrix[:, 2]


def bone_matrix(source_a, source_b, target_a, target_b, transverse_scale):
    """Map BOTH attachment points exactly; independently size cloth width."""
    source_a, source_b, target_a, target_b = map(
        lambda p: np.asarray(p, float), (source_a, source_b, target_a, target_b))
    source_axis = source_b - source_a
    target_axis = target_b - target_a
    source_length, target_length = np.linalg.norm(source_axis), np.linalg.norm(target_axis)
    if min(source_length, target_length, transverse_scale) <= 0:
        raise ValueError('Nonzero bone lengths and positive width are required')
    u, v = source_axis/source_length, target_axis/target_length
    source_basis = np.column_stack((np.array([u[1], -u[0]]), u))
    target_basis = np.column_stack((np.array([v[1], -v[0]])*transverse_scale,
                                    v*target_length/source_length))
    linear = target_basis @ source_basis.T
    return np.column_stack((linear, target_a - linear @ source_a))


def rigid_matrix(source_anchor, target_anchor, scale, angle=0.):
    """Uniform scale and rotation, used for the face and shoes."""
    co, si = np.cos(angle), np.sin(angle)
    linear = scale * np.array([[co, -si], [si, co]])
    return np.column_stack((linear, np.asarray(target_anchor) - linear @ source_anchor))


@dataclass
class Cutout:
    name: str
    image: np.ndarray

    def __post_init__(self):
        if self.image is None or self.image.ndim != 3 or self.image.shape[2] != 4:
            raise ValueError(f'{self.name}: an RGBA cutout is required')
        self.height, self.width = self.image.shape[:2]
        self.premultiplied = self.image.astype(np.float32)/255
        self.premultiplied[..., :3] *= self.premultiplied[..., 3:4]
        self.mipmaps = [self.premultiplied]
        for level in range(1, 4):
            self.mipmaps.append(cv2.resize(self.premultiplied,
                (max(1, self.width//2**level), max(1, self.height//2**level)), interpolation=cv2.INTER_AREA))
        ys, xs = np.nonzero(self.image[..., 3] > 32)
        if not len(xs):
            raise ValueError(f'{self.name}: empty alpha')
        points = np.column_stack((xs, ys)).astype(np.float32)
        self.hull = cv2.convexHull(points).reshape(-1, 2)

    @classmethod
    def load(cls, name, path):
        return cls(name, cv2.imread(str(Path(path)), cv2.IMREAD_UNCHANGED))

    def local(self, normalized):
        return np.asarray(normalized, float) * [self.width-1, self.height-1]

    def warp(self, matrix):
        corners = np.array([[0, 0], [self.width, 0], [self.width, self.height], [0, self.height]])
        mapped = corners @ matrix[:, :2].T + matrix[:, 2]
        origin = np.floor(mapped.min(axis=0)).astype(int) - 2
        end = np.ceil(mapped.max(axis=0)).astype(int) + 2
        local_matrix = matrix.copy()
        local_matrix[:, 2] -= origin
        # Prefilter tiny textures before rotating: avoids crawling denim/laces.
        min_scale = np.linalg.svd(matrix[:, :2], compute_uv=False).min()
        level = int(np.clip(np.floor(np.log2(.75/max(min_scale, 1e-8))), 0, len(self.mipmaps)-1))
        source = self.mipmaps[level]
        scale = np.array([self.width/source.shape[1], self.height/source.shape[0]])
        # Pixel-center mapping for an area-resized source.
        local_matrix[:, 2] += local_matrix[:, :2] @ ((scale-1)/2)
        local_matrix[:, :2] *= scale
        warped = cv2.warpAffine(source, local_matrix, tuple(end-origin),
                                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return warped, origin


def composite_cutout(frame, warped, origin):
    """Blend a premultiplied float cutout into a BGR uint8 frame."""
    x, y = map(int, origin)
    h, w = warped.shape[:2]
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(frame.shape[1], x+w), min(frame.shape[0], y+h)
    if x1 <= x0 or y1 <= y0:
        return
    src = warped[y0-y:y1-y, x0-x:x1-x]
    dst = frame[y0:y1, x0:x1]
    dst[:] = np.clip(np.rint(src[..., :3]*255 + dst*(1-src[..., 3:4])), 0, 255).astype(np.uint8)
