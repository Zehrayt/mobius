"""
SpriteManager -- PNG loading, fallback placeholder rendering.

When a sprite file is missing, render a colored placeholder with name label.
When file exists, return the PNG image (alpha blended later).
"""

from __future__ import annotations

import hashlib
import warnings
from pathlib import Path
from typing import Optional
import numpy as np
import cv2


class SpriteManager:
    """Manages sprite loading and fallback placeholders."""

    def __init__(self, assets_dir: str = "assets"):
        self.assets_dir = Path(assets_dir)
        self._cache: dict[str, Optional[np.ndarray]] = {}  # sprite_path -> image or None
        self._placeholder_colors: dict[str, tuple] = {}  # sprite_name -> BGR color

    def _get_placeholder_color(self, sprite_name: str) -> tuple:
        """Generate consistent placeholder color for this sprite."""
        if sprite_name not in self._placeholder_colors:
            # Use sprite name hash to pick a color
            h = int.from_bytes(hashlib.sha256(sprite_name.encode()).digest()[:4], 'big') % 12
            colors = [
                (200, 100, 100),  # reddish
                (100, 200, 100),  # greenish
                (100, 100, 200),  # bluish
                (200, 200, 100),  # yellowish
                (200, 100, 200),  # magenta
                (100, 200, 200),  # cyan
                (180, 120, 80),   # brown
                (160, 80, 160),   # purple
                (80, 160, 160),   # teal
                (160, 160, 80),   # olive
                (100, 150, 200),  # sky
                (200, 150, 100),  # peach
            ]
            self._placeholder_colors[sprite_name] = colors[h]
        return self._placeholder_colors[sprite_name]

    def load_sprite(self, sprite_path: str) -> Optional[np.ndarray]:
        """
        Load sprite PNG. Returns image (H, W, 4) with alpha, or None if missing.
        Caches result.
        """
        if sprite_path in self._cache:
            return self._cache[sprite_path]

        full_path = self.assets_dir / sprite_path

        if full_path.exists():
            img = cv2.imread(str(full_path), cv2.IMREAD_UNCHANGED)
            if img is not None:
                # Ensure BGRA
                if img.ndim == 2:
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGRA)
                elif img.shape[2] == 3:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
                if img.dtype == np.uint16:
                    img = (img / 257).astype(np.uint8)
                self._cache[sprite_path] = img
                return img

        # File not found or couldn't read
        warnings.warn(f'Asset eksik veya okunamiyor: {full_path}; placeholder kullaniliyor.',
                      RuntimeWarning, stacklevel=2)
        self._cache[sprite_path] = None
        return None

    def get_placeholder_image(
        self,
        sprite_name: str,
        size: tuple = (100, 100)
    ) -> np.ndarray:
        """
        Generate a placeholder image (colored rect + text).
        size: (w, h)
        Returns: (h, w, 4) BGRA image.
        """
        w, h = size
        color = self._get_placeholder_color(sprite_name)

        # Create colored background
        img = np.zeros((h, w, 4), dtype=np.uint8)
        img[:, :, :3] = color
        img[:, :, 3] = 200  # semi-transparent

        # Draw border
        cv2.rectangle(img, (0, 0), (w-1, h-1), (255, 255, 255, 255), 2)

        # Add text label (truncate if needed)
        text = sprite_name[:20]
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        thickness = 1

        # Get text size to center it
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        x = (w - text_size[0]) // 2
        y = (h + text_size[1]) // 2

        cv2.putText(img, text, (x, y), font, font_scale, (0, 0, 0, 255), thickness)

        return img

    def get_sprite(
        self,
        sprite_path: str,
        fallback_size: tuple = (100, 100),
        fallback=None,
    ) -> np.ndarray:
        """
        Get sprite image. If file missing, return placeholder.
        Always returns (h, w, 4) BGRA.
        """
        img = self.load_sprite(sprite_path)
        if img is not None:
            return img

        # Return placeholder
        sprite_name = Path(sprite_path).stem
        return fallback() if fallback is not None else self.get_placeholder_image(sprite_name, fallback_size)
