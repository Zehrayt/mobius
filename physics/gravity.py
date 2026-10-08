"""Gravity policy for the 30 Hz biped: one physical value, or legacy replay.

The legacy policy exists only to reproduce steps 29-31. It deliberately
retains their low walking gravity and real-scale force/fall gravity.
"""
from dataclasses import dataclass
import math

PIXELS_PER_METRE = 184.0 / 0.9
SIMULATION_FPS = 30
STANDARD_GRAVITY = 9.81
REAL_GRAVITY = STANDARD_GRAVITY * PIXELS_PER_METRE / SIMULATION_FPS**2
LEGACY_WALK_GRAVITY = 0.065


@dataclass(frozen=True)
class GravityPolicy:
    mode: str = 'unified'

    def __post_init__(self):
        if self.mode not in ('unified','legacy'):
            raise ValueError("gravity_mode must be 'unified' or 'legacy'")

    @property
    def walking(self):
        return REAL_GRAVITY if self.mode == 'unified' else LEGACY_WALK_GRAVITY

    @property
    def falling(self):
        return REAL_GRAVITY

    @property
    def muscle_load(self):
        return REAL_GRAVITY

    def capture_frequency(self, height):
        return math.sqrt(self.walking / height) if self.mode == 'unified' else .045
