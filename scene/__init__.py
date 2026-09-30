"""
scene/ -- production-oriented scene/timeline/actor system.

Separation of concerns:
  - physics/ (generic verlet/IK/collision motoru) characterları BILMIYOR
  - scene/ (actors, timeline, camera) production sahnelerini üretme yapıyor
  - demo/ her ikisini concrete bir sahneye bağlıyor

Bu katman:
  - REUSABLE timeline desteği (seconds-based, tween/easing, zaman bağımsız)
  - PNG sprite compositing (alpha blend, position, scale, rotation)
  - Asset fallback (dosya yoksa placeholder)
  - Simple camera system (zoom, pan)
  - Prop attachment system
"""

from .timeline import Timeline, Easing, Action, ActionType
from .actor import Actor, ActorMode, Expression, RigAdapter
from .sprite import SpriteManager
from .camera import Camera
from .compositing import composite_frame

__all__ = [
    "Timeline",
    "Easing",
    "Action",
    "ActionType",
    "Actor",
    "RigAdapter",
    "ActorMode",
    "Expression",
    "SpriteManager",
    "Camera",
    "composite_frame",
]
