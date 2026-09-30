"""Scene entities and the explicit boundary to future gait/IK adapters."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol
import numpy as np


class ActorMode(Enum):
    SIMPLE_SPRITE = 'simple'
    PROCEDURAL_RIG = 'procedural'


class Expression(Enum):
    NEUTRAL = 'neutral'
    HAPPY = 'happy'
    SMILE = 'smile'
    SURPRISED = 'surprised'
    SAD = 'sad'


class RigAdapter(Protocol):
    """Adapter owns physics stepping and maps bones into actor-local pixels.

    Existing frame-dependent physics should use a fixed-step accumulator inside
    update, independently of the render FPS. parts() returns local-space Actors
    (one per PNG body part); anchor() returns a local-space bone position.
    """
    def update(self, dt_seconds: float) -> None: ...
    def walk_to(self, target_x: float, duration: float) -> None: ...
    def reach(self, target: tuple) -> None: ...
    def set_pose(self, points: dict) -> None: ...
    def look_at(self, target: tuple) -> None: ...
    def anchor(self, name: str) -> np.ndarray: ...
    def parts(self) -> list[Actor]: ...


@dataclass(eq=False)
class Actor:
    name: str
    mode: ActorMode = ActorMode.SIMPLE_SPRITE
    position: np.ndarray = field(default_factory=lambda: np.zeros(2))
    scale: float = 1.0
    rotation: float = 0.0  # positive = visually counterclockwise, as in OpenCV
    z_index: int = 0
    visible: bool = True
    alpha: float = 1.0
    expression: str | None = None
    sprite_name: str | None = None
    expressions: dict[str, str] = field(default_factory=dict)
    anchors: dict[str, tuple] = field(default_factory=lambda: {'center': (0, 0), 'hands': (0, 0)})
    attached_props: dict = field(default_factory=dict)
    rig: RigAdapter | None = None
    parallax_depth: float = 1.0
    # Secondary motion is additive and never modifies the timeline base position.
    render_offset: np.ndarray = field(default_factory=lambda: np.zeros(2))
    render_rotation: float = 0.0
    look_target: tuple | None = None
    _parent: Actor | None = field(default=None, init=False, repr=False)

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float)

    @property
    def sprite_key(self):
        return self.expressions.get(self.expression, self.sprite_name)

    def set_expression(self, expression):
        self.expression = expression.value if isinstance(expression, Expression) else expression

    def attach_prop(self, prop_actor, prop_name=None, anchor='center', offset=(0, 0)):
        ancestor = self
        while ancestor is not None:
            if ancestor is prop_actor:
                raise ValueError('Prop attachment would create a cycle')
            ancestor = ancestor._parent
        self.anchor_position(anchor)  # reject unknown anchors early
        if prop_actor._parent is not None:
            raise ValueError('Detach prop from its current parent before attaching it')
        name = prop_name or prop_actor.name
        if name in self.attached_props:
            raise ValueError(f'Prop name already attached: {name}')
        self.attached_props[name] = (prop_actor, {'anchor': anchor, 'offset': np.asarray(offset, dtype=float)})
        prop_actor._parent = self

    def anchor_position(self, name):
        if self.mode == ActorMode.PROCEDURAL_RIG:
            return np.asarray(self._require_rig().anchor(name), dtype=float)
        if name not in self.anchors:
            raise ValueError(f'{self.name}: unknown anchor {name!r}')
        return np.asarray(self.anchors[name], dtype=float)

    @staticmethod
    def rotate_vector(vector, angle):
        c, s = np.cos(np.radians(angle)), np.sin(np.radians(angle))
        return np.array([[c, s], [-s, c]]) @ np.asarray(vector)

    def world_transform(self):
        pos = np.asarray(self.position) + self.render_offset
        angle = self.rotation + self.render_rotation
        if self._parent is None:
            return pos, self.scale, angle, self.alpha, self.visible
        parent = self._parent
        p, scale, rot, alpha, visible = parent.world_transform()
        info = next(info for prop, info in parent.attached_props.values() if prop is self)
        local = parent.anchor_position(info['anchor']) + info['offset'] + pos
        return (p + self.rotate_vector(local * scale, rot), scale*self.scale,
                rot+angle, alpha*self.alpha, visible and self.visible)

    def detach_prop(self, prop_name):
        if prop_name not in self.attached_props:
            raise KeyError(prop_name)
        prop, _ = self.attached_props[prop_name]
        p, s, r, a, v = prop.world_transform()
        del self.attached_props[prop_name]
        prop._parent = None
        prop.position = p - prop.render_offset
        prop.scale, prop.rotation, prop.alpha, prop.visible = s, r-prop.render_rotation, a, v
        return prop

    def look_at(self, target):
        self.look_target = tuple(target)
        if self.mode == ActorMode.PROCEDURAL_RIG:
            self._require_rig().look_at(self.look_target)
        else:
            # Whole-image tilt is the only available gaze gesture in sprite mode.
            dx, dy = np.asarray(target)-self.world_transform()[0]
            self.rotation = float(np.clip(-np.degrees(np.arctan2(dy, max(abs(dx), 1))), -6, 6))

    def _require_rig(self):
        if self.rig is None:
            raise NotImplementedError(f'{self.name}: this operation requires a RigAdapter')
        return self.rig

    def walk_to(self, target_x, duration, *, timeline=None, at=0.0):
        if self.mode == ActorMode.PROCEDURAL_RIG:
            return self._require_rig().walk_to(target_x, duration)
        if timeline is None:
            raise ValueError('SIMPLE_SPRITE.walk_to requires timeline= and at=')
        from .actions import ActionType
        return timeline.tween(at, at+duration, ActionType.MOVE_ACTOR, actor=self,
                              target=(target_x, self.position[1]), easing='ease_in_out')

    def reach(self, target):
        return self._require_rig().reach(target)

    def set_pose(self, skeleton_points):
        return self._require_rig().set_pose(skeleton_points)
