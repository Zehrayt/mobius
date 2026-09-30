"""Reusable action bindings; no scene or character names belong here."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable
import numpy as np


class ActionType(Enum):
    SHOW_ACTOR = 'show'
    HIDE_ACTOR = 'hide'
    MOVE_ACTOR = 'move'
    SCALE_ACTOR = 'scale'
    ROTATE_ACTOR = 'rotate'
    SET_EXPRESSION = 'expression'
    ATTACH_PROP = 'attach_prop'
    DETACH_PROP = 'detach_prop'
    LOOK_AT = 'look_at'
    MOVE_CAMERA = 'move_camera'
    CAMERA_ZOOM = 'zoom'
    FADE_IN = 'fade_in'
    FADE_OUT = 'fade_out'
    CALLBACK = 'callback'


@dataclass
class Action:
    time: float
    action_type: ActionType
    duration: float = 0.0
    easing: Callable[[float], float] = lambda t: t
    actor: Any = None
    target: Any = None
    scale: float | None = None
    angle: float | None = None
    expression: str | None = None
    prop: Any = None
    prop_anchor: str = 'center'
    prop_offset: tuple = (0, 0)
    look_target: Any = None
    camera_target: Any = None
    zoom_target: float | None = None
    alpha_target: float | None = None
    callback: Callable | None = None

    def end_time(self):
        return self.time + self.duration


def binding(action, camera):
    """Return (object, attribute, destination) for interpolated actions."""
    a, kind = action.actor, action.action_type
    fields = {
        ActionType.MOVE_ACTOR: (a, 'position', action.target),
        ActionType.SCALE_ACTOR: (a, 'scale', action.scale),
        ActionType.ROTATE_ACTOR: (a, 'rotation', action.angle),
        ActionType.MOVE_CAMERA: (camera, 'position', action.camera_target),
        ActionType.CAMERA_ZOOM: (camera, 'zoom', action.zoom_target),
        ActionType.FADE_IN: (a, 'alpha', 1.0 if action.alpha_target is None else action.alpha_target),
        ActionType.FADE_OUT: (a, 'alpha', 0.0 if action.alpha_target is None else action.alpha_target),
    }
    return fields.get(kind)


def validate(action, camera):
    if not np.isfinite(action.time) or action.time < 0:
        raise ValueError('Action time must be finite and nonnegative')
    if not np.isfinite(action.duration) or action.duration < 0:
        raise ValueError('Action duration must be finite and nonnegative')
    if not callable(action.easing):
        raise ValueError('Easing must be callable')
    b = binding(action, camera)
    if b is not None:
        obj, attr, end = b
        if obj is None or end is None:
            raise ValueError(f'{action.action_type.value} requires a target object and value')
        value = np.asarray(end, dtype=float)
        if not np.all(np.isfinite(value)) or value.shape != ((2,) if attr == 'position' else ()):
            raise ValueError(f'Invalid {attr} target')
        if attr in ('scale', 'zoom') and end <= 0:
            raise ValueError(f'{attr} must be positive')
        if attr == 'alpha' and not 0 <= end <= 1:
            raise ValueError('Alpha must be between 0 and 1')
    else:
        if action.duration:
            raise ValueError('Discrete actions cannot have a duration')
        if action.action_type == ActionType.CALLBACK:
            if not callable(action.callback):
                raise ValueError('CALLBACK requires a callable')
        elif action.actor is None:
            raise ValueError('Actor action requires an actor')
        elif action.action_type in (ActionType.ATTACH_PROP, ActionType.DETACH_PROP) and action.prop is None:
            raise ValueError('Prop action requires a prop')
        elif action.action_type == ActionType.SET_EXPRESSION and action.expression is None:
            raise ValueError('Expression is required')
        elif action.action_type == ActionType.LOOK_AT:
            if np.shape(action.look_target) != (2,) or not np.all(np.isfinite(action.look_target)):
                raise ValueError('look_target must be a finite world point')


def apply_discrete(action):
    a, k = action.actor, action.action_type
    if k == ActionType.SHOW_ACTOR:
        a.visible = True
    elif k == ActionType.HIDE_ACTOR:
        a.visible = False
    elif k == ActionType.SET_EXPRESSION:
        a.set_expression(action.expression)
    elif k == ActionType.ATTACH_PROP:
        a.attach_prop(action.prop, anchor=action.prop_anchor, offset=action.prop_offset)
    elif k == ActionType.DETACH_PROP:
        a.detach_prop(action.prop if isinstance(action.prop, str) else action.prop.name)
    elif k == ActionType.LOOK_AT:
        a.look_at(action.look_target)
    elif k == ActionType.CALLBACK:
        action.callback()
