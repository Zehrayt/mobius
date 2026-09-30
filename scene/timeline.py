"""Forward-only seconds clock with exact event boundaries and fixed tween origins.

Call update(t) with absolute, nondecreasing seconds. Skipped timestamps process
all intervening events, including tween endpoints. Later actions replace earlier
ones on the same property; equal-time events use insertion order. Build a fresh
scene to replay/seek backwards (callbacks may have external side effects).
"""
from __future__ import annotations
import copy
import math
import numpy as np
from .actions import Action, ActionType, binding, validate, apply_discrete


class Easing:
    linear = staticmethod(lambda t: t)
    ease_in_out = staticmethod(lambda t: 2*t*t if t < .5 else 1 - (-2*t+2)**2/2)
    ease_out = staticmethod(lambda t: 1 - (1-t)**2)
    ease_in_quad = staticmethod(lambda t: t*t)
    ease_in_out_quad = ease_in_out
    ease_out_quad = ease_out


class Timeline:
    def __init__(self, camera=None):
        self.camera = camera
        self.actions: list[Action] = []
        self.current_time = None
        self._next = 0
        self._active = {}

    def add_action(self, time, action_type, **kwargs):
        if self.current_time is not None:
            raise RuntimeError('Schedule actions before playback')
        action_type = ActionType(action_type)
        easing = kwargs.get('easing', Easing.linear)
        if isinstance(easing, str):
            if easing not in ('linear', 'ease_in_out', 'ease_out'):
                raise ValueError(f'Unknown easing: {easing}')
            kwargs['easing'] = getattr(Easing, easing)
        action = Action(float(time), action_type, **kwargs)
        validate(action, self.camera)
        self.actions.append(action)
        self.actions.sort(key=lambda a: a.time)
        return action

    at = add_action

    def tween(self, start, end, action_type, **kwargs):
        if end <= start:
            raise ValueError('Tween end must be later than start')
        return self.add_action(start, action_type, duration=end-start, **kwargs)

    def _sample(self, time):
        for key, (action, obj, attr, start, end) in list(self._active.items()):
            progress = self.get_tween_progress(action, time)
            setattr(obj, attr, self.lerp(start, end, progress))
            if time >= action.end_time():
                del self._active[key]

    def update(self, current_time):
        current_time = float(current_time)
        if not math.isfinite(current_time) or current_time < 0:
            raise ValueError('Time must be finite and nonnegative')
        if self.current_time is not None and current_time < self.current_time:
            raise ValueError('Timeline is forward-only; rebuild the scene to replay')
        fired = []
        while self._next < len(self.actions) and self.actions[self._next].time <= current_time:
            action = self.actions[self._next]
            self._sample(action.time)
            b = binding(action, self.camera)
            if b is None:
                apply_discrete(action)
            else:
                obj, attr, end = b
                key = (id(obj), attr)
                self._active.pop(key, None)
                if action.action_type == ActionType.FADE_IN:
                    obj.visible = True
                if action.duration == 0:
                    setattr(obj, attr, copy.deepcopy(end))
                else:
                    self._active[key] = (action, obj, attr, copy.deepcopy(getattr(obj, attr)), copy.deepcopy(end))
            self._next += 1
            fired.append(action)
        self._sample(current_time)
        self.current_time = current_time
        return fired

    @staticmethod
    def get_tween_progress(action, current_time):
        if not action.duration:
            return 1.0
        return action.easing(float(np.clip((current_time-action.time)/action.duration, 0, 1)))

    @staticmethod
    def lerp(start, end, t):
        result = np.asarray(start) + (np.asarray(end)-np.asarray(start))*t
        return float(result) if result.ndim == 0 else result
