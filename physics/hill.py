"""
hill.py -- Adim 28: Hill kas modeli, kuvvet-hiz iliskisi (boyutsuz).

Konsantrik (kas kisaliyor, hiz s >= 0):
    f(s) = (1 - s/vmax) / (1 + s/(k vmax))            k = 0.25 (egrilik)
Eksantrik (kas zorla uzuyor, s < 0) -- van Soest & Bobbert (1993) formu:
    f(s) = 1.5 - 0.5 (1 + s/vmax) / (1 - 7.56 s/(k vmax))
f(0) = 1 (izometrik), s -> vmax'ta f -> 0, eksantrikte f -> ~1.48 platosu.

Hiz burada EKLEM acisal hizi (rad/kare), vmax eklemin en buyuk acisal hizi.
Insan diz/kalca ekstansor-fleksorleri icin ~10-12 rad/s; 30 fps'de 0.4 rad/kare.
"""
from __future__ import annotations

HILL_K = 0.25
JOINT_VMAX = 0.4      # rad/kare (~12 rad/s)


def force_velocity(s: float, vmax: float = JOINT_VMAX, k: float = HILL_K) -> float:
    if vmax <= 0.0:
        return 1.0
    if s >= 0.0:
        if s >= vmax:
            return 0.0
        return (1.0 - s / vmax) / (1.0 + s / (k * vmax))
    s = max(s, -vmax)
    return 1.5 - 0.5 * (1.0 + s / vmax) / (1.0 - 7.56 * s / (k * vmax))
