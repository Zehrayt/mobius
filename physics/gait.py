"""
gait.py -- FABRIK tabanlı "ayak basma" (foot-planting) yürüyüş state machine'i.

Bu dosyanın tamamı bu projede sıfırdan yazılmıştır (üçüncü taraf bir
projeden alınmamıştır). `demo/step2_leg_reach.py` içinde prototiplenen
Leg sınığı, step3'te de aynı mantığı tekrar kullanabilmek için buraya
genelleştirilerek taşındı.

Mantık: bir bacak iki fazdan oluşur --
  stance : ayak yerde sabit durur, kalça üzerinden geçer.
  swing  : ayak, kalça hedeften (STRIDE_RELEASE kadar) geride kalınca
           kalkar, kalçanın (STRIDE_AHEAD kadar) ilerisindeki yeni basış
           noktasına -- hafif bir kaldırma eğrisiyle (LIFT_HEIGHT) -- taşınır.
Hiçbir kare elle anahtarlanmaz (keyframe yok); zamanlama tamamen kalçanın
o anki konumuna göre tetiklenir.
"""
from __future__ import annotations

import numpy as np

from physics.fabrik import FabrikChain2D


def _smoothstep(t: float) -> float:
    """Swing fazının x ilerlemesi için ease-in/ease-out eğrisi (3t^2 - 2t^3).
    Ayak kalkarken sıfırdan, inerken sıfıra yumuşak hızlanıp yavaşlar --
    sabit hızlı (lineer) bir süpürme yerine gerçek bir bacağın atalet/kas
    ivmesine daha yakın bir his verir. Bu da saat/zaman tabanlı bir eğri
    DEĞİL: girdi olarak yine bacağın kendi swing_t ilerlemesini alıyor,
    sadece o ilerlemeyi lineer yerine ease-in/out olarak yeniden eşliyor."""
    return t * t * (3.0 - 2.0 * t)


class FootPlantingLeg:
    def __init__(
        self,
        hip_pos: np.ndarray,
        segment_lengths: list[float],
        ground_y: float,
        stride_release: float = 18.0,
        stride_ahead: float = 28.0,
        swing_duration_frames: int = 10,
        lift_height: float = 22.0,
        initial_planted_offset: float = 0.0,
        knee_limits: tuple[float, float] | None = None,
        knee_bend_sign: float = 1.0,
    ):
        self.chain = FabrikChain2D(hip_pos, segment_lengths)
        self.ground_y = ground_y
        self.stride_release = stride_release
        self.stride_ahead = stride_ahead
        self.swing_duration_frames = swing_duration_frames
        self.lift_height = lift_height
        self.knee_limits = knee_limits
        self.knee_bend_sign = knee_bend_sign

        self.state = "stance"
        self.planted = np.array([hip_pos[0] + initial_planted_offset, ground_y])
        self.swing_start = self.planted.copy()
        self.swing_target = self.planted.copy()
        self.swing_t = 0.0

    def update(self, hip_pos: np.ndarray) -> np.ndarray:
        if self.state == "stance":
            foot = self.planted
            if hip_pos[0] - self.planted[0] > self.stride_release:
                self.state = "swing"
                self.swing_start = self.planted.copy()
                self.swing_target = np.array([hip_pos[0] + self.stride_ahead, self.ground_y])
                self.swing_t = 0.0
                foot = self.swing_start
        else:
            self.swing_t += 1.0 / self.swing_duration_frames
            t = min(self.swing_t, 1.0)
            x_t = _smoothstep(t)
            x = self.swing_start[0] + (self.swing_target[0] - self.swing_start[0]) * x_t
            lift = np.sin(np.pi * t) * self.lift_height
            foot = np.array([x, self.ground_y - lift])
            if self.swing_t >= 1.0:
                self.planted = self.swing_target.copy()
                self.state = "stance"
                foot = self.planted

        self.chain.set_base(hip_pos)
        self.chain.solve(foot)
        if self.knee_limits is not None:
            self.chain.clamp_joint_angles(*self.knee_limits, bend_sign=self.knee_bend_sign)
        return foot
