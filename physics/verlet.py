"""
verlet.py -- genel amaçlı nokta/çubuk (point-stick) Verlet integration sistemi.

Kaynak / Attribution
---------------------
Bu modül, https://github.com/austinweis/python-verlet-integration
projesindeki `src/rag.py` dosyasından (Apache License 2.0) uyarlanmıştır.
Orijinal lisans metni: third_party_licenses/LICENSE-apache-2.0-python-verlet-integration.txt

Orijinale göre yapılan değişiklikler (Apache-2.0 madde 4(b) gereği belirtilir):
  - Ham Python listeleri yerine numpy array temsili kullanıldı.
  - Sınır (bounds) çarpışması opsiyonel hale getirildi (video modunda zemin
    dışında bir sınıra ihtiyaç yok).
  - Tip belirteçleri (type hints) ve dataclass tabanlı bir API eklendi.
  - `Rag` sınıfı `VerletSystem` olarak yeniden adlandırıldı ve dış modüllerin
    (fabrik.py, demo scriptleri) kullanacağı şekilde küçük yardımcı metodlar
    eklendi (`pin`, `add_point`, `add_stick`).
  - `clamp_direction()` eklendi (orijinalde yok) -- serbest bırakılan çok
    parçalı bir verlet zinciri (ör. kalça->omuz->baş) fizikte iyi bilinen
    bir sorun olan "kaotik çift sarkaç" (double pendulum) davranışına
    girip takla atabilir; bu fonksiyon bir segmentin yönünü sabit bir
    referans vektöre göre sınırlayarak gövde/boyun gibi yarı-rijit
    parçaları kararlı tutar.

Algoritma (Störmer-Verlet integration + distance-constraint relaxation)
kavramsal olarak Thomas Jakobsen'in "Advanced Character Physics" makalesine
dayanır ve birçok açık kaynak projede aynı şekilde uygulanır; burada asıl
alınan şey orijinal projenin temiz Python API tasarımıdır.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

GRAVITY = np.array([0.0, 0.05])
FRICTION = 0.02
RELAX_ITERS = 8


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm < 1e-9:
        return vector
    return vector / norm


def clamp_direction(points: np.ndarray, prev_points: np.ndarray,
                     i_anchor: int, i_free: int,
                     reference_dir: np.ndarray, max_deviation_deg: float) -> None:
    """`points[i_anchor] -> points[i_free]` segmentinin yönünü sabit bir
    `reference_dir` vektörüne göre `max_deviation_deg` ile simetrik olarak
    sınırlar (segment uzunluğu korunur). `prev_points[i_free]` de yeni
    konuma eşitlenir ki clamp bir sonraki karede sahte bir hız (velocity)
    tepmesi yaratmasın.

    NOT: Referans olarak *başka bir serbest noktanın* yönünü (ör. çok kısa
    ve bu yüzden sayısal olarak gürültülü bir "driver->hip" vektörünü)
    kullanmak kararsızdır -- küçük konum gürültüleri büyük açı sıçramalarına
    dönüşür. Bunun yerine sabit bir global yön (ör. yukarı = (0,-1)) ya da
    zaten bu fonksiyonla stabilize edilmiş bir önceki segmentin yönü
    verilmelidir.

    Kullanım: serbest (pinned olmayan) çok parçalı bir zincirin (ör.
    kalça->omuz->baş) fizikte "çift sarkaç" (double pendulum) gibi kaotik
    dönmesini engelleyip yarı-rijit bir gövde/boyun hissi vermek için
    `VerletSystem.step()` çağrısından hemen sonra kullanılır.
    """
    reference_dir = _unit(np.asarray(reference_dir, dtype=float))
    vec = points[i_free] - points[i_anchor]
    length = float(np.linalg.norm(vec))
    direction = vec / (length + 1e-9)

    cross_z = reference_dir[0] * direction[1] - reference_dir[1] * direction[0]
    dot = float(np.clip(np.dot(reference_dir, direction), -1.0, 1.0))
    signed_angle = float(np.degrees(np.arctan2(cross_z, dot)))

    clamped = float(np.clip(signed_angle, -max_deviation_deg, max_deviation_deg))
    if clamped == signed_angle:
        return

    theta = np.radians(clamped)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    rotated = np.array([
        reference_dir[0] * cos_t - reference_dir[1] * sin_t,
        reference_dir[0] * sin_t + reference_dir[1] * cos_t,
    ])
    points[i_free] = points[i_anchor] + rotated * length
    prev_points[i_free] = points[i_free]


@dataclass
class VerletSystem:
    """Noktalar (points) ve aralarındaki mesafe kısıtlamalarından (sticks)
    oluşan genel bir Verlet integration sistemi. Zincir, ağaç ya da
    kapalı iskelet (torso + kollar + bacaklar) gibi keyfi graf yapılarını
    destekler.
    """

    points: np.ndarray            # (N, 2) güncel pozisyonlar
    prev_points: np.ndarray       # (N, 2) bir önceki adımın pozisyonları
    sticks: list[tuple[int, int, float]] = field(default_factory=list)
    pinned: set[int] = field(default_factory=set)
    gravity: np.ndarray = field(default_factory=lambda: GRAVITY.copy())
    friction: float = FRICTION

    @classmethod
    def empty(cls) -> "VerletSystem":
        return cls(points=np.zeros((0, 2)), prev_points=np.zeros((0, 2)))

    def add_point(self, pos: Sequence[float], pinned: bool = False) -> int:
        idx = len(self.points)
        pos = np.asarray(pos, dtype=float).reshape(1, 2)
        self.points = np.vstack([self.points, pos]) if self.points.size else pos.copy()
        self.prev_points = np.vstack([self.prev_points, pos]) if self.prev_points.size else pos.copy()
        if pinned:
            self.pinned.add(idx)
        return idx

    def add_stick(self, i: int, j: int, length: float | None = None) -> None:
        if length is None:
            length = float(np.linalg.norm(self.points[j] - self.points[i]))
        self.sticks.append((i, j, length))

    def pin(self, idx: int, pos: Sequence[float] | None = None) -> None:
        """Bir noktayı sabitler (statics). pos verilirse önce oraya taşır."""
        if pos is not None:
            self.points[idx] = pos
            self.prev_points[idx] = pos
        self.pinned.add(idx)

    def set_pinned_position(self, idx: int, pos: Sequence[float]) -> None:
        """Pinlenmiş bir noktayı her karede dışarıdan (ör. bir anchor script'i)
        güncellemek için kullanılır."""
        self.points[idx] = pos

    def step(self, dt: float = 1.0) -> None:
        self._integrate(dt)
        for _ in range(RELAX_ITERS):
            self._satisfy_sticks()
            self._reapply_pins()

    # -- iç adımlar -----------------------------------------------------
    def _integrate(self, dt: float) -> None:
        velocity = (self.points - self.prev_points) * max(1.0 - self.friction, 0.0)
        new_points = self.points + velocity + self.gravity * (dt * dt)
        self.prev_points = self.points.copy()
        self.points = new_points
        self._reapply_pins()

    def _reapply_pins(self) -> None:
        for idx in self.pinned:
            self.prev_points[idx] = self.points[idx]

    def _satisfy_sticks(self) -> None:
        for i, j, rest_length in self.sticks:
            pi, pj = self.points[i], self.points[j]
            delta = pj - pi
            dist = float(np.linalg.norm(delta)) + 1e-9
            diff = (dist - rest_length) / dist
            i_pinned = i in self.pinned
            j_pinned = j in self.pinned
            if i_pinned and j_pinned:
                continue
            if i_pinned:
                self.points[j] -= delta * diff
            elif j_pinned:
                self.points[i] += delta * diff
            else:
                self.points[i] += delta * diff * 0.5
                self.points[j] -= delta * diff * 0.5
