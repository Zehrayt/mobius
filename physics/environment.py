"""
environment.py -- sahne/dünya ile ilgili, karakterin kendi iskeletinden
BAĞIMSIZ öğeler: zemin/temas bölgeleri (`Terrain`) ve rüzgar gust
üreteci (`GustWind`).

Refactor notu (kullanıcının "sorumlulukların ayrılması" geri bildirimi
üzerine): daha önce her demo kendi `floor_fn`/`friction_fn` kapanışını
(closure) ve kendi `wind_x(t)` fonksiyonunu ayrı ayrı, birbirinden
habersiz tanımlıyordu (`demo/step7_squash_stretch.py`,
`step8_collision_friction.py`, `step10_secondary_wind.py`'nin bu
refactor öncesi halleri git geçmişinde duruyor). Mantık BİREBİR AYNI --
sadece tek, parametrize edilebilir bir yerde topluca tanımlandı.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Terrain:
    """Düz (x'e bağlı olmayan) bir zemin yüksekliği + x aralığına göre
    değişen temas sürtünmesi bölgelerinden oluşan basit bir zemin
    temsili. `physics/collision.collide_ground()`'un beklediği
    `floor_fn`/`friction_fn` çağrılabilirlerini üretir.

    `zones`: `(x0, x1, friction)` üçlülerinden bir liste -- bir x bu
    aralıklardan birine düşerse o bölgenin sürtünmesi, düşmezse
    `default_friction` kullanılır. Zemin yüksekliği şu an sabit
    (`ground_y`) -- x'e bağlı bir yükseklik (basamak/rampa) ileride
    `zones` ile aynı desende bir `height_zones` alanıyla eklenebilir,
    ama bu henüz hiçbir demoda ihtiyaç olmadığı için eklenmedi (YAGNI)."""

    ground_y: float
    default_friction: float
    zones: list[tuple[float, float, float]] = field(default_factory=list)

    def floor_fn(self, x: float) -> float:
        return self.ground_y

    def friction_fn(self, x: float) -> float:
        for x0, x1, friction in self.zones:
            if x0 <= x <= x1:
                return friction
        return self.default_friction


@dataclass
class GustWind:
    """Birkaç uyumsuz (incommensurate) sinüs frekansının toplamı + sabit
    bir taban değer + bir başlangıç/ramp süresi -- düzensiz "gust"
    rüzgarını taklit eder. BİLEREK tek bir periyodik `sin(t)` DEĞİL
    (bkz. `physics/verlet.py`'deki `wind`/`wind_scale` notu): bu, dış bir
    ÇEVRESEL kuvveti sürüyor, karakterin kendi kinematiğini değil --
    projenin "sin(time) karakterin kendi hareketini değil, dış bir
    çevresel etkiyi sürebilir" ilkesine uygun.

    `components`: `(genlik, frekans_hz, faz)` üçlülerinden bir liste."""

    base: float
    components: list[tuple[float, float, float]] = field(default_factory=list)
    start_t: float = 0.0
    ramp_t: float = 1.0

    def value(self, t: float) -> float:
        if t < self.start_t:
            return 0.0
        gust = sum(
            amp * np.sin(2 * np.pi * freq * t + phase)
            for amp, freq, phase in self.components
        )
        raw = self.base + gust
        ramp = min((t - self.start_t) / self.ramp_t, 1.0) if self.ramp_t > 0 else 1.0
        return float(raw * ramp)
