"""
ragdoll.py -- aktif (IK) / pasif (ragdoll) fizik harmanı için paylaşılan
yardımcılar (bkz. demo/step9_ragdoll_blend.py).

Refactor notu (kullanıcının "sorumlulukların ayrılması" geri bildirimi
üzerine): bu blend mantığı daha önce `step9_ragdoll_blend.py` içinde
hardcode idi (her nokta için elle `pas*(1-blend)+active*blend` yazımı,
`clamp_direction` açısının elle hesaplanan bir `max_lean` ile
gevşetilmesi, `driver`'ın elle hesaplanan bir hedefe pinlenmesi, vb.).
Aynı mantık BİREBİR AYNI davranışla (aynı formüller, sadece isimlendirilip
tek bir yere taşındı) buraya çıkarıldı ki gelecekte başka bir demo/karakter
de aynı aktif/pasif harman mekanizmasını tekrar yazmadan kullanabilsin.
"""
from __future__ import annotations

import numpy as np


def blend_point(passive_pos: np.ndarray, active_pos: np.ndarray, blend: float) -> np.ndarray:
    """`blend`=1 -> tam aktif (IK'nin önerdiği konum), 0 -> tam pasif
    (fiziğin kendi başına ürettiği konum). İkisi arasında lineer karışım."""
    return passive_pos * (1.0 - blend) + active_pos * blend


def blend_prev_points(prev_pos: np.ndarray, blended_pos: np.ndarray, blend: float) -> np.ndarray:
    """`blend=1`'de `prev_points` de tam senkron edilir -- IK'nin ani
    konum düzeltmesinin sahte bir hız/yerçekimi tepmesi yaratmaması
    için (`VerletSystem.set_pinned_position()` ile aynı mantık).
    `blend=0`'da `prev_points`'e hiç dokunulmaz -- fizik kendi
    momentumuyla devam eder. Ara değerlerde bu iki uç arasında
    kısmi (orantılı) bir senkronizasyon uygulanır."""
    return prev_pos * (1.0 - blend) + blended_pos * blend


def blended_max_angle(blend: float, active_max_deg: float, passive_max_deg: float = 180.0) -> float:
    """`clamp_direction()`'a verilecek maksimum sapma açısını `blend`'e
    göre ayarlar -- `blend=1`'de dar (aktif/kontrollü duruş),
    `blend=0`'da pratik olarak sınırsız (pasif/serbest -- gövde/boyun
    stabilizasyonu da IK ile BİRLİKTE devre dışı kalır)."""
    return passive_max_deg - blend * (passive_max_deg - active_max_deg)


def blended_friction(blend: float, active_friction: float, passive_friction: float) -> float:
    """`VerletSystem.friction`'ı `blend`'e göre ayarlar -- pasif modda
    genellikle daha yüksek tutulur (ek bir eklem sönümlemesi olmadığı
    için enerjinin makul bir sürede sönümlenmesine yardımcı olur;
    bkz. `demo/step9_ragdoll_blend.py`'deki dürüst sınır notu)."""
    return blend * active_friction + (1.0 - blend) * passive_friction


def driver_follow_target(hip_last_pos: np.ndarray, walk_target: np.ndarray, blend: float) -> np.ndarray:
    """Pinned "driver" noktasının hedef konumu: `blend=1`'de her zamanki
    gibi yürüyüşün belirlediği hedefe, `blend=0`'da kalçanın KENDİ bir
    önceki karedeki fizik-konumuna -- yani pratikte kalçayı da serbest
    bırakır (driver-hip çubuğu zaten sıfır mesafeye yakınsadığı için
    bu durumda hiçbir çekme kuvveti kalmaz)."""
    return hip_last_pos * (1.0 - blend) + walk_target * blend
