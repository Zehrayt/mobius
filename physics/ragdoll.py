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


def lerp_blend(blend: float, active_val: float, passive_val: float) -> float:
    """Herhangi bir sayısal parametreyi `blend`'e göre `active_val`
    (blend=1) ile `passive_val` (blend=0) arasında doğrusal karıştırır.

    DÜZELTME (3. tur ek -- omurga yay-sönümleme): önceden bu formül
    SADECE `blended_max_angle()` içinde, açı sınırına özel olarak
    yazılıydı. Şimdi aynı formülü yay sabiti (stiffness) ve sönümleme
    (damping) için de kullanmamız gerekti (bkz. `apply_angular_spring()`),
    bu yüzden jenerik bir isimle buraya çıkarıldı -- `blended_max_angle()`
    davranışı BİREBİR AYNI kalacak şekilde bunu çağırıyor (aşağıda)."""
    return passive_val - blend * (passive_val - active_val)


def blended_max_angle(blend: float, active_max_deg: float, passive_max_deg: float = 180.0) -> float:
    """`clamp_direction()`'a verilecek maksimum sapma açısını `blend`'e
    göre ayarlar -- `blend=1`'de dar (aktif/kontrollü duruş),
    `blend=0`'da pratik olarak sınırsız (pasif/serbest -- gövde/boyun
    stabilizasyonu da IK ile BİRLİKTE devre dışı kalır)."""
    return lerp_blend(blend, active_max_deg, passive_max_deg)


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


def transition_impulse_vector(
    current_velocity: np.ndarray,
    velocity_gain: float,
    up_kick: float,
) -> np.ndarray:
    """Aktif->pasif (ragdoll) geçiş ANINDA enjekte edilecek tek seferlik
    darbe (impulse) vektörünü hesaplar.

    Neden gerekli (kullanıcı geri bildirimi -- "aktiften pasife hatali
    gecis / momentum aktarilmiyor"): gecis oncesi kod SADECE friction'i
    (blended_friction) ve govde/boyun sertligini (blended_max_angle)
    yumusatiyordu -- karakterin O ANKI hizina hicbir sey EKLEMIYORDU,
    yani ragdoll karakterin kendi mevcut yuruyus hizinin dogal
    surtunmeyle sonumlenmesiyle basliyordu (olculdu: kalca vx gecisten
    ~0.3s sonra 1.85'ten 0.34'e friction ile duz bir sekilde dusuyor) --
    bu, bir "darbe" hissinden cok yumusak bir "rolantiye alma" gibi
    goruluyor.

    Bu fonksiyon, karakterin KENDI o anki vektorel hizini (`current_velocity`,
    yon dahil) alip `velocity_gain` ile buyuterek (kullanicinin tam olarak
    istedigi sey: "karakterin vektorel hizini ragdoll dugumlerine aktar")
    + kucuk bir dikey "kalkis" bileseni (`up_kick`, negatif = yukari,
    gercek bir darbenin oturttugu havaya kalkma hissi icin) dondurur.

    DURUST NOT: bu demoda (step9/step13) "darbe"nin nedeni ile ilgili
    fiziksel bir carpisma (ör. bagimsiz topa cikma) YOK -- KNOCKDOWN_T
    sadece zamanlanmis bir tetikleyici (bkz. modul dokstring'i). Yani
    buradaki impulse GERCEK bir carpismadan turetilen bir tepki-kuvveti
    degil, "karakter bir seye carpmis GIBI davransin" diye elle
    tasarlanmis, karakterin kendi hizina orantili sabit bir darbe."""
    return current_velocity * velocity_gain + np.array([0.0, up_kick])


def apply_impulse(points: np.ndarray, prev_points: np.ndarray, indices: list[int], impulse: np.ndarray) -> None:
    """Verilen noktaların `prev_points`'ini `impulse` kadar GERİYE kaydırır
    -- Verlet hızı `points - prev_points` olduğu için bu, bir sonraki
    `step()`'te noktaya ANINDA `+impulse` kadar ek hız kazandırmış olur
    (mevcut hızın ÜSTÜNE eklenir, üzerine yazmaz)."""
    for i in indices:
        prev_points[i] = prev_points[i] - impulse
