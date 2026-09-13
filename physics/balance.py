"""
balance.py -- kütle merkezi (basitleştirilmiş "üst gövde" vekili) ile
destek tabanı arasındaki farka orantılı kol dengeleme yardımcıları
(bkz. demo/step12_balance.py).

Refactor notu (kullanıcının "sorumlulukların ayrılması" geri bildirimi
üzerine): bu hesap daha önce `step12_balance.py` içinde hardcode idi.
Mantık BİREBİR AYNI, buraya taşındı ki ileride başka bir demo/karakter
de aynı "CoM - destek farkı -> orantılı kol tepkisi" mekanizmasını
tekrar yazmadan kullanabilsin.

DÜRÜST SINIR: bu gerçek bir ters-dinamik (inverse dynamics)/rigid-body
coupling hesabı DEĞİLDİR -- basit bir orantılı (negatif) geri besleme.
Kollar "doğru yöne" hareket eder ama bu iskelette kolların kütlesi
gövdenin gerçek kütle merkezini fiziksel olarak geri çekecek kadar
büyük/bağlı değildir -- oyun animasyonlarında yaygın bir "ucuz tepki
jesti" (cheap reactive gesture) tekniğidir.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np


def upper_body_com_x(points: np.ndarray, indices: Sequence[int]) -> float:
    """Basitleştirilmiş üst-gövde kütle merkezi vekili: verilen nokta
    indekslerinin (ör. kalça+omuz+baş) yatay ortalaması. Tam kütle-
    ağırlıklı bir hesap DEĞİL (kol/bacak kütleleri dahil değil)."""
    return float(np.mean([points[i][0] for i in indices]))


def support_x(stance_x: Sequence[float], fallback_x: Sequence[float]) -> float:
    """Destek tabanının yatay konumu: o an zeminde duran (stance) ayağın/
    ayakların ortalama x'i. Hiçbiri stance değilse (nadir bir geçiş
    karesi) `fallback_x`'in ortalamasına düşer."""
    if stance_x:
        return float(np.mean(stance_x))
    return float(np.mean(fallback_x))


def counter_balance_offset(error: float, gain_x: float, gain_y: float, max_err: float) -> tuple[float, float]:
    """`error = com_x - support_x` büyüklüğüne orantılı (kırpılmış) bir
    kol düzeltme ofseti döndürür: `(yatay, dikey)`. Yatay ofset hatanın
    TERSİ yönünde (geri/karşı denge); dikey ofset her zaman yukarı
    (kolları kaldırmak dönme ataletini arttırıp dengeyi kolaylaştırır --
    gerçek bir insan refleksine benzer bir sezgisel, ama tam bir rigid-
    body hesaba bağlanmıyor, bkz. modül notu)."""
    clipped = float(np.clip(error, -max_err, max_err))
    bal_x = -clipped * gain_x
    bal_y = -min(abs(error), max_err) * gain_y
    return bal_x, bal_y
