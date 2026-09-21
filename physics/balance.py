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

7. TUR EKİ (kullanıcı talebi -- "Aktif Denge ve Refleks / Center of Mass
Recovery"): yukarıdaki `counter_balance_offset` SÜREKLİ, küçük-genlikli bir
kozmetik jesttir -- gerçek bir "tehlike" eşiği yoktur, `BAL_MAX_ERR`'de
erken doyar ve GERÇEK bir "düşme" durumunda (destek tabanının tamamen
dışına çıkma) hiçbir ekstra tepki üretmez; adım zamanlaması da salt kalça-
mesafesi eşiğine bağlıdır (bkz. `physics/gait.py`), dengeye hiç bakmaz.
Bu bölümdeki yeni fonksiyonlar bunun üzerine (ONU DEĞİŞTİRMEDEN) bir
"tehlike tespiti" + "büyütülmüş tepki" katmanı ekliyor:
  - `support_interval()`: destek tabanını artık TEK bir nokta değil, o an
    zeminde duran ayağın/ayakların fiziksel genişliğini de sayan bir
    ARALIK olarak modelliyor.
  - `outside_interval_error()`: kütle merkezinin bu aralığın GERÇEKTEN
    dışına çıkıp çıkmadığını (0 = güvenlii) ölçüyor -- eski `error =
    com_x - support_x` tek-nokta farkından farkı olarak, aralık İÇİNDE
    kalan normal yürüyüş salınımını "tehlike" saymıyor.
  - `FallRiskMonitor`: histerezisli (giriş/çıkış eşiği farklı) bir durum
    makinesi -- tek bir gürültülü kare "tehlike"yi tetikleyip hemen
    kapatmasın diye (bkz. `gait.py`'deki `FootPlantingLeg` state machine
    ile aynı ruh).
  - `emergency_counter_balance_offset()`: `counter_balance_offset` ile
    AYNI şekil/yön mantığı, ama gerçek tehlike büyüklüğüne göre ölçeklenen
    daha büyük kazanç/tavan (kolları "ters yöne savurma" -- kullanıcının
    kendi tabiriyle).
DÜRÜST SINIR: bu hâlâ gerçek bir ters-dinamik/rigid-body denge kontrolcüsü
DEĞİL -- eşik tabanlı bir refleks anahtarlama katmanı. `support_interval`
ayak GENİŞLİĞİNİ sabit bir `FOOT_HALF_LEN` sabitiyle yaklaşıklıyor (gerçek
ayak geometrisi/ağırlık merkezi hesaba katılmıyor); `outside_interval_
error` yalnızca ANLIK konuma bakıyor, gerçek biyomekanikteki "extrapolated
center of mass" (hız/momentum projeksiyonu) gibi bir ÖNGÖRÜ içermiyor --
yani düşme hızlıysa geç kalabilir. Bkz. `demo/step12_balance.py` ve
`README.md` → "7. tur" için ölçümler ve ekstra-adım entegrasyonu.
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


def reach_pulldown_offset(
    overrun_px: float,
    gain_x: float,
    gain_y: float,
    max_offset: float,
    travel_dir: float = 1.0,
) -> tuple[float, float]:
    """Kullanıcı geri bildirimi -- "kemik esnemesi / kütle merkezi
    bağlantısızlığı": bir bacağın hedef ayak konumu kendi menzilini
    (`FootPlantingLeg.last_overrun_px`) aştığında, MİMARİ OLARAK DOĞRU
    tepki bacağın kendisini (sabit uzunluklu segmentleri) germek DEĞİL,
    kalçanın/gövdenin (basitleştirilmiş CoM) hedefe doğru AŞAĞI ve İLERİ
    eğilmesidir -- tıpkı gerçek bir insanın erişemediği bir adımda
    belini/kalçasını öne-aşağı sarkıtması gibi.

    `overrun_px` kadar bir asma miktarını, sonraki karede uygulanacak bir
    (yatay, dikey) kalça ofsetine çevirir -- `travel_dir` (+1/-1) yürüyüş
    yönünü belirtir (ileri ofset bu yönde uygulanır). `max_offset` ile
    kırpılır ki tek bir aşırı-erişim karesi kalçayı gerçekçi olmayan bir
    mesafeye fırlatmasın.

    DÜRÜST SINIR: `counter_balance_offset` gibi bu da gerçek bir ters-
    dinamik hesap DEĞİL -- orantılı bir düzeltme. Ama `counter_balance_
    offset`'ten farklı olarak kolları değil, doğrudan kalçanın/gövdenin
    hedef konumunu etkiler (bkz. çağıran kod: `driver` hedefine eklenir)."""
    magnitude = float(np.clip(overrun_px, 0.0, max_offset))
    return magnitude * gain_x * travel_dir, magnitude * gain_y

# ---------------------------------------------------------------------
# 7. tur eki: destek-poligonu tabanlı tehlike tespiti + büyütülmüş refleks
# (bkz. modül dokstring'inin "7. TUR EKİ" bölümü)
# ---------------------------------------------------------------------

FOOT_HALF_LEN = 12.0  # gerçekçi bir ayağın yaklaşık yarı-uzunluğu (px) --
                      # tek bir sabit, bu iskeletin ayak geometrisi yok.


def support_interval(stance_foot_x: Sequence[float], foot_half_len: float = FOOT_HALF_LEN,
                      fallback_x: Sequence[float] = ()) -> tuple[float, float]:
    """O an ZEMİNDE DURAN (stance) ayağın/ayakların x konumlarından gerçek
    bir destek ARALIĞI (interval) hesaplar: `(min(stance_x) - foot_half_len,
    max(stance_x) + foot_half_len)`. Tek ayak stance'taysa bile ayağın
    kendi fiziksel uzunluğu kadar bir pay veriyor -- eski `support_x()`'in
    tek NOKTA varsayımından (gerçek bir ayağın sıfır genişliği yokmuş gibi
    davranması) farkı bu.

    `stance_foot_x` boşsa (ikisi de swing -- nadir bir geçiş karesi)
    `fallback_x`'in ortalaması etrafında aynı yarı-genişlikte bir aralık
    döner (destek tabanının GEÇİCİ olarak belirsiz olduğu bu anlarda kaba
    bir tahmin -- gerçek bir çift-havada anı, insan yürüyüşünde de kısa
    süreli belirsizdir)."""
    if stance_foot_x:
        return (min(stance_foot_x) - foot_half_len, max(stance_foot_x) + foot_half_len)
    mid = float(np.mean(list(fallback_x))) if fallback_x else 0.0
    return (mid - foot_half_len, mid + foot_half_len)


def outside_interval_error(com_x: float, interval: tuple[float, float]) -> float:
    """Kütle merkezinin `interval = (lo, hi)` destek aralığının GERÇEKTEN
    dışına çıkma mesafesi. İçerideyse `0.0` (normal yürüyüş salınımı --
    hatta CoM aralığın kenarına yakın olsa bile -- burada "tehlike"
    SAYILMAZ, bu `counter_balance_offset`'in sürekli küçük tepkisinden
    farkı). Pozitif = aralığın ÖNÜNDE (ileri düşme riski), negatif =
    ARKASINDA (geriye düşme riski)."""
    lo, hi = interval
    if com_x < lo:
        return com_x - lo
    if com_x > hi:
        return com_x - hi
    return 0.0


class FallRiskMonitor:
    """Histerezisli tehlike durumu makinesi: `outside_interval_error()`'ün
    MUTLAK değeri `enter_px`'i aşınca tehlikeye girer, `exit_px`'in ALTINA
    inince çıkar (`exit_px < enter_px` olmalı) -- tek bir gürültülü karenin
    tehlikeyi tetikleyip hemen kapatmasını (çırpınma/"chatter") önler,
    `physics/gait.py`'deki `FootPlantingLeg` durum makinesiyle aynı
    tasarım ilkesi.

    DÜRÜST SINIR: sadece ANLIK `error` büyüklüğüne bakıyor, hız/ivme
    (momentum) hesaba katmıyor -- bkz. modül dokstring'i."""

    def __init__(self, enter_px: float, exit_px: float):
        assert exit_px < enter_px, "exit_px, enter_px'ten kucuk olmali (histerezis)"
        self.enter_px = enter_px
        self.exit_px = exit_px
        self.in_danger = False

    def update(self, error: float) -> bool:
        mag = abs(error)
        if self.in_danger:
            if mag < self.exit_px:
                self.in_danger = False
        else:
            if mag > self.enter_px:
                self.in_danger = True
        return self.in_danger


def emergency_counter_balance_offset(error: float, gain_x: float, gain_y: float,
                                      max_err: float) -> tuple[float, float]:
    """`counter_balance_offset` ile AYNI şekil/yön mantığı (hatanın TERSİ
    yönünde yatay ofset + her zaman yukarı dikey ofset), ama çağıran kodun
    (bkz. `demo/step12_balance.py`) `FallRiskMonitor` tehlike bildirdiğinde
    kullanması amaçlanan, çok daha büyük `gain_x`/`max_err` değerleriyle --
    normal yürüyüş salınımının küçük kozmetik jestinden ayrı, "kollarını
    ters yöne savurma" büyüklüğünde bir tepki. Fonksiyonun kendisi
    `counter_balance_offset`'in birebir aynısı -- ayrı bir isim + ayrı
    parametre seti olarak tutulması, çağıranın normal/acil durumu net
    şekilde ayırt edebilmesi için (iki farklı sabit seti, bkz. step12)."""
    return counter_balance_offset(error, gain_x, gain_y, max_err)

