"""
fabrik.py -- 2D FABRIK (Forward And Backward Reaching Inverse Kinematics) çözücü.

Kaynak / Attribution
---------------------
Bu modül, https://github.com/Yanneeh/Fabrik-Inverse-kinematics
projesindeki `fabrikSolver.py` dosyasındaki `Segment2D` / `FabrikSolver2D`
sınıflarından (MIT License, Copyright (c) 2020 Yannick van Diermeen)
uyarlanmıştır. Orijinal lisans metni:
third_party_licenses/LICENSE-mit-fabrik-inverse-kinematics.txt

MIT lisansı gereği yukarıdaki telif bildirimi ve lisans metni korunmuştur.

Orijinale göre yapılan değişiklikler:
  - 3D sınıflar (Segment3D / FabrikSolver3D) ve matplotlib görselleştirme
    (`plot`) kısmı kaldırıldı -- bu proje kendi OpenCV render pipeline'ını
    kullanıyor.
  - `max_iterations` limiti eklendi (orijinalde hedef ulaşılamaz durumda
    `compute()` sonsuz döngüye girebiliyordu).
  - Segment açıları yerine doğrudan `set_base()` ile her karede taban
    noktasının (ör. kalça/omuz) dışarıdan güncellenebilmesi eklendi --
    hareketli bir karakterde FABRIK zincirinin köküdür.
  - Değişken adları ve tip belirteçleri (type hints) eklendi.
  - `clamp_joint_angles()` eklendi (orijinalde yok) -- eklem açı sınırları
    (ör. dizin tersine bükülmemesi) için solve-sonrası bir clamp adımı.

FABRIK algoritmasının kendisi Aristidou & Lasenby (2011) tarafından
yayınlanmış, patentsiz, akademik olarak açık bir yöntemdir; burada MIT'den
alınan asıl şey orijinal projenin Python/numpy implementasyon detaylarıdır.
"""
from __future__ import annotations

import numpy as np


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm < 1e-9:
        return vector
    return vector / norm


def _circular_distance_deg(a_deg: float, b_deg: float) -> float:
    """Iki acinin (derece) DAIRESEL mesafesi -- 350 ile 10 arasindaki
    mesafe 340 degil 20 olmali (cember uzerinde kisa yol). `_nearest_valid_bend_deg`
    icin gerekli: bir aralikin (arc) UC NOKTALARINA olan gercek aci
    mesafesini bulmak icin, yoksa 3. tur'un "yansitma" (reflect) hatasi
    tekrarlanir (asagida acikliyor)."""
    d = abs(a_deg - b_deg) % 360.0
    return d if d <= 180.0 else 360.0 - d


def _nearest_valid_bend_deg(signed_angle_deg: float, min_bend_deg: float, max_bend_deg: float, bend_sign: float) -> float:
    """Bir eklemin buyuk sinyalli acisini (`signed_angle_deg`), gecerli
    "buyukluk" araligina ([min_bend_deg, max_bend_deg], isareti
    `bend_sign` ile sabit) EN KUCUK aci-mesafesi ile katlar.

    DUZELTME (3. tur EKI -- omurga yay-sonumleme calismasi sirasinda
    ragdoll'un bacaklarinin havada "asili/dolasmis" gorunmesi uzerine
    bulundu, bkz. commit mesaji): round-3'un "yansitma" (reflect)
    fonksiyonu -- `magnitude = clip(|signed_angle|, min, max); clamped =
    ±magnitude (bend_sign'a gore SABIT isaretle)` -- kucuk yanlis-taraf
    acilari icin dogru calisiyordu (ör. +5° -> -8°, 13° luk kucuk bir
    duzeltme) AMA BUYUK yanlis-taraf acilari icin KENDI SOYLEDIGI
    "sureklilik her kosulda korunur" iddiasini ihlal ediyordu: sayisal
    tanida (diag_fall2.py benzeri bir script) olculdu -- knockdown darbesi
    sonrasi dizin ham (kisitlanmamis) acisi fiziksel olarak surekli
    +82°'den +174°'ye yukseliyor (bacagin momentumla yukari savrulmasi,
    GERCEK bir fizik olayi), ama eski kod HER KAREDE bu buyuklugu KORUYUP
    isareti ZORLA ters cevirdigi icin (ör. +142.83° -> -142.83°) ayak,
    diz etrafinda tek karede ~74°-164° lik GORUNUR bir sicrama yapiyordu
    (+bu konum prev_points'e de yazildigi icin hiz sifirlaniyor, yani
    bacak birkaç kare boyunca havada "donuyor" gibi kaliyordu -- olculdu:
    sag ayak y=304px'ten y=138px'e ~0.4s icinde sicrayip oradan ~1.1s
    boyunca dusmuyor).

    Gercek duzeltme: "buyuklugu koru, isareti zorla" YERINE, gecerli
    araligin (bir [lo, hi] yay/arc) CEMBER UZERINDE EN YAKIN UC NOKTASINA
    katla -- bir araligin disindaki bir noktaya en yakin gecerli nokta
    HER ZAMAN o araligin iki ucundan biridir (temel bir geometri gercegi),
    magnitude'u degil ACI MESAFESINI minimize eder. Bu hem eski docstring'in
    KUCUK yanlis-taraf ornegini (+5° -> -8°, degismedi, asagida dogrulandi)
    HEM DE yukaridaki BUYUK yanlis-taraf/yuksek-hizli ragdoll durumunu
    (+142.83° -> -150° gibi -- SADECE ~67° luk bir katlama, ~164° yerine)
    dogru sekilde kapsiyor. Zaten gecerli bir aci (`lo <= signed <= hi`)
    HICBIR degisiklik olmadan aynen donuyor -- eski davranisla BIREBIR ayni."""
    if bend_sign >= 0:
        lo, hi = min_bend_deg, max_bend_deg
    else:
        lo, hi = -max_bend_deg, -min_bend_deg
    if lo <= signed_angle_deg <= hi:
        return signed_angle_deg
    if _circular_distance_deg(signed_angle_deg, lo) <= _circular_distance_deg(signed_angle_deg, hi):
        return lo
    return hi


class FabrikChain2D:
    """Sabit uzunluklu segmentlerden oluşan bir 2D IK zinciri (ör. bir bacak:
    kalça -> diz -> ayak bileği)."""

    def __init__(self, base: np.ndarray, segment_lengths: list[float], margin_of_error: float = 0.5):
        self.base = np.asarray(base, dtype=float)
        self.lengths = list(segment_lengths)
        self.arm_length = float(sum(self.lengths))
        self.margin_of_error = margin_of_error

        # Başlangıçta zinciri tabandan aşağı doğru düz bir çizgi olarak kur.
        points = [self.base.copy()]
        for length in self.lengths:
            points.append(points[-1] + np.array([0.0, length]))
        self.points = np.array(points)  # shape (n_segments + 1, 2), index 0 == base

    def set_base(self, pos: np.ndarray) -> None:
        """Zincirin kök noktasını (ör. kalça pozisyonu) her karede günceller."""
        delta = np.asarray(pos, dtype=float) - self.base
        self.base = np.asarray(pos, dtype=float)
        self.points[0] = self.base
        # Diğer noktaları da aynı ötelemeyle taşı ki bir sonraki solve() daha
        # az iterasyonla yakınsasın (soğuk başlangıçtan kaçınmak için).
        self.points[1:] += delta

    def is_reachable(self, target: np.ndarray) -> bool:
        return bool(np.linalg.norm(self.base - np.asarray(target)) <= self.arm_length)

    def solve(self, target: np.ndarray, max_iterations: int = 12) -> np.ndarray:
        """Zinciri hedefe ulaştırmaya çalışır, güncellenmiş nokta dizisini
        döndürür (son eleman uç-efektör / ayak konumudur)."""
        target = np.asarray(target, dtype=float)

        if not self.is_reachable(target):
            # Hedef menzil dışındaysa: zinciri tamamen gerip hedefe doğrultur.
            for i in range(len(self.points) - 1):
                r = _unit(target - self.points[i])
                self.points[i + 1] = self.points[i] + r * self.lengths[i]
            return self.points

        n = len(self.points) - 1
        for _ in range(max_iterations):
            if np.linalg.norm(self.points[-1] - target) < self.margin_of_error:
                break

            # Forward reaching: uç-efektörü hedefe kilitle, geriye doğru çöz.
            self.points[-1] = target
            for i in range(n - 1, -1, -1):
                r = _unit(self.points[i] - self.points[i + 1])
                self.points[i] = self.points[i + 1] + r * self.lengths[i]

            # Backward reaching: kökü orijinal tabana kilitle, ileri doğru çöz.
            self.points[0] = self.base
            for i in range(n):
                r = _unit(self.points[i + 1] - self.points[i])
                self.points[i + 1] = self.points[i] + r * self.lengths[i]

        return self.points

    def clamp_joint_angles(self, min_bend_deg: float, max_bend_deg: float, bend_sign: float = 1.0) -> None:
        """FABRIK çözümünden hemen sonra çağrılır. Zincirdeki her ara eklemin
        (interior joint) bir önceki segmente göre büküm açısını
        [min_bend_deg, max_bend_deg] aralığına sabitler ve büküm yönünü
        (`bend_sign`: +1 = saat yönünün tersi, -1 = saat yönü) zorlar --
        örneğin bir dizin tersine bükülmesini (hyperextension) engellemek
        için kullanılır. Segment uzunlukları korunur; bunun karşılığında
        uç-efektör hedefe tam ulaşamayabilir -- fiziksel olarak geçerli bir
        poz elde etmenin kabul edilen bedeli budur. (FABRIK'in kendisinde
        yerleşik bir açı kısıtlaması yoktur; bu, literatürde yaygın olan bir
        "solve sonrası clamp" tekniğidir, orijinal projeden alınmamıştır.)

        DÜZELTME (3. tur kullanıcı geri bildirimi -- "flamingo bacağı /
        tersine bükülen dizler"): sayısal tanı (bkz. `diag_round3.py`,
        commit mesajında özetlendi) BU projede normal yürüyüş sırasında
        gözle görülür bir ters-bükülme KARESİ bulamadı (t=3.2s/4.2s'de her
        iki dizin ham açısı da beklenen işaret aralığındaydı) -- kullanıcının
        somut iddiası bu haliyle DOĞRULANMADI. Ama inceleme GERÇEK, farklı
        bir kırılganlık ortaya çıkardı: eski kod `clip(signed_angle,
        -max,-min)` kullanıyordu -- yani FABRIK'in doğal çözümü YANLIŞ
        tarafa (ör. +50°) düşerse, sonuç doğrudan en yakın SINIRA
        (-8°, neredeyse düz bacak) SIÇRIYORDU: süreksiz, ani bir "pop".
        Bu proje zaten dizi sık sık 8° sınırına yakın tutuyor (bkz.
        "Ertelenen konular" -- tembel FABRIK çözümü), yani bu sıçramanın
        tetiklenmesi için gereken sinyal gürültüsü marjı ince. Kullanıcının
        önerdiği gerçek mimari düzeltme -- "açı 180'i geçtiği an kodun dizi
        ZORLA doğru tarafa katlaması" -- burada bir SINIRA kenetlemek değil,
        büküm BÜYÜKLÜĞÜNÜ koruyarak doğru tarafa YANSITMAK (reflect) olarak
        uygulanıyor: `magnitude = clip(|signed_angle|, min, max); clamped =
        ±magnitude (bend_sign'a göre)`.

        DÜZELTME/GERİ ALMA (3. tur EKİ -- kullanıcı geri bildirimi "robotik
        ve sakat yürüyüş"): bu fonksiyon kısa bir süre `_nearest_valid_
        bend_deg()`'in çember-üzerinde-en-yakın-uç-noktaya-katlama mantığını
        kullanacak şekilde değiştirilmişti (ragdoll'daki bir sıçrama hatasını
        düzeltirken). AMA bu, buradaki AKTİF/IK bacağı için YANLIŞ bir
        değişiklikti -- ölçüldü: yürüyüş döngüsündeki ham diz açısı aralığı
        [-173°,+174°]'ten [-58°,+58°]'e daralıp bacak neredeyse hiç
        bükülemeyen, kazık yutmuş gibi bir "peg-leg" yürüyüşe dönüştü
        (kullanıcı geri bildirimi doğrulandı). Sebep: bu fonksiyonun
        `points`'i, `FabrikChain2D.solve()` tarafından HER KAREDE sıfırdan,
        SÜREKLİ bir hedefe göre yeniden hesaplanıyor -- yani burada
        `clamp_joint_angle_points()`'teki gibi bir "hız donması" riski YOK
        (bu sınıfta `prev_points` kavramı bile yok). Büyük yanlış-taraf
        açılarındaki potansiyel sıçrama da pratikte zararsız çünkü bir
        sonraki karede FABRIK zaten hedefe göre baştan çözüyor. Yani bu
        fonksiyon için orijinal "büyüklüğü koru" yaklaşımının (doğal, derin
        diz bükümü verir) `_nearest_valid_bend_deg()`'in ürettiği sıçrama-
        güvenli ama SIĞ (neredeyse düz bacağa katlayan) sonuçtan daha iyi
        olduğu ortaya çıktı. `_nearest_valid_bend_deg()` GERÇEKTEN gerekli
        olduğu tek yer -- ani darbe/momentum altındaki SERBEST (Verlet)
        ragdoll bacağı -- `clamp_joint_angle_points()`'te (aşağıda) hâlâ
        kullanılıyor, sadece BURADA (aktif/IK zincirinde) geri alındı."""
        n = len(self.points)
        if n < 3:
            return  # tek segmentli zincirde ara eklem yok

        prev_dir = _unit(self.points[1] - self.points[0])
        for i in range(1, n - 1):
            seg_vec = self.points[i + 1] - self.points[i]
            length = float(np.linalg.norm(seg_vec))
            out_dir = seg_vec / (length + 1e-9)

            cross_z = prev_dir[0] * out_dir[1] - prev_dir[1] * out_dir[0]
            dot = float(np.clip(np.dot(prev_dir, out_dir), -1.0, 1.0))
            signed_angle = float(np.degrees(np.arctan2(cross_z, dot)))

            magnitude = float(np.clip(abs(signed_angle), min_bend_deg, max_bend_deg))
            clamped = magnitude if bend_sign >= 0 else -magnitude

            theta = np.radians(clamped)
            cos_t, sin_t = np.cos(theta), np.sin(theta)
            rotated_dir = np.array([
                prev_dir[0] * cos_t - prev_dir[1] * sin_t,
                prev_dir[0] * sin_t + prev_dir[1] * cos_t,
            ])
            self.points[i + 1] = self.points[i] + rotated_dir * length
            prev_dir = rotated_dir

    @property
    def end_effector(self) -> np.ndarray:
        return self.points[-1]


def clamp_joint_angle_points(
    points: np.ndarray,
    prev_points: np.ndarray,
    i_base: int,
    i_mid: int,
    i_end: int,
    min_bend_deg: float,
    max_bend_deg: float,
    bend_sign: float = 1.0,
) -> None:
    """`clamp_joint_angles()` ile AYNI matematik, ama bir `FabrikChain2D`
    üzerinde değil, doğrudan bir `VerletSystem`'in ham `points`/`prev_points`
    dizileri üzerinde çalışır -- 3 nokta (`i_base` -> `i_mid` -> `i_end`,
    ör. kalça -> diz -> ayak) arasındaki TEK eklemin büküm açısını sınırlar.

    Neden gerekli (kullanıcı geri bildirimi -- "anatomik bütünlük"):
    `clamp_joint_angles()` SADECE aktif/IK ile çözülen `FabrikChain2D`
    üzerinde çağrılıyordu (bkz. `physics/gait.py`). Karakterin "pasif"
    (ragdoll) bacak temsili -- `body.points` içindeki diz/ayak noktaları --
    sadece rijit `stick`'lerle (SABİT UZUNLUK) bağlı serbest Verlet
    noktalarıdır; hiçbir açı kısıtı yoktu, yani ragdoll'da bacak fiziksel
    olarak imkansız açılara (dizin tam tersine bükülmesi ya da kendi
    üzerine tamamen katlanması) serbestçe gidebiliyordu -- ölçüldü:
    tam pasif karelerde diz iç açısı 0.3°-142.6° arasında, HİÇBİR sınır
    olmadan salınıyordu.

    Bu fonksiyon, o pasif temsile de AYNI açı kısıtını (aktif/IK modunda
    kullanılanla aynı ya da istenirse daha gevşek bir aralıkla) uygulamak
    için var -- "IK kapalıyken bile eklem açısı fiziksel olarak makul
    kalsın" fikri, çünkü gerçek bir eklemin hareket açıklığı bilinç
    dışında (baygın/ragdoll) da bilinçli (yürürken) olduğundan farklı
    DEĞİLDİR; sadece kim kontrol ettiği değişir.

    DÜZELTME (3. tur EKİ -- kullanıcı geri bildirimi "hızı sıfırlayan açı
    kısıtlamaları" / "uçan bacaklar"): bu fonksiyon ÖNCEDEN `clamp_
    direction()` ile AYNI kuralı izliyordu -- düzeltilen noktanın
    `prev_points`'ini doğrudan YENİ konumuna eşitleyip hızını (points-
    prev_points farkını) SIFIRLIYORDU. Kullanıcının tespit ettiği kusur
    tam olarak buydu: ragdoll serbest düşerken diz açısı sınırın hemen
    dışında SABİT kalıyorsa (ör. -6° iken sınır -8°), bu düzeltme HER
    KAREDE tetikleniyor, her seferinde yerçekiminin bacağa o kareye kadar
    kazandırdığı açısal hızı siliyordu -- net etki: bacak fiilen
    "frenleniyor", yerçekimine rağmen düşemeyip geniş bir "V" (splits)
    pozunda havada asılı kalıyordu (ölçüldü: sağ ayak ~1+ saniye boyunca
    y=140-180px bandında donup düşmedi). **Gerçek düzeltme (kullanıcının
    önerisi -- momentum-koruyan kısıtlama):** nokta yeni konuma
    IŞINLANMAK yerine, düzeltmenin uyguladığı OFSET hem `points`'e hem
    `prev_points`'e AYNI MİKTARDA ekleniyor -- pozisyon düzelirken hız
    (points-prev_points farkı) MATEMATİKSEL OLARAK korunuyor (bkz. kod).
    Bu, `clamp_direction()`'da KASITLI olarak DEĞİŞTİRİLMEDİ (bkz.
    `physics/verlet.py`'nin modül dokstring'indeki kapsam notu) -- SADECE
    burada, kullanıcının somut olarak tespit ettiği yerde uygulanıyor.
    """
    p_base, p_mid, p_end = points[i_base], points[i_mid], points[i_end]
    prev_dir = _unit(p_mid - p_base)
    seg_vec = p_end - p_mid
    length = float(np.linalg.norm(seg_vec))
    out_dir = seg_vec / (length + 1e-9)

    cross_z = prev_dir[0] * out_dir[1] - prev_dir[1] * out_dir[0]
    dot = float(np.clip(np.dot(prev_dir, out_dir), -1.0, 1.0))
    signed_angle = float(np.degrees(np.arctan2(cross_z, dot)))

    # DUZELTME (3. tur -- bkz. clamp_joint_angles() dokstring'indeki ayni
    # yorum): sinira "clip" yerine gecerli araligin EN YAKIN UC NOKTASINA
    # katlama -- bkz. `_nearest_valid_bend_deg()` (3. tur EKI: eski
    # "buyuklugu koru, isareti zorla" yontemi buyuk yanlis-taraf acilarinda
    # kendi sureklilik iddiasini ihlal ediyordu, orada acikliyor).
    clamped = _nearest_valid_bend_deg(signed_angle, min_bend_deg, max_bend_deg, bend_sign)

    if clamped == signed_angle:
        return

    theta = np.radians(clamped)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    rotated_dir = np.array([
        prev_dir[0] * cos_t - prev_dir[1] * sin_t,
        prev_dir[0] * sin_t + prev_dir[1] * cos_t,
    ])
    new_end = points[i_mid] + rotated_dir * length
    correction = new_end - points[i_end]
    points[i_end] = new_end
    # DUZELTME (3. tur EKI, bkz. yukaridaki docstring): ISINLAMA + hiz
    # sifirlama YERINE, ayni ofset prev_points'e de eklenir -- boylece
    # hiz (points-prev_points farki) bu duzeltmeden ONCEKI degeriyle
    # MATEMATIKSEL OLARAK AYNI kalir (momentum-koruyan kisitlama).
    prev_points[i_end] = prev_points[i_end] + correction
