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
        """
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

            if bend_sign >= 0:
                clamped = float(np.clip(signed_angle, min_bend_deg, max_bend_deg))
            else:
                clamped = float(np.clip(signed_angle, -max_bend_deg, -min_bend_deg))

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

    `prev_points[i_end]` yeni konuma eşitlenir (bkz. `clamp_direction`) ki
    bu düzeltme bir sonraki karede sahte bir hız sıçraması yaratmasın.
    """
    p_base, p_mid, p_end = points[i_base], points[i_mid], points[i_end]
    prev_dir = _unit(p_mid - p_base)
    seg_vec = p_end - p_mid
    length = float(np.linalg.norm(seg_vec))
    out_dir = seg_vec / (length + 1e-9)

    cross_z = prev_dir[0] * out_dir[1] - prev_dir[1] * out_dir[0]
    dot = float(np.clip(np.dot(prev_dir, out_dir), -1.0, 1.0))
    signed_angle = float(np.degrees(np.arctan2(cross_z, dot)))

    if bend_sign >= 0:
        clamped = float(np.clip(signed_angle, min_bend_deg, max_bend_deg))
    else:
        clamped = float(np.clip(signed_angle, -max_bend_deg, -min_bend_deg))

    if clamped == signed_angle:
        return

    theta = np.radians(clamped)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    rotated_dir = np.array([
        prev_dir[0] * cos_t - prev_dir[1] * sin_t,
        prev_dir[0] * sin_t + prev_dir[1] * cos_t,
    ])
    points[i_end] = points[i_mid] + rotated_dir * length
    prev_points[i_end] = points[i_end]
