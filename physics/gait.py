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
    """Ease-in/ease-out eğrisi (3t^2 - 2t^3): swing ilerlemesi sıfırdan
    başlayıp sıfıra biterken yumuşak hızlanıp yavaşlar -- sabit hızlı
    (lineer) bir süpürme yerine gerçek bir bacağın atalet/kas ivmesine
    daha yakın bir his verir. Zaman tabanlı DEĞİL: girdi bacağın kendi
    swing_t ilerlemesi, sadece lineer yerine ease-in/out'a yeniden eşliyor."""
    return t * t * (3.0 - 2.0 * t)


def _quadratic_bezier(p0: np.ndarray, control: np.ndarray, p1: np.ndarray, t: float) -> np.ndarray:
    """İkinci dereceden (quadratic) Bezier eğrisi: B(t) = (1-t)^2*p0 +
    2(1-t)t*control + t^2*p1.

    DÜZELTME (3. tur kullanıcı geri bildirimi -- "ağırlık transferi ve ayak
    sürüklenmesi / buz pateni hissi"): eski kod, swing fazının yatay (x)
    ilerlemesi için `_smoothstep(t)`, dikey (y) kalkışı için ise BAĞIMSIZ
    bir `sin(pi*t)` eğrisi kullanıyordu -- ikisi de aynı `t`'ye bağlı olsa
    da matematiksel olarak farklı eğriler, yani ayağın izlediği yol tek bir
    tutarlı yay (arc) DEĞİL, iki ayrı fonksiyonun çakıştırılmasıydı. Sayısal
    tanı (`diag_round3.py`) STANCE fazında ayağın zaten tam sabit kaldığını
    (sol ayak std=0.0000px) doğruladı -- yani "kayma" swing eğrisinin
    KENDİSİNDE değildi. Yine de kullanıcının somut önerisi olan tek-
    parametreli Bezier eğrisine geçmek matematiksel olarak daha temiz ve
    daha kolay ayarlanabilir (tek bir kontrol noktası = kalkış yüksekliği +
    yatay "sekme" şekli), bu yüzden benimsendi. Kontrol noktası, düz
    çizginin (`p0`-`p1`) ortasının `lift_height`'in İKİ KATI kadar üstüne
    konur -- ikinci derece Bezier'in t=0.5'teki değeri kontrol noktasının
    sadece YARISI kadar ağırlık taşıdığı için (bkz. çağıran kod)."""
    one_minus_t = 1.0 - t
    return (one_minus_t ** 2) * p0 + 2.0 * one_minus_t * t * control + (t ** 2) * p1


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
        # DUZELTME (kullanici geri bildirimi -- "kemik esnemesi / kutle
        # merkezi baglantisizligi"): hedef ayak konumu bacagin menzilini
        # (chain.arm_length) astiginda FABRIK zinciri tamamen gerilip
        # dogrultuluyor (bkz. FabrikChain2D.solve()'daki is_reachable()==
        # False dali) -- bu, dizin GORSEL olarak kilitlenmis/dumduz
        # gorunmesine yol aciyor. `last_overrun_px`, cagiran kodun (demo)
        # bu asma miktarini okuyup kalcayi/govdeyi (CoM'u) asagi+ileri
        # cekerek hedefi GERCEKTEN erisilebilir hale getirmesine izin
        # verir -- "bacak germek" yerine "govde egilir" mimarisi.
        self.last_overrun_px = 0.0

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
            t_raw = min(self.swing_t, 1.0)
            t = _smoothstep(t_raw)
            # DUZELTME (3. tur -- bkz. _quadratic_bezier() dokstring'i):
            # tek bir Bezier egrisi -- kontrol noktasi duz cizginin
            # ortasinin 2*lift_height ustunde, ki t=0.5'te gercek tepe
            # yuksekligi tam `lift_height` olsun.
            mid_x = (self.swing_start[0] + self.swing_target[0]) / 2.0
            control = np.array([mid_x, self.ground_y - 2.0 * self.lift_height])
            foot = _quadratic_bezier(self.swing_start, control, self.swing_target, t)
            if self.swing_t >= 1.0:
                self.planted = self.swing_target.copy()
                self.state = "stance"
                foot = self.planted

        self.chain.set_base(hip_pos)
        self.last_overrun_px = max(0.0, float(np.linalg.norm(foot - hip_pos)) - self.chain.arm_length)
        self.chain.solve(foot)
        if self.knee_limits is not None:
            self.chain.clamp_joint_angles(*self.knee_limits, bend_sign=self.knee_bend_sign)
            # DUZELTME (3. tur kullanici geri bildirimi -- "ayak surukleniyor
            # / buz pateni"): sayisal tani, STANCE ayaginin -- FABRIK hedefi
            # tam olarak sabit `self.planted` olsa bile -- diz-aci clamp'inin
            # yan etkisiyle (bkz. clamp_joint_angles() -- uc-efektor hedefe
            # tam ulasamayabilir) birkac piksel oynadigini olctu (sag ayak:
            # stance karelerinde std=4.31px, bazi karelerde zeminden ~12.5px
            # sapma). Bu, "kayma" degil ama gorsel olarak ayni izlenimi
            # verebilecek bir titreme. STANCE'ta ayagin ZATEN hareket etmemesi
            # GEREKTIGI icin (bu, swing'teki -- kabul edilebilir -- erisim
            # odununden FARKLI, orada zaten dokumante edilmis bir esneme var),
            # diz acisi kisitini bacagin GENEL duruşu icin uygulamaya devam
            # edip uc-efektoru (ayak) HER ZAMAN tam olarak hedefine geri
            # kenetliyoruz.
            if self.state == "stance":
                self.chain.points[-1] = self.planted.copy()
        return foot
