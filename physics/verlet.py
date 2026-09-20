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
  - `_integrate()` düzeltildi: orijinal `move_dynamic_points`/kısıtlama
    mantığı pinned (statics) noktaları da entegre edip sadece çıkan hatayı
    `prev_points` üzerinden "gizliyordu" -- her karede pinned bir noktanın
    hedefi değiştiğinde (ör. bir omuz tutamağının karşı-bacak itkisiyle
    ileri-geri hareket etmesi) bu, gözle görülür bir titremeye (jitter)
    yol açıyordu. Artık pinned noktalar `_integrate()`'te tamamen muaf
    tutuluyor; konumları yalnızca `pin()` / `set_pinned_position()` ile
    belirleniyor.
  - `wind` / `wind_scale` eklendi (orijinalde yok) -- kuyruk/pelerin gibi
    hiçbir hedefe bağlanmayan ("ikincil fizik") serbest verlet zincirlerini
    rüzgar/hava direnciyle hareket ettirebilmek için: `wind`, `gravity`
    gibi her karede eklenen genel bir ivme vektörüdür; `wind_scale` ise
    her noktanın bu rüzgara ne kadar tepki vereceğini ayarlayan nokta
    başına bir çarpandır (ör. bir kuyruğun ucu köküne göre daha fazla
    savrulsun diye) -- bkz. `add_point(..., wind_scale=...)`.
  - `collide_ground()` eklenmişti (orijinalde yok), **ARTIK BU DOSYADA
    DEĞİL** -- kullanıcının "sorumlulukların ayrılması" (separation of
    concerns) geri bildirimi üzerine bir refactor adımında
    `physics/collision.py`'ye (bağımsız bir fonksiyon olarak,
    `collide_ground(body, floor_fn, friction_fn)`) taşındı. `VerletSystem`
    böylece "zemin"/"çarpışma" gibi sahneye özgü kavramlardan tamamen
    habersiz, jenerik bir nokta/çubuk motoru olarak kalıyor. Davranış
    BİREBİR AYNI kaldı -- bkz. `physics/collision.py`'nin kendi
    dokstring'i.
  - `add_stick(..., compliance=...)` eklendi (orijinalde yok) -- bir çubuğun
    ne kadar "esnek" olduğunu 0 (tam rijit, orijinal davranış) ile
    1'e yakın bir değer arasında ayarlar. `_satisfy_sticks()`'teki geometrik
    düzeltme her relaksasyon iterasyonunda `(1 - compliance)` ile ölçeklenir
    -- yani çubuk sınırlı iterasyon sayısında hedef uzunluğa TAM
    yakınsayamaz ve hız/momentumun etkisiyle geçici olarak esner, sonra
    birkaç kare içinde geri toparlanır (squash & stretch). Bu, XPBD/PBD
    literatüründeki "compliance" kavramının basitleştirilmiş bir hali --
    gerçek XPBD `dt^2/stiffness` ile ölçekler, burada sadece iterasyon
    başına doğrusal bir karışım oranı kullanılıyor.
  - **Kütle (mass) hiyerarşisi eklendi** (3. tur eki -- kullanıcı geri
    bildirimi "kağıt bebek etkisi": tüm Verlet düğümleri eşit kütleymiş
    gibi davranıyordu, yani ağır bir gövde hafif bir bacağı gerçekçi
    şekilde SÜRÜKLEYEMİYORDU). `add_point(..., mass=...)` ile her noktaya
    bir kütle atanabiliyor; `_satisfy_sticks()`'teki çubuk düzeltmesi artık
    50/50 sabit bölünmüyor, standart PBD/XPBD nokta-kütle formülüyle
    (`w=1/kütle`; `frac_i = w_i/(w_i+w_j)`) TERS KÜTLEYLE ağırlıklanıyor
    -- ağır nokta az hareket eder, hafif nokta farkı kapatmak için çok
    hareket eder. `mass` verilmezse (varsayılan 1.0, TÜM demolar için
    geçerli) ya da iki nokta eşit kütledeyse formül `frac=0.5` verir --
    yani ESKİ davranışla BİREBİR AYNI (bkz. commit mesajındaki regresyon
    doğrulaması). **Dürüst sınır:** yerçekimi hâlâ kütleden bağımsız bir
    ivme olarak uygulanıyor (gerçek fizikte de doğru budur -- F=ma,
    a=g kütleden bağımsızdır), yani kütlenin GÖRÜNÜR etkisi SADECE çubuk
    kısıtlaması ihlal edildiğinde (ör. ani bir darbe/çarpışma anında)
    ortaya çıkar; bu gerçek bir rijit-cisim eylemsizlik-momenti simülasyonu
    değil, PBD literatüründeki "ters kütle ağırlıklı düzeltme" yaklaşımı.
  - `apply_angular_spring()` eklendi (3. tur eki, aşağıda ayrıntılı
    docstring'i var) -- omurga/boyun için basit bir açısal Hooke yasası
    (yay-sönümleme) yardımcısı.
  - **`clamp_direction()`'ın `prev_points`'i sıfırlama davranışı KORUNDU**
    (3. tur eki -- kullanıcı geri bildirimi "hızı sıfırlayan kısıtlamalar"
    genel eleştirisine rağmen, BİLİNÇLİ bir kapsam kararı): kullanıcının
    somut şikayeti (`clamp_joint_angle_points()`'in ragdoll bacaklarını
    "V-splits" pozunda dondurması, bkz. `physics/fabrik.py`) SADECE o
    fonksiyonda momentum-koruyan bir düzeltmeyle giderildi. `clamp_
    direction()` gövde/boyun/kol için ÇOK daha geniş bir alanda (hem
    aktif hem pasif modda) kullanılıyor -- momentumu orada da korumak,
    `step3`'ün "çift sarkaç" (double pendulum) kaosunu geri getirme
    riski taşıyor (bu fonksiyon zaten TAM OLARAK o kaosu engellemek için
    yazılmıştı) ve ayrı bir doğrulama turu gerektirir. Bu yüzden bu
    turda DEĞİŞTİRİLMEDİ -- olası bir sonraki iş.
    `compliance=0.0`
    (varsayılan) ile eski davranıştan hiçbir fark yok, bu yüzden mevcut
    hiçbir demo/iskelet etkilenmedi.

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
                     reference_dir: np.ndarray, max_deviation_deg: float,
                     preserve_momentum: bool = False) -> None:
    """`points[i_anchor] -> points[i_free]` segmentinin yönünü sabit bir
    `reference_dir` vektörüne göre `max_deviation_deg` ile simetrik olarak
    sınırlar (segment uzunluğu korunur).

    `preserve_momentum=False` (VARSAYILAN, ESKİ DAVRANIŞ): `prev_points[i_free]`
    yeni konuma DOĞRUDAN eşitlenir -- yani `i_free`'nin `i_anchor`'a göre açısal
    hızı bu düzeltmede SIFIRLANIR. Bu, 3. tur eki dokstring notunda açıklanan
    BİLİNÇLİ kapsam kararıydı (round 4'te `clamp_joint_angle_points()`'e
    uygulanan momentum-koruyan düzeltme buraya BİLEREK taşınmadı -- gerekçe:
    bu fonksiyon TÜM gövde/boyun/kol eklemlerinde, hem aktif hem pasif modda
    kullanılıyor ve momentum'u HER YERDE korumak `step3`'ün "çift sarkaç"
    kaosunu geri getirme riski taşıyordu).

    `preserve_momentum=True` (5. tur EKİ -- kullanıcı geri bildirimi
    "sabit ~87px/s'lik dikey kaçış / vektörel yanlılık"): nokta yeni konuma
    IŞINLANMAK yerine, düzeltmenin uyguladığı OFSET hem `points`'e hem
    `prev_points`'e AYNI MİKTARDA eklenir -- `clamp_joint_angle_points()`
    (`physics/fabrik.py`) ile BİREBİR AYNI matematik. Bu parametre EKLENDİ
    (fonksiyonun geri kalanı DEĞİŞMEDİ), varsayılanı `False` -- yani mevcut
    HİÇBİR çağrı yeri (neck, kol konileri) etkilenmedi, SADECE `demo/
    step9_ragdoll_blend.py`/`step13_full_integration_test.py`'deki gövde-eğim
    (torso-lean, hip->shoulder, referans=UP) çağrısı `True` olarak güncellendi.

    NEDEN SADECE ORADA (kütle-ağırlıklandırma DEĞİL, hedefli momentum
    koruma): kullanıcı "tüm kısıtlama sistemini kütle-ağırlıklı hale getir"
    önerisini getirdiğinde, önce (round 4 sonrası) bir ΔY-toplayıcı tanısı
    kuruldu -- her kısıtlama fonksiyonunun kendi net Y yerdeğiştirmesini
    ayrı bir toplayıcıda biriktirip 60s'lik bir kararlı-durum penceresinde
    karşılaştırdık. Sonuç: `zemin` (collide_ground) sızıntıya SIFIR katkı
    veriyordu (karakter zeminden tamamen kopmuştu -- "Asimetrik Zemin
    Çarpışması" hipotezi ELENDİ); `açı` kategorisindeki sızıntının
    >%75'i TEK BİR çağrıdan geliyordu -- işte bu (torso-lean) -- çünkü
    gövde açısı `PASSIVE_TORSO_MAX_DEG`'e (75°) çarpıp orada kilitlenince
    bu clamp HER KAREDE tetikleniyor ve her seferinde omuzun kalçaya göre
    açısal hızını sıfırlıyordu (`clamp_joint_angle_points()`'in round-4
    ÖNCESİ "V-splits" kusuruyla AYNI mekanizma, farklı eklemde). Kütle-
    ağırlıklı `clamp_direction` (anchor/free arası düzeltme paylaşımı)
    AYRICA denendi -- kaçışı ~2x geciktirdi ama ORTADAN KALDIRMADI; oysa
    SADECE bu tek çağrıyı momentum-koruyan yapmak kaçışı -87px/s'den
    -9.7px/s'ye indirdi (9x) VE davranışı tekdüze kaçıştan SINIRLI
    salınıma çevirdi -- bkz. `README.md`'nin "5. tur" bölümü ve
    `/tmp/diag_dy_accum.py`, `/tmp/diag_dy_accum2.py`, `/tmp/
    diag_confirm_torso.py` (cihaz üzerindeki tanı script'leri, repo'ya
    dahil değil). TÜM çağrıları aynı anda momentum-koruyan yapmak (round
    4/5 arası ayrıca denendi) "çift sarkaç" riskini geri getirip kararsız
    kaldı -- bu yüzden değişiklik SADECE en baskın sızıntı kaynağına,
    cerrahi şekilde uygulandı.

    DÜRÜST SINIR: bu, sızıntıyı SIFIRLAMIYOR -- geriye kalan ~-9.7px/s
    (r_elbow_hand kol-konisi + mesafe ağına dağılmış küçük paylar +
    boyun) hâlâ tanısı tamamlanmamış, gelecekteki bir tur için not edildi.

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
    new_free = points[i_anchor] + rotated * length
    if preserve_momentum:
        correction = new_free - points[i_free]
        points[i_free] = new_free
        prev_points[i_free] = prev_points[i_free] + correction
    else:
        points[i_free] = new_free
        prev_points[i_free] = points[i_free]


def apply_angular_spring(points: np.ndarray, prev_points: np.ndarray,
                          i_anchor: int, i_free: int,
                          reference_dir: np.ndarray, rest_deg: float,
                          stiffness: float, damping: float) -> float:
    """`points[i_anchor] -> points[i_free]` segmentinin `reference_dir`'e
    göre açısını basit bir açısal Hooke yasası (`tork = -k*theta - c*omega`)
    ile "geri çağıran" bir yay-sönümleme (spring-damper) uygular.

    EKLENME NEDENİ (3. tur eki -- kullanıcı isteği "omurgaya gerçek tork ve
    yay-sönümleme"): `clamp_direction()` (yukarıda) SERT bir duvar --
    açı sınırı aşılana kadar hiçbir direnç yok, aşılınca ANINDA sınıra
    kenetleniyor. Bu, ragdoll'u ya "tamamen serbest" ya da "aniden
    donmuş" gibi gösteriyordu (bkz. `physics/ragdoll.py`'nin
    `blended_max_angle()`'ı ile uygulanan pasif tavan). Bu fonksiyon
    onun YERİNE değil, YANINA (birlikte) kullanılmak üzere tasarlandı:
    `clamp_direction()` dış güvenlik duvarı olarak kalıyor (yay yanlış
    ayarlansa bile eklem asla anatomik olarak imkansız bir açıya
    kilitlenemez), bu fonksiyon ise o duvara çarpmadan ÖNCE, mesafeyle
    orantılı yumuşak bir direnç (kas tonusu hissi) sağlıyor.

    Matematik:
      - `theta` -- segmentin `reference_dir`'e göre işaretli açısı (derece,
        `clamp_direction()` ile AYNI atan2/cross-product yöntemi).
      - `omega` (açısal hız) -- persistan bir durum SAKLAMIYORUZ (bu motor
        zaten hiçbir yerde ek durum tutmuyor, sadece points/prev_points);
        bunun yerine `i_free` ve `i_anchor` noktalarının Verlet hızlarının
        (points-prev_points) farkının segmente DİK (tanjantiyel) bileşenini
        segment uzunluğuna bölerek anlık açısal hızı DOĞRUDAN türetiyoruz.
      - `tork = -stiffness*(theta-rest_deg) - damping*omega` -- açısal
        ivme (birim atalet momenti varsayımıyla -- motor zaten her noktayı
        örtük "birim kütle" kabul ediyor, bkz. `physics/verlet.py` genel
        tasarımı).
      - Bu açısal ivme, `i_free` noktasının TANJANSİYEL bir çizgisel
        ivmesine çevrilip -- `physics.ragdoll.apply_impulse()` ile AYNI
        numarayla -- `prev_points[i_free]`'e (points'e DEĞİL) uygulanıyor.
        Böylece bu kare `points`'te ANİ bir pozisyon sıçraması olmuyor;
        etki bir sonraki karenin `_integrate()`'inde hıza (points-
        prev_points farkına) yansıyarak DOĞAL bir ivmelenme gibi hissediliyor
        -- gerçek bir kuvvetin davranışı, bir "clamp"ınki değil.

    `stiffness`/`damping` = 0 verilirse fonksiyon hiçbir şey yapmaz (etkisiz).
    Dönen değer: mevcut `theta` (derece) -- çağıran kod isterse
    loglama/tanılama için kullanabilir.

    DÜRÜST SINIR: bu gerçek bir rijit-cisim tork/eylemsizlik-momenti hesabı
    DEĞİL -- `physics/balance.py`'deki orantılı geri beslemeyle AYNI ruhta,
    basitleştirilmiş bir "yeterince inandırıcı" (cheap but convincing)
    yaklaşım. Moment kolu/eylemsizlik momenti gerçek kütle dağılımına göre
    hesaplanmıyor (motor hiç kütle taşımıyor)."""
    if stiffness == 0.0 and damping == 0.0:
        return 0.0

    reference_dir = _unit(np.asarray(reference_dir, dtype=float))
    anchor = points[i_anchor]
    free = points[i_free]
    vec = free - anchor
    length = float(np.linalg.norm(vec))
    direction = vec / (length + 1e-9)

    cross_z = reference_dir[0] * direction[1] - reference_dir[1] * direction[0]
    dot = float(np.clip(np.dot(reference_dir, direction), -1.0, 1.0))
    theta_deg = float(np.degrees(np.arctan2(cross_z, dot)))
    theta_err_rad = np.radians(theta_deg - rest_deg)

    # direction'a dik (tanjansiyel) birim vektor -- cross_z ile AYNI donme
    # yonu kuralina uyacak sekilde (yukaridaki atan2 ile tutarli).
    perp = np.array([-direction[1], direction[0]])
    v_free = free - prev_points[i_free]
    v_anchor = anchor - prev_points[i_anchor]
    angular_vel = float(np.dot(v_free - v_anchor, perp)) / (length + 1e-9)

    angular_accel = -stiffness * theta_err_rad - damping * angular_vel
    tangential_accel = perp * (angular_accel * length)

    # apply_impulse() ile AYNI numara (bkz. physics/ragdoll.py): points'e
    # degil, prev_points'e (geriye) uygulanir -- bu kareye pozisyon
    # sicramasi degil, bir sonraki kareye hiz degisimi ekler.
    prev_points[i_free] = prev_points[i_free] - tangential_accel
    return theta_deg


@dataclass
class VerletSystem:
    """Noktalar (points) ve aralarındaki mesafe kısıtlamalarından (sticks)
    oluşan genel bir Verlet integration sistemi. Zincir, ağaç ya da
    kapalı iskelet (torso + kollar + bacaklar) gibi keyfi graf yapılarını
    destekler.
    """

    points: np.ndarray            # (N, 2) güncel pozisyonlar
    prev_points: np.ndarray       # (N, 2) bir önceki adımın pozisyonları
    sticks: list[tuple[int, int, float, float]] = field(default_factory=list)  # (i, j, rest_length, compliance)
    pinned: set[int] = field(default_factory=set)
    gravity: np.ndarray = field(default_factory=lambda: GRAVITY.copy())
    friction: float = FRICTION
    wind: np.ndarray = field(default_factory=lambda: np.zeros(2))
    wind_scale: np.ndarray = field(default_factory=lambda: np.zeros(0))  # (N,) nokta başına rüzgar çarpanı
    masses: np.ndarray = field(default_factory=lambda: np.zeros(0))  # (N,) nokta başına kütle (bkz. add_point)

    @classmethod
    def empty(cls) -> "VerletSystem":
        return cls(points=np.zeros((0, 2)), prev_points=np.zeros((0, 2)))

    def add_point(self, pos: Sequence[float], pinned: bool = False, wind_scale: float = 1.0, mass: float = 1.0) -> int:
        idx = len(self.points)
        pos = np.asarray(pos, dtype=float).reshape(1, 2)
        self.points = np.vstack([self.points, pos]) if self.points.size else pos.copy()
        self.prev_points = np.vstack([self.prev_points, pos]) if self.prev_points.size else pos.copy()
        self.wind_scale = np.append(self.wind_scale, float(wind_scale))
        self.masses = np.append(self.masses, float(mass))
        if pinned:
            self.pinned.add(idx)
        return idx

    def add_stick(self, i: int, j: int, length: float | None = None, compliance: float = 0.0) -> None:
        """`compliance`: bkz. modül dokstring'i -- 0.0 tam rijit (varsayılan,
        eski davranış), >0 çubuğun momentum altında geçici esnemesine
        (squash & stretch) izin verir."""
        if length is None:
            length = float(np.linalg.norm(self.points[j] - self.points[i]))
        self.sticks.append((i, j, length, compliance))

    def pin(self, idx: int, pos: Sequence[float] | None = None) -> None:
        """Bir noktayı sabitler (statics). pos verilirse önce oraya taşır."""
        if pos is not None:
            self.points[idx] = pos
            self.prev_points[idx] = pos
        self.pinned.add(idx)

    def set_pinned_position(self, idx: int, pos: Sequence[float]) -> None:
        """Pinlenmiş bir noktayı her karede dışarıdan (ör. bir anchor script'i)
        güncellemek için kullanılır. `prev_points` de aynı konuma eşitlenir --
        aksi halde bir sonraki `_integrate()` çağrısı bu noktanın "hızını"
        (points - prev_points farkını) sıfır olmayan bir değer sanıp üstüne
        ekstra bir sahte hız + yerçekimi payı bindirir. Hedef konum karesel
        (ör. bir "itki" profiliyle ileri-geri) değiştiğinde bu sahte hız
        işareti sürekli yön değiştirir ve gözle görülür bir titremeye
        dönüşür -- bkz. `_integrate()`'teki pinned nokta muafiyeti."""
        self.points[idx] = pos
        self.prev_points[idx] = pos

    def step(self, dt: float = 1.0) -> None:
        self._integrate(dt)
        for _ in range(RELAX_ITERS):
            self._satisfy_sticks()
            self._reapply_pins()

    # -- iç adımlar -----------------------------------------------------
    def _integrate(self, dt: float) -> None:
        velocity = (self.points - self.prev_points) * max(1.0 - self.friction, 0.0)
        # Rüzgar da yerçekimi gibi bir ivme olarak eklenir, ama nokta başına
        # `wind_scale` ile ölçeklenir -- ör. bir kuyruk zincirinin ucu
        # (wind_scale büyük) kökünden (wind_scale küçük/0) çok daha fazla
        # savrulsun diye. `wind_scale` boşsa (ör. add_point hiç çağrılmadan
        # doğrudan nokta atanmışsa) rüzgar etkisi sıfır kabul edilir.
        if self.wind_scale.shape[0] == len(self.points) and np.any(self.wind):
            wind_accel = self.wind[None, :] * self.wind_scale[:, None]
        else:
            wind_accel = 0.0
        candidate = self.points + velocity + (self.gravity + wind_accel) * (dt * dt)
        self.prev_points = self.points.copy()
        if self.pinned:
            # Pinned noktalar fizikten TAMAMEN muaf: konumları yalnızca
            # `pin()` / `set_pinned_position()` ile dışarıdan belirlenir.
            # Önceki implementasyon bu noktaları da entegre edip
            # (hız + yerçekimi ekleyip) sadece `_reapply_pins()` ile
            # `prev_points`'i senkronluyordu -- ama `points`'teki kaymayı
            # asla geri almıyordu. Sonuç: her karede küçük bir konum hatası
            # birikiyor, bu da omuz/kol gibi hızlı yön değiştiren pinned
            # noktalarda gözle görülür titremeye (jitter) yol açıyordu.
            mask = np.ones(len(self.points), dtype=bool)
            for idx in self.pinned:
                mask[idx] = False
            self.points = np.where(mask[:, None], candidate, self.points)
        else:
            self.points = candidate
        self._reapply_pins()

    # NOT: `collide_ground()` artık burada değil -- bkz. `physics/collision.py`
    # (modül dokstring'indeki refactor notu).

    def _reapply_pins(self) -> None:
        for idx in self.pinned:
            self.prev_points[idx] = self.points[idx]

    def _satisfy_sticks(self) -> None:
        has_masses = self.masses.shape[0] == len(self.points)
        for i, j, rest_length, compliance in self.sticks:
            pi, pj = self.points[i], self.points[j]
            delta = pj - pi
            dist = float(np.linalg.norm(delta)) + 1e-9
            diff = (dist - rest_length) / dist
            if compliance:
                # Tam düzeltmenin sadece bir kısmı uygulanır -- çubuk bu
                # relaksasyon iterasyonunda hedef uzunluğa tam ulaşmaz,
                # bu da (sınırlı RELAX_ITERS altında) gözle görülür geçici
                # esnemeye (squash & stretch) izin verir.
                diff *= max(1.0 - compliance, 0.0)
            i_pinned = i in self.pinned
            j_pinned = j in self.pinned
            if i_pinned and j_pinned:
                continue
            if i_pinned:
                self.points[j] -= delta * diff
            elif j_pinned:
                self.points[i] += delta * diff
            else:
                # KUTLE HIYERARSISI (bkz. modul dokstring'indeki "Kutle
                # (mass) hiyerarsisi" notu): duzeltme artik 50/50 degil,
                # TERS KUTLEYLE agirlikli -- standart PBD/XPBD nokta-kutle
                # kisitlama formulu (w=1/m; frac_i=w_i/(w_i+w_j)). Daha
                # AGIR nokta daha AZ hareket eder, daha HAFIF nokta
                # farki KAPATMAK icin daha COK hareket eder -- ör. agir
                # bir kalca ile hafif bir ayak arasindaki cubuk gerildiginde
                # ayak kalcaya dogru cok daha fazla cekilir, kalca neredeyse
                # yerinde kalir (gercek bir govde agirliginin bacagi
                # surukelemesi hissi). `masses` atanmamissa (has_masses
                # False, ör. bu sistemi kullanan TUM diger demolar) ya da
                # iki nokta esit kutledeyse frac_i=frac_j=0.5 olur --
                # yani ESKI davranisla BIREBIR AYNI (kutle atanmamis/esit
                # her yerde hicbir sayisal fark YOK, bkz. commit mesaji
                # dogrulamasi).
                if has_masses:
                    mi = max(float(self.masses[i]), 1e-9)
                    mj = max(float(self.masses[j]), 1e-9)
                    wi, wj = 1.0 / mi, 1.0 / mj
                    total = wi + wj
                    frac_i = wi / total
                    frac_j = wj / total
                else:
                    frac_i = frac_j = 0.5
                self.points[i] += delta * diff * frac_i
                self.points[j] -= delta * diff * frac_j
