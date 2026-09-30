"""
active_gait.py -- `FootPlantingLeg`'in DINAMIK (fizik-tabanli, "Active
Ragdoll") varyanti: `ActiveFootPlantingLeg`.

Bu dosyanin tamami bu proje icin sifirdan yazilmistir.

MIMARI (8. tur -- kullanicinin "Kinematik-Pin Kalca" elestirisi +
"polimorfizm/mimari ayrim" talebine dogrudan cevap): onceki turlarda
(step2-step13) kalca hep bir `driver` (kinematik/pinned) noktaya rijit
bir cubukla (length=3.0) baglanip DISARIDAN scriptlenen bir hizla
(WALK_SPEED) suruklendi. Kullanicinin sayisal olarak dogrulanan elestirisi:
bu, kalcayi fiili olarak dis kuvvetlerden TAMAMEN izole ediyor -- hip'e
DOGRUDAN 220px'lik bir hiz-darbesi uygulandiginda bile hip driver'dan
sadece 3.0px sapabiliyordu (bkz. commit mesaji/README "8. tur" -- izole
tanilama). Yani "destek poligonu disina cikma" tehlikesi sadece driver_x
MANUEL kaydirildiginda "gerceklesebiliyordu" -- gercek bir fiziksel darbe
hicbir sey ifade etmiyordu.

`gait.py`'deki mevcut `FootPlantingLeg` KASITLI OLARAK degistirilmedi --
step1-step11 (ve step12/13'un "tehlike disi" kareleri) kanarya sayilarini
korumak icin. Bunun yerine bu yeni sinif ondan KALITIM alip SADECE "ne
zaman adim atilir" (stride_release) ve "ayak nereye basar" (swing_target)
kararlarini fizik-tabanli hale getiriyor -- swing egrisi (Bezier), FABRIK
cozumu, diz-aci kisitlamasi, `trigger_emergency_step()` DEGISMEDEN miras
alinip kullaniliyor.

KUTLE MERKEZI IZDUSUMU / CAPTURE POINT (Dogrusal Ters Sarkac Modeli --
Linear Inverted Pendulum / LIP, robotik biped literaturunde standart bir
kavram -- bkz. J. Pratt ve digerleri, "Capture Point: A Step toward
Humanoid Push Recovery", 2006): kalcanin o anki konumundan VE hizindan,
"eger bacaklara SIFIR ek kuvvet uygulanmazsa kalcanin nihayetinde nereye
kadar gidip duracagi" (yakalama/capture noktasi) hesaplanir:

    xcp = hip_x + capture_gain * hip_vx / omega0,   omega0 = sqrt(g / L)

g: yercekimi ivmesi (KARE BASINA -- bu motorun `VerletSystem.step(dt=1.0)`
kuralina uygun, saniye bazli DEGIL -- bkz. Mimari refactor bolumundeki
"kare hizindan bagimsizlik" durust siniri, bu da AYNI sekilde gecerli);
L: bacagin tam acilim uzunlugu (`chain.arm_length`, ters sarkacin kolu).

Normal `stride_release` (SABIT piksel esigi) yerine, adim atma karari
ARTIK bu yakalama noktasinin o an zeminde duran ayagin (destek) NE KADAR
onune gectigine bakiyor: kalca ne kadar hizliysa (buyuk momentum),
yakalama noktasi o kadar ileride -- yani adim HEM daha erken tetikleniyor
HEM de daha uzaga atiliyor (kullanicinin istedigi tam olarak bu:
"Kalca hizli gidiyorsa, ayak cok daha uzaga ve daha erken atilmali").

DURUST SINIR (v1 kapsami, bilincli basitlestirmeler -- bkz. commit
mesaji/README icin tam liste): (1) tek-destekli (single support)
alternatif yuruyus modeli varsayiliyor -- cift-destek (double support)
fazi yok, bu yuzden CAGIRAN KODUN (bkz. demo/step14) her karede "diger
bacak zaten havadaysa bu bacak ASLA released olamaz" kuralini
`other_leg_swinging` ile disaridan uygulamasi GEREKIYOR (bu sinif kendi
basina bunu garanti ETMEZ); (2) stance bacagi FIZIKSEL olarak (kalcanin
yukseklik dinamigi acisindan) TAM ACIK/rijit bir kol (`chain.arm_length`)
varsayiliyor -- diz FABRIK/gorsel amacli hafif bukulebilir ama bu, kalca-
yukseklik fizigini ETKILEMIYOR (bkz. demo/step14'teki ayri "anchor"
noktasi + rijit cubuk) -- bu, robotikte "compass gait" olarak bilinen,
standart bir basitlestirme; (3) [12. TURDE KISMEN COZULDU -- asagiya
bkz.] `swing_duration_frames` v1'de SABIT kaliyordu (hizla
olceklenmiyordu).

12. TUR -- DINAMIK SALINIM SURESI v2 + HAVADA YENIDEN HEDEFLEME v2
(9. turda REDDEDILEN ilk versiyonlarin defterini kapatan, 10. turun
compliance=0.85 yumusatmasi UZERINE yeniden test edilmis hali):

9. turda, salinim suresini VE hedefini TEK SEFERDE (kalkis anindaki xcp'ye
gore) sabitleyip sonra ANI bir sicramayla degistirmek (rijit "teleport"),
kalcanin hizinde 2.4 kat'lik (13.86 -> 33.55) kontrolsuz bir sicramaya yol
acmis ve REDDEDILMISTI. Kok neden ayni zamanda 10. turdaki anchor-kalca
rijitligiyle de ORTAKTI: HER IKI ozellik de "ani, kesikli" bir geometrik
degisikligi rijit bir cubuga anlik olarak dayatiyordu.

Bu turda, AYNI iki ozellik, SUREKLI/YUMUSATILMIS bir bicimde yeniden
uygulandi:

  * Dinamik Salinim Suresi v2: suresi TEK SEFERDE degil, HER KAREDE
    o anki `hip_vx`'e gore yeniden hedeflenir (DST_BASE_VX=2.0 referans
    hiza gore ters orantili), sonra `_active_swing_duration` bu hedefe
    dogru birinci-derece bir gecikmeyle (DST_SMOOTH_RATE) yumusatilir --
    yani sure ASLA bir kareden digerine sicramaz, sadece yavasca surer.
    DST_MIN_DURATION/DST_MAX_DURATION sinirlari patolojik (asiri kisa/
    uzun) degerleri once keser.

  * Havada Yeniden Hedefleme v2: swing sirasinda yeni bir darbe gelirse
    (xcp degisirse), `swing_target` ANLIK ISINLANMA ile degil, kare
    basina en fazla MAR_MAX_STEP_PX kadar kayan SONUMLU (damped) bir
    vektorle yeni hedefe dogru itilir. `MAR_LOCK_IN_T`, salinimin son
    yarisinda (swing_t >= 0.5) hedefi DONDURUR -- inisin hemen oncesinde
    ayagin planlayabilecegi kararli/"taahhut edilmis" bir hedef birakmak
    icin (aksi halde inis anina cok yakin bir yeniden hedefleme, inis
    kinematigini bozardi).

DOGRULAMA (izole `_proto12/`, gercek repo dosyalarindan TURETILEN kopyalar
uzerinde, ust uste 3 farkli senaryo ile):
  - Normal yuruyus + buyuk itki (150px, t=7.0s): tepe hip_vx 14.58-17.51
    araliginda kaldi (9. turun reddedilen 33.55'ine KIYASLA saglikli);
    65s uzun-sure kararlilik testinde NaN yok, 185 adim (v1 ile BIREBIR
    ayni), max ardisik ayni-bacak adimi=1 (v1 ile ayni).
  - Tokezleme (15px, t=3.0s, DARBE HER ZAMAN destek/stance fazinda
    gerceklesiyor): MAR'in etkisi burada OLCULEMEYECEK kadar kucuk
    (tokez_peak 14.53 -> 14.69) -- cunku darbe zaten YERDEYKEN geliyor,
    henuz baslamamis bir salinimin hedefini "yeniden" hedefleyecek bir
    sey yok. Bu senaryo MAR'i test etmek icin YANLIS senaryo oldugu
    ANLASILDI (bkz. asagidaki "Havada-iken darbe" testi, dogru senaryo).
  - Havada-iken darbe (YENI, ADI-USTUNDE-DOGRU test: darbe, bir bacak
    fiilen swing_t=0.5'te HAVADAYKEN veriliyor): burada MAR'in gercek,
    olculebilir faydasi ortaya cikti --
      * Inis dogrulugu: v1 (MAR'siz) inis noktasi darbeden TAMAMEN
        etkilenmiyor (153.87px, darbe havadayken geldigi icin hedef
        zaten kalkista donmustu ve DEGISMIYOR) -- yani v1, ayaktayken
        gelen darbeye adapte olamiyor. MAR (cap=8, lockin=0.5) inis
        noktasini gercek yakalama noktasina dogru kaydiriyor
        (153.87 -> 167.22px).
      * Tokezleme-sonrasi ayni-bacak ardisik adimi (kelebek etkisiyle
        "sekme/topallama" riski, bkz. 11. tur'un REDDEDILME gerekcesi):
        MAR bunu KOTULESTIRMIYOR, TERSINE IYILESTIRIYOR -- max ardisik
        2-4 (v1, MAR'siz) -> 1-2 (MAR ile), cunku ayak gercege daha
        yakin bir yere basinca duzeltici bir ekstra adima daha az
        ihtiyac kaliyor.
      * Bedeli: darbe anindaki tepe hip_vx hafifce artiyor (14.68 ->
        14.85, ~%1; 40px'lik daha sert bir darbede 39.81 -> 43.92,
        ~%10) -- swing_target'i kaydirmak, FABRIK/IK zinciri uzerinden
        govdeye kucuk bir ek tepki besliyor. cap=16 bu bedeli daha da
        buyutuyor (tepe hip_vx 47.39) inis-dogrulugu kazanci cok az
        artarken (167 -> 175px) -- bu yuzden cap=8 secildi (cap=16
        DEGIL): 9./10./11. turlarin ogrettigi "daha fazla asiri-
        agresif parametre = gizli/gecikmis cokus" deseniyle tutarli.
      * 65s uzun-sure kararlilik: MAR (cap=4/8, tek basina VEYA DST ile
        birlikte) NaN uretmiyor, adim sayisi v1 ile BIREBIR ayni (185)
        -- rahatsizlik olmadan yuruyen normal kararli yuruyuste xcp
        zaten swing_target'a yakin oldugundan MAR'in etkisi dogal
        olarak sifira yakin kaliyor (atil degil, sadece GEREKMIYOR).

KARAR: Her iki ozellik de kabul edildi. DST v2, katiksiz bir kazanc
(hicbir olcumde v1'den kotu degil). MAR v2'nin faydasi SENARYOYA BAGLI
(sadece GERCEKTEN havadayken gelen darbelerde olculebilir) ama gercek ve
dogrulanmis; bedeli kucuk ve iyi karakterize edilmis (tepe hiz artisi),
11. turda reddedilen Rest Length Lerping'in aksine (o, ana metrigi
KOTULESTIRIYORDU -- burada ana metrik/ardisik-adim IYILESIYOR).
"""
from __future__ import annotations

import numpy as np

from physics.gait import FootPlantingLeg

# 12. tur -- Dinamik Salinim Suresi v2 parametreleri.
DST_BASE_VX = 2.0       # referans hiz (demo/step14'teki TARGET_VX ile ayni)
DST_MIN_VX = 0.5        # sifira bolmeyi onlemek icin taban hiz
DST_MIN_DURATION = 4.0  # kare -- asiri kisa/sert salinimi kes
DST_MAX_DURATION = 20.0 # kare -- asiri uzun/suruklenen salinimi kes
DST_SMOOTH_RATE = 0.15  # birinci-derece gecikme orani (0-1, kucuk=yumusak)

# 12. tur -- Havada Yeniden Hedefleme v2 parametreleri.
MAR_MAX_STEP_PX = 8.0   # kare basina swing_target'in kayabilecegi azami piksel
MAR_LOCK_IN_T = 0.5     # bu swing_t'den sonra hedef DONAR (inis kararliligi icin)

# Adim 17 (Faz A) -- Kinematik Kontak-Faz Sensoru. step16 laboratuvarinin
# (checkpoint 4/5) DOGRULANMIS uc-fazli etiket setini (heel_strike /
# flat_foot / toe_off) AYNEN kullanir, ama faz, segment-ayak penetrasyonundan
# (heel_pen/toe_pen -- bu motorda stance bacagi hala TEK noktali oldugu icin
# yok) DEGIL, stance bacaginin MUTLAK acisindan okunur: ayak->kalca vektorunun
# dikeyle yaptigi aci (step14'un `leg_angle_deg` tanimiyla ayni: atan2(dx, -dy),
# yuruyus yonu +x). Kalca ayagin GERISINDEYSE (aci < -olu_bant) bacak one
# yatik -> yuk topukta (heel_strike); ONUNDEYSE (aci > +olu_bant) yuk parmak
# ucunda (toe_off); arada flat_foot. SAF GOZLEMCI: hicbir durum/konum/hiz
# DEGISTIRMEZ -- step14'un 360 karelik parmak izi sensor eklendikten sonra
# bit-bit ayni kaldi (bkz. README "Adim 17").
CONTACT_DEADBAND_DEG = 2.0
PHASE_SWING = "swing"
PHASE_HEEL_STRIKE = "heel_strike"
PHASE_FLAT_FOOT = "flat_foot"
PHASE_TOE_OFF = "toe_off"


# Adim 18 (Faz B) -- toe_off tetikli "yakalama adimi" (opportunistic override).
# Tetik: stance bacagi toe_off'ta VE bacak acisi normal yuruyusun ulasamadigi
# bir asim acisinda (60s itkisiz kosuda stance acisi hic +14.4 dereceyi
# gecmiyor) -- ya da FallRiskMonitor tehlike bildirirken bacak toe_off'ta.
# Eylem: tek-destek kuralini BOZMADAN (a) havadaki bacak varsa onun kalan
# salinimini FAZB_CATCH_FRAMES'e sikistir, (b) yoksa toe_off bacagini hemen
# sikistirilmis bir salinimla birak. Hedef her karede, inis anindaki tahmini
# kalca konumuna dogru (kare basina sinirli) kayar.
FAZB_TOE_OFF_OVERRUN_DEG = 20.0
FAZB_CATCH_FRAMES = 3.0
FAZB_TARGET_LEAD_PX = 10.0
FAZB_MAX_PREDICT_PX = 60.0
FAZB_MAX_RETARGET_PX = 25.0
# Capture-point hedefinin kalcaya gore ust siniri. xcp = hip + vx/OMEGA0 (=22*vx):
# itki sonrasi vx~20 iken normal birakis hedefi kalcanin 400+ px ONUNE
# dusuyordu (bacak boyu 184 px) -- ayak 260 px ileriye basip karakteri geri
# firlatiyordu. Normal yuruyuste hedef kalcanin ~+30 px onunde oldugu icin bu
# sinir itkisiz yuruyuste HIC devreye girmiyor (60 s kosu bit-bit ayni).
FAZB_MAX_STEP_AHEAD_PX = 70.0


def classify_contact_phase(leg_angle_deg: float, deadband_deg: float = CONTACT_DEADBAND_DEG) -> str:
    """Faz A kinematik sensoru -- stance bacaginin mutlak acisindan faz."""
    if leg_angle_deg < -deadband_deg:
        return PHASE_HEEL_STRIKE
    if leg_angle_deg > deadband_deg:
        return PHASE_TOE_OFF
    return PHASE_FLAT_FOOT


class ActiveFootPlantingLeg(FootPlantingLeg):
    """`FootPlantingLeg`'in capture-point tabanli, dinamik-hizli varyanti.

    Ek parametreler:
      capture_gain: xcp formulundeki hip_vx carpani (varsayilan 1.0 --
        teorik LIP formulunun kendisi, ayarlanabilir bir "agresiflik"
        kazanci olarak da kullanilabilir).
      support_margin: xcp, o an zeminde duran ayagin ne kadar ONUNE
        gecince adim tetiklenir (eski `stride_release`'in dogrudan
        karsiligi, ama artik SABIT bir kalca-kaymasi degil, xcp'ye
        uygulanan bir esik).
      swing_lead_margin: yeni ayak, xcp'nin ne kadar ILERISINE basar
        (eski `stride_ahead`'in dogrudan karsiligi).
      omega0: sqrt(g/L) -- cagiran kod hesaplayip geciriyor (bu sinif
        kendi basina g/L bilmiyor, sahne sabitlerine bagli olmasin diye)."""

    def __init__(self, *args, capture_gain: float = 1.0, support_margin: float = 60.0,
                 swing_lead_margin: float = -15.0, omega0: float = 0.05,
                 knee_forward_seed: float | None = None, **kwargs):
        super().__init__(*args, **kwargs)
        # Adim 17 -- FABRIK diz dali tohumu (None = eski davranis). +1/-1:
        # yuruyus yonu. Bkz. `_seed_knee_branch()`.
        self.knee_forward_seed = knee_forward_seed
        self.foot_target = self.planted.copy()
        self.catch_active = False   # Adim 18 (Faz B) -- yakalama salinimi suruyor mu
        self.faz_b_enabled = True   # False: Adim 17 davranisi (karsilastirma icin)
        self.capture_gain = capture_gain
        self.support_margin = support_margin
        self.swing_lead_margin = swing_lead_margin
        self.omega0 = omega0
        # 13. tur eki -- Stribeck (statik/kinetik) kalici kayma durumu:
        # is_slipping=True oldugu surece ayak "stance" FSM etiketini
        # KORUR (bkz. modul dokstring'i -- cagiran kodun stance-bacak
        # tespiti buna bagli) ama artik zeminle kilitli DEGILDIR --
        # slip_velocity kendi ivmelenen/sonumlenen dinamigiyle
        # surer (bkz. demo/step14_active_biped.py).
        self.is_slipping = False
        self.slip_velocity = 0.0
        # Adim 17 (Faz A) -- kinematik kontak-faz sensoru (saf gozlemci).
        self.leg_angle_deg = 0.0
        self.contact_phase = PHASE_FLAT_FOOT if self.state == "stance" else PHASE_SWING

    def sense_contact_phase(self, hip_pos: np.ndarray) -> str:
        """Faz A: `planted` (stance'ta anchor ile ayni nokta) -> kalca
        vektorunun mutlak acisini olcup `contact_phase`'i gunceller. Swing
        sirasinda faz 'swing'dir ve aci ayagin o anki konumuna gore
        raporlanir (sadece log icin)."""
        foot = self.planted if self.state == "stance" else self.chain.points[-1]
        dx = float(hip_pos[0] - foot[0])
        dy = float(hip_pos[1] - foot[1])
        self.leg_angle_deg = float(np.degrees(np.arctan2(dx, -dy)))
        if self.state == "stance":
            self.contact_phase = classify_contact_phase(self.leg_angle_deg)
        else:
            self.contact_phase = PHASE_SWING
        return self.contact_phase

    def update(self, hip_pos: np.ndarray, hip_vx: float = 0.0, hold_release: bool = False,
               other_leg_swinging: bool = False) -> np.ndarray:
        """`other_leg_swinging=True` iken (diger bacak zaten havadaysa) bu
        bacak STANCE'ta kalmaya ZORLANIR -- bkz. sinif dokstring'indeki
        "tek-destekli model" siniri: iki bacagin AYNI ANDA havada olmasi
        (kisa bir "ucus fazi") bu v1'de desteklenmiyor, cunku o anda
        kalcayi fiziksel olarak tutan HICBIR sey kalmaz (anchor cubugu da
        boşta kalir) -- izole tanilama bunun kontrolsuz bir hiz-sicramasina
        yol actigini gosterdi (bkz. commit mesaji/README)."""
        if self.state == "stance" and not hold_release and not other_leg_swinging:
            xcp = hip_pos[0] + self.capture_gain * hip_vx / self.omega0
            if xcp - self.planted[0] > self.support_margin and self.faz_b_enabled and self.toe_off_overrun():
                # Adim 18 (Faz B): kalca bu ayagi asiri hizla gecmisken (toe_off
                # asim acisi) normal capture-point birakisi xcp'yi (hip_vx/omega0)
                # yuzlerce piksel ileriye atiyordu -- bunun yerine sikistirilmis
                # yakalama salinimi.
                self.launch_catch_step(hip_pos[0], hip_vx)
            elif xcp - self.planted[0] > self.support_margin:
                self.state = "swing"
                self.swing_start = self.planted.copy()
                self.swing_target = np.array([self._reach_clamp(xcp + self.swing_lead_margin, hip_pos[0]), self.ground_y])
                self.swing_t = 0.0
                self._active_swing_duration = float(self.swing_duration_frames)
                self.emergency_step_active = False
        elif self.state == "swing" and self.catch_active:
            # Adim 18 (Faz B): yakalama salinimi -- hedef, INIS anindaki
            # tahmini kalca konumuna dogru her karede sinirli hizla kayar
            # (sikistirilmis salinimda kalca ayagi gecmeye devam ediyor).
            desired = self._catch_target_x(hip_pos[0], hip_vx)
            delta = desired - self.swing_target[0]
            cap = FAZB_MAX_RETARGET_PX + abs(hip_vx)   # kalcanin kendi hizindan geri kalmasin
            self.swing_target[0] += max(-cap, min(cap, delta))
        elif self.state == "swing" and not self.emergency_step_active:
            # 12. tur -- Dinamik Salinim Suresi v2: HER KAREDE o anki
            # hip_vx'e gore bir hedef sure hesapla, sonra suresi ANI
            # DEGIL, birinci-derece bir gecikmeyle bu hedefe dogru surukle
            # (bkz. modul dokstring'i -- 9. turun ani/sert versiyonunun
            # reddedilme gerekcesi).
            target_duration = self.swing_duration_frames * (
                DST_BASE_VX / max(abs(hip_vx), DST_MIN_VX)
            )
            target_duration = min(DST_MAX_DURATION, max(DST_MIN_DURATION, target_duration))
            self._active_swing_duration += (
                target_duration - self._active_swing_duration
            ) * DST_SMOOTH_RATE
            self._active_swing_duration = max(self._active_swing_duration, DST_MIN_DURATION)

            # 12. tur -- Havada Yeniden Hedefleme v2: salinimin sadece ILK
            # yarisinda (MAR_LOCK_IN_T), yeni bir darbe xcp'yi degistirdiyse
            # swing_target'i kare basina en fazla MAR_MAX_STEP_PX kayan
            # sonumlu bir vektorle yeni hedefe dogru it (ANLIK ISINLANMA
            # DEGIL -- bkz. 9. turun reddedilen "teleport" versiyonu).
            if self.swing_t < MAR_LOCK_IN_T:
                xcp = hip_pos[0] + self.capture_gain * hip_vx / self.omega0
                desired_target_x = self._reach_clamp(xcp + self.swing_lead_margin, hip_pos[0])
                delta = desired_target_x - self.swing_target[0]
                step = max(-MAR_MAX_STEP_PX, min(MAR_MAX_STEP_PX, delta))
                self.swing_target[0] += step
        # hold_release=True HER ZAMAN gecirilir ki ebeveynin KENDI (sabit
        # esikli) stride_release kontrolu bu sinif icin ASLA devreye
        # girmesin -- release karari SADECE yukarida, capture-point'e gore.
        self._seed_knee_branch(hip_pos)
        foot = super().update(hip_pos, hold_release=True)
        if self.state == "stance":
            self.catch_active = False
        self.foot_target = np.array(foot, dtype=float).copy()
        self.sense_contact_phase(hip_pos)
        return foot

    # -- Adim 18 (Faz B) ---------------------------------------------------
    def toe_off_overrun(self) -> bool:
        """Faz A sensoru: stance'ta, toe_off fazinda VE normal yuruyusun
        ulasamadigi bir asim acisinda mi? (kalca ayagi hizla geciyor)"""
        return (self.state == "stance" and self.contact_phase == PHASE_TOE_OFF
                and self.leg_angle_deg > FAZB_TOE_OFF_OVERRUN_DEG)

    def _reach_clamp(self, target_x: float, hip_x: float) -> float:
        if not self.faz_b_enabled:
            return target_x
        return max(hip_x - FAZB_MAX_STEP_AHEAD_PX, min(hip_x + FAZB_MAX_STEP_AHEAD_PX, target_x))

    def _catch_target_x(self, hip_x: float, hip_vx: float) -> float:
        remaining = max(0.0, (1.0 - self.swing_t) * self._active_swing_duration)
        ahead = max(-FAZB_MAX_PREDICT_PX, min(FAZB_MAX_PREDICT_PX, hip_vx * remaining))
        lead = FAZB_TARGET_LEAD_PX if hip_vx >= 0 else -FAZB_TARGET_LEAD_PX
        return hip_x + ahead + lead

    def compress_swing(self, hip_x: float, hip_vx: float, frames: float = FAZB_CATCH_FRAMES) -> bool:
        """Havadaki bacagin KALAN salinimini `frames` kareye sikistirir.
        `swing_t` degismez (Bezier konumu sicramaz); sadece kare basina
        artis hizi (1/_active_swing_duration) buyur. DST/MAR bu salinim
        icin devre disi (emergency_step_active), hedefi yakalama mantigi
        surer. Zaten daha kisa kaldiysa dokunmaz."""
        if self.state != "swing" or self.swing_t >= 1.0:
            return False
        remaining = (1.0 - self.swing_t) * self._active_swing_duration
        if remaining <= frames + 1e-9 and self.catch_active:
            return False
        new_remaining = min(remaining, frames)
        self._active_swing_duration = max(1.0, new_remaining / (1.0 - self.swing_t))
        self.emergency_step_active = True
        self.catch_active = True
        return True

    def launch_catch_step(self, hip_x: float, hip_vx: float, frames: float = FAZB_CATCH_FRAMES) -> bool:
        """toe_off'taki stance bacagini HEMEN, sikistirilmis bir salinimla
        birakir (cagiran kod diger bacagin havada OLMADIGINI garanti eder)."""
        if self.state != "stance":
            return False
        self.state = "swing"
        self.swing_start = self.planted.copy()
        self.swing_t = 0.0
        self._active_swing_duration = max(1.0, float(frames))
        self.emergency_step_active = True
        self.catch_active = True
        self.is_slipping = False
        self.slip_velocity = 0.0
        self.swing_target = np.array([self._catch_target_x(hip_x, hip_vx), self.ground_y])
        return True

    def _seed_knee_branch(self, hip_pos: np.ndarray) -> None:
        """Adim 17 -- FABRIK her karede bir onceki noktalardan baslar; bacak
        duzlesip yeniden bukuldugunde dogal cozum dizin YANLIS tarafina
        (yuruyus yonune gore geriye) dusebiliyor. `clamp_joint_angles()`
        bunu buyuklugu koruyup ayna-yansitarak duzeltirken uyluk yonunu
        sabit tuttugu icin AYAGI hedefinden 100+ px geriye savuruyordu
        (bkz. README "Adim 17" -- Bilge derisi giydirilince goruldu). Cozum:
        cozumden ONCE diz noktasini yuruyus yonunun ileri tarafina (~30
        derece) tohumlamak -- FABRIK dogru dala yakinsar, clamp bir no-op
        olur. Zincir noktalari kalca dinamigine geri beslenmez (anchor =
        planted), bu yuzden Verlet noktalari bit-bit ayni kalir."""
        if self.knee_forward_seed is None:
            return
        chain = self.chain
        d = chain.points[-1] - hip_pos
        n = float(np.linalg.norm(d))
        u = d / n if n > 1e-6 else np.array([0.0, 1.0])
        perp = np.array([-u[1], u[0]])
        if perp[0] * self.knee_forward_seed < 0:
            perp = -perp
        l1 = chain.lengths[0]
        chain.points[0] = hip_pos
        chain.points[1] = hip_pos + u * l1 * 0.866 + perp * l1 * 0.5

    def apply_slip(self, delta_x: float) -> None:
        """13. tur -- Kinetik Sürtünme Sınırı (Slipping): itki üreten
        taraf (bkz. demo/step14_active_biped.py) zemin sürtünmesinin
        (`Terrain.friction_fn`) izin verdiği azami tepki kuvvetini
        (`F_MAX = mu * THRUST_CAP`) aştığını tespit ettiğinde, kalan
        (excess) kuvveti buraya -- ayağın bastığı noktaya (`planted`) bir
        kayma (kinetik slip) olarak uygular. Bu metod SADECE STANCE
        fazında çağrılmalıdır (bu sınıf kendi başına kontrol etmiyor --
        `update()` ile aynı sözleşme, çağıran kod garanti eder)."""
        self.planted[0] += delta_x
