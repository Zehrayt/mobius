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
standart bir basitlestirme; (3) `swing_duration_frames` SABIT kaliyor
(hizla olceklenmiyor) -- ilk surum.
"""
from __future__ import annotations

import numpy as np

from physics.gait import FootPlantingLeg


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

    def __init__(self, *args, capture_gain: float = 1.0, support_margin: float = 6.0,
                 swing_lead_margin: float = 12.0, omega0: float = 0.05, **kwargs):
        super().__init__(*args, **kwargs)
        self.capture_gain = capture_gain
        self.support_margin = support_margin
        self.swing_lead_margin = swing_lead_margin
        self.omega0 = omega0

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
            if xcp - self.planted[0] > self.support_margin:
                self.state = "swing"
                self.swing_start = self.planted.copy()
                self.swing_target = np.array([xcp + self.swing_lead_margin, self.ground_y])
                self.swing_t = 0.0
                self._active_swing_duration = float(self.swing_duration_frames)
                self.emergency_step_active = False
        # hold_release=True HER ZAMAN gecirilir ki ebeveynin KENDI (sabit
        # esikli) stride_release kontrolu bu sinif icin ASLA devreye
        # girmesin -- release karari SADECE yukarida, capture-point'e gore.
        return super().update(hip_pos, hold_release=True)
