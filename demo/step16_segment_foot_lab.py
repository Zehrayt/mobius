"""
Adim 16 -- Segment Foot DINAMIK dogrulama laboratuvari (checkpoint 2).

BAGLAM: kullanicinin `step15_segment_foot.py` (checkpoint 1 -- kinematik/
pinned kalca ile emergent heel-to-toe roll) uzerine getirdigi 3 somut
muhendislik itirazindan sonra, once bir "checkpoint 2" onerildi (serbest/
dinamik kalca + gercek yuk altinda per-node surtunme), kullanici bunu
"once temizle, parametre taramalarini tamamla, bagimsiz bir laboratuvar
modulu olarak commit'le" diye geri gonderdi -- HAKLI olarak: ilk
checkpoint 2 denemesindeki `FOOT_STICK_COMPLIANCE=0.3` ve `LOAD_GAIN=0.05`
degerleri ilk denemede sezgisel secilmisti, sistemli dogrulanmamisti.

Bu dosya o dogrulamayi yapiyor. Iki ayri, TAMAMLANMIS bulgu ve BIR ACIK
(cozulmemis) sorun var -- asagida DURUST SINIR bolumune bakin.

===========================================================================
BULGU 1 -- ESNEKLIK (COMPLIANCE) TARAMASI: 0.3 GERCEKTEN GUVENLI BIR SECIM
===========================================================================
Iki BAGIMSIZ test ile capraz dogrulandi:

(a) Izole "ring-down" testi -- yercekimi VE ic surtunme KAPALI (worst-case,
    en az sonumlu senaryo), ankle SABIT, heel'e TEK bir darbe (15px),
    sadece ankle-heel/ankle-toe cubugunun kendi salinim/sonumleme
    davranisi olculdu (buyuk kalca-sarkaci kirliligi OLMADAN):

      compliance | ilk_dev | max|dev| | 120.kare_dev | sonum_orani
         0.00     |  0.069  |  0.069   |   -0.0000    |   0.0000
         0.30     |  0.223  |  0.223   |    0.0010    |   0.0049
         0.60     |  0.457  |  0.457   |    0.0067    |   0.0159
         0.70     |  0.364  |  0.692   |    0.0126    |   0.0196
         0.80     | -0.138  |  1.009   |    0.0255    |   0.0273  <- ISARET TERSINE DONUYOR
         0.90     | -1.577  |  2.265   |    0.0677    |   0.0323
         0.95     | -2.944  |  4.535   |    0.1585    |   0.0378
         0.99     | -4.514  | 12.412   |    0.5628    |   0.0922

    0.7'ye kadar davranis saglikli (kucuk, hizla sonen sapmalar). 0.8'den
    itibaren ilk-kare sapmasi ISARET DEGISTIRIYOR (beklenen gerilme yerine
    sikisma gorunuyor) VE genlik/sonum orani hizla buyuyor -- kullanicinin
    "sunger etkisi" tam olarak bu.

(b) Capraz-kontrol -- AYNI tarama, GERCEK sistemde (yercekimi+surtunme+
    itki AÇIK, tek seferlik 40px darbe): heel_dev_max 0.44 (c=0.1) -> 0.83
    (c=0.3) -> 14.35 (c=0.9) -- (a)'nin izole bulgusuyla AYNI egilim,
    AYNI sira: 0.3 hala kucuk/saglikli tarafta, buyume 0.5-0.7 civarinda
    hizlaniyor.

SONUC: `FOOT_STICK_COMPLIANCE=0.3`, hip cubugundaki 0.85'in ayak-cubuklari
icin dogrudan karsiligi DEGIL (COK daha kisa bir cubuk, farkli kutle
orani) -- ama kendi olcegi icinde, kendi sistemli taramasiyla dogrulanmis,
guvenli bir secim. ~0.7 civari, gelecekte "daha esnek/hassas ayak"
denenirse yaklasilmamasi gereken sinir olarak not edildi.

===========================================================================
BULGU 2 -- YUK-KAZANCI (LOAD_GAIN) TARAMASI: 0.3'E KADAR SAKIN, 0.5'TE HAFIF CIRPINMA
===========================================================================
Sabit yuruyus itkisi altinda (compliance=0.3 sabit, 200 kare), LOAD_GAIN
0.0'dan 0.3'e kadar heel/toe kayma-durumu HIC "toggle" (acil/kapan)
yapmiyor (0 -> 0 -> ... -> 0). 0.5'te ilk kez hafif cirpinma beliriyor
(heel:2, toe:2 toggle). `LOAD_GAIN=0.05` (13. tur eki'nden devralinan
mertebe) bu tarama icinde rahat, genis bir guvenlik payiyla oturuyor.

===========================================================================
DURUST SINIR -- ACIK/COZULMEMIS SORUN: per-node YATAY STRES formulu
===========================================================================
Kullanicinin istedigi "pogo testi"ni (ikinci/swing bacak OLMADAN, tek
ayagin en agir darbe altinda heel/toe surtunmelerinin ayrisip ayrismadigi)
kurarken IKI ayri, GERCEK sorun bulundu -- İKİSİ DE COZULMEDI, bilerek
YARIM birakildi:

1) Bu izole "tek ayak" rigi govde/karsi-denge/adim-atma icermiyor --
   serbest kalca + rijit ARM_LENGTH cubugu, matematiksel olarak ters bir
   sarkac (inverted pendulum). Aktif bir duzeltme mekanizmasi (step14'un
   torso-lean clamp'i, ya da bir sonraki adimi atma refleksi) olmadan
   HERHANGI bir yatay darbe er ya da gec kalcayi devirir -- bu, ayak
   modelinin degil, riginin govdesiz olmasinin dogal sonucu. Yani "tam
   hayatta kalma" bu checkpoint'te ANLAMLI bir metrik DEGIL; analiz
   SADECE darbe-hemen-sonrasi (ilk ~25 kare) pencereye odaklanabilir.

2) O kisa pencerede bile, per-node yatay "stres" formulu (step14'un
   `total_stress = desired_thrust + STRESS_GAIN*stress_vec[0]` tekniginin
   dogrudan kopyasi, ankle-heel/ankle-toe icin) GUVENLI bir calisma
   noktasi BULAMADI. `NODE_STRESS_GAIN` taramasi:

     gain= 1..8   -> 20px darbede bile HICBIR kayma tetiklenmiyor
     gain=10      -> darbe YOKKEN bile (normal yuruyus) toe 18 kare kayiyor
     gain=15..30  -> normal yuruyuste HER IKI nod da spurious kayiyor

   Yani "buyuk darbede tetiklenir, normal yuruyuste tetiklenmez" diye bir
   GUVENLI ARALIK bulunamadi -- step14'un `stress_vec` formulu (uzun
   anchor-kalca cubugu uzerinde, dikkatli bir LOAD_GAIN/STRESS_GAIN
   taramasiyla, bkz. "13. tur eki") buradaki cok daha KISA ankle-heel/
   ankle-toe cubuklarina naif bir sekilde tasinamiyor. Bu, muhtemelen
   YANLIS nicelik olculuyor olmasindan kaynaklaniyor (stick-stretch yerine,
   ornegin `collide_ground()`'un o karede uyguladigi gercek dikey
   duzeltme miktari -- bir carpma/impuls vekili -- daha dogru bir sinyal
   olabilir) -- ama bu HENUZ TEST EDILMEDI, sadece bir hipotez.

BU YUZDEN: pogo testi / asimetrik-kayma-ayrisma iddiasi bu turda
DOGRULANAMADI, commit'e dahil EDILMEDI. Sadece BULGU 1 ve BULGU 2
(compliance/load_gain taramalari, saglam ve tekrarlanabilir) asagidaki
kodla dogrulanip commit'lendi. Per-node stres formulunun dogru fiziksel
niceligi bulmak, tam entegrasyondan ONCE cozulmesi gereken bir SONRAKI
ayri adim.

===========================================================================
EK (checkpoint 3) -- collide_ground() TABANLI GERCEK NORMAL KUVVET VEKILI:
KISMEN DOGRULANDI, DAHA DERIN bir sorun bulundu
===========================================================================
Kullanicinin hipotezi ("yanlis nicelik olculuyor -- stick-stretch yerine
collide_ground()'un o karede uyguladigi gercek dikey duzeltme miktari
daha dogru bir sinyal olabilir") test edildi. Iki ayri bulgu cikti --
biri DOGRULANDI, digeri sorunu COZMEDI ama daha NET hale getirdi:

1) DOGRULANDI -- ama pogo/push testinin KENDISI yanlis senaryoymus: yatay
   push testi izole edildiginde (bkz. `_proto16c/diag_penetration.py`,
   repoya alinmadi), bir yatay darbenin bu rijit tek-bacak geometrisinde
   ayagi zemine DAHA COK BASTIRMADIGI, tam tersine zeminden KALDIRDIGI
   (ters-sarkac devrilmesi -- ayni "govdesiz rig" bulgusuyla tutarli)
   ortaya cikti: push arttikca heel/toe penetrasyonu (ΔY) SIFIRA
   dusuyor, hicbir zaman buyumuyor. Yani orijinal pogo testi, friksiyon/
   kayma davranisini olcmek icin dogru arac degil -- bir DEVRILME testi,
   bir DARBE/YUK testi degil.

   Bunun yerine gercek bir HEEL-STRIKE (topuk-once inis) senaryosu
   kuruldu: ayak, checkpoint 1'in (Adim 15) dogruladigi topuk-once egimle,
   kucuk bir yukseklikten ileri hizla dusuruluyor. Bu GERCEK bir temas
   ani urettigi icin, ΔY (penetrasyon) izole olcumde net bir darbe
   sinyali gosterdi: temas anindaki tepe deger (~0.7-1.3), sakin/yerlesik
   durumdaki gurultu tabanindan (~0.2-0.3) 3-5 KAT daha buyuk -- eski
   `dev` (cubuk-gerilmesi) sinyalinin HICBIR zaman ulasamadigi bir ayrim.
   Ayni Verlet-native mantikla turetilen bir "shear" (yatay kisit-
   duzeltmesi -- adim-oncesi/sonrasi konum farkindan, tam da ΔY'nin
   yatay eslenigi) darbe aninda (~0.35) sakin durumdan (~0.01-0.02) yine
   15-30 kat buyuk cikti. Yani IZOLE olcumde kullanicinin hipotezi tam
   isabetliydi: ΔY/shear, `dev`'den cok daha temiz bir sinyal.

2) COZULMEDI -- gercek yuruyus itkisi (`thrust_gain=1.0`) ile birlestirilip
   `K_NORMAL` (ΔY'yi friksiyon-limitine cevirmek icin gerekli yeni bir
   kazanc) tarandiginda, AYNI temel ayrim sorunu farkli bir yuzeyde geri
   geldi: hicbir `K_NORMAL` degeri, hem (a) normal yuruyusu (darbe/inis
   YOK, sadece surekli itki altinda ayak zeminde) sahte-tetiklemeden
   birakip hem de (b) gercek heel-strike inisini dogru tetikleyemedi.
   Kucuk K_NORMAL'da IKISI de tetikleniyor (ayrim yok); buyuk K_NORMAL'da
   heel-strike TETIKLENMEZ HALE geliyor (hassasiyet kayboluyor) AMA normal
   yuruyuste topugun agirligi one dogru KAYARKEN (checkpoint 2'de zaten
   bulunan "sürekli itki agirligi one tasir" davranisi) penetrasyon SIFIRA
   yaklastikca friksiyon limiti de sifira cokuyor ve kalan kucuk sayisal
   gurultu bu sifira-yakin esigi trivial olarak asip sahte "kayma" olarak
   isaretleniyor -- bkz. `_proto16c/lab3.py` tam tarama tablosu (ornek:
   K_NORMAL=4.0 -> normal yuruyuste heel_slip=11 kare, YOK darbe; ayni
   K_NORMAL'da gercek heel-strike'ta heel_slip=0 -- ISTENENIN TAM TERSI).

SONUC (guncellenmis): kullanicinin ΔY/shear hipotezi izole olcumde
DOGRU CIKTI (dev'den cok daha iyi SNR) -- ama bu, TEK BASINA, gercek
yuruyus dinamigi + gercek darbe arasindaki ayrimi cozmuyor. Kok neden
muhtemelen kullanicinin MESAJINDA ZATEN ONERILEN, henuz UYGULANMAMIS bir
onceki adim: heel-strike/flat-foot/toe-off FAZ durum makinesi olmadan,
"normal" ile "anormal" yerel sinyali MUTLAK bir esikle ayirmak yapisal
olarak mumkun degil gibi gorunuyor -- cunku "normal" un kendisi (agirlik
aktariminin dogal roll'u sirasinda) zaten penetrasyonu sifira yaklastirip
ayni "az-yuk" rejimine giriyor ki bu tam da darbe-sonrasi rejimle CAKISAN
bolge. Faz bilgisi (su an flat-foot mu, toe-off'a mi giriyor) olmadan,
mutlak esik tabanli hicbir formul (ister dev, ister ΔY/shear) bu ikisini
guvenilir sekilde ayiramiyor. Bu, tam `active_gait.py` entegrasyonundan
ONCE -- hatta per-node stres formulunden bile ONCE -- cozulmesi gereken,
daha temel bir on-kosul olarak yeniden cerceveleniyor.

Bu ek de (checkpoint 2'nin geri kalani gibi) commit'e YENI bir slip-
tetikleme mekanizmasi olarak DAHIL EDILMEDI -- sadece dogrulanmis/
dogrulanmamis bulgular, izole test kodu ile birlikte belgeleniyor
(bkz. `run_heelstrike()` asagida, `_proto16c/` silinmeden once buradan
gercek repoya tasindi).

Cikti: konsol raporu (video uretmiyor -- bu bir olcum/tarama laboratuvari).
"""
from __future__ import annotations

import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from physics.verlet import VerletSystem
from physics.collision import collide_ground
from physics.environment import Terrain

GROUND_Y = 220.0
ANKLE_HEIGHT = 10.0
FOOT_LEN = 26.0
ARM_LENGTH = 150.0

TUNED_GRAVITY = np.array([0.0, 0.065])
TUNED_FRICTION = 0.045
GROUND_FRICTION = 0.9

TARGET_VX = 2.0
THRUST_GAIN = 1.0
THRUST_CAP = 1.5
LOAD_FACTOR_MIN = 0.1
LOAD_FACTOR_MAX = 3.0
MU_STATIC = 1.2
KINETIC_RATIO = 0.6
SLIP_ACCEL_GAIN = 1.0
SLIP_DECAY = 0.85
SLIP_STOP_VEL = 0.05

# -- bu turda dogrulanan, TAVSIYE EDILEN degerler (bkz. yukaridaki BULGU 1/2) --
RECOMMENDED_FOOT_STICK_COMPLIANCE = 0.3
RECOMMENDED_LOAD_GAIN = 0.05


def build_body(foot_stick_compliance: float):
    sys_ = VerletSystem.empty()
    sys_.gravity = TUNED_GRAVITY.copy()
    sys_.friction = TUNED_FRICTION
    idx: dict = {}
    idx["hip"] = sys_.add_point([0.0, GROUND_Y - ANKLE_HEIGHT - ARM_LENGTH], mass=1.0)
    idx["ankle"] = sys_.add_point([0.0, GROUND_Y - ANKLE_HEIGHT], mass=0.6)
    idx["heel"] = sys_.add_point([-FOOT_LEN / 2.0, GROUND_Y], mass=0.3)
    idx["toe"] = sys_.add_point([FOOT_LEN / 2.0, GROUND_Y], mass=0.3)
    sys_.add_stick(idx["hip"], idx["ankle"], length=ARM_LENGTH, compliance=0.0)
    heel_dist = float(np.hypot(FOOT_LEN / 2.0, ANKLE_HEIGHT))
    sys_.add_stick(idx["ankle"], idx["heel"], length=heel_dist, compliance=foot_stick_compliance)
    sys_.add_stick(idx["ankle"], idx["toe"], length=heel_dist, compliance=foot_stick_compliance)
    sys_.add_stick(idx["heel"], idx["toe"], length=FOOT_LEN, compliance=0.0)
    return sys_, idx, heel_dist


class NodeSlipState:
    def __init__(self):
        self.is_slipping = False
        self.slip_velocity = 0.0


def node_stress(body, idx, node_name, rest_len):
    ankle, node = idx["ankle"], idx[node_name]
    vec = body.points[node] - body.points[ankle]
    length = float(np.linalg.norm(vec))
    dev = length - rest_len
    direction = vec / length if length > 1e-6 else np.array([0.0, 1.0])
    return dev, direction


def run(n_frames=300, foot_stick_compliance=RECOMMENDED_FOOT_STICK_COMPLIANCE,
        load_gain=RECOMMENDED_LOAD_GAIN, node_stress_gain=1.0,
        big_push_t=None, big_push_px=0.0, thrust_gain=THRUST_GAIN, target_vx=TARGET_VX):
    """Tek bacak, dinamik (serbest kalca) segment-foot rigi -- video YOK,
    sadece sayisal log. Bkz. dosya dokstring'i icin DURUST SINIR (govdesiz/
    karsi-dengesiz oldugu icin "hayatta kalma" burada anlamli bir metrik
    degil, ve node_stress_gain HENUZ guvenli bir deger BULUNAMADI)."""
    body, idx, heel_dist = build_body(foot_stick_compliance)
    hip, ankle, heel, toe = idx["hip"], idx["ankle"], idx["heel"], idx["toe"]
    terrain = Terrain(ground_y=GROUND_Y, default_friction=GROUND_FRICTION, zones=[])
    heel_state, toe_state = NodeSlipState(), NodeSlipState()

    log = {k: [] for k in ["hip_x", "hip_y", "hip_vx", "heel_dev", "toe_dev",
                            "heel_load", "toe_load", "heel_contact", "toe_contact",
                            "heel_slip", "toe_slip", "heel_slip_toggle", "toe_slip_toggle"]}
    pushed = False
    prev_heel_slip = prev_toe_slip = False

    for f in range(n_frames):
        hip_pos_before = body.points[hip].copy()
        hip_prev = body.prev_points[hip].copy()
        hip_vx = hip_pos_before[0] - hip_prev[0]

        if big_push_t is not None and not pushed and f >= big_push_t:
            body.prev_points[hip][0] -= big_push_px
            pushed = True

        desired_thrust = thrust_gain * (target_vx - hip_vx)
        desired_thrust = max(-THRUST_CAP, min(THRUST_CAP, desired_thrust))

        heel_dev, heel_dir = node_stress(body, idx, "heel", heel_dist)
        toe_dev, toe_dir = node_stress(body, idx, "toe", heel_dist)
        heel_contact = body.points[heel][1] >= GROUND_Y - 1e-6
        toe_contact = body.points[toe][1] >= GROUND_Y - 1e-6

        def load_factor(dev):
            return max(LOAD_FACTOR_MIN, min(LOAD_FACTOR_MAX, 1.0 - load_gain * dev))

        heel_load = load_factor(heel_dev) if heel_contact else 0.0
        toe_load = load_factor(toe_dev) if toe_contact else 0.0

        f_max_static_heel = MU_STATIC * THRUST_CAP * heel_load
        f_max_kinetic_heel = f_max_static_heel * KINETIC_RATIO
        f_max_static_toe = MU_STATIC * THRUST_CAP * toe_load
        f_max_kinetic_toe = f_max_static_toe * KINETIC_RATIO

        # bkz. dosya dokstring'i "ACIK SORUN" -- bu formul (step14'un
        # stress_vec tekniginin per-node kopyasi) GUVENLI bir gain
        # ARALIGI olmadan calisiyor; varsayilan node_stress_gain=1.0 ile
        # cagirilirsa PRATIKTE hicbir zaman tetiklenmez (bkz. kalibrasyon
        # tablosu) -- bilerek boyle birakildi, bir sonraki turun cozecegi
        # acik bir sorun olarak.
        heel_stress = desired_thrust + node_stress_gain * heel_dev * heel_dir[0] if heel_contact else 0.0
        toe_stress = desired_thrust + node_stress_gain * toe_dev * toe_dir[0] if toe_contact else 0.0

        for name, state, stress, f_max_s, f_max_k, contact in [
            ("heel", heel_state, heel_stress, f_max_static_heel, f_max_kinetic_heel, heel_contact),
            ("toe", toe_state, toe_stress, f_max_static_toe, f_max_kinetic_toe, toe_contact),
        ]:
            if not contact:
                state.is_slipping = False
                state.slip_velocity = 0.0
                continue
            if not state.is_slipping and abs(stress) > f_max_s:
                state.is_slipping = True
                state.slip_velocity = 0.0
            if state.is_slipping:
                excess = (stress - math.copysign(f_max_k, stress)) if abs(stress) > f_max_k else 0.0
                state.slip_velocity += -excess * SLIP_ACCEL_GAIN
                state.slip_velocity *= SLIP_DECAY
                node_idx = idx[name]
                body.points[node_idx][0] += state.slip_velocity
                body.prev_points[node_idx][0] += state.slip_velocity
                if abs(state.slip_velocity) < SLIP_STOP_VEL and abs(stress) <= f_max_k:
                    state.is_slipping = False
                    state.slip_velocity = 0.0

        body.prev_points[hip][0] -= desired_thrust
        body.step(dt=1.0)
        collide_ground(body, terrain.floor_fn, terrain.friction_fn)

        log["hip_x"].append(float(body.points[hip][0]))
        log["hip_y"].append(float(body.points[hip][1]))
        log["hip_vx"].append(float(hip_vx))
        log["heel_dev"].append(float(heel_dev))
        log["toe_dev"].append(float(toe_dev))
        log["heel_load"].append(float(heel_load))
        log["toe_load"].append(float(toe_load))
        log["heel_contact"].append(bool(heel_contact))
        log["toe_contact"].append(bool(toe_contact))
        log["heel_slip"].append(heel_state.is_slipping)
        log["toe_slip"].append(toe_state.is_slipping)
        log["heel_slip_toggle"].append(heel_state.is_slipping != prev_heel_slip)
        log["toe_slip_toggle"].append(toe_state.is_slipping != prev_toe_slip)
        prev_heel_slip, prev_toe_slip = heel_state.is_slipping, toe_state.is_slipping

        if not np.all(np.isfinite(body.points)):
            return {"nan": True, "frame": f}

    return {k: np.array(v) for k, v in log.items()}


def run_ringdown(foot_stick_compliance: float, impulse_px: float = 15.0, n_frames: int = 120):
    """Izole ring-down: yercekimi/surtunme KAPALI, ankle SABIT, heel'e tek
    darbe -- SADECE ankle-heel/ankle-toe cubugunun kendi salinim/sonumleme
    davranisini olcer (bkz. dosya dokstring'i BULGU 1)."""
    sys_ = VerletSystem.empty()
    sys_.gravity = np.array([0.0, 0.0])
    sys_.friction = 0.0
    ankle = sys_.add_point([0.0, 0.0], pinned=True)
    heel = sys_.add_point([-FOOT_LEN / 2.0, ANKLE_HEIGHT], mass=0.3)
    toe = sys_.add_point([FOOT_LEN / 2.0, ANKLE_HEIGHT], mass=0.3)
    heel_dist = float(np.hypot(FOOT_LEN / 2.0, ANKLE_HEIGHT))
    sys_.add_stick(ankle, heel, length=heel_dist, compliance=foot_stick_compliance)
    sys_.add_stick(ankle, toe, length=heel_dist, compliance=foot_stick_compliance)
    sys_.add_stick(heel, toe, length=FOOT_LEN, compliance=0.0)
    sys_.prev_points[heel][0] -= impulse_px

    dev_log = []
    for f in range(n_frames):
        sys_.step(dt=1.0)
        vec = sys_.points[heel] - sys_.points[ankle]
        dev_log.append(float(np.linalg.norm(vec)) - heel_dist)
        if not np.all(np.isfinite(sys_.points)):
            return {"nan": True, "frame": f}
    return {"dev": np.array(dev_log)}


def run_heelstrike(drop_height=15.0, fwd_vel=1.5, heel_lead_deg=15.0, n_frames=150,
                    k_normal=1.0, thrust_gain=THRUST_GAIN, target_vx=TARGET_VX):
    """checkpoint 3: GERCEK bir heel-strike (topuk-once inis) senaryosu --
    ayak, Adim 15'in (checkpoint 1) dogruladigi topuk-once egimle, kucuk
    bir yukseklikten (drop_height) ileri hizla (fwd_vel) dusuruluyor.
    Pogo/push testinin YERINE gecmiyor (o hala izole, ayri bir sinama) --
    onun YANLIS senaryo oldugu bulgusundan (bkz. dosya dokstring'i EK
    bolumu) sonra, friksiyon/yuk sinyalini GERCEK bir temas darbesiyle
    test etmek icin ayrica kuruldu.

    Friksiyon limiti burada `dev` (cubuk-gerilmesi) DEGIL, iki Verlet-
    native buyuklukten turetiliyor: `pen` (ΔY, collide_ground'dan HEMEN
    ONCE olculen penetrasyon derinligi -- gercek Normal Kuvvet vekili) ve
    `shear` (ΔX, ayni karede adim-oncesi/sonrasi konum farkindan -- gercek
    kesme/surtunme-kuvveti vekili, `pen`'in yatay eslenigi). Ikisi de AYNI
    kokten (Verlet kisit-cozucusunun konum-duzeltmesi) geldigi icin ayni
    olcekte ve `dev`'den cok daha temiz (bkz. dosya dokstring'i)."""
    sys_ = VerletSystem.empty()
    sys_.gravity = TUNED_GRAVITY.copy()
    sys_.friction = TUNED_FRICTION
    idx = {}
    ankle_y = GROUND_Y - ANKLE_HEIGHT - drop_height
    idx["hip"] = sys_.add_point([0.0, ankle_y - ARM_LENGTH], mass=1.0)
    idx["ankle"] = sys_.add_point([0.0, ankle_y], mass=0.6)
    theta = math.radians(heel_lead_deg)
    heel_dist = float(np.hypot(FOOT_LEN / 2.0, ANKLE_HEIGHT))
    base_ang = math.atan2(ANKLE_HEIGHT, FOOT_LEN / 2.0)
    idx["heel"] = sys_.add_point([-heel_dist * math.cos(base_ang - theta),
                                   ankle_y + heel_dist * math.sin(base_ang - theta)], mass=0.3)
    idx["toe"] = sys_.add_point([heel_dist * math.cos(base_ang + theta),
                                  ankle_y + heel_dist * math.sin(base_ang + theta)], mass=0.3)
    sys_.add_stick(idx["hip"], idx["ankle"], length=ARM_LENGTH, compliance=0.0)
    sys_.add_stick(idx["ankle"], idx["heel"], length=heel_dist, compliance=RECOMMENDED_FOOT_STICK_COMPLIANCE)
    sys_.add_stick(idx["ankle"], idx["toe"], length=heel_dist, compliance=RECOMMENDED_FOOT_STICK_COMPLIANCE)
    sys_.add_stick(idx["heel"], idx["toe"], length=FOOT_LEN, compliance=0.0)
    for name in ("hip", "ankle", "heel", "toe"):
        sys_.prev_points[idx[name]][0] += fwd_vel

    hip, heel, toe = idx["hip"], idx["heel"], idx["toe"]
    terrain = Terrain(ground_y=GROUND_Y, default_friction=GROUND_FRICTION, zones=[])
    heel_state, toe_state = NodeSlipState(), NodeSlipState()

    log = {k: [] for k in ["heel_pen", "toe_pen", "heel_shear", "toe_shear", "heel_slip", "toe_slip"]}
    for f in range(n_frames):
        hip_pos_before = sys_.points[hip].copy()
        hip_prev = sys_.prev_points[hip].copy()
        hip_vx = hip_pos_before[0] - hip_prev[0]
        desired_thrust = thrust_gain * (target_vx - hip_vx)
        desired_thrust = max(-THRUST_CAP, min(THRUST_CAP, desired_thrust))
        sys_.prev_points[hip][0] -= desired_thrust

        pre_points = sys_.points.copy()
        pre_prev = sys_.prev_points.copy()
        sys_.step(dt=1.0)

        shear = {}
        for name in ("heel", "toe"):
            i = idx[name]
            inertial_x = pre_points[i, 0] + (pre_points[i, 0] - pre_prev[i, 0])
            shear[name] = float(sys_.points[i, 0]) - float(inertial_x)

        heel_pen = max(0.0, float(sys_.points[heel][1]) - GROUND_Y)
        toe_pen = max(0.0, float(sys_.points[toe][1]) - GROUND_Y)
        heel_contact = heel_pen > 1e-9
        toe_contact = toe_pen > 1e-9

        f_max_static_heel = MU_STATIC * k_normal * heel_pen
        f_max_kinetic_heel = f_max_static_heel * KINETIC_RATIO
        f_max_static_toe = MU_STATIC * k_normal * toe_pen
        f_max_kinetic_toe = f_max_static_toe * KINETIC_RATIO

        for name, state, stress, f_max_s, f_max_k, contact in [
            ("heel", heel_state, shear["heel"], f_max_static_heel, f_max_kinetic_heel, heel_contact),
            ("toe", toe_state, shear["toe"], f_max_static_toe, f_max_kinetic_toe, toe_contact),
        ]:
            if not contact:
                state.is_slipping = False
                state.slip_velocity = 0.0
                continue
            if not state.is_slipping and abs(stress) > f_max_s:
                state.is_slipping = True
                state.slip_velocity = 0.0
            if state.is_slipping:
                excess = (stress - math.copysign(f_max_k, stress)) if abs(stress) > f_max_k else 0.0
                state.slip_velocity += -excess * SLIP_ACCEL_GAIN
                state.slip_velocity *= SLIP_DECAY
                node_idx = idx[name]
                sys_.points[node_idx][0] += state.slip_velocity
                sys_.prev_points[node_idx][0] += state.slip_velocity
                if abs(state.slip_velocity) < SLIP_STOP_VEL and abs(stress) <= f_max_k:
                    state.is_slipping = False
                    state.slip_velocity = 0.0

        collide_ground(sys_, terrain.floor_fn, terrain.friction_fn)

        log["heel_pen"].append(heel_pen)
        log["toe_pen"].append(toe_pen)
        log["heel_shear"].append(shear["heel"])
        log["toe_shear"].append(shear["toe"])
        log["heel_slip"].append(heel_state.is_slipping)
        log["toe_slip"].append(toe_state.is_slipping)

        if not np.all(np.isfinite(sys_.points)):
            return {"nan": True, "frame": f}
    return {k: np.array(v) for k, v in log.items()}


def main() -> None:
    print("=== BULGU 1a: izole ring-down (yercekimi/surtunme KAPALI, worst-case) ===")
    print(f"{'compliance':>10} {'ilk_dev':>9} {'max|dev|':>10} {'120.kare_dev':>13} {'sonum_orani':>12} {'NaN?':>6}")
    for c in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.98, 0.99]:
        r = run_ringdown(c)
        if r.get("nan"):
            print(f"{c:10.2f} {'--':>9} {'--':>10} {'--':>13} {'--':>12} {'EVET@'+str(r['frame']):>6}")
            continue
        dev = r["dev"]
        max_abs = np.max(np.abs(dev))
        early_amp = np.max(np.abs(dev[:10]))
        late_amp = np.max(np.abs(dev[-10:]))
        ratio = late_amp / early_amp if early_amp > 1e-9 else 0.0
        print(f"{c:10.2f} {dev[0]:9.3f} {max_abs:10.3f} {dev[-1]:13.4f} {ratio:12.4f} {'hayir':>6}")

    print("\n=== BULGU 1b: capraz-kontrol -- GERCEK sistemde (itki+darbe ON), 40px darbe ===")
    print(f"{'compliance':>10} {'heel_dev_max':>13} {'toe_dev_max':>12} {'NaN?':>6}")
    for c in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]:
        r = run(n_frames=200, foot_stick_compliance=c, load_gain=RECOMMENDED_LOAD_GAIN,
                big_push_t=80, big_push_px=40.0)
        if r.get("nan"):
            print(f"{c:10.2f} {'--':>13} {'--':>12} {'EVET@'+str(r['frame']):>6}")
            continue
        w = slice(75, 200)
        print(f"{c:10.2f} {np.abs(r['heel_dev'][w]).max():13.2f} {np.abs(r['toe_dev'][w]).max():12.2f} {'hayir':>6}")

    print("\n=== BULGU 2: LOAD_GAIN taramasi -- kayma-durumu cirpinmasi (chatter) ===")
    print(f"{'load_gain':>10} {'heel_slip_toggle':>17} {'toe_slip_toggle':>16} {'NaN?':>6}")
    for lg in [0.0, 0.01, 0.02, 0.05, 0.1, 0.2, 0.3, 0.5]:
        r = run(n_frames=200, foot_stick_compliance=RECOMMENDED_FOOT_STICK_COMPLIANCE, load_gain=lg)
        if r.get("nan"):
            print(f"{lg:10.2f} {'--':>17} {'--':>16} {'EVET@'+str(r['frame']):>6}")
            continue
        print(f"{lg:10.2f} {int(r['heel_slip_toggle'].sum()):17d} {int(r['toe_slip_toggle'].sum()):16d} {'hayir':>6}")

    print("\n=== ACIK SORUN (bkz. dosya dokstring'i): NODE_STRESS_GAIN icin guvenli aralik YOK ===")
    print("20px darbe:")
    for gain in [1.0, 8.0, 10.0, 15.0, 20.0, 30.0]:
        r = run(n_frames=110, node_stress_gain=gain, big_push_t=80, big_push_px=20.0,
                thrust_gain=0.0, target_vx=0.0)
        w = slice(80, 105)
        hs, ts = int(r["heel_slip"][w].sum()), int(r["toe_slip"][w].sum())
        print(f"  gain={gain:5.1f} (darbe VAR) -> heel_slip={hs:3d} toe_slip={ts:3d}")
    print("normal yuruyus (darbe YOK -- 0 olmali):")
    for gain in [1.0, 8.0, 10.0, 15.0, 20.0, 30.0]:
        r = run(n_frames=90, node_stress_gain=gain)
        hs, ts = int(r["heel_slip"].sum()), int(r["toe_slip"].sum())
        print(f"  gain={gain:5.1f} (darbe YOK) -> heel_slip={hs:3d} toe_slip={ts:3d}")

    print("\n=== EK (checkpoint 3) BULGU 3: izole heel-strike'ta ΔY/shear SNR'i (dev ile karsilastir) ===")
    r = run_heelstrike(drop_height=15.0, fwd_vel=1.5, heel_lead_deg=15.0, n_frames=150, k_normal=1e9, thrust_gain=0.0, target_vx=0.0)
    settle = slice(100, 150)
    impact_w = slice(20, 35)
    print(f"heel_pen: darbe_max={r['heel_pen'][impact_w].max():.4f}  yerlesik_max={np.abs(r['heel_pen'][settle]).max():.4f}"
          f"  oran={r['heel_pen'][impact_w].max() / max(np.abs(r['heel_pen'][settle]).max(), 1e-9):.1f}x")
    print(f"toe_pen:  darbe_max={r['toe_pen'][impact_w].max():.4f}  yerlesik_max={np.abs(r['toe_pen'][settle]).max():.4f}"
          f"  oran={r['toe_pen'][impact_w].max() / max(np.abs(r['toe_pen'][settle]).max(), 1e-9):.1f}x")
    print(f"heel_shear: darbe_max={np.abs(r['heel_shear'][impact_w]).max():.4f}  yerlesik_max={np.abs(r['heel_shear'][settle]).max():.4f}"
          f"  oran={np.abs(r['heel_shear'][impact_w]).max() / max(np.abs(r['heel_shear'][settle]).max(), 1e-9):.1f}x")
    print(f"toe_shear:  darbe_max={np.abs(r['toe_shear'][impact_w]).max():.4f}  yerlesik_max={np.abs(r['toe_shear'][settle]).max():.4f}"
          f"  oran={np.abs(r['toe_shear'][impact_w]).max() / max(np.abs(r['toe_shear'][settle]).max(), 1e-9):.1f}x")

    print("\n=== EK (checkpoint 3) ACIK SORUN 2: K_NORMAL icin de guvenli aralik YOK (yuruyus+heel-strike birlikte) ===")
    print(f"{'k_normal':>9} {'yuruyus_heel':>13} {'yuruyus_toe':>12} {'strike_heel':>12} {'strike_toe':>11}")
    for kn in [0.2, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 8.0, 20.0]:
        rw = run_heelstrike(drop_height=0.0, fwd_vel=0.0, heel_lead_deg=0.0, n_frames=200, k_normal=kn)
        rs = run_heelstrike(drop_height=15.0, fwd_vel=1.5, heel_lead_deg=15.0, n_frames=150, k_normal=kn)
        wh, wt = int(rw["heel_slip"].sum()), int(rw["toe_slip"].sum())
        sh, st = int(rs["heel_slip"].sum()), int(rs["toe_slip"].sum())
        print(f"{kn:9.1f} {wh:13d} {wt:12d} {sh:12d} {st:11d}")
    print("(hicbir K_NORMAL 'yuruyus=0,0 VE strike>0' satirini birlikte vermiyor -- bkz. dosya dokstring'i EK bolumu)")


if __name__ == "__main__":
    main()
