"""
Adim 14 (8. tur -- kullanicinin "Kinematik-Pin Kalca" elestirisi ve
"Active Ragdoll" talebi): kalcanin (hip) ARTIK bir `driver`/pin noktasina
degil, SADECE zeminde duran bacagin kendi rijit-kol geometrisine (ters
sarkac) + bir "kalca-uzatici kas" itkisine (push-off) bagli, TAMAMEN
SERBEST bir Verlet noktasi oldugu ilk calisir iki-bacakli dinamik yuruyus
prototipi.

MIMARI (bkz. `physics/active_gait.py`'nin tam dokstring'i icin gerekce/
capture-point matematigi): `gait.py`/`FootPlantingLeg` DEGISTIRILMEDI --
step1-13 kanaryalari korunuyor. Bu demo, ondan kalitim alan yeni
`ActiveFootPlantingLeg` sinifini kullaniyor. Kalca-destek fizigi: `hip`
serbest bir Verlet noktasi, `anchor` (o an zeminde duran bacagin
`planted` konumuna HER karede tasinan, pinned) bir nokta, ikisi arasinda
`chain.arm_length` uzunlugunda rijit bir cubuk -- bu, kalcayi "o an
zeminde duran ayagin ustunde donen bir ters sarkac" gibi davranmaya
zorluyor (robotikte "compass gait" olarak bilinen standart basitlestirme).

ITKI (push-off): gercek insan yuruyusundeki kalca-uzatici/baldir kasi
itkisinin kaba bir modeli -- HER karede (sadece bir bacak zemindeyken)
kalcaya, HEDEF hizla (`TARGET_VX`) mevcut hiz arasindaki farka ORANTILI
(P-kontrolcu) bir ileri/geri ivme uygulaniyor. DURUST NOT: ilk denemede
bu SABIT/kosulsuz bir itki olarak denendi -- sonuc SINIRSIZ hizlanan bir
karakterdi (itkinin geri besleme olmadan surekli enerji eklemesi, hicbir
sey onu frenlemiyordu). Hedef-hiz farkina orantili hale getirilince
(asagidaki THRUST_GAIN/TARGET_VX) sistem kendiliginden KARARLI bir
sabit-hiz yuruyuse yakinsiyor -- bkz. commit mesaji/README'deki tanilama.

Cikti: outputs/step14_active_biped.mp4
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from physics.verlet import VerletSystem, clamp_direction, apply_angular_spring, apply_angular_couple
from physics.fabrik import clamp_joint_angle_points
from physics.trunk_yaw import (TrunkYaw, yaw_momentum, side_sign, HIP_HALF_WIDTH,
                               SHOULDER_HALF_WIDTH)
from physics.hill import force_velocity
from physics.leg_mass import (LegMassModel, THIGH_MASS, SHANK_MASS,
                              THIGH_AXIS_FRAC, SHANK_AXIS_FRAC)
from physics.active_gait import ActiveFootPlantingLeg, PHASE_TOE_OFF, PHASE_HEEL_STRIKE
from physics.environment import Terrain
from physics.balance import support_interval, outside_interval_error, FallRiskMonitor, upper_body_com_x
from physics.arms import PhysicalArms, ArmGains
from physics.bracing import FallBracing, constrain_fallen_elbows
from physics.spine import FallenSpine, WAIST_RADIUS, hinge_drive, SoftHinge
from physics.gravity import GravityPolicy, REAL_GRAVITY, LEGACY_WALK_GRAVITY
from physics.ground_recovery import GroundRecovery
import math

W, H = 640, 400
FPS = 30
DURATION_S = 12
N_FRAMES = FPS * DURATION_S

HIP_Y = 170.0
GROUND_Y = 330.0
TORSO_LEN = 55.0
HEAD_STICK_LEN = 30.0
HEAD_RADIUS = 15

LEG_SEGMENT_LEN = 92.0
ARM_LENGTH = LEG_SEGMENT_LEN * 2.0  # chain.arm_length ile ayni (iki segment)
SWING_DURATION_FRAMES = 10
LIFT_HEIGHT = 22.0
KNEE_LIMITS = (8.0, 150.0)
# Adim 17 duzeltmesi: -1.0 idi. Bu sahne +x yonunde yurudugu icin -1.0,
# FABRIK dizini SALINIMDA karelerin %100'unde GERIYE (kus/ters diz) bukuyordu
# -- cubuk-adamda fark edilmemisti, Bilge derisi giydirilince gorundu (bkz.
# README "Adim 17"). Zincir noktalari kalca dinamigine HIC geri beslenmiyor
# (anchor = planted); isaret degisince 360 karenin tum Verlet noktalari
# bit-bit ayni kaldi, sadece cizilen diz tarafi duzeldi. step3-13 KENDI
# sabitlerini kullaniyor, DOKUNULMADI.
KNEE_BEND_SIGN = 1.0

UP = np.array([0.0, -1.0])
TORSO_MAX_LEAN_DEG = 12.0
NECK_MAX_TILT_DEG = 18.0
NECK_POSTURE_K = 0.5
NECK_POSTURE_C = 0.8

TUNED_GRAVITY = np.array([0.0, LEGACY_WALK_GRAVITY])  # historical replay only
TUNED_FRICTION = 0.045

# -- Dinamik kalca-itkisi (bkz. modul dokstring'i -- P-kontrolcu) -------
TARGET_VX = 2.0     # hedef ileri hiz (px/kare) -- eski WALK_SPEED=60px/s'in
                     # 30 FPS'teki karsiligi ~2.0px/kare ile KARSILASTIRILABILIR
                     # kilmak icin bilincli olarak secildi (birebir esitlemek
                     # ZORUNLU degil -- bu artik EMERGENT bir hiz, DIKTE
                     # EDILEN degil).
THRUST_GAIN = 1.0
THRUST_CAP = 1.5     # tek karede uygulanabilecek max itki/fren (kararlilik siniri)

# -- 13. tur: Kinetik Sürtünme Sınırı (Slipping) -- `physics/environment.py`
# `Terrain`'i (önceki turlardan hazır duran altyapı) ilk kez buraya
# kablolüyoruz. F_MAX = mu * THRUST_CAP: gerçek F_N = m*g yerine
# THRUST_CAP'i referans "normal kuvvet" olarak kullanıyoruz -- bu motorda
# itki zaten doğrudan kinematik bir hız-değişimi (gerçek bir F=ma kuvveti
# DEĞİL), bu yüzden gerçek bir F_N ile boyutsal olarak karşılaştırılamaz;
# bunun yerine mu=1.0 "zemin en az kas kadar güçlü tutuyor" (hiçbir zaman
# kaymıyor), mu<1.0 kaymaya başlatıyor şeklinde sezgisel/empirik bir ölçek
# olarak tanımlandı (bkz. README "13. tur" -- dürüstçe belgelenen bir
# basitleştirme, `dt=1.0`/`OMEGA0` ampirik seçimleriyle AYNI ruhta).
# -- 13. tur eki -- kullanicinin elestirisi uzerine 3 mimari acigin
# kapatilmasi: (1) Agirlik aktarimi yok sayilmisti (F_MAX sabit
# THRUST_CAP'e bagliydi, bacaga binen GERCEK dikey yuk hic hesaba
# katilmiyordu); (2) statik/kinetik (Stribeck) ayrimi yoktu -- tek
# sinir vardi, kayma tetiklenince ANI olarak "sizip" ayni kare icinde
# geri kilitleniyordu, gercek bir suruklenme (slip phase) yasanmiyordu;
# (3) mekanizma SADECE kendi urettigi itkiye (1D) bakiyordu -- disaridan
# gelen bir darbe, kendi P-kontrolculu itkisi THRUST_CAP'e ONCEDEN
# sikistirildigi icin (bkz. yukaridaki THRUST_CAP clamp), normal zeminde
# (mu_static=1.2>1.0) HICBIR ZAMAN kaymayi tetikleyemiyordu -- yapisal
# bir kor nokta.
#
# ANAHTAR CIKARIM: anchor-kalca cubugunun (`build_body()`'deki
# `ARM_LENGTH` uzunlugunda, compliance=0.85 olan cubuk) relaksasyon-
# SONRASI gercek boyu (`body.points[hip] - body.points[anchor]`), HICBIR
# `physics/verlet.py` degisikligi gerektirmeden zaten var olan, FIZIKSEL
# olarak GERCEK bir nicelik -- ve rest-length'ten (ARM_LENGTH) sapmasi
# (`stretch_dev`), itkiyi, yercekimi/carpma kaynakli dikey yuklenmeyi
# VE disaridan gelen darbeleri (STUMBLE/BIG_PUSH, ikisi de `hip`'in
# prev_points'ini degistirerek uygulaniyor) TEK bir olcumde yakalar --
# cunku hepsi ayni yoldan (hip'in konumunu/hizini degistirerek) cubugun
# gercek gerilme/sikisma durumunu etkiliyor.
#
#   * Agirlik aktarimi: `stretch_dev`'in ISARETI fiziksel olarak
#     anlamli -- SIKISMA (- dev, hip anchor'a beklenenden YAKIN --
#     agir bir inis/heel-strike'ta olur) f_n_effective'i YUKSELTIYOR
#     (daha fazla tutunma); GERILME (+ dev, kalkis/havalanma) f_n'i
#     DUSURUYOR. Izole testte dogrulandi: AYNI 60px yatay darbe, agir
#     inis anında (compression) daha GEC ve daha KUCUK bir kaymaya yol
#     acarken, hafif/kalkis anında (tension) daha ERKEN ve daha BUYUK
#     bir kaymaya yol aciyor (bkz. README "13. tur eki").
#   * Cok yonlu (omnidirectional) stres: `total_stress = desired_thrust
#     + STRESS_GAIN * stress_vec[0]` -- stress_vec, stretch_dev'in
#     cubugun GERCEK anlik dogrultusuna izdusumu (Verlet mesafe
#     kisitlamasinin kendi duzeltme kuvvetiyle AYNI matematik). Bu,
#     desired_thrust THRUST_CAP'e sikismis olsa bile, BUYUK bir dis
#     darbenin normal zeminde (mu_static=1.2) bile kaymayi
#     tetikleyebilmesini SAGLIYOR -- izole testte dogrulandi (bkz.
#     README): 150px'lik bir darbe, ARTIK buzsuz zeminde de olculebilir
#     (53+ kare) bir kayma tepkisi uretiyor; eskiden bu YAPISAL olarak
#     IMKANSIZDI.
#   * Statik/kinetik (Stribeck): `mu_kinetic = mu_static * KINETIC_RATIO`
#     (HER ZAMAN kucuk) iki AYRI `Terrain` orneginden (`terrain_static`,
#     `terrain_kinetic`) okunuyor -- `Terrain` sinifinin kendisi
#     DEGISTIRILMEDI (step8-13 ile geriye-uyumluluk). Kayma tetiklenince
#     (`is_slipping=True`) ayak o kareden itibaren KALICI olarak
#     kilitsiz kalir; `slip_velocity` kendi ivmelenen (kinetik esigi asan
#     "excess" kadar) ve sonumlenen (SLIP_DECAY) dinamigiyle surer, ta ki
#     hem HIZ (SLIP_STOP_VEL altina) hem de STRES (kinetik sinirin
#     altina) durana kadar -- tek karelik "sizinti" DEGIL, coklu-kare
#     gercek bir suruklenme fazi (izole testte 60+ ardisik kare
#     dogrulandi, bkz. README).
#
# DURUST SINIR (bilerek kapsam disi birakildi, README'de acikca
# belirtiliyor): bu motorda TUM dis darbeler (STUMBLE_KICK_PX,
# BIG_PUSH_KICK_PX) zaten SADECE yatay (x ekseni) -- gercek bir dikey
# darbe modeli yok, `Terrain.ground_y` sabit (egim/slope destegi henuz
# yok). O yuzden "cok yonlu/2B" burada "yatay eksendeki TUM kaynaklarin
# (itki + dis darbe + cubuk gerilimi) BIRLESIMI" anlamina geliyor,
# harfiyen "gercek dikey kayma" degil -- gercek dikey yukleme zaten
# agirlik-aktarimi (f_n_effective) yoluyla dolayli olarak modelleniyor
# (fizikte de dogru yer: yercekimi kaymaya degil, tutunma TAVANINA
# etki eder).
GROUND_MU_STATIC = 1.2    # normal zemin, STATIK ust sinir -- THRUST_CAP'ten daima buyuk
ICE_ZONES: list = [(250.0, 450.0, 0.15)]  # (x0, x1, mu_static) uclulerinden liste -- Terrain.zones ile ayni format
KINETIC_RATIO = 0.6       # mu_kinetic = mu_static * KINETIC_RATIO (Stribeck: kinetik HER ZAMAN statikten kucuk)
SLIP_ACCEL_GAIN = 1.0     # kinetik esigi asan stresin slip_velocity'ye ne kadar ivme kazandirdigi (eski SLIP_GAIN)
SLIP_DECAY = 0.85         # her karede slip_velocity'ye uygulanan sonum (surtunme freni) -- <1.0, kayma dogal olarak yavaslar
SLIP_STOP_VEL = 0.05      # bu esigin altina inince (VE stres kinetik siniri asmiyorsa) slip biter, stance'a kilitlenir
LOAD_GAIN = 0.02          # cubuk sikismasinin/gerilmesinin f_n_effective'i ne kadar degistirdigi (bkz. README -- sweep)
LOAD_FACTOR_MIN = 0.3     # asiri gerilme (kalkis/havalanma aninda) altinda bile taban bir tutunma birak
LOAD_FACTOR_MAX = 3.0     # asiri sikisma (agir inis) ustunde patolojik/kararsiz buyumeyi kes
STRESS_GAIN = 0.1         # cubugun gercek yatay sapmasinin (dis darbe/momentum) toplam strese katkisi (bkz. README -- sweep)

# -- 13. tur eki 3 -- kullanicinin eki-2'ye getirdigi 2 elestiri (bkz.
# README "13. tur eki 3"): (1) "45 derece sihirli sayi geriye gidistir" --
# sabit bir aci, kutle merkezinin (COM) destek poligonuna gore NEREDE
# oldugunu bilmiyor (COM kayan bacakla AYNI yondeyse 45 derece bile
# guvenli olabilir, tersiyse 20 derece bile olumcul olabilir); (2)
# "whip-crack tesadüf degil, kotu fizigin ciglaligidir" -- compliance'i
# kayma hizina bagli dinamik degistirmek bir yay sabitini yuk altinda
# degistirmekti, gercek bir "dikey Normal Kuvvet kaybi" DEGIL.
#
# COZUM: ikisi de YENI bir mekanizma icat ETMEDEN cozuldu -- `physics/
# balance.py`'de zaten var olan, `demo/step12_balance.py` ve `step13_
# full_integration_test.py`'de zaten kullanilan ve dogrulanmis
# `FallRiskMonitor`/`support_interval`/`outside_interval_error` (kutle
# merkezi vs. GERCEK destek araligi, histerezisli) bu dosyaya hic
# baglanmamisti -- kod incelemesiyle dogrulanan, dogrudan bir gozden
# kacirma. (1) icin: acil adim artik SABIT bir aciya degil, com_x'in
# support_interval()'in (stance ayagin gercek genisligi) GERCEKTEN
# disina cikip cikmadigina bakiyor. (2) icin: ANCHOR_HIP_COMPLIANCE_BASE
# ARTIK HICBIR ZAMAN degistirilmiyor (compliance HER ZAMAN sabit 0.85) --
# dinamik compliance (eski SLIP_COMPLIANCE_MAX/SAT_VEL) TAMAMEN KALDIRILDI.
#
# DOGRULAMA (izole `_proto_eki3/demo/ablation.py`, gercek repo dosyalarindan
# TURETILEN kopyalar + 5 konfigurasyonluk ablation + siddet taramasi):
# standart senaryoda (15px tokez + 150px buyuk itki) ESKI (aci+dinamik-
# compliance) kombinasyonu frame 233'te dusuyordu; YENI (COM-kriteri +
# SABIT compliance) kombinasyonu bu senaryoyu TAMAMEN atlatiyor (dusme
# YOK) -- VE bacak acisi hicbir zaman ~54 dereceyi asmiyor (eskisi -89
# dereceye kadar cikiyordu). Siddet taramasi GERCEK bir kirilma noktasi
# buldu: tek darbede ~1000px civari (700px'te hala hayatta), veya
# 300px+500px'lik ardisik CIFT darbede dusuyor -- yani "hic dusmuyor"
# degil, cok daha yuksek VE karakterize edilmis bir esik. KRITIK CAPRAZ-
# KONTROL: COM-kriterini SABIT compliance yerine dinamik compliance ile
# birlikte calistirinca sonuc DAHA KOTU cikiyor (frame 225'te dusuyor,
# sabit-compliance'in "hic dusmuyor"undan) -- yani dinamik compliance'i
# KALDIRMAK sadece daha durust degil, OLCULEBILIR sekilde daha iyi
# (kullanicinin "termodinamik yalan" elestirisinin dogrudan sayisal
# dogrulamasi). 20s rahatsiz-edilmemis regresyon: 0 acil-adim
# tetiklemesi, ort. hip_vx=1.898 (hedef 2.0) -- FALL_RISK_ENTER_PX/
# EXIT_PX (step12/13'ten AYNEN alinan 45.0/20.0) normal yuruyuste
# hicbir yanlis-pozitif uretmiyor.
ANCHOR_HIP_COMPLIANCE_BASE = 0.85  # normal stance -- 10. turdaki deger, degismedi; ARTIK HER ZAMAN SABIT (bkz. yukarisi)
FOOT_HALF_LEN = 12.0               # physics.balance.FOOT_HALF_LEN ile ayni varsayilan -- support_interval() icin
FALL_RISK_ENTER_PX = 45.0          # step12/13'ten AYNEN alindi (bkz. yukaridaki dogrulama -- retune GEREKMEDI)
FALL_RISK_EXIT_PX = 20.0
EMERGENCY_STEP_LEAD_PX = 15.0
EMERGENCY_SWING_SPEEDUP = 2.5

# -- Historical capture-point tuning; step 32 uses GravityPolicy.sqrt(g/L).
# DURUST BULGU (izole tanilama sirasinda kesfedildi): LIP formulunun
# TEORIK degeri omega0=sqrt(g/L) = sqrt(0.065/184) = 0.0188 buraya
# DOGRUDAN konulunca sistem KARARSIZ cikti (ort. hip_vx hedefin 1.8 kati,
# ara sira 25px/kare'ye varan sicramalar) -- cunku bu motor HER ZAMAN
# dt=1.0 ile calisiyor (bkz. "Mimari refactor" bolumundeki onceden
# belgelenmis "kare hizindan bagimsiz degil" siniri); LIP formulunun
# surekli-zaman (saniye bazli) turetimi bu ayrik/kare-bazli sisteme
# DOGRUDAN aktarilmiyor. Ampirik bir tarama (0.03/0.045/0.08) 0.045'in
# kararli oldugunu gosterdi -- bu deger o yuzden TEORIK degil, OLCULEREK
# secildi (bkz. commit mesaji/README).
OMEGA0 = 0.045
# Adim 18: 6.0/12.0 idi. Faz A sensoru (Adim 17) bu degerlerle kalcanin
# ayagin onune HIC gecmedigini gosterdi: xcp = hip + vx/OMEGA0 ~ hip + 44px
# oldugu icin bacak, kalca ayagin ~38px GERISINDEYKEN birakiliyordu -- 60s
# itkisiz koşuda 171 stance'in 0'i tam yuvarlanma, stance karelerinin %99'u
# heel_strike (topukta yuruyen, geriye yaslanmis karakter). 60/-15: bacak
# kalca ayagin ~16px ONUNE gecince birakilir, yeni ayak xcp'nin 15px GERISINE
# basar -> 124 stance'in 123'u HS->FF->TO. Bedeli (itki dayanikliligi) Faz B
# ile ele alindi, bkz. README "Adim 18".
SUPPORT_MARGIN = 60.0
SWING_LEAD_MARGIN = -15.0
UNIFIED_SUPPORT_MARGIN = 35.0
UNIFIED_SWING_LEAD_MARGIN = 0.0
LEGACY_SUPPORT_MARGIN = 6.0      # Adim 17 oncesi "topuk yuruyusu" -- karsilastirma icin
LEGACY_SWING_LEAD_MARGIN = 12.0

# Adim 18 (Faz B) -- toe_off tetikli yakalama adimi (bkz. physics/active_gait.py
# FAZB_* sabitleri ve README "Adim 18"). False: Adim 17 davranisi (yalnizca
# FallRiskMonitor -> trigger_emergency_step).
FAZ_B_ENABLED = True

# Adim 19b -- yakalama sonrasi kinetik sok emilimi. Yakalama ayagi kalcanin
# altina indiginde anchor-kalca cubugu (ARM_LENGTH, compliance 0.85) cok
# sikismis haldedir; tam boyuna tek-iki karede yaylanip kalcayi 57 px/kare
# firlatiyordu. SADECE yakalama inisinden sonra (anchor o bacaga gectiginde)
# cubugun dinlenme boyu bir YUKSELIS HIZI SINIRLAYICISIYLA ARM_LENGTH'e doner:
#   rest_k = min(ARM, max(rest_{k-1}, d_now) + SHOCK_RISE_CAP_PX)
# (monoton -- destek asla gevsemez, kalca sarkmaz). Kalca SHOCK_FLOOR_PX'in
# altindaysa sonumleme birakilir (tam rijit boy): -500 px geri itki taramasinda
# yumusaklik cokusu buyutuyordu. compliance DEGISMIYOR (13. tur eki 3), normal
# adimlarda HIC devreye girmez (11. tur Rest Length Lerping'in reddedilme
# sebebi normal handoff'lari bozmasiydi; itkisiz yuruyus bit-bit ayni).
# Denenip reddedilen: birinci derece low-pass (rate 0.1-0.5) -- -500 px'te
# HER oranda dustu.
# Adim 21 -- govde/boyun kelepcesi: True = Adim 8'deki momentum-koruyan
# kelepce (siniri asan hiz prev_points'te SAKLANIR -> "hayalet hiz");
# "inelastic" = momentum korunur ama siniri asan tanjantiyel bilesen silinir.
TORSO_CLAMP_MODE = "inelastic"
POSTURE_TARGET_DEG = 3.0   # hafif one egim (+ = yuruyus yonu)
POSTURE_K = 0.2            # Adim 22 -- olculup ayarlanacak
POSTURE_C = 0.8
# Adim 22 -- salinim bacagina anatomik kutle (physics/leg_mass.py): ters dinamik
# kalca yuku govdeye tepki olarak uygulanir.
LEG_MASS_ENABLED = True
# Adim 22 -- kutleli bacaklarla itki/capture-point TUM GOVDE kutle merkezinin
# hizini okur (kalca noktasinin degil): salinim bacagi ile govde arasindaki
# IC momentum alisverisi kalcayi yavaslatir ama sistemin COM'unu degil.
LEG_MASS_COM_CONTROL = True
# Adim 22 -- itki uygulama noktasi. "hip": eski model, hiz degisimi yalnizca
# kalca noktasina (govde cubugu kalcadan itilir, geride kalip -12 derecelik
# duvara yaslaniyordu). "com": zemin tepkisi kutle merkezinden gecer (LIP
# varsayimi) -- ayni hiz degisimi govde+kol noktalarinin HEPSINE; itki govdeye
# egim momenti vermez, govde durusunu kalca kaslari (POSTURE_*) tasir.
THRUST_MODE = "com"
STALE_SLIP_RESET = True

# Adim 20 -- fiziksel kollar (physics/arms.py). "off": Adim 19 govdesi (kol
# yok, bit-bit ayni); "passive": sadece yercekimi/koni/dirsek yayi;
# "drive": + bacaga ters PD omuz torku (momentum dengeleme).
# Adim 22: "cancel": kollar bacaklarin dikey eksen (yaw) momentumunu olcup
# onu sifirlayacak acisal hizi hedefler (physics/arms.py drive_cancel) --
# capraz salinim buradan KENDILIGINDEN cikar (sol kol ~ sag bacak +0.73).
ARMS_MODE = "cancel"
ARM_REFLEX = False   # Adim 21: varsayilan KAPALI -- etkisi gurultu bandinda (README Adim 21)
ARM_REFLEX_DEG = 70.0      # refleks hedef acisi (asagiya gore); isaret reflex_dir'den
# Adim 21: eski "12 kare sabit hedef, sonra ani kesinti" yerine: ARM_REFLEX_HOLD
# kare tam agirlik, sonra agirlik kare basina ARM_REFLEX_DECAY ile ustel soner.
ARM_REFLEX_HOLD = 3
ARM_REFLEX_DECAY = 0.75
ARM_REFLEX_OFF_LEVEL = 0.02
ARM_EXIT_DAMPING = 2.0        # refleks sonerken ek omuz sonumu (kol yercekimiyle salinip arkaya savrulmasin)
ARM_EXIT_LEVEL = 0.001
# "signed": yon COM sapmasina gore (ARM_REFLEX_SIGN); "fixed": her itkide ayni
# yon (ARM_REFLEX_FIXED_DIR, +1 = kollar ONE) -- asimetrik refleks.
ARM_REFLEX_POLICY = "signed"
ARM_REFLEX_FIXED_DIR = 1.0
ARM_PASSIVE_DURING_CATCH = False   # Faz B yakalama salinimi surerken bacak-izleme torku kapali
ARM_REFLEX_SIGN = 1.0      # +1: kollar COM sapmasi yonune savrulur (yel degirmeni) -- olculen en iyi; -1: tersine (COM.u geri cek)

# Adim 19c -- Faz B tetigi, bir sonraki karenin tahmini bacak acisini da okur
# (bkz. ActiveFootPlantingLeg.predict_contact).
PREDICTIVE_SENSOR_ENABLED = True
SHOCK_ABSORB_ENABLED = True
SHOCK_RISE_CAP_PX = 15.0
SHOCK_FLOOR_PX = 90.0
SHOCK_ABSORB_DONE_PX = 0.5
# Adim 24 -- temas tabanli, ivme sinirli bacak uzatma servosu.
# "rate_cap" (Adim 19b): dinlenme boyu kare basina en fazla SHOCK_RISE_CAP_PX
#   (15 px/kare ~ 2.2 m/s!) uzar; kalca SHOCK_FLOOR_PX altindaysa sonumleme
#   birakilir -> tam rijit boy, tek karede 49-60 px "yay" sicramasi (olculdu,
#   150 px itki, kare 217).
# "servo": dinlenme boyu bir hiz+ivme sinirli servoyla ARM_LENGTH'e uzar
#   (diz ekstansorlerinin sinirli kuvveti); asla mevcut boydan kisa degil
#   (destek gevsemez). Taban kurali yok.
# Tetik: "catch" = yalnizca Faz B yakalama inisi; "contact" = HER inis, temas
#   anindaki sikisma (ARM_LENGTH - |kalca-ayak|) SHOCK_CONTACT_MIN_PX'i
#   asiyorsa. Normal yuruyuste sikisma <= 1.3 px (60 s olcum), yani itkisiz
#   yuruyus bit-bit ayni.
SHOCK_MODE = "force"
# Adim 27 -- "force": kuvvet sinirli bacak aktuatoru. Sok emilimi sirasinda
# bacak (anchor-kalca cubugunun dinlenme boyu r) gercek AGIRLIGA karsi diz
# torkunun uretebildigi kuvvetle hareket eder:
#   F_max(d) = act * TAU_KNEE / (l * sin(beta)),  cos(beta) = d / 2l
#   a = (sum F_max - M g_gercek) / M      (iki ayak yerdeyse kuvvetler toplanir)
# Uzatma ivmesi <= a; kas gevserse govde g_gercek ile yavaslar (fren). a < 0
# ise bacak agirligi tasiyamaz ve zorla bukulur (cokme). Temas aninda r'nin
# hizi kalcanin bacak boyunca gercek hizidir (inis hizi kuvvetle sonumlenir).
# Adim 27'nin bu iki-yercekimi ayrimi yalnizca gravity_mode="legacy" icin
# korunur. Adim 32 varsayilaninda integrasyon, kas yuku ve yakalama ayni
# GravityPolicy degerini kullanir.
TAU_KNEE_MAX_NM = 200.0              # tek bacak diz ekstansoru (~2.9 Nm/kg, 70 kg)
TORQUE_UNIT_NM = 0.291               # 1 tork birimi (bkz. physics/active_gait.py)
G_REAL_PX = REAL_GRAVITY   # shared 30 Hz physical scale
BODY_MASS_TOTAL = 5.18               # govde+kollar 3.52 + iki bacak 1.66
KNEE_FLEX_MIN_DIST = 2.0 * 92.0 * np.sin(np.radians(15.0))   # diz 150 der bukuk: kalca-ayak ~48 px
ACT_RATE = 0.57                      # aktivasyon: kare basina (1 - act) * 0.57 (zaman sabiti ~1.2 kare, ~40 ms)
ACT_INITIAL = 0.2                    # hazirliksiz temasta kas aktivasyonu
LEG_FORCE_CAP_W = 6.0
HILL_ENABLED = True                  # Adim 28: kuvvet-hiz iliskisi (physics/hill.py)
# Adim 29 -- cokus sonrasi yere yigilma (ragdoll gecisi). Cokus aninda bacaklar
# kinematik IK'dan cikip kutleli Verlet zincirlerine (kalca-diz-ayak, 92+92 px)
# donusur; anchor cubugu (ayak yapistirici) kapanir; yere temas eden her nokta
# esnek olmayan zemin + kinetik surtunme gorur; diz menteşe sinirli (0-150 der);
# gövde kelepcesi genisler. Yercekimi bu durumda GERCEK olcektir (G_REAL_PX):
# Adim 27'deki karar -- bacak ivmeleri ve zaman olcegi gercek, dusus de oyle.
FALLEN_KNEE_MASS = 0.45              # uyluk payi (bacak 0.83 = 0.45 + 0.38)
FALLEN_FOOT_MASS = 0.38
FALLEN_FINAL_STATIC_LOCK = True
FALLEN_KINETIC_MU = 0.6             # yerden kalkmada kinetik Coulomb (statik 0.8)
FALLEN_STATIC_MU = 0.8               # Adim 30: yuke bagli statik surtunme (PBD normal duzeltmesine oranla)
FALLEN_GROUND_MU = 0.6               # zemine degen noktada yatay hiz kare basina bu oranda sonumlenir
FALLEN_TORSO_LIMIT_DEG = 179.0       # yigilmada govde-kalca acisi serbest (gevsek govde)
# temas yaricaplari (px; 184 px = 0.9 m): kalca eklemi oturunca ~10 cm yerde
FALLEN_RADIUS = {"hip": 20.0, "shoulder": 14.0, "head": 16.0, "knee": 12.0, "foot": 6.0, "arm": 5.0}
FALLEN_KNEE_MIN_DEG = 0.0
FALLEN_KNEE_MAX_DEG = 150.0
FALLEN_KNEE_BEND_SIGN = 1.0
# Yigilmada govde kaslari: True = gevsek (durus kuvvet cifti kapali). Acik
# birakilinca ayarli yercekimine gore ayarlanmis kuvvet cifti gercek yercekimine
# karsi govdeyi tutmaya calisip zemin surtunmesiyle kare basina 0.4-0.5 px
# "surunme" uretiyordu (olculdu).
FALLEN_LIMP = True
RELAX_ITERS_FALLEN = 24
GN_RELAX_ITERS_FALLEN = 8            # Adim 29 tabani: impuls modu ve omurga carpani bunu kullanir
# Adim 30 -- koruyucu kol refleksi (bracing). Tetik: bacak agirligi tasiyamiyor
# (kuvvet dengesi a < 0 ve bacak bukuluyor, BRACE_YIELD_FRAMES kare) ya da cokus.
# Kollar dusus tarafinda dunyaya sabit bir noktaya (kalca + yon * BRACE_PLANT_FROM_HIP)
# uzanir; yerdeki el omuz-el kolonu olur (README "Adim 30"). Govde durulunca kollar
# onu eksantrik olarak indirir, boyun gevser, sonra refleks soner.
BRACE_ENABLED = True
BRACE_YIELD_FRAMES = 2
BRACE_PLANT_FROM_HIP = 50.0         # px: elin dusecegi nokta, kalcadan (govde 55 px)
BRACE_FREEZE_PX = 25.0              # el zemine bu kadar yaklasinca hedef (ve yon) donar
BRACE_LOOKAHEAD = 3.0               # kare: dusus yonu = omuz-kalca konumu + 3 kare goreli hiz
BRACE_ELBOW_K = 0.3
BRACE_ELBOW_C = 0.6
BRACE_SETTLE_SPEED = 0.5            # px/kare: omuz bundan yavassa "duruldu"
BRACE_SETTLE_FRAMES = 15
BRACE_RELAX_RATE = 0.9              # indirme bitince refleks agirligi kare basina
BRACE_LOWER_RATE = 0.6              # px/kare: durulduktan sonra kolon boyunun kisalma hizi
BRACE_NECK_RELAX_DEG = 0.7          # der/kare: indirmede boyun siniri 18 -> 60 der
# Koruyucu refleks tetigi -- carpma ani (TTC). Tek yercekiminde cokus cogu zaman
# bacak servosu hic devreye girmeden, kalca destek yuksekliginin altina inince
# algilaniyor (support_height); servo "yield" sinyali olusmuyor ve refleks cokus
# karesinde aciliyordu (-200 px itki: bas 4 kare sonra yere vurdu, olculdu).
# Kollar bacak yukunu degil, BAS ve GOGSUN (omuz noktasi) zemine kalan suresini
# dinler: balistik TTC = mevcut yukseklik, dusey hiz ve yercekiminden. Ayakta
# duran govdede basin TTC'si ~16 kare (serbest dusus suresi) oldugu icin esik
# yuruyuste kendiliginden tetiklenmez. Adim 24/25'teki ayak TTC kapisinin govde karsiligi.
BRACE_TTC_FRAMES = 10.0             # kare (~330 ms): 4/6/8/10/12/14 taramasinda en dengeli (README)
BRACE_TTC_IN_LEGACY = False         # iki yercekimli eski kosul Adim 30 kanaryalarini korur
NECK_FALL_K = 0.0                   # dususte boyun acisal sertligi (tarama: README)
NECK_FALL_C = 0.0                   # dususte boyun acisal sonumu
NECK_FALL_END_C = 0.0               # ilerleyici sonum: sinira (FALLEN_NECK_LIMIT_DEG) yaklastikca eklenir
NECK_FALL_END_START_DEG = 30.0      # ilerleyici sonumun basladigi boyun acisi
# Dususte boyun: cozucu ici sonumlu esnek aci kisiti (README "Boyun").
# "passive" (varsayilan): yalniz koruyucu refleks kapaliyken (bracing=False) --
# pasif dususte kamci 854 -> 558 der/s, bas darbesi 9.2 -> 7.0 px/kare; refleks
# acikken kollar soku emer, sert boyun basi govdeyle indirip darbeyi 4.3 -> 5.1
# yapiyordu. "always" | "off". NECK_XPBD_ENABLED = True her durumda zorlar.
NECK_XPBD_MODE = "passive"
NECK_XPBD_ENABLED = False
NECK_XPBD_COMPLIANCE = 0.01         # rad / genellestirilmis kuvvet (dt = 1 kare)
NECK_XPBD_BETA = 16.0               # sonum (XPBD gamma = compliance * beta)
NECK_FALL_ALPHA_MAX = 0.08          # kare basina en buyuk acisal hiz duzeltmesi (rad/kare)
BRACE_DIR_MIN_PX = 8.0              # omuz-kalca yatay farki (3 kare ileri) bundan kucukse yon belirsiz
# Kalkma override'i: XCoM destek araliginin OVERRIDE_XCOM_MARGIN disinda ya da
# bas/gogus OVERRIDE_MIN_VY ustunde inerken TTC <= BRACE_TTC_FRAMES, OVERRIDE_FRAMES ust uste.
OVERRIDE_XCOM_MARGIN = 30.0         # px (normal kalkis/yuruyuste en fazla 15.5, olculdu)
OVERRIDE_MIN_COM_HEIGHT = 60.0      # px: XCoM olcutu diz ustunden itibaren (KM 106+), yerde degil
OVERRIDE_MIN_VY = 2.0               # px/kare
OVERRIDE_MIN_GAP = 80.0             # px: bas/gogus zeminden bu kadar yuksekte (yerdeki fazlarda TTC hep kisa)
OVERRIDE_FRAMES = 4
RECOVERY_IDLE_STATES = ("waiting", "needs_roll", "failed")
COULOMB_RECOVERY = False            # True: yerden kalkmada kare butceli saf Coulomb (README "Durust fizik modu")
# Kol kolonu: eli yere degmis ve refleksi aktif kol, omuz-el dogrultusunda yalniz
# itebilen bir destek; dirsek ekstansor kapasitesini (TAU_ELBOW_MAX_NM, eksantrikte
# Hill 1.5x) asan carpma hizi kolu bukerek (dinlenme boyunu kisaltarak) yutulur.
FALLEN_HAND_MU = 0.8                 # el-zemin statik surtunme katsayisi (koni yarim acisi ~39 der)
TAU_ELBOW_MAX_NM = 60.0              # tek kol triseps (~0.85 Nm/kg)
ARM_SEG_MEAN = 33.0                  # (34 + 32) / 2
ARM_SUPPORT_MASS = 1.0               # omuz noktasinin bir kola dusen etkin kutlesi (omuz+bas)/2
ELBOW_FLEX_MIN_DIST = float(np.sqrt(34.0 ** 2 + 32.0 ** 2 - 2 * 34.0 * 32.0 * np.cos(np.radians(40.0))))  # dirsek 140 der
FALLEN_NECK_IN_LOOP = True           # Adim 30: yerde boyun siniri relaksasyon dongusunde (kalca-bas en kisa mesafe)
FALLEN_NECK_LIMIT_DEG = 60.0         # yerde bas zemine yaslanabilsin (dik durusta 25-ish)
FALLEN_STATIC_STICK_PX = 0.6         # temas noktasinin bu hizin altindaki yatay kaymasi durur                # neredeyse duz bacakta F -> sonsuz; agirligin 6 kati ile sinirla
SHOCK_TRIGGER = "contact"
SHOCK_CONTACT_MIN_PX = 8.0
SHOCK_EXT_ACCEL = 1.0
SHOCK_EXT_VMAX = 3.0
# Adim 25 -- inise hazirlik (pre-activation). Salinimdaki ayagin zemine kalan
# suresi (ActiveFootPlantingLeg.time_to_contact) PREACT_FRAMES'e inince bacak
# ekstansorleri temastan ONCE kasilmaya baslar: hazirlik hizi her kare
# SHOCK_EXT_ACCEL artar (VMAX tavanli). Temas sok servosunu tetiklerse servo
# sifirdan degil bu hizla baslar. 0 = kapali (Adim 24).
PREACT_FRAMES = 3
# Adim 26 -- ayak rocker'i (ayak bilegi stratejisi, buyuk hareket formu).
# Durus bacaginin pivotu ayak bilegine sabitti: kalca ayagin 150 px onune
# gectiginde bacak uzunlugu sabit oldugu icin kalca h = sqrt(L^2 - dx^2)
# yayina iniyordu (150 px itkide dususun cogu bu geometri). Gercek ayakta
# topuk kalkar ve govde parmak ucu uzerinden yuvarlanir (forefoot rocker,
# plantar fleksiyon); geri tarafta topuk uzerinden (heel rocker). Pivot,
# kalca normal yuruyusun hic ulasmadigi bir mesafeyi (ROCKER_START_*) gectikten
# sonra kalcayla birlikte, en fazla ayak uzunlugu kadar kayar.
# Ayak: 0.26 m ~ 53 px; bilekten parmak ucuna ~40 px, topuga ~13 px.
# Normal yuruyuste kalca-pivot dx -18.4 ... +29.4 px (60 s olcum).
ROCKER_ENABLED = True
ROCKER_TOE_PX = 40.0
ROCKER_HEEL_PX = 13.0
ROCKER_START_FWD_PX = 35.0
ROCKER_START_BACK_PX = 25.0
# Adim 26 -- kalca stratejisi: COM destek araliginin disina ciktiginda govde
# durus hedefi kayar (derece / px hata), kuvvet cifti (apply_angular_couple)
# govdeyi dondururken tepkisi kalcayi ters yone iter. Isaret: + = COM ondeyken
# govde ONE egilir (kalca geri itilir). Tavan: govde kelepcesi 12 derecenin altinda.
HIP_STRATEGY_GAIN = 0.3
HIP_STRATEGY_MAX_DEG = 9.0

# -- Sahne olaylari: bir kucuk (ABSORBE EDILEN) ve bir buyuk (GERCEK
#    DUSMEYE yol acan) darbe.
#    DURUST BULGU (izole harness_preserve.py taramasi ile bulundu):
#    preserve_momentum=True duzeltmesinden SONRA (gercek govde
#    eylemsizligi ile), eski STUMBLE_KICK_PX=30 degeri artik guvenli
#    degil -- gecikmeli bir dusmeye yol aciyordu (frame 141, tokezleme
#    frame 90'dan ~50 kare sonra). Kucuk itkiler taramasi:
#      5px, 10px, 15px  -> hayatta kaldi
#      20px, 25px       -> dustu (frame 163, 171)
#    Gercek absorbe/dusme siniri 15px-20px arasinda; STUMBLE_KICK_PX
#    bu nedenle 15px'e cekildi (dogrulanmis en buyuk 'hayatta kalan'
#    deger). Eski 30px degeri artik 'buyuk itki' kategorisine daha
#    yakindi ve bu adimin gostermek istedigi seyi (kucuk bir
#    tokezlemenin gercekten absorbe edilmesini) dogru temsil etmiyordu.
STUMBLE_T = 3.0
STUMBLE_KICK_PX = 15.0
BIG_PUSH_T = 7.0
BIG_PUSH_KICK_PX = 150.0

FALL_HIP_Y_THRESHOLD = GROUND_Y - 10.0  # bkz. main() -- sadece raporlama icin
FALLEN_ENTRY_ON_LOW_HIP = True          # Adim 30: kalca bacak destek sinirinin altinda -> yigilma


def build_body(gravity_y: float | None = None) -> tuple[VerletSystem, dict]:
    sys_ = VerletSystem.empty()
    sys_.gravity = np.array([0.,REAL_GRAVITY if gravity_y is None else gravity_y])
    sys_.friction = TUNED_FRICTION
    idx: dict = {}

    # ESKI mimarideki `driver` (kinematik/pin, scriptlenmis hizla suruklenen)
    # noktasi YOK -- `hip` artik TAMAMEN serbest. `anchor`, HER karede o an
    # zeminde duran bacagin `planted` konumuna tasinan (pinned) bir nokta;
    # `hip`'e ARM_LENGTH uzunlugunda RIJIT bir cubukla bagli -- bu, kalcayi
    # "o an zeminde duran ayagin ustunde donen bir ters sarkac" gibi
    # davranmaya zorluyor (bkz. modul dokstring'i).
    idx["hip"] = sys_.add_point([0.0, HIP_Y], mass=1.0)
    idx["anchor"] = sys_.add_point([0.0, GROUND_Y], pinned=True)
    # 13. tur eki 3 -- compliance ARTIK HICBIR ZAMAN degismiyor (bkz.
    # yukaridaki eki-3 yorum bloğu), bu yuzden dinamik mutasyon icin
    # index tutmaya gerek kalmadi (eski "anchor_hip_stick" indeksi
    # kaldirildi).
    sys_.add_stick(idx["anchor"], idx["hip"], length=ARM_LENGTH, compliance=ANCHOR_HIP_COMPLIANCE_BASE)

    idx["shoulder"] = sys_.add_point([0.0, HIP_Y - TORSO_LEN])
    sys_.add_stick(idx["hip"], idx["shoulder"], length=TORSO_LEN)

    idx["head"] = sys_.add_point([0.0, HIP_Y - TORSO_LEN - HEAD_STICK_LEN])
    sys_.add_stick(idx["shoulder"], idx["head"], length=HEAD_STICK_LEN)

    return sys_, idx


def make_leg(hip_pos: np.ndarray, initial_planted_offset: float,
             support_margin: float | None = None, swing_lead_margin: float | None = None) -> ActiveFootPlantingLeg:
    return ActiveFootPlantingLeg(
        hip_pos,
        segment_lengths=[LEG_SEGMENT_LEN, LEG_SEGMENT_LEN],
        ground_y=GROUND_Y,
        swing_duration_frames=SWING_DURATION_FRAMES,
        lift_height=LIFT_HEIGHT,
        initial_planted_offset=initial_planted_offset,
        knee_limits=KNEE_LIMITS,
        knee_bend_sign=KNEE_BEND_SIGN,
        capture_gain=1.0,
        support_margin=SUPPORT_MARGIN if support_margin is None else support_margin,
        swing_lead_margin=SWING_LEAD_MARGIN if swing_lead_margin is None else swing_lead_margin,
        omega0=OMEGA0,
        knee_forward_seed=KNEE_BEND_SIGN,  # Adim 17: +x yonunde ileri diz dali
    )


def draw_frame(body: VerletSystem, idx: dict, legs: list[ActiveFootPlantingLeg],
               camera_offset: float, hip_vx: float, fell: bool) -> np.ndarray:
    frame = np.full((H, W, 3), 22, dtype=np.uint8)
    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (85, 85, 85), 2)

    def to_screen(p):
        return (int(p[0] + camera_offset), int(p[1]))

    body_color = (90, 220, 90) if not fell else (90, 90, 220)
    leg_colors = [(90, 220, 90), (230, 160, 60)]

    torso_ids = [idx["hip"], idx["shoulder"]] if "waist" not in idx else [idx["hip"], idx["waist"], idx["shoulder"]]
    for a, b in zip(torso_ids, torso_ids[1:]):
        cv2.line(frame, to_screen(body.points[a]), to_screen(body.points[b]), body_color, 4, cv2.LINE_AA)
    cv2.line(frame, to_screen(body.points[idx["shoulder"]]), to_screen(body.points[idx["head"]]), body_color, 3, cv2.LINE_AA)
    cv2.circle(frame, to_screen(body.points[idx["head"]]), HEAD_RADIUS, body_color, 2, cv2.LINE_AA)

    # Adim 20: fiziksel kollar (varsa) -- omuz -> dirsek -> el
    if len(body.points) >= idx["head"] + 5:
        sh = body.points[idx["shoulder"]]
        for k, color in ((0, (90, 220, 90)), (2, (230, 160, 60))):
            e, h = body.points[idx["head"] + 1 + k], body.points[idx["head"] + 2 + k]
            cv2.line(frame, to_screen(sh), to_screen(e), color, 3, cv2.LINE_AA)
            cv2.line(frame, to_screen(e), to_screen(h), color, 3, cv2.LINE_AA)

    for leg, color in zip(legs, leg_colors):
        pts = leg.chain.points
        for i in range(len(pts) - 1):
            cv2.line(frame, to_screen(pts[i]), to_screen(pts[i + 1]), color, 4, cv2.LINE_AA)
        for p in pts:
            cv2.circle(frame, to_screen(p), 5, (255, 255, 255), -1, cv2.LINE_AA)

    status = "DUSTU" if fell else "yuruyor"
    color_txt = (110, 110, 255) if fell else (210, 210, 210)
    cv2.putText(frame, f"hip_vx: {hip_vx:+.2f}px/kare (hedef {TARGET_VX:.1f})   durum: {status}",
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color_txt, 1, cv2.LINE_AA)
    cv2.putText(frame, "Adim 14 (8. tur): kalca ARTIK kinematik-pin DEGIL -- capture-point tabanli dinamik biped",
                (12, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


class ActiveBipedSim:
    """Adim 14'un fizik dongusu, kare-kare adimlanabilen (stepable) bir
    nesne olarak. Adim 17 (Bilge derisi) refactor'u: dongu govdesi
    main()'den BIREBIR tasindi -- davranis DEGISMEDI (refactor oncesi/
    sonrasi 360 karenin tum Verlet + FABRIK noktalari bit-bit ayni,
    bkz. README "Adim 17"). Boylece ayni fizigi hem bu dosyanin cubuk-
    adam cizimi hem de `demo/step17_bilge_physics_skin.py`'nin 16 parcali
    Bilge derisi, fizik kodunu KOPYALAMADAN kullanabiliyor.

    Sahne olaylari (tokezleme/buyuk itki) ve capture-point marjlari
    varsayilan olarak modul sabitlerinden gelir; test/sahne/deney icin
    kwargs ile degistirilebilir (varsayilanlar step14'u DEGISTIRMEZ)."""

    def __init__(self, stumble_t: float | None = None, stumble_kick_px: float | None = None,
                 big_push_t: float | None = None, big_push_kick_px: float | None = None,
                 fps: int = FPS, support_margin: float | None = None,
                 swing_lead_margin: float | None = None, faz_b: bool | None = None,
                 shock_absorb: bool | None = None, predictive_sensor: bool | None = None,
                 arms_mode: str | None = None, arm_reflex: bool | None = None,
                 arm_gains: ArmGains | None = None, torso_clamp_mode=None,
                 posture_k: float | None = None, posture_c: float | None = None,
                 leg_mass: bool | None = None, thrust_mode: str | None = None,
                 catch_timing: str | None = None, hip_torque_max: float | None = None,
                 closing_ttc: float | None = None, shock_mode: str | None = None,
                 shock_trigger: str | None = None, preactivation: int | None = None,
                 rocker: bool | None = None, hip_strategy_gain: float | None = None,
                 hill: bool | None = None, brace: bool | None = None,
                 bracing: bool | str | None = None, fall_solver: str | None = None, articulated_spine: bool = True,
                 gravity_mode: str = "unified", balance_recovery: bool = True,
                 upright_head: bool = True, ground_recovery: bool = False,
                 recovery_reposition: bool = False, recovery_transfer: bool = False,
                 recovery_rise: bool = False, recovery_stand: bool = False,
                 recovery_walk: bool = False):
        self.gravity_policy = GravityPolicy(gravity_mode)
        if ground_recovery and (gravity_mode!='unified' or not articulated_spine or
                                arms_mode=='off' or not balance_recovery):
            raise ValueError('Ground recovery requires unified gravity, arms, spine and balance recovery')
        if recovery_reposition and not ground_recovery:
            raise ValueError('Repositioning requires ground_recovery=True')
        if recovery_transfer and not (ground_recovery and recovery_reposition):
            raise ValueError('Foot transfer requires ground_recovery=True and recovery_reposition=True')
        if recovery_rise and not recovery_transfer:
            raise ValueError('Kneeling rise requires recovery_transfer=True')
        if recovery_stand and not recovery_rise:
            raise ValueError('Standing recovery requires recovery_rise=True')
        if recovery_walk and not recovery_stand:
            raise ValueError('Walking recovery requires recovery_stand=True')
        if recovery_walk:
            from physics.walk_recovery import WalkRecovery
            self._recovery_factory = WalkRecovery
        elif recovery_stand:
            from physics.stand_recovery import StandRecovery
            self._recovery_factory = StandRecovery
        elif recovery_rise:
            from physics.kneel_rise import KneelRise
            self._recovery_factory = KneelRise
        elif recovery_transfer:
            from physics.foot_transfer import FootTransfer
            self._recovery_factory = FootTransfer
        elif ground_recovery:
            self._recovery_factory = lambda: GroundRecovery(reposition=recovery_reposition)
        else:
            self._recovery_factory = None
        self.recovery = self._recovery_factory() if self._recovery_factory is not None else None
        # Override: kalkma sirasinda dusus algilanirsa (XCoM destek disinda ya da
        # bas/gogus hizla zemine yaklasiyor) kalkma kesilir, koruyucu refleks yeniden
        # kurulur; durulunca kalkma bastan baslar (bkz. _check_recovery_override).
        self.recovery_aborts = []    # (kare, kesilen durum, neden)
        self._override_frames = 0
        self.balance_recovery = balance_recovery and gravity_mode == "unified"
        # yon kapisi (BRACE_DIR_MIN_PX) TTC tetigiyle birlikte: eski kosulda kapali
        self.brace_dir_gate = gravity_mode == "unified" or BRACE_TTC_IN_LEGACY
        self.upright_head = upright_head and gravity_mode == "unified"
        if gravity_mode == "unified" and fps != FPS:
            raise ValueError("Unified biped is calibrated at 30 Hz; render resampling is separate")
        # Adim 30 birlesimi: varsayilan koruyucu refleks Zehra'nin kol kolonu
        # refleksidir (bracing=True / brace=True). Gul Nihal'in impuls tabanli
        # FallBracing'i bracing="impulse" ile secilir; False ikisini de kapatir.
        if brace is not None:
            mode = "reflex" if brace else "off"
        elif bracing is None or bracing is True:
            mode = "reflex" if BRACE_ENABLED else "off"
        elif bracing is False:
            mode = "off"
        elif bracing == "impulse":
            mode = "impulse"
        else:
            raise ValueError("bracing must be True, False or 'impulse'")
        self.bracing_mode = mode
        self.stumble_t = STUMBLE_T if stumble_t is None else stumble_t
        self.stumble_kick_px = STUMBLE_KICK_PX if stumble_kick_px is None else stumble_kick_px
        self.big_push_t = BIG_PUSH_T if big_push_t is None else big_push_t
        self.big_push_kick_px = BIG_PUSH_KICK_PX if big_push_kick_px is None else big_push_kick_px
        self.fps = fps
        self.dt = 1.0 / fps
        self.faz_b = FAZ_B_ENABLED if faz_b is None else faz_b
        self.torso_clamp_mode = TORSO_CLAMP_MODE if torso_clamp_mode is None else torso_clamp_mode
        self.posture_k = POSTURE_K if posture_k is None else posture_k
        self.posture_c = POSTURE_C if posture_c is None else posture_c
        use_lm = LEG_MASS_ENABLED if leg_mass is None else leg_mass
        self.leg_mass = LegMassModel(np.array([0.,self.gravity_policy.walking])) if use_lm else None
        self.predictive_sensor = PREDICTIVE_SENSOR_ENABLED if predictive_sensor is None else predictive_sensor
        self.fazb_events = []   # (kare, bacak, 'compress'|'launch', 'toe'|'heel', bacak_acisi)
        self.catch_frames_log = []   # Adim 23: (inis karesi, bacak, kalkistan inise kare)
        self.shock_absorb = SHOCK_ABSORB_ENABLED if shock_absorb is None else shock_absorb
        self.shock_pending = None   # yakalama inisi yapan bacak (anchor ona gecince baslar)
        self.shock_rest = None      # None: sok emilimi aktif degil
        self.shock_events = []      # (kare, baslangic_dinlenme_boyu)
        self.shock_vel = 0.0
        self.shock_mode = SHOCK_MODE if shock_mode is None else shock_mode
        self.preact_frames = PREACT_FRAMES if preactivation is None else preactivation
        self.rocker = ROCKER_ENABLED if rocker is None else rocker
        self.rocker_log = []
        self.hip_strategy_gain = hip_strategy_gain
        self.last_real_error = 0.0
        self.shock_pending_v0 = 0.0
        self.shock_act = 1.0
        self.collapsed = False
        self.fallen_legs = None     # Adim 29: {"l": (diz, ayak), "r": ...} Verlet indeksleri
        self.brace = self.bracing_mode == "reflex"
        self.bracing = False
        self.brace_level = 0.0
        self.brace_dir = 0.0
        self.brace_start = None
        self.brace_plant_x = None
        self.brace_lowering = False
        self.brace_neck_deg = None
        self.brace_lowered = False
        self.yield_frames = 0
        self.settle_count = 0
        self.brace_log = []         # Adim 30: (kare, seviye, hedef_aci_der)
        self.impact_log = []        # Adim 30: (kare, nokta, dikey carpma hizi px/kare)
        self._contact_prev = None
        self.arm_column = {}        # Adim 30: kol -> dinlenme boyu (yalniz itme), el yerdeyken
        self.last_contact_ttc = None
        self.neck_soft = None
        self.brace_trigger = None   # 'ttc' | 'yield' | 'collapse' | 'override'
        self.arm_yield_log = []     # (kare, kol, cozulemeyen hiz px/kare)
        self.hill = HILL_ENABLED if hill is None else hill
        self.collapse_frame = None
        self.collapse_reason = None
        self.fall_bracing = FallBracing() if self.bracing_mode == "impulse" else None
        # Yigilma cozucusu: "column" Adim 30'un (24 tur, yuke bagli statik surtunme,
        # son kilit, giris duzeltmesi, dongu ici boyun); "gn" Gul Nihal'in dalindakini
        # birebir korur. Varsayilan: impuls refleksi ya da yerden kalkma ile "gn", digerlerinde "column".
        if fall_solver not in (None, "column", "gn"):
            raise ValueError("fall_solver must be None, 'column' or 'gn'")
        self.gn_fall = (fall_solver == "gn") or (fall_solver is None and
                                                 (self.fall_bracing is not None or ground_recovery))
        # Yerde kollar: Gul Nihal'in kutle agirlikli dirsek cozucusu (dongu icinde) impuls
        # refleksiyle ve yerden kalkma zinciriyle; zincir hangi refleksle olursa olsun bu
        # kol cozumuyle dogrulandi (yuruyus kol kisiti yerde kalkmayi bozuyordu, olculdu).
        # yerden kalkma: saf Coulomb surtunmesi (yuke bagli statik + kinetik, yapistirici yok)
        self.coulomb_friction = ground_recovery and COULOMB_RECOVERY
        self.gn_fallen_arms = self.gn_fall and (self.fall_bracing is not None or ground_recovery)
        self.articulated_spine = articulated_spine
        self.spine = None
        self.ground_contacts = []
        self.ground_projection_impulses = []
        self.leg_force_log = []     # Adim 27: (kare, d, F_toplam/W, a, r, v)
        self.preact_log = []        # Adim 25: (temas karesi, bacak, temastaki hazirlik hizi)
        self.shock_trigger = SHOCK_TRIGGER if shock_trigger is None else shock_trigger
        self.contact_log = []       # Adim 24: (kare, bacak, temas sikismasi px)

        self.body, self.idx = build_body(self.gravity_policy.walking)
        self.hip_base_mass = float(self.body.masses[self.idx["hip"]])
        self.thrust_mode = THRUST_MODE if thrust_mode is None else thrust_mode
        self.arms_mode = ARMS_MODE if arms_mode is None else arms_mode
        self.arm_reflex = ARM_REFLEX if arm_reflex is None else arm_reflex
        self.arms = (PhysicalArms(self.body, self.idx["shoulder"], arm_gains)
                     if self.arms_mode != "off" else None)
        self.upper_ids = [self.idx["hip"], self.idx["shoulder"], self.idx["head"]]
        if self.arms is not None:
            self.upper_ids += [i for pair in self.arms.idx.values() for i in pair]
        self.reflex_level = 0.0
        self.reflex_hold = 0
        self.reflex_dir = 0.0
        self.terrain_static = Terrain(ground_y=GROUND_Y, default_friction=GROUND_MU_STATIC,
                                      zones=list(ICE_ZONES))
        kinetic_zones = [(x0, x1, mu * KINETIC_RATIO) for (x0, x1, mu) in ICE_ZONES]
        self.terrain_kinetic = Terrain(ground_y=GROUND_Y, default_friction=GROUND_MU_STATIC * KINETIC_RATIO,
                                       zones=kinetic_zones)

        hip = self.idx["hip"]
        half = ARM_LENGTH * 0.15
        if gravity_mode == "unified":
            support_margin = UNIFIED_SUPPORT_MARGIN if support_margin is None else support_margin
            swing_lead_margin = UNIFIED_SWING_LEAD_MARGIN if swing_lead_margin is None else swing_lead_margin
        self.left_leg = make_leg(self.body.points[hip].copy(), -half, support_margin, swing_lead_margin)
        self.right_leg = make_leg(self.body.points[hip].copy(), +half, support_margin, swing_lead_margin)
        for leg in (self.left_leg, self.right_leg):
            leg.balance_recovery = self.balance_recovery
            leg.gravity_y = self.gravity_policy.walking
            leg.omega0 = self.gravity_policy.capture_frequency(ARM_LENGTH)
            leg.faz_b_enabled = self.faz_b
            leg.predictive_sensor = self.predictive_sensor
            leg.catch_timing = catch_timing          # None: physics.active_gait.CATCH_TIMING
            leg.hip_torque_max = hip_torque_max      # None: physics.active_gait.HIP_TORQUE_MAX
            leg.closing_ttc_frames = closing_ttc     # None: physics.active_gait.FAZB_CLOSING_TTC_FRAMES
            leg.hill = hill                          # None: physics.active_gait.HILL_ENABLED
        # sag bacak baslangicta swing'de -- alternatif adimla baslamasi icin.
        self.right_leg.state = "swing"
        self.right_leg.swing_start = self.right_leg.planted.copy()
        self.right_leg.swing_target = np.array([half + 20.0, GROUND_Y])
        self.right_leg.swing_t = 0.0

        self.stumbled = False
        self.big_pushed = False
        self.fell = False
        self.fall_frame = None
        self.step_events = []
        self.slip_events = []
        self.emergency_step_events = []  # 13. tur eki 3 -- FallRiskMonitor tabanli acil adim tetiklemeleri
        self.risk_monitor = FallRiskMonitor(FALL_RISK_ENTER_PX, FALL_RISK_EXIT_PX)
        self.hip_x_log = []
        self.hip_y_log = []
        self.hip_vx_log = []
        self.frame = 0
        self.last_hip_vx = 0.0
        self.last_ctrl_vx = 0.0
        self.trunk_yaw = TrunkYaw()
        self._yaw_prev = None
        self.last_leg_yaw = 0.0
        self.last_arm_yaw = 0.0
        self.last_in_danger = False
        self.nan = False

    def com_x(self) -> float:
        """Govde (kalca+omuz+bas) -- kollar varsa kutle-agirlikli olarak dahil."""
        idx, p = self.idx, self.body.points
        base = [idx["hip"], idx["shoulder"], idx["head"]]
        if self.spine is not None:
            ids = self.upper_ids
            return float(np.average(p[ids, 0], weights=self.body.masses[ids]))
        if self.arms is None:
            return upper_body_com_x(p, base)
        ids = base + [i for pair in self.arms.idx.values() for i in pair]
        m = self.body.masses[ids].copy()
        m[0] = self.hip_base_mass   # Adim 22: kalcaya eklenen ortuk bacak kutlesi denge olcusunu degistirmesin
        return float(np.sum(p[ids, 0] * m) / np.sum(m))

    def _update_trunk_yaw(self) -> None:
        """Adim 22 -- 2.5B govde yaw defteri (bkz. physics/trunk_yaw.py).
        Hizlar konum farkindan (bu kare - onceki kare), kalcaya gore."""
        p = self.body.points
        hip_x = float(p[self.idx["hip"], 0])
        now = {"hip": hip_x, "foot_l": float(self.left_leg.foot_target[0]),
               "foot_r": float(self.right_leg.foot_target[0])}
        if self.arms is not None:
            for side in ("l", "r"):
                e, h = self.arms.idx[side]
                now["e" + side], now["h" + side] = float(p[e, 0]), float(p[h, 0])
        prev = self._yaw_prev
        self._yaw_prev = now
        if prev is None:
            return
        v_hip = now["hip"] - prev["hip"]
        legs = []
        for side in ("l", "r"):
            v_foot = now["foot_" + side] - prev["foot_" + side]
            lat = side_sign(side) * HIP_HALF_WIDTH
            for frac, mm in ((THIGH_AXIS_FRAC, THIGH_MASS), (SHANK_AXIS_FRAC, SHANK_MASS)):
                legs.append((lat, mm, frac * (v_foot - v_hip)))
        arms = []
        if self.arms is not None:
            for side in ("l", "r"):
                e, h = self.arms.idx[side]
                lat = side_sign(side) * SHOULDER_HALF_WIDTH
                for key, i in (("e", e), ("h", h)):
                    arms.append((lat, float(self.body.masses[i]), now[key + side] - prev[key + side] - v_hip))
        self.last_leg_yaw = yaw_momentum(legs)
        self.last_arm_yaw = yaw_momentum(arms)
        self.trunk_yaw.update(self.last_leg_yaw, self.last_arm_yaw)

    def _neck_min_dist(self) -> float | None:
        """Adim 30: koruyucu refleks boyun tonusunu da kapsar -- bas govdeyle
        hizali tutulur. Boyun acisi siniri, kalca-bas arasinda yalniz itebilen
        bir en kisa mesafe olarak yigilma relaksasyon dongusunde cozulur:
        |kalca-bas|^2 = L_g^2 + L_b^2 + 2 L_g L_b cos(theta). Disaridaki
        clamp_direction ile 18 derece denendi: yerde bas her kare dusup geri
        kaldirilarak 4 px/kare titresiyor, eller zeminden sekiyordu (olculdu;
        Adim 29'daki 18 derece bulgusunun aynisi)."""
        if not self.collapsed or self.gn_fall:
            return None
        if self.bracing and self.brace_neck_deg is not None:
            th = np.radians(self.brace_neck_deg)
        elif FALLEN_NECK_IN_LOOP:
            th = np.radians(FALLEN_NECK_LIMIT_DEG)
        else:
            return None
        # omurga varsa boyun siniri bel-bas arasinda (bel-omuz segmenti; bkz. neck_root)
        root_len = self.spine.segment_length if self.spine is not None else TORSO_LEN
        return float(np.sqrt(root_len ** 2 + HEAD_STICK_LEN ** 2 + 2 * root_len * HEAD_STICK_LEN * np.cos(th)))

    def _brace_side(self) -> float:
        """Omzun inecegi taraf: govde kalca etrafinda devrilir, belirleyici
        omzun KALCAYA GORE konumu ve hizidir. Mutlak omuz hizi denendi: geri
        itkide kalca geri kacarken govde one katlaniyor, refleks eli kalcanin
        arkasina uzatip omuz cizgisinden kaciriyordu (olculdu)."""
        idx, p, q = self.idx, self.body.points, self.body.prev_points
        sh, hp = idx["shoulder"], idx["hip"]
        return 1.0 if self._brace_lean() >= 0.0 else -1.0

    def _brace_lean(self) -> float:
        """Omzun kalcaya gore (BRACE_LOOKAHEAD kare ilerisi) yatay konumu, px."""
        idx, p, q = self.idx, self.body.points, self.body.prev_points
        sh, hp = idx["shoulder"], idx["hip"]
        rel = p[sh][0] - p[hp][0]
        v_rel = (p[sh][0] - q[sh][0]) - (p[hp][0] - q[hp][0])
        return float(rel + BRACE_LOOKAHEAD * v_rel)

    def contact_ttc(self) -> float | None:
        """Bas ve gogsun (omuz) zemine balistik carpma suresi, kare; en kucugu.
        t: y + v t + g t^2 / 2 = zemin - yaricap (g > 0 oldugundan her zaman
        tanimli; yukari giden nokta icin daha uzun). Yerdeki nokta 0 dondurur."""
        idx, p, q = self.idx, self.body.points, self.body.prev_points
        g = float(self.gravity_policy.falling)
        best = None
        for name in ("head", "shoulder"):
            i = idx[name]
            gap = (GROUND_Y - FALLEN_RADIUS[name]) - float(p[i, 1])
            v = float(p[i, 1] - q[i, 1])
            if gap <= 0.0:
                t = 0.0
            else:
                disc = v * v + 2.0 * g * gap
                t = (-v + float(np.sqrt(disc))) / g
            best = t if best is None else min(best, t)
        return best

    def _impact_imminent(self) -> bool:
        if self.collapsed and self.bracing:
            return False
        if self.gravity_policy.mode != "unified" and not BRACE_TTC_IN_LEGACY:
            return False
        ttc = self.contact_ttc()
        self.last_contact_ttc = ttc
        return ttc is not None and ttc <= BRACE_TTC_FRAMES

    def _neck_xpbd_active(self) -> bool:
        if NECK_XPBD_ENABLED or NECK_XPBD_MODE == "always":
            return True
        return (NECK_XPBD_MODE == "passive" and self.bracing_mode == "off"
                and self.gravity_policy.mode == "unified")

    def push(self, kick_px: float, point: str = "hip") -> None:
        """Dis darbe: noktaya yatay hiz (px/kare) ekler -- buyuk itkiyle ayni olcek
        (taban kalca kutlesine gore momentum)."""
        i = self.idx[point]
        self.body.prev_points[i][0] -= kick_px * self.hip_base_mass / float(self.body.masses[i])

    def _support_xcom_margin(self) -> float | None:
        """XCoM'un (KM + v / omega0) bu karedeki temas noktalarinin x araligina
        gore disarida kalan mesafesi (px; <= 0 icerde). Temas yoksa None."""
        body = self.body
        p, q, m = body.points, body.prev_points, body.masses
        contacts = getattr(self, "contact_ids", [])
        if not contacts:
            return None
        free = [i for i in range(len(p)) if i not in body.pinned]
        com = np.average(p[free], axis=0, weights=m[free])
        v = np.average((p - q)[free], axis=0, weights=m[free])
        h = max(GROUND_Y - float(com[1]), 1.0)
        omega = float(np.sqrt(float(body.gravity[1]) / h))
        xcom = float(com[0] + v[0] / omega)
        if h < OVERRIDE_MIN_COM_HEIGHT:
            return None      # yatarken/el-diz ustunde destek araligi anlamli degil
        lo, hi = float(p[contacts, 0].min()), float(p[contacts, 0].max())
        return max(lo - xcom, xcom - hi)

    def _check_recovery_override(self) -> None:
        rec = self.recovery
        if rec is None or rec.state in RECOVERY_IDLE_STATES:
            self._override_frames = 0
            return
        reason = None
        margin = self._support_xcom_margin()
        if margin is not None and margin > OVERRIDE_XCOM_MARGIN:
            reason = "xcom"
        else:
            idx, p, q = self.idx, self.body.points, self.body.prev_points
            falling = max(float(p[idx[n], 1] - q[idx[n], 1]) for n in ("head", "shoulder"))
            gap = min(GROUND_Y - FALLEN_RADIUS[n] - float(p[idx[n], 1]) for n in ("head", "shoulder"))
            ttc = self.contact_ttc()
            if (falling > OVERRIDE_MIN_VY and gap > OVERRIDE_MIN_GAP and ttc is not None
                    and ttc <= BRACE_TTC_FRAMES):
                reason = "ttc"
        self._override_frames = self._override_frames + 1 if reason else 0
        if self._override_frames < OVERRIDE_FRAMES:
            return
        self.recovery_aborts.append((self.frame, rec.state, reason))
        self._override_frames = 0
        self.recovery = self._recovery_factory()
        self._rearm_fall_reflex()

    def _rearm_fall_reflex(self) -> None:
        """Kalkma kesildi: etkin koruyucu refleks yeniden kurulur ve hemen acilir."""
        if self.fall_bracing is not None:
            self.fall_bracing = FallBracing()      # bosta; gogus TTC'si kisalinca kendisi uzanir
        if self.brace and self.arms is not None:
            self.bracing = True
            self.brace_trigger = "override"
            self.brace_level = 1.0
            self.brace_start = self.frame
            self.brace_dir = self._brace_side()
            self.brace_plant_x = None
            self.brace_lowering = False
            self.brace_lowered = False
            self.brace_neck_deg = None
            self.settle_count = 0
            self.arm_column = {}

    def _update_brace(self, f: int) -> None:
        """Adim 30 -- koruyucu refleksin acilmasi/sonmesi."""
        if not self.brace or self.arms is None:
            return
        idx, p, q = self.idx, self.body.points, self.body.prev_points
        imminent = self._impact_imminent()
        if not self.bracing and (self.collapsed or self.yield_frames >= BRACE_YIELD_FRAMES or imminent):
            self.brace_trigger = ("ttc" if imminent else "yield" if self.yield_frames >= BRACE_YIELD_FRAMES
                                  else "collapse")
            self.brace_dir = self._brace_side()
            self.bracing = True
            self.brace_level = 1.0
            self.brace_start = f
        if not self.bracing:
            return
        if not self.collapsed and self.yield_frames == 0 and self.shock_rest is None and not imminent:
            # bacak toparlandi (cokus olmadi) -- refleks soner
            self.brace_level *= BRACE_RELAX_RATE
        elif self.collapsed:
            v = float(np.linalg.norm(p[idx["shoulder"]] - q[idx["shoulder"]]))
            self.settle_count = self.settle_count + 1 if v < BRACE_SETTLE_SPEED else 0
            if self.settle_count >= BRACE_SETTLE_FRAMES:
                # durulma: once kollar eksantrik olarak govdeyi yere indirir
                # (kolon boyu BRACE_LOWER_RATE ile kisalir), sonra refleks soner
                self.brace_lowering = True
            if self.brace_neck_deg is None:
                self.brace_neck_deg = NECK_MAX_TILT_DEG
            if self.brace_lowering:
                # bas da yavasca birakilir: govdeyi bir payanda gibi tutmasin, birden de dusmesin
                self.brace_neck_deg = min(FALLEN_NECK_LIMIT_DEG, self.brace_neck_deg + BRACE_NECK_RELAX_DEG)
            if self.brace_lowering and self.brace_lowered and self.brace_neck_deg >= FALLEN_NECK_LIMIT_DEG:
                self.brace_level *= BRACE_RELAX_RATE
        if self.brace_level < 0.01 and not self.collapsed:
            self.bracing = False
            self.brace_level = 0.0

    @property
    def fall_reflex_active(self) -> bool:
        """Yerde koruyucu refleks hala surulyor mu (yerden kalkma bunu bekler)."""
        if self.fall_bracing is not None:
            return bool(self.fall_bracing.active)
        return bool(self.bracing and self.brace_level > 0.01)

    def _collapse(self, reason):
        if self.collapsed:
            return
        self.collapsed = True
        self.collapse_frame = self.frame
        self.collapse_reason = reason
        if not self.fell:
            self.fell = True
            self.fall_frame = self.frame
        self._enter_fallen()

    def _enter_fallen(self) -> None:
        """Adim 29 -- cokus: bacaklar kinematikten kutleli Verlet zincirlerine gecer."""
        body, idx = self.body, self.idx
        hip = idx["hip"]
        v_hip = body.points[hip] - body.prev_points[hip]
        self.fallen_legs = {}
        for key, leg in (("l", self.left_leg), ("r", self.right_leg)):
            knee = leg.chain.points[1].copy()
            foot = (np.array([leg.planted[0], GROUND_Y]) if leg.state == "stance"
                    else np.asarray(leg.foot_target, float).copy())
            if not self.gn_fall:
                knee, foot = self._fallen_leg_init(body.points[hip].copy(), knee, foot)
            k_i = body.add_point(knee, mass=FALLEN_KNEE_MASS)
            f_i = body.add_point(foot, mass=FALLEN_FOOT_MASS)
            # diz kalcayla birlikte hareket ediyordu; yerdeki ayak duruyordu
            body.prev_points[k_i] = knee - 0.5 * v_hip
            body.prev_points[f_i] = foot if leg.state == "stance" else foot - v_hip
            body.add_stick(hip, k_i, length=LEG_SEGMENT_LEN)
            body.add_stick(k_i, f_i, length=LEG_SEGMENT_LEN)
            self.fallen_legs[key] = (k_i, f_i)
        i0, j0, r0, _ = body.sticks[0]
        body.sticks[0] = (i0, j0, r0, 1.0)          # anchor cubugu: compliance 1 -> etkisiz
        body.masses[hip] = self.hip_base_mass       # ortuk bacak kutlesi artik ayri noktalarda
        # Unified policy has no acceleration switch at collapse.
        if self.gravity_policy.mode == "legacy":
            body.gravity = np.array([0.0, self.gravity_policy.falling])
        if self.articulated_spine:
            self.spine = FallenSpine(body, hip, idx["shoulder"])
            idx["waist"] = self.spine.waist
            self.upper_ids.append(self.spine.waist)
        self.elbow_side = {}
        if self.arms is not None:
            sh = idx["shoulder"]
            for key, (e_i, h_i) in self.arms.idx.items():
                d, v = body.points[h_i] - body.points[sh], body.points[e_i] - body.points[sh]
                c = -(d[0] * v[1] - d[1] * v[0])
                self.elbow_side[key] = 1.0 if c >= 0.0 else -1.0

    def _fallen_leg_init(self, hp_: np.ndarray, knee: np.ndarray, foot: np.ndarray):
        """Adim 30 duzeltmesi -- yigilmaya giriste kinematik bacagi gecerli bir
        ragdoll durusuna cevirir. Cokus gec tetiklenip kalca zemine cok
        yaklastiysa FABRIK dizi zeminin 58-71 px ALTINA bukebiliyor ya da bacagi
        150 dereceden fazla katlayabiliyordu (kalca-ayak 30 px); zemin/menteşe
        siniri ilk karede bunu duzeltirken cubuklar kalcayi 70-150 px yukari
        firlatiyordu (olculdu). Gecerli durus (diz zemin ustunde, geri bukulme
        yok, bukulme <= 150 der) aynen korunur; degilse yeni olusan AYAK noktasi
        zemin boyunca kalcadan uzaklastirilir ve diz iki kemik IK ile dogru
        tarafta yeniden kurulur (govdenin momentumuna dokunulmaz)."""
        l = LEG_SEGMENT_LEN
        knee_floor = GROUND_Y - FALLEN_RADIUS["knee"]

        def valid(k, ft):
            u, v = ft - hp_, k - hp_
            n = float(np.linalg.norm(u))
            return (k[1] <= knee_floor and n >= KNEE_FLEX_MIN_DIST - 1e-6 and
                    -(u[0] * v[1] - u[1] * v[0]) * FALLEN_KNEE_BEND_SIGN >= 0.0 and
                    abs(float(np.linalg.norm(v)) - l) < 2.0 and abs(float(np.linalg.norm(k - ft)) - l) < 2.0)

        if valid(knee, foot):
            return knee, foot

        def ik(ft):
            u = ft - hp_
            n = float(np.linalg.norm(u))
            h = float(np.sqrt(max(l * l - 0.25 * n * n, 0.0)))
            perp = np.array([-u[1], u[0]]) / n
            if -(u[0] * perp[1] - u[1] * perp[0]) * FALLEN_KNEE_BEND_SIGN < 0.0:
                perp = -perp
            return hp_ + 0.5 * u + perp * h

        dy = float(foot[1] - hp_[1])
        sx = 1.0 if foot[0] >= hp_[0] else -1.0
        n0 = max(float(np.linalg.norm(foot - hp_)), KNEE_FLEX_MIN_DIST * 1.01, abs(dy) + 0.5)
        best = None
        for n in np.arange(n0, 2.0 * l - 0.5, 2.0):
            ft = np.array([hp_[0] + sx * float(np.sqrt(max(n * n - dy * dy, 0.0))), foot[1]])
            k = ik(ft)
            if best is None:
                best = (k, ft)
            if k[1] <= knee_floor:
                return k, ft
        if best is None:   # ayak bacak boyundan uzakta (salinim hedefi): erisilebilir uca cek
            u = foot - hp_
            ft = hp_ + u / float(np.linalg.norm(u)) * (2.0 * l - 0.5)
            best = (ik(ft), ft)
        return best

    def _fallen_constraints(self) -> None:
        """Adim 29 -- yigilma: diz menteşe siniri + esnek olmayan zemin + kinetik surtunme.
        Zemin, cubuk cozucusuyle AYNI relaksasyon dongusunde uygulanir: yalniz
        sonda uygulandiginda cubuklar her kare noktalari zemine geri itiyor ve
        surtunmeyle birlikte kare basina ~1 px "surunme" uretiyordu (olculdu)."""
        body, idx = self.body, self.idx
        p, q = body.points, body.prev_points
        hip = idx["hip"]
        radius = np.zeros(len(p))
        radius[hip] = FALLEN_RADIUS["hip"]
        radius[idx["shoulder"]] = FALLEN_RADIUS["shoulder"]
        radius[idx["head"]] = FALLEN_RADIUS["head"]
        if self.spine is not None:
            radius[self.spine.waist] = WAIST_RADIUS
        for k_i, f_i in self.fallen_legs.values():
            radius[k_i] = FALLEN_RADIUS["knee"]
            radius[f_i] = FALLEN_RADIUS["foot"]
        if self.arms is not None:
            for e_i, h_i in self.arms.idx.values():
                radius[e_i] = FALLEN_RADIUS["arm"]
                radius[h_i] = FALLEN_RADIUS["arm"]
        mask = np.ones(len(p), dtype=bool)
        mask[idx["anchor"]] = False
        floor = GROUND_Y - radius
        x_before = p[:, 0].copy()
        incoming_vy = (p - q)[:, 1].copy()
        contact = np.zeros(len(p), dtype=bool)
        normal_projection = np.zeros(len(p))
        # Diz menteşesi momentum koruyarak: (1) en fazla 150 derece bukulme =
        # kalca-ayak mesafesi >= KNEE_FLEX_MIN_DIST (kutle agirlikli, yalniz itme);
        # (2) geri bukulme yok: diz kalca-ayak cizgisinin arkasina dusunce aynalanir.
        # fabrik.clamp_joint_angle_points yalnizca uc noktayi oynattigi icin zeminle
        # birlikte kare basina 0.5-2 px "surunme" uretiyordu (olculdu).
        m_ = body.masses
        x0 = getattr(self, "_fallen_x0", None)
        pairs = [(hip, k_i, f_i, KNEE_FLEX_MIN_DIST) for k_i, f_i in self.fallen_legs.values()]
        if self.arms is not None and not self.gn_fall:
            pairs += [(idx["shoulder"], e_i, h_i, ELBOW_FLEX_MIN_DIST) for e_i, h_i in self.arms.idx.values()]

        def knee_flex_limit():
            # Adim 30: ayni momentum koruyan bukulme siniri dirsekler icin de (omuz-el >= 140 der)
            for a_i, _m, b_i, dmin in pairs[len(self.fallen_legs):]:
                dvec = p[b_i] - p[a_i]
                dist = float(np.linalg.norm(dvec))
                if 1e-6 < dist < dmin:
                    wi, wj = 1.0 / m_[a_i], 1.0 / m_[b_i]
                    corr = dvec / dist * (dmin - dist)
                    p[a_i] -= corr * wi / (wi + wj)
                    p[b_i] += corr * wj / (wi + wj)
            for k_i, f_i in self.fallen_legs.values():
                dvec = p[f_i] - p[hip]
                dist = float(np.linalg.norm(dvec))
                if 1e-6 < dist < KNEE_FLEX_MIN_DIST:
                    wi, wj = 1.0 / m_[hip], 1.0 / m_[f_i]
                    corr = dvec / dist * (KNEE_FLEX_MIN_DIST - dist)
                    p[hip] -= corr * wi / (wi + wj)
                    p[f_i] += corr * wj / (wi + wj)

        # Adim 30: kol kolonlari -- el yerde ve refleks aktifken omuz-el yalniz itme destegi
        columns = []
        if self.arms is not None and self.bracing and self.brace_level > 0.5:
            sh = idx["shoulder"]
            for key, (e_i, h_i) in self.arms.idx.items():
                on_ground = p[h_i][1] >= floor[h_i] - 0.5
                if not on_ground:
                    self.arm_column.pop(key, None)
                    continue
                dvec = p[sh] - p[h_i]
                d = float(np.linalg.norm(dvec))
                if d < 1e-6:
                    continue
                r = self.arm_column.get(key, min(d, 34.0 + 32.0))
                # omzun ele dogru (sikistiran) hizi; kapasitenin durduramadigi kismi kolu buker
                u = dvec / d
                v_c = -float(np.dot(p[sh] - q[sh], u))
                a_cap = self.arm_column_capacity(r, v_c)
                if v_c > a_cap:
                    r = max(ELBOW_FLEX_MIN_DIST, r - (v_c - a_cap))
                    self.arm_yield_log.append((self.frame, key, round(v_c - a_cap, 2)))
                r = min(r, d) if v_c <= 0.0 else r
                if self.brace_lowering:
                    r = max(ELBOW_FLEX_MIN_DIST, r - BRACE_LOWER_RATE)
                self.arm_column[key] = r
                columns.append((sh, h_i, r))

        if self.brace_lowering and not self.brace_lowered:
            sh_down = p[idx["shoulder"]][1] >= floor[idx["shoulder"]] - 0.5
            at_min = all(r <= ELBOW_FLEX_MIN_DIST + 1e-6 for _a, _b, r in columns)
            self.brace_lowered = sh_down or at_min

        neck_min = self._neck_min_dist()
        # boyun koku: tek parca govdede kalca, omurgada bel (bas omuzun ust parcasina gore sinirlanir)
        neck_root = self.spine.waist if self.spine is not None else hip
        if neck_min is not None:
            columns.append((idx["head"], neck_root, neck_min))
        neck_col = len(columns) - 1 if neck_min is not None else None

        def arm_columns():
            for c_i, (a_i, b_i, r) in enumerate(columns):
                if c_i == neck_col:
                    # boyun tonusu: kok-bas en kisa mesafe, kutle agirlikli (zemine dayanmiyor)
                    dvec = p[a_i] - p[b_i]
                    dist = float(np.linalg.norm(dvec))
                    if 1e-6 < dist < r:
                        corr = dvec / dist * (r - dist)
                        wi, wj = 1.0 / m_[a_i], 1.0 / m_[b_i]
                        p[a_i] += corr * wi / (wi + wj)
                        p[b_i] -= corr * wj / (wi + wj)
                    continue
                dvec = p[a_i] - p[b_i]
                dist = float(np.linalg.norm(dvec))
                if 1e-6 < dist < r:
                    corr = dvec / dist * (r - dist)
                    # yerdeki el: kolun dikeyle acisi surtunme konisinin icindeyse zemin
                    # tepkiyi karsilar (el kaymaz, tum duzeltme omuza); disindaysa el kayar
                    if abs(dvec[0]) <= FALLEN_HAND_MU * abs(dvec[1]):
                        p[a_i] += corr
                    else:
                        wi, wj = 1.0 / m_[a_i], 1.0 / m_[b_i]
                        p[a_i] += corr * wi / (wi + wj)
                        p[b_i] -= corr * wj / (wi + wj)

        def knee_branch_limit():
            # Solve the knee half-plane together with ground and lengths.
            # The old post-solve single-point reset shortened both bones and
            # repeatedly nudged a resting body in nearly straight poses.
            for k_i, f_i in self.fallen_legs.values():
                a, b = p[k_i]-p[hip], p[f_i]-p[k_i]
                area = (a[0]*b[1]-a[1]*b[0])*FALLEN_KNEE_BEND_SIGN
                if area >= 0:
                    continue
                ids = [hip, k_i, f_i]
                gradient = np.array([[-b[1], b[0]],
                                     [a[1]+b[1], -a[0]-b[0]],
                                     [-a[1], a[0]]])*FALLEN_KNEE_BEND_SIGN
                inverse_mass = 1/m_[ids]
                effective = float(np.sum(inverse_mass[:, None]*gradient**2))
                p[ids] -= area/max(effective, 1e-9)*inverse_mass[:, None]*gradient

        # Coupled waist, arm and ground contacts need 64 iterations to keep
        # torso lengths and settling stable; step-30 and legacy counts stay unchanged.
        if self.gn_fall:
            iterations = GN_RELAX_ITERS_FALLEN * (8 if self.spine is not None else 2 if self.gn_fallen_arms else 1)
        else:
            # kol kolonu refleksi: Adim 30'un 24 turu; omurgada en az 64 tur
            iterations = max(RELAX_ITERS_FALLEN, GN_RELAX_ITERS_FALLEN * (8 if self.spine is not None else 1))
        iterations *= getattr(self.recovery, 'constraint_iterations_multiplier', 1)
        dn = normal_projection     # bu kare zeminin noktaya verdigi toplam normal duzeltme
        tcorr = np.zeros(len(p))   # Coulomb: bu kare noktaya verilen toplam tegetsel duzeltme
        neck_xpbd = None
        if self._neck_xpbd_active():
            if self.neck_soft is None:
                self.neck_soft = SoftHinge(NECK_XPBD_COMPLIANCE, NECK_XPBD_BETA)
            self.neck_soft.begin_frame()
            neck_xpbd = [idx["waist"] if self.spine is not None else hip, idx["shoulder"], idx["head"]]
            prev_frame = q.copy()      # x_n: karenin basi (Verlet oncesi) konumlar
        for _ in range(iterations):
            arm_columns()
            below = mask & (p[:, 1] > floor)
            contact |= below
            dn[below] += p[below, 1] - floor[below]
            p[below, 1] = floor[below]
            if x0 is not None and len(x0) == len(p):
                # statik surtunme (konum tabanli PBD): bu kare az kaymis temas noktasi
                # kare basindaki yerine kilitlenir -- cubuk duzeltmeleri de onu kaydiramaz.
                # Adim 30: esik yuke bagli (Coulomb, PBD: |dx_t| <= mu_s * dx_n); sabit
                # 0.6 px esikte kolun itisi oturan kalcayi her kare 2 px geri kaydiriyordu.
                if self.coulomb_friction:
                    # Saf Coulomb, KARE butcesiyle: bir temas noktasinin bu karedeki TOPLAM
                    # tegetsel duzeltmesi <= mu * TOPLAM normal duzeltme. Iterasyon basina
                    # kontrol (eski) kapasiteyi ~iterasyon sayisi kadar katliyordu: diz ustunde
                    # KM destegin 19 px disinda iken govde ~2100 N'luk surtunme ciftiyle
                    # tutuluyordu (Coulomb siniri ~270 N, olculdu).
                    drift = np.where(contact, p[:, 0] - x0, 0.0)
                    self.friction_budget_log = (dn, tcorr)
                    used = np.abs(tcorr)
                    static_left = np.maximum(FALLEN_STATIC_MU * dn - used, 0.0)
                    kinetic_left = np.maximum(FALLEN_KINETIC_MU * dn - used, 0.0)
                    budget = np.where(np.abs(drift) <= static_left, np.abs(drift), kinetic_left)
                    corr = np.sign(drift) * np.minimum(np.abs(drift), budget)
                    p[:, 0] -= corr
                    tcorr += corr
                    lim = None
                elif self.gn_fall:
                    lim = FALLEN_STATIC_STICK_PX
                else:
                    lim = np.maximum(FALLEN_STATIC_STICK_PX, FALLEN_STATIC_MU * dn)
                if lim is not None:
                    stick = contact & (np.abs(p[:, 0] - x0) < lim)
                    p[stick, 0] = x0[stick]
            body._satisfy_sticks()
            knee_flex_limit()
            if self.balance_recovery:
                knee_branch_limit()
            if self.spine is not None:
                self.spine.constrain()
            if neck_xpbd is not None:
                self.neck_soft.project(p, prev_frame, m_, neck_xpbd)
            if self.arms is not None and self.gn_fallen_arms:
                constrain_fallen_elbows(self.arms)
        for k_i, f_i in self.fallen_legs.values():
            if self.balance_recovery:
                continue
            dvec = p[f_i] - p[hip]
            n = float(np.linalg.norm(dvec))
            v = p[k_i] - p[hip]
            # hiperekstansiyon siniri (0 derece): diz kalca-ayak cizgisinin arkasina
            # gecerse cizgiye geri konur (esnek olmayan). Aynalama denendi: duz bacakta
            # diz her kare ileri-geri atlayip 40 px/kare hiz pompaliyordu (olculdu).
            if n > 1e-6 and -(dvec[0] * v[1] - dvec[1] * v[0]) / n * FALLEN_KNEE_BEND_SIGN < 0.0:
                u = dvec / n
                along = p[hip] + u * float(np.dot(v, u))
                fwd = np.array([-u[1], u[0]]) * (-FALLEN_KNEE_BEND_SIGN)
                if -(dvec[0] * fwd[1] - dvec[1] * fwd[0]) / n * FALLEN_KNEE_BEND_SIGN < 0.0:
                    fwd = -fwd
                p[k_i] = along + fwd * 0.5
                q[k_i] = p[k_i].copy()
        if self.arms is not None and not self.gn_fall:
            sh = idx["shoulder"]
            for key, (e_i, h_i) in self.arms.idx.items():
                sgn = self.elbow_side.get(key, 1.0)
                dvec = p[h_i] - p[sh]
                n = float(np.linalg.norm(dvec))
                v = p[e_i] - p[sh]
                if n > 1e-6 and -(dvec[0] * v[1] - dvec[1] * v[0]) * sgn < 0.0:
                    u = dvec / n
                    along = p[sh] + u * float(np.dot(v, u))
                    side = np.array([-u[1], u[0]])
                    if -(dvec[0] * side[1] - dvec[1] * side[0]) * sgn < 0.0:
                        side = -side
                    p[e_i] = along + side * 0.5
                    q[e_i] = p[e_i].copy()
        below = mask & (p[:, 1] > floor)
        contact |= below
        normal_projection[below] += p[below, 1] - floor[below]
        p[below, 1] = floor[below]
        if FALLEN_FINAL_STATIC_LOCK and not self.gn_fall and x0 is not None and len(x0) == len(p):
            # Adim 30: statik kilit son kez, cubuk gecisinden SONRA. Dongu cubuklarla
            # bittigi icin kilitli temas noktalari her kare ~0.02 px kayiyordu; yerdeki
            # govde surekli "surunuyordu" (480 karede 1.4 px, iterasyon sayisina gore
            # yonu bile degisiyordu -- olculdu). Kalan cubuk hatasi sonraki karede cozulur.
            lim = np.maximum(FALLEN_STATIC_STICK_PX, FALLEN_STATIC_MU * dn)
            stick = contact & (np.abs(p[:, 0] - x0) < lim)
            p[stick, 0] = x0[stick]
        # Adim 30: yeni temas eden noktalarin dikey carpma hizi
        names = {hip: "kalca", idx["shoulder"]: "omuz", idx["head"]: "bas"}
        if self.arms is not None:
            for key, (e_i, h_i) in self.arms.idx.items():
                names[e_i] = "dirsek_" + key
                names[h_i] = "el_" + key
        prevc = self._contact_prev if self._contact_prev is not None and len(self._contact_prev) == len(p) else np.zeros(len(p), bool)
        for i in np.where(contact & ~prevc)[0]:
            if i in names:
                self.impact_log.append((self.frame, names[i], round(float(p[i][1] - q[i][1]), 2)))
        self._contact_prev = contact.copy()
        # Contact solver input speed, before its inelastic velocity reset.
        # This is a point-mass diagnostic, not a measured physical impulse.
        self.contact_ids = [int(i) for i in np.flatnonzero(contact)]
        for point in np.flatnonzero(contact):
            self.ground_contacts.append((self.frame, int(point), max(0.0, float(incoming_vy[point]))))
            self.ground_projection_impulses.append(
                (self.frame, int(point), float(normal_projection[point] * body.masses[point])))
        # temas eden noktalar: dikey hiz sifir (esnek olmayan), yatay hiz surtunmeyle soner
        q[contact, 1] = p[contact, 1]
        if self.coulomb_friction:
            # surtunme yukaridaki konum duzeltmesinde (kare butceli); yatay hiz Verlet'ten
            if self.arms is not None and self.bracing and self.brace_level > 0.01:
                for _e_i, h_i in self.arms.idx.values():
                    if contact[h_i]:
                        q[h_i, 0] = p[h_i, 0]      # aktif kavrama yalniz refleks surerken
            return
        q[contact, 0] = p[contact, 0] - (p[contact, 0] - q[contact, 0]) * (1.0 - FALLEN_GROUND_MU)
        if self.arms is not None and self.bracing:
            # Adim 30: refleksle uzanan avuc zemine yapisir (avuc ici-zemin, aktif kavrama):
            # yatay hiz temas karesinde sifirlanir; yuk altinda kayma kolon/koni kuralinda
            for _e_i, h_i in self.arms.idx.values():
                if contact[h_i]:
                    q[h_i, 0] = p[h_i, 0]
        # statik surtunme (Coulomb): yavas kayan temas noktasi yapisir
        slow = contact & (np.abs(p[:, 0] - q[:, 0]) < FALLEN_STATIC_STICK_PX)
        q[slow, 0] = p[slow, 0]

    def arm_column_capacity(self, r: float, v_c: float) -> float:
        """Adim 30 -- bir kolun omuz-el dogrultusunda bir karede durdurabildigi
        en buyuk omuz hizi (px/kare): dirsek ekstansor torku / (l sin beta) /
        etkin kutle; cos beta = r / 2l. Sikisan (eksantrik) kolda Hill ~1.5x.
        Duz kol (sin beta -> 0) kemik kolonudur: kapasite cok buyur."""
        c = min(max(r / (2.0 * ARM_SEG_MEAN), 0.0), 1.0)
        sinb = max(float(np.sqrt(1.0 - c * c)), 0.05)
        fv = force_velocity(-v_c / (ARM_SEG_MEAN * sinb)) if self.hill else 1.0
        tau = TAU_ELBOW_MAX_NM / TORQUE_UNIT_NM * fv
        return tau / (ARM_SEG_MEAN * sinb) / ARM_SUPPORT_MASS

    def leg_force_capacity(self, d: float) -> float:
        """Adim 27 -- iki kemikli bacagin kalca-ayak dogrultusunda uretebildigi en
        buyuk kuvvet (tork birimi / px): F = tau / (l sin beta), cos beta = d/2l."""
        l = LEG_SEGMENT_LEN
        c = min(max(d / (2.0 * l), 0.0), 1.0)
        sinb = float(np.sqrt(max(1.0 - c * c, 0.0)))
        tau = TAU_KNEE_MAX_NM / TORQUE_UNIT_NM
        cap = LEG_FORCE_CAP_W * BODY_MASS_TOTAL * self.gravity_policy.muscle_load
        return cap if sinb * l * cap <= tau else tau / (l * sinb)

    def _hip_strategy_deg(self) -> float:
        g = HIP_STRATEGY_GAIN if self.hip_strategy_gain is None else self.hip_strategy_gain
        if g == 0.0:
            return 0.0
        return float(np.clip(g * self.last_real_error, -HIP_STRATEGY_MAX_DEG, HIP_STRATEGY_MAX_DEG))

    def _rocker_shift(self, leg, hip_x: float) -> float:
        """Adim 26 -- durus pivotunun ayak bileginden kaymasi (+ parmak ucu, - topuk)."""
        if not self.rocker:
            return 0.0
        dx = hip_x - float(leg.planted[0])
        if dx > ROCKER_START_FWD_PX:
            shift = min(ROCKER_TOE_PX, dx - ROCKER_START_FWD_PX)
        elif dx < -ROCKER_START_BACK_PX:
            shift = -min(ROCKER_HEEL_PX, -ROCKER_START_BACK_PX - dx)
        else:
            shift = 0.0
        self.rocker_log.append(shift)
        return shift

    def system_com_vx(self) -> float:
        """Adim 22: govde + kollar + iki bacagin (pergel ekseni segmentleri)
        kutle-agirlikli yatay hizi. Bacak segmenti i: v = (1-f_i) v_kalca + f_i v_ayak."""
        idx, p, q = self.idx, self.body.points, self.body.prev_points
        if self.spine is not None:
            ids = [i for i in range(len(p)) if i not in self.body.pinned]
            return float(np.average((p-q)[ids, 0], weights=self.body.masses[ids]))
        ids = [idx["hip"], idx["shoulder"], idx["head"]]
        if self.arms is not None:
            ids += [i for pair in self.arms.idx.values() for i in pair]
        m = self.body.masses[ids].copy()
        m[0] = self.hip_base_mass
        mom = float(np.sum(m * (p[ids, 0] - q[ids, 0])))
        tot = float(np.sum(m))
        v_hip = float(p[idx["hip"], 0] - q[idx["hip"], 0])
        for leg in self.legs:
            h = self.leg_mass.hist.get(id(leg), [])
            # Son ayak hizi [t-1, t]: tepki bir onceki karenin SONUNDA prev_points'e
            # islendigi icin govde hizi bu araligin ivmesini zaten iceriyor.
            v_foot = float(h[-1][0][0] - h[-2][0][0]) if len(h) >= 2 else 0.0
            for frac, mm in ((THIGH_AXIS_FRAC, THIGH_MASS), (SHANK_AXIS_FRAC, SHANK_MASS)):
                mom += mm * ((1.0 - frac) * v_hip + frac * v_foot)
                tot += mm
        return mom / tot

    @property
    def legs(self) -> list:
        return [self.left_leg, self.right_leg]

    def step(self) -> float:
        """Tek bir kare ilerlet; o karenin (itki ONCESI olculen) hip_vx'ini dondur."""
        body, idx = self.body, self.idx
        hip = idx["hip"]
        anchor = idx["anchor"]
        left_leg, right_leg = self.left_leg, self.right_leg
        f = self.frame
        t = f * self.dt
        hip_pos_before = body.points[hip].copy()
        hip_prev = body.prev_points[hip].copy()
        hip_vx = hip_pos_before[0] - hip_prev[0]
        ctrl_vx = self.system_com_vx() if (self.leg_mass is not None and LEG_MASS_COM_CONTROL) else hip_vx
        self.last_ctrl_vx = ctrl_vx

        # Itkiler DURTU (momentum) olarak: kick_px, taban kalca kutlesine
        # (1.0) verilen hiz. Adim 22'de kalcaya ortuk bacak kutlesi eklenince
        # ayni hiz daha buyuk momentum olurdu -- kiyas adil kalsin diye bolunur.
        kick_scale = self.hip_base_mass / float(body.masses[hip])
        if not self.stumbled and t >= self.stumble_t:
            body.prev_points[hip][0] -= self.stumble_kick_px * kick_scale
            self.stumbled = True
        if not self.big_pushed and t >= self.big_push_t:
            body.prev_points[hip][0] -= self.big_push_kick_px * kick_scale
            self.big_pushed = True

        if self.balance_recovery:
            # The impulse is already in Verlet velocity; sense it this frame.
            ctrl_vx = (self.system_com_vx() if self.leg_mass is not None and LEG_MASS_COM_CONTROL
                       else body.points[hip, 0] - body.prev_points[hip, 0])
            self.last_ctrl_vx = ctrl_vx

        stance_leg = None
        for leg in (left_leg, right_leg):
            if leg.state == "stance":
                stance_leg = leg
        if STALE_SLIP_RESET:
            # Adim 21: anchor (yuk) yalnizca stance_leg'de. Iki ayak birden
            # yerdeyken YUKSUZ ayagin kayma durumu donuk kaliyordu
            # (is_slipping=True, slip_velocity sabit -4 px/kare) ve anchor
            # o ayaga gectigi an kayma 50+ kare surup gidiyordu.
            for leg in (left_leg, right_leg):
                if leg is not stance_leg and leg.state == "stance" and leg.is_slipping:
                    leg.is_slipping = False
                    leg.slip_velocity = 0.0
        in_danger = False
        self.last_real_error = 0.0
        if stance_leg is not None and not self.fell:
            desired_thrust = THRUST_GAIN * (TARGET_VX - ctrl_vx)
            desired_thrust = max(-THRUST_CAP, min(THRUST_CAP, desired_thrust))

            # 13. tur eki -- gercek 2B stres: bir onceki karenin relaksasyon
            # SONRASI anchor-kalca cubugu (bkz. yukaridaki "13. tur eki"
            # yorum bloğu icin tam gerekce/dogrulama).
            stretch_vec = body.points[hip] - body.points[anchor]
            stretch_len = float(np.linalg.norm(stretch_vec))
            stretch_dev = stretch_len - ARM_LENGTH  # + gerilme (tension), - sikisma (compression)
            stretch_dir = stretch_vec / stretch_len if stretch_len > 1e-6 else np.array([0.0, -1.0])
            stress_vec = stretch_dev * stretch_dir

            load_factor = 1.0 - LOAD_GAIN * stretch_dev
            load_factor = max(LOAD_FACTOR_MIN, min(LOAD_FACTOR_MAX, load_factor))
            f_n_effective = (self.gravity_policy.walking if self.gravity_policy.mode == "unified"
                             else THRUST_CAP) * load_factor

            total_stress = desired_thrust + STRESS_GAIN * stress_vec[0]

            mu_static = self.terrain_static.friction_fn(stance_leg.planted[0])
            mu_kinetic = self.terrain_kinetic.friction_fn(stance_leg.planted[0])
            f_max_static = mu_static * f_n_effective
            f_max_kinetic = mu_kinetic * f_n_effective

            # Stribeck: statik/kinetik esik ayrimi + KALICI kayma fazi
            # (bkz. yukaridaki yorum bloğu).
            if not stance_leg.is_slipping and abs(total_stress) > f_max_static:
                stance_leg.is_slipping = True
                stance_leg.slip_velocity = 0.0

            if stance_leg.is_slipping:
                applied_thrust = math.copysign(min(abs(desired_thrust), f_max_kinetic), desired_thrust) \
                    if desired_thrust != 0.0 else 0.0
                if abs(total_stress) > f_max_kinetic:
                    excess = total_stress - math.copysign(f_max_kinetic, total_stress)
                else:
                    excess = 0.0
                stance_leg.slip_velocity += -excess * SLIP_ACCEL_GAIN
                stance_leg.slip_velocity *= SLIP_DECAY
                stance_leg.apply_slip(stance_leg.slip_velocity)
                self.slip_events.append((f, "l" if stance_leg is left_leg else "r",
                                         float(excess), float(stance_leg.slip_velocity)))
                if abs(stance_leg.slip_velocity) < SLIP_STOP_VEL and abs(total_stress) <= f_max_kinetic:
                    stance_leg.is_slipping = False
                    stance_leg.slip_velocity = 0.0
            else:
                applied_thrust = desired_thrust

            # 13. tur eki 3 -- eski SABIT-aci kriteri (MAX_SLIP_LEG_ANGLE_DEG)
            # KALDIRILDI: artik gercek COM-vs-destek-araligi (FallRiskMonitor)
            # kullaniliyor. BILEREK `is_slipping`'e bagli DEGIL.
            com_x = self.com_x()
            interval = support_interval([stance_leg.planted[0]], foot_half_len=FOOT_HALF_LEN,
                                         fallback_x=[left_leg.swing_target[0], right_leg.swing_target[0]])
            real_error = outside_interval_error(com_x, interval)
            self.last_real_error = float(real_error)
            in_danger = self.risk_monitor.update(real_error)
            # Adim 18 (Faz B): stance bacagi toe_off'ta ve kalca onu asiri
            # hizla geciyorsa (ya da tehlike + toe_off), tek-destek kuralini
            # bozmadan yakalama adimi: havadaki bacak varsa ONUN salinimini
            # sikistir, yoksa toe_off bacagini sikistirilmis salinimla birak.
            fazb_handled = False
            # Adim 19c -- prediktif sensor: bu karenin itkisi prev_points'te,
            # kalcanin bir sonraki konumu Verlet kuraliyla tahmin edilir.
            stance_leg.predict_contact(body.points[hip], body.prev_points[hip], com_vx=ctrl_vx)
            front = stance_leg.catch_overrun() if self.faz_b else None
            # Adim 24 -- kapanma hizi kapisi tehlike yolunu da kapsar: COM destek
            # araliginin disinda ama ona TTC <= FAZB_CLOSING_TTC_FRAMES icinde
            # varacaksa (or. yakalama ayagi bilerek onune basildi) adim atilmaz.
            closing_soon = self.faz_b and stance_leg._closing_soon()
            if closing_soon:
                fazb_handled = True
            if self.faz_b and front is None and in_danger and not closing_soon:
                # tehlike + dogru cephe: COM onde & toe_off / COM geride & heel_strike
                if real_error > 0 and stance_leg.contact_phase == PHASE_TOE_OFF:
                    front = "toe"
                elif real_error < 0 and stance_leg.contact_phase == PHASE_HEEL_STRIKE:
                    front = "heel"
            # Adim 20 -- kol refleksi: tehlike ya da Faz B yakalamasi aninda
            # kollar, COM sapma yonune gore sabit bir denge acisina gider.
            if self.arms is not None and self.arm_reflex and (in_danger or front is not None):
                direction = (1.0 if front == "toe" else -1.0) if front is not None else float(np.sign(real_error))
                if direction != 0.0:
                    self.reflex_dir = (ARM_REFLEX_SIGN * direction if ARM_REFLEX_POLICY == "signed"
                                       else ARM_REFLEX_FIXED_DIR)
                    self.reflex_level = 1.0
                    self.reflex_hold = ARM_REFLEX_HOLD
            if front is not None:
                other = right_leg if stance_leg is left_leg else left_leg
                side = "l" if stance_leg is left_leg else "r"
                if other.state == "swing":
                    if other.compress_swing(hip_pos_before[0], ctrl_vx):
                        other.catch_start_frame = f
                        self.fazb_events.append((f, "r" if side == "l" else "l", "compress", front,
                                                 round(stance_leg.leg_angle_deg, 1)))
                elif stance_leg.launch_catch_step(hip_pos_before[0], ctrl_vx):
                    stance_leg.catch_start_frame = f
                    self.fazb_events.append((f, side, "launch", front, round(stance_leg.leg_angle_deg, 1)))
                fazb_handled = True
            other_swinging = (right_leg if stance_leg is left_leg else left_leg).state == "swing"
            if in_danger and not fazb_handled and not (self.faz_b and other_swinging):
                # Adim 20: Faz B aciksa eski acil adim, diger bacak HAVADAYKEN
                # stance bacagini firlatamaz (tek-destek kurali; kollu 500 px
                # itkide bu yoldan 1 karelik cift-havada olculdu).
                target_x = com_x + np.sign(real_error) * EMERGENCY_STEP_LEAD_PX
                if stance_leg.trigger_emergency_step(target_x, speedup=EMERGENCY_SWING_SPEEDUP):
                    stance_leg.is_slipping = False
                    stance_leg.slip_velocity = 0.0
                    self.emergency_step_events.append((f, "l" if stance_leg is left_leg else "r", round(float(real_error), 1)))

            body.set_pinned_position(anchor, [stance_leg.planted[0] + self._rocker_shift(stance_leg, hip_pos_before[0]),
                                              GROUND_Y])
            if self.thrust_mode == "com":
                body.prev_points[self.upper_ids, 0] -= applied_thrust
            else:
                body.prev_points[hip][0] -= applied_thrust

        # Adim 19b -- yakalama sonrasi sok emilimi (bkz. SHOCK_ABSORB_* notu)
        # anchor'in GERCEKTEN yakalama bacagina gectigi karede baslar (iki bacak
        # da stance iken anchor diger bacakta kalabiliyor -- o durumda bekle)
        if stance_leg is not None and self.shock_pending is not None and stance_leg is self.shock_pending:
            d = float(np.linalg.norm(body.points[hip] - np.array([stance_leg.planted[0], GROUND_Y])))
            self.shock_rest = min(ARM_LENGTH, d)
            if self.shock_mode == "force":
                # temas: r'nin hizi kalcanin bacak boyunca gercek hizi; kas aktivasyonu
                # hazirliga gore (Adim 25: TTC ile temastan once kasilma)
                hp = body.points[hip] - body.prev_points[hip]
                u = (body.points[hip] - np.array([stance_leg.planted[0], GROUND_Y])) / max(d, 1e-6)
                self.shock_vel = float(np.dot(hp, u))
                self.shock_act = (max(ACT_INITIAL, min(1.0, self.shock_pending_v0))
                                  if self.preact_frames > 0 else ACT_INITIAL)
            else:
                self.shock_vel = self.shock_pending_v0 if self.preact_frames > 0 else 0.0
            self.shock_pending = None
            self.shock_events.append((f, round(self.shock_rest, 1)))
        if self.collapsed:
            pass   # Adim 29: anchor cubugu kapali, bacaklar ragdoll (bkz. _enter_fallen)
        elif self.shock_rest is not None and self.shock_mode == "force":
            i0, j0, _, c0 = body.sticks[0]
            if stance_leg is not None:
                d_now = float(np.linalg.norm(body.points[hip] - body.points[anchor]))
                W = BODY_MASS_TOTAL * self.gravity_policy.muscle_load
                # Adim 28: Hill kuvvet-hiz -- diz acisal hizi w = r_hizi / (l sin beta);
                # bacak uzarken (konsantrik) kuvvet duser, zorla bukulurken (eksantrik) artar
                def hill_factor(d_leg: float) -> float:
                    if not self.hill:
                        return 1.0
                    c = min(max(d_leg / (2.0 * LEG_SEGMENT_LEN), 0.0), 1.0)
                    sinb = max(float(np.sqrt(1.0 - c * c)), 0.05)
                    return force_velocity(self.shock_vel / (LEG_SEGMENT_LEN * sinb))
                f_tot = self.leg_force_capacity(d_now) * self.shock_act * hill_factor(self.shock_rest)
                other = right_leg if stance_leg is left_leg else left_leg
                if other.state == "stance":
                    d2 = float(np.linalg.norm(body.points[hip] - np.array([other.planted[0], GROUND_Y])))
                    # Adim 28: diger bacak yalnizca BUKUKSE yuk paylasir -- duz bacagin
                    # uzama payi yok, kalcayi yukari ivmelendiremez (olculdu: duz bacagin
                    # 6W tavani tek karede 15 px/kare firlatma uretiyordu)
                    if d2 < ARM_LENGTH - SHOCK_CONTACT_MIN_PX:
                        f_tot += self.leg_force_capacity(d2) * self.shock_act * hill_factor(d2)
                a_up = (f_tot - W) / BODY_MASS_TOTAL
                self.yield_frames = self.yield_frames + 1 if (a_up < 0.0 and self.shock_vel < 0.0) else 0
                err = ARM_LENGTH - self.shock_rest
                g = self.gravity_policy.muscle_load
                v_des = float(np.sqrt(g * g / 4.0 + 2.0 * g * max(err, 0.0)) - g / 2.0)
                v_des = min(v_des, err)
                self.shock_vel += float(np.clip(v_des - self.shock_vel, -g, a_up))
                self.shock_rest = min(ARM_LENGTH, self.shock_rest + self.shock_vel)
                if self.shock_vel > 0.0:
                    self.shock_rest = min(ARM_LENGTH, max(self.shock_rest, min(d_now, ARM_LENGTH)))
                if self.shock_rest <= KNEE_FLEX_MIN_DIST:
                    self.shock_rest = KNEE_FLEX_MIN_DIST
                    self.shock_vel = 0.0
                    self._collapse("leg_force")
                self.shock_act += (1.0 - self.shock_act) * ACT_RATE
                self.leg_force_log.append((f, round(d_now, 1), round(f_tot / W, 2), round(a_up, 2),
                                           round(self.shock_rest, 1), round(self.shock_vel, 2)))
            if self.collapsed:
                self.shock_rest = None                  # Adim 29: cubuk _enter_fallen'da kapatildi
            else:
                body.sticks[0] = (i0, j0, self.shock_rest, c0)
                if ARM_LENGTH - self.shock_rest < SHOCK_ABSORB_DONE_PX and self.shock_vel >= 0.0:
                    self.shock_rest = None
        elif self.shock_rest is not None and self.shock_mode == "servo":
            i0, j0, _, c0 = body.sticks[0]
            if stance_leg is not None:
                d_now = float(np.linalg.norm(body.points[hip] - body.points[anchor]))
                err = ARM_LENGTH - self.shock_rest
                mag = float(np.sqrt(SHOCK_EXT_ACCEL ** 2 / 4.0 + 2.0 * SHOCK_EXT_ACCEL * max(err, 0.0))
                            - SHOCK_EXT_ACCEL / 2.0)
                v_des = min(mag, SHOCK_EXT_VMAX, err)
                self.shock_vel += float(np.clip(v_des - self.shock_vel, -SHOCK_EXT_ACCEL, SHOCK_EXT_ACCEL))
                self.shock_rest = min(ARM_LENGTH, max(self.shock_rest + self.shock_vel, d_now))
            body.sticks[0] = (i0, j0, self.shock_rest, c0)
            if ARM_LENGTH - self.shock_rest < SHOCK_ABSORB_DONE_PX:
                self.shock_rest = None
        elif self.shock_rest is not None:
            i0, j0, _, c0 = body.sticks[0]
            if stance_leg is not None:
                # hiz siniri: dinlenme boyu, cubugun SU ANKI boyunun en fazla
                # SHOCK_RISE_CAP_PX fazlasi olabilir -- kalca asla gevsemis bir
                # destekle sarkmaz (cokus onlenir), sadece yukselis hizi sinirlanir.
                d_now = float(np.linalg.norm(body.points[hip] - body.points[anchor]))
                self.shock_rest = min(ARM_LENGTH, max(self.shock_rest, d_now) + SHOCK_RISE_CAP_PX)
                if GROUND_Y - body.points[hip][1] < SHOCK_FLOOR_PX:
                    # kalca tehlikeli alcaklikta: sonumleme birakilir, tam rijit
                    # boy -- yumusaklik cokusu buyutmesin (-500 px taramasi)
                    self.shock_rest = ARM_LENGTH
            body.sticks[0] = (i0, j0, self.shock_rest, c0)
            if ARM_LENGTH - self.shock_rest < SHOCK_ABSORB_DONE_PX:
                self.shock_rest = None
        elif body.sticks[0][2] != ARM_LENGTH:
            i0, j0, _, c0 = body.sticks[0]
            body.sticks[0] = (i0, j0, ARM_LENGTH, c0)

        # 13. tur eki 3 -- dinamik-compliance ("muz kabugu") hilesi TAMAMEN
        # KALDIRILDI; anchor-kalca cubugu HER ZAMAN sabit
        # ANCHOR_HIP_COMPLIANCE_BASE ile kurulur (bkz. build_body()).

        # Real gravity can carry the hip below recoverable knee height before
        # a catch landing starts the force-limited leg servo. Route that
        # failure through the same ragdoll transition, not only a 'fell' flag.
        if self.gravity_policy.mode == "unified" and not self.collapsed:
            predicted_hip_y = (body.points[hip, 1] +
                               (body.points[hip, 1]-body.prev_points[hip, 1]) * (1-body.friction) +
                               body.gravity[1])
            if predicted_hip_y >= GROUND_Y-KNEE_FLEX_MIN_DIST:
                self._collapse("support_height")
                self.shock_rest = None

        if self.arms is not None:
            hp = body.points[hip]
            leg_angles = {k: float(np.arctan2(leg.chain.points[-1][0] - hp[0], leg.chain.points[-1][1] - hp[1]))
                          for k, leg in (("l", left_leg), ("r", right_leg))}
            reflex = None
            if self.arm_reflex and self.reflex_level > ARM_REFLEX_OFF_LEVEL:
                reflex = self.reflex_dir * ARM_REFLEX_DEG
            catching = any(l.catch_active for l in (left_leg, right_leg))
            drive_on = self.arms_mode == "drive" and not (ARM_PASSIVE_DURING_CATCH and catching)
            self._update_brace(f)
            if self.bracing and self.brace_level > 0.01:
                # Adim 30: koruyucu refleks -- kollar dusus yonunde zemine uzanir
                sh = body.points[idx["shoulder"]]
                # hedef dunyaya sabit: omzun govde kalca etrafinda devrilince inecegi
                # noktanin altina (kalca + yon * BRACE_PLANT_FROM_HIP). Omze gore hedef
                # denendi: el omzu kovalayip zemine 11 px/kare yatay hizla carpiyor,
                # kayarak kolu yataya yatiriyordu (olculdu).
                hands_y = max(body.points[h][1] for _e, h in self.arms.idx.values())
                side = self._brace_side()
                # yeniden planlama: hicbir el yuk tasimiyorken govde ters tarafa devriliyorsa
                # (ornegin ellere sekip geriye oturma) kollar yeni dusus tarafina uzanir
                replan = not self.arm_column and side != self.brace_dir
                # Yon belirsizken (dik cokme: omuz kalcanin ustunde) dunyaya sabit hedef
                # KILITLENMEZ; kollar omzun altina "hazir" uzanir. TTC ile erken acilan
                # refleks hedefi bu anda kilitleyip elleri kalcanin altina koyuyor, govde
                # sonra devrilince kolon omzu tutamiyordu (olculdu, README birlesim bolumu).
                ambiguous = (self.brace_dir_gate and not self.arm_column and
                             abs(self._brace_lean()) < BRACE_DIR_MIN_PX)
                if ambiguous:
                    self.brace_plant_x = None
                elif self.brace_plant_x is None or hands_y < GROUND_Y - BRACE_FREEZE_PX or replan:
                    self.brace_dir = side
                    self.brace_plant_x = float(body.points[idx["hip"]][0]) + self.brace_dir * BRACE_PLANT_FROM_HIP
                target = np.array([sh[0] if ambiguous else self.brace_plant_x, GROUND_Y])
                ang = float(np.degrees(np.arctan2(target[0] - sh[0], max(target[1] - sh[1], 1.0))))
                self.brace_log.append((f, round(self.brace_level, 3), round(ang, 1)))
                self.arms.drive(leg_angles, enabled=True, reflex_target_deg=ang,
                                reflex_weight=self.brace_level, passive_base=self.collapsed,
                                skip=tuple(self.arm_column))
            elif self.collapsed:
                self.arms.drive(leg_angles, enabled=False)   # Adim 29: yigilmis -- kollar pasif
                if self.fall_bracing is not None:
                    self.fall_bracing.drive(self.arms, idx["head"], GROUND_Y, self.gravity_policy.falling, f)
            elif self.arms_mode == "cancel" and reflex is None:
                # Adim 22 -- kollar bacaklarin yaw momentumunu iptal etmeyi hedefler
                self.arms.drive(leg_angles, enabled=False)   # yalnizca bacak acisi gecmisini gunceller
                self.arms.drive_cancel(self.last_leg_yaw)
            else:
                self.arms.drive(leg_angles, enabled=(drive_on or reflex is not None),
                                reflex_target_deg=reflex, reflex_weight=self.reflex_level,
                                exit_damping=ARM_EXIT_DAMPING if self.reflex_level > ARM_EXIT_LEVEL else 0.0)
            # Adim 21 -- cikis sonumlemesi: tetik ARM_REFLEX_HOLD kare tam
            # agirlikta tutulur, sonra agirlik ustel soner (zombi kilidi yok).
            if self.reflex_hold > 0:
                self.reflex_hold -= 1
            else:
                self.reflex_level *= ARM_REFLEX_DECAY

        # Adim 22 -- aktif govde durusu (kalca ekstansor/fleksor kaslari): govdeyi
        # hedef egime cekmeye calisan PD tork (apply_angular_spring: konuma degil
        # prev_points'e). Yoksa govde itki altinda karelerin %100'unde -12
        # derecelik kelepce duvarinda duruyor ve momentum alisverisini duvar yutuyor.
        if (self.posture_k > 0.0 or self.posture_c > 0.0) and not (self.collapsed and FALLEN_LIMP):
            apply_angular_couple(body.points, body.prev_points, body.masses, hip, idx["shoulder"], UP,
                                 POSTURE_TARGET_DEG + self._hip_strategy_deg(), self.posture_k, self.posture_c)

        if self.collapsed:
            self._fallen_x0 = body.points[:, 0].copy()   # Adim 29: statik surtunme icin kare basi konum
        if self.spine is not None:
            self.spine.drive()
        if (self.collapsed and not self.gn_fall and self.gravity_policy.mode == "unified"
                and (NECK_FALL_K > 0.0 or NECK_FALL_C > 0.0 or NECK_FALL_END_C > 0.0)):
            # Boyun kaslari dususte: bas ust govdeye (bel-omuz) gore yay/sonumle tutulur.
            # Omurgali govdede gogus yere degince bas eylemsizlikle kamci gibi savruluyordu.
            root = idx["waist"] if self.spine is not None else hip
            hinge_drive(body, [root, idx["shoulder"], idx["head"]], NECK_FALL_K, NECK_FALL_C,
                        end_damping=NECK_FALL_END_C, end_start=np.radians(NECK_FALL_END_START_DEG),
                        end_limit=np.radians(FALLEN_NECK_LIMIT_DEG), limit=NECK_FALL_ALPHA_MAX)
        if self.recovery is not None:
            self.recovery.telemetry_frame = self.frame
            self.recovery.sim_ref = self
            self.recovery.drive(self)
            if getattr(self.recovery, "_targets", None):
                self.recovery.apply_joint_motors(self)
        if self.upright_head and not self.collapsed:
            # A damped neck controller, released completely in the ragdoll.
            # Like the torso controller, its equal/opposite forces add no
            # linear momentum; this remains an upright-reference PD model.
            apply_angular_couple(body.points, body.prev_points, body.masses,
                                 idx["shoulder"], idx["head"], UP, 0.0, NECK_POSTURE_K, NECK_POSTURE_C)
        body.step(dt=1.0)
        # preserve_momentum=True -- bkz. modulun 8. tur notu: varsayilan
        # (False) govde/boyun kelepcesi uzerinden kalcanin KENDI hizini da
        # sessizce sifirlayip dusme esigini maskeliyordu.
        if self.spine is None:
            clamp_direction(body.points, body.prev_points, hip, idx["shoulder"], UP,
                            FALLEN_TORSO_LIMIT_DEG if self.collapsed else TORSO_MAX_LEAN_DEG,
                            preserve_momentum=self.torso_clamp_mode)
        torso_root = idx["waist"] if self.spine is not None else hip
        torso_dir = body.points[idx["shoulder"]] - body.points[torso_root]
        if not (self.collapsed and FALLEN_NECK_IN_LOOP and not self.gn_fall):
            clamp_direction(body.points, body.prev_points, idx["shoulder"], idx["head"], torso_dir,
                            FALLEN_NECK_LIMIT_DEG if self.collapsed else NECK_MAX_TILT_DEG,
                            preserve_momentum=self.torso_clamp_mode)
        if self.arms is not None and self.gn_fall:
            if not (self.collapsed and self.gn_fallen_arms):
                self.arms.constrain()
        elif self.arms is not None:
            lvl = self.brace_level if self.bracing else 0.0
            # yuk alan (kolonu olan) kolda dirsek ekstansorleri kolonun icinde: cift yok
            self.arms.constrain(fallen=self.collapsed, brace_k=BRACE_ELBOW_K * lvl, brace_c=BRACE_ELBOW_C * lvl,
                                skip=tuple(self.arm_column))
        if self.collapsed:
            self._fallen_constraints()
        if self.recovery is not None:
            self.recovery.observe(self, GROUND_Y)
            self._check_recovery_override()

        hip_pos = body.points[hip]
        # NOT: bu dongu bilerek SIRALI (once sol, sonra sag) calisir -- bkz.
        # git gecmisi/README 8. tur: "simetrik" degerlendirme cift-havada
        # (double-swing) durumunu geri getirip frame 72'de ani dusmeye yol
        # acti. Sirali degerlendirme tek-destek kuralini garanti eder.
        for leg in (left_leg, right_leg):
            other = right_leg if leg is left_leg else left_leg
            was_stance = leg.state == "stance"
            was_swing = leg.state == "swing"
            was_catch = leg.catch_active
            if self.collapsed:
                # Adim 29: bacak ragdoll -- IK zinciri fizikten okunur (cizim/olcum icin)
                k_i, f_i = self.fallen_legs["l" if leg is left_leg else "r"]
                leg.chain.points[0] = hip_pos.copy()
                leg.chain.points[1] = body.points[k_i].copy()
                leg.chain.points[2] = body.points[f_i].copy()
                leg.planted = np.array([body.points[f_i][0], GROUND_Y])
                leg.foot_target = body.points[f_i].copy()
                leg.state = "stance"
                continue
            leg.update(hip_pos, hip_vx=ctrl_vx, other_leg_swinging=(other.state == "swing"))
            if was_swing and was_catch and leg.state == "stance":
                # Adim 23: yakalama adiminin gercek suresi (kalkistan inise)
                dur = (leg.last_catch_frames if leg._catch_timing() == "torque"
                       else f - getattr(leg, "catch_start_frame", f))
                self.catch_frames_log.append((f, "l" if leg is left_leg else "r", dur))
                if self.shock_absorb and self.shock_trigger == "catch":
                    self.shock_pending = leg
            if was_swing and leg.state == "stance" and self.shock_absorb and self.shock_trigger == "contact":
                comp = ARM_LENGTH - float(np.linalg.norm(hip_pos - np.array([leg.planted[0], GROUND_Y])))
                self.contact_log.append((f, "l" if leg is left_leg else "r", round(comp, 1)))
                if comp > SHOCK_CONTACT_MIN_PX:
                    self.shock_pending = leg
                    self.shock_pending_v0 = getattr(leg, "preact_v", 0.0)
                    self.preact_log.append((f, "l" if leg is left_leg else "r", round(self.shock_pending_v0, 2)))
            # Adim 25 -- inise hazirlik: temastan PREACT_FRAMES once ekstansorler kasilir
            if leg.state == "swing" and self.preact_frames > 0:
                ttc = leg.time_to_contact(hip_pos)
                if ttc is not None and ttc <= self.preact_frames:
                    if self.shock_mode == "force":
                        # kas aktivasyonu temastan once yukselir (ACT_RATE ile)
                        a0 = getattr(leg, "preact_v", 0.0) or ACT_INITIAL
                        leg.preact_v = a0 + (1.0 - a0) * ACT_RATE
                    else:
                        leg.preact_v = min(SHOCK_EXT_VMAX, getattr(leg, "preact_v", 0.0) + SHOCK_EXT_ACCEL)
                else:
                    leg.preact_v = 0.0
            else:
                leg.preact_v = 0.0
            if was_stance and leg.state == "swing":
                self.step_events.append((f, "l" if leg is left_leg else "r"))
            if was_swing and leg.state == "stance":
                # 13. tur eki -- her yeni ayak basisi TAZE bir statik-surtunme
                # sansiyla baslar.
                leg.is_slipping = False
                leg.slip_velocity = 0.0
        if self.shock_pending is not None and self.shock_pending.state != "stance":
            self.shock_pending = None   # anchor ona hic gecmeden tekrar kalkti -- bekleyen emilim iptal

        if not self.fell and hip_pos[1] > FALL_HIP_Y_THRESHOLD:
            self.fell = True
            self.fall_frame = f
        if FALLEN_ENTRY_ON_LOW_HIP and not self.gn_fall and not self.collapsed and hip_pos[1] > GROUND_Y - KNEE_FLEX_MIN_DIST:
            # Adim 30 duzeltmesi: kalca, iki kemikli bacagin en katlanmis halinden
            # (kalca-ayak 48 px) daha alcaga indiyse bacak destegi fiziksel olarak
            # bitmistir -- kuvvet bacagi cokusu tetiklememis olsa da (ornegin iki
            # ayak havadayken) yigilma durumuna girilir. Oncesinde >= ~5.6 m/s
            # itkilerde kalca zeminin 184 px ALTINA iniyordu (Adim 29'da da; olculdu).
            self._collapse("support_height")

        if self.leg_mass is not None and not self.collapsed:
            # Adim 22 -- bacak kutlesi: bu karenin ayak konumlariyla merkezi
            # fark ivmesi (ornek t-1) -> tepki simdi, bir sonraki karenin
            # hizina (1 kare gecikme; bkz. physics/leg_mass.py).
            self.leg_mass.observe((left_leg, right_leg))
            F, tau, n_sw = self.leg_mass.hip_load((left_leg, right_leg), body.points[hip])
            body.masses[hip] = self.hip_base_mass + n_sw * LegMassModel.implicit_hip_mass()
            LegMassModel.apply_reaction(body.points, body.prev_points, body.masses, hip, idx["shoulder"], F, tau)

        self._update_trunk_yaw()

        self.hip_x_log.append(hip_pos[0])
        self.hip_y_log.append(hip_pos[1])
        self.hip_vx_log.append(hip_vx)
        self.last_hip_vx = hip_vx
        self.last_in_danger = bool(in_danger)
        if not np.all(np.isfinite(body.points)):
            self.nan = True
        self.frame += 1
        return hip_vx


def main() -> None:
    sim = ActiveBipedSim()
    body, idx = sim.body, sim.idx
    hip = idx["hip"]
    left_leg, right_leg = sim.left_leg, sim.right_leg

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step14_active_biped.mp4")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    for f in range(N_FRAMES):
        hip_vx = sim.step()
        hip_pos = body.points[hip]
        camera_offset = W / 2 - hip_pos[0]
        frame = draw_frame(body, idx, [left_leg, right_leg], camera_offset, hip_vx, sim.fell)
        writer.write(frame)

        if sim.nan:
            print(f"UYARI: NaN/inf @ frame {f}")
            break

    writer.release()
    print(f"wrote {out_path}")

    hip_x_log = np.array(sim.hip_x_log)
    hip_y_log = np.array(sim.hip_y_log)
    hip_vx_log = np.array(sim.hip_vx_log)

    st = int(STUMBLE_T * FPS)
    bp = int(BIG_PUSH_T * FPS)
    print()
    print(f"=== Adim 14 -- dinamik biped raporu ({N_FRAMES} kare, {DURATION_S}s) ===")
    print(f"toplam adim sayisi: {len(sim.step_events)}  -> {sim.step_events}")
    # DURUST DUZELTME: onceki kontrol "dusme karesi tokezlemeden >20
    # kare sonra ise EVET (absorbe edildi)" diyordu -- ama bu, buyuk
    # itkiden (t=BIG_PUSH_T) ONCE gerceklesen bir dusmeyi de yanlislikla
    # "absorbe edildi" olarak raporluyordu (orn. frame 141'deki dusme,
    # aslinda STUMBLE_KICK_PX=30 tokezlemesinden kaynaklaniyordu, ama
    # st+20=110'dan buyuk oldugu icin yanlislikla EVET yazdiriyordu).
    # Dogru mantik: dusme, buyuk itki karesinden (bp) ONCE olduysa bu
    # tokezlemenin kendisinden kaynaklanmis demektir -- absorbe edilmemis.
    stumble_caused_fall = sim.fell and sim.fall_frame is not None and sim.fall_frame < bp
    print(f"tokezleme (t={STUMBLE_T}s, {STUMBLE_KICK_PX:.0f}px): "
          f"frame {st}-{st+15} hip_vx araligi=[{hip_vx_log[st:st+15].min():.2f}, {hip_vx_log[st:st+15].max():.2f}]"
          f"  (absorbe edildi mi: {'HAYIR' if stumble_caused_fall else 'EVET'})")
    print(f"buyuk itki (t={BIG_PUSH_T}s, {BIG_PUSH_KICK_PX:.0f}px) sonrasi: dustu={sim.fell}"
          f"  (dusme karesi: {sim.fall_frame}, t={sim.fall_frame/FPS if sim.fall_frame else None})")
    print(f"son hip_y: {hip_y_log[-1]:.2f} (GROUND_Y={GROUND_Y}, dusme esigi={FALL_HIP_Y_THRESHOLD})")
    print(f"ortalama hip_vx (buyuk itkiden ONCE, kararli yuruyus): {hip_vx_log[:bp].mean():.3f}px/kare (hedef={TARGET_VX})")
    print(f"kayma (slip) olaylari (kare sayisi -- aktif kayma boyunca HER kare 1 olay): {len(sim.slip_events)}")
    if sim.slip_events:
        print(f"  ilk 8: {sim.slip_events[:8]}")
        print(f"  son 8: {sim.slip_events[-8:]}")
        slipping_frames = [s[0] for s in sim.slip_events]
        streaks, cur = [], 1
        for i in range(1, len(slipping_frames)):
            if slipping_frames[i] == slipping_frames[i - 1] + 1:
                cur += 1
            else:
                streaks.append(cur)
                cur = 1
        streaks.append(cur)
        print(f"  en uzun ardisik kayma serisi (kare -- Stribeck suruklenme fazinin gercekten kalici oldugunun kaniti): {max(streaks)}")
    print(f"acil kurtarma adimi (FallRiskMonitor/COM-vs-destek-araligi) tetiklemeleri: {len(sim.emergency_step_events)}  -> {sim.emergency_step_events}")
    print(f"Faz B yakalama adimlari (toe_off tetikli): {len(sim.fazb_events)}  -> {sim.fazb_events}")
    print(f"en dusuk kalca yuksekligi (buyuk itkiden sonra): {GROUND_Y - max(sim.hip_y_log[bp:]):.1f}px")
    rise = -np.diff(np.array(sim.hip_y_log))[bp:]
    print(f"itki sonrasi en hizli kalca yukselisi: {rise.max():.1f}px/kare  (sok emilimi: {sim.shock_events})")


if __name__ == "__main__":
    main()
