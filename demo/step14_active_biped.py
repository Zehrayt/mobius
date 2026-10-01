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
from physics.trunk_yaw import (TrunkYaw, yaw_momentum, side_sign, HIP_HALF_WIDTH,
                               SHOULDER_HALF_WIDTH)
from physics.leg_mass import (LegMassModel, THIGH_MASS, SHANK_MASS,
                              THIGH_AXIS_FRAC, SHANK_AXIS_FRAC)
from physics.active_gait import ActiveFootPlantingLeg, PHASE_TOE_OFF, PHASE_HEEL_STRIKE
from physics.environment import Terrain
from physics.balance import support_interval, outside_interval_error, FallRiskMonitor, upper_body_com_x
from physics.arms import PhysicalArms, ArmGains
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

TUNED_GRAVITY = np.array([0.0, 0.065])
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

# -- Capture-point (destek/adim) parametreleri (bkz. active_gait.py) ----
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


def build_body() -> tuple[VerletSystem, dict]:
    sys_ = VerletSystem.empty()
    sys_.gravity = TUNED_GRAVITY.copy()
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

    cv2.line(frame, to_screen(body.points[idx["hip"]]), to_screen(body.points[idx["shoulder"]]), body_color, 4, cv2.LINE_AA)
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
                 catch_timing: str | None = None, hip_torque_max: float | None = None):
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
        self.leg_mass = LegMassModel(TUNED_GRAVITY) if use_lm else None
        self.predictive_sensor = PREDICTIVE_SENSOR_ENABLED if predictive_sensor is None else predictive_sensor
        self.fazb_events = []   # (kare, bacak, 'compress'|'launch', 'toe'|'heel', bacak_acisi)
        self.catch_frames_log = []   # Adim 23: (inis karesi, bacak, kalkistan inise kare)
        self.shock_absorb = SHOCK_ABSORB_ENABLED if shock_absorb is None else shock_absorb
        self.shock_pending = None   # yakalama inisi yapan bacak (anchor ona gecince baslar)
        self.shock_rest = None      # None: sok emilimi aktif degil
        self.shock_events = []      # (kare, baslangic_dinlenme_boyu)

        self.body, self.idx = build_body()
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
        self.left_leg = make_leg(self.body.points[hip].copy(), -half, support_margin, swing_lead_margin)
        self.right_leg = make_leg(self.body.points[hip].copy(), +half, support_margin, swing_lead_margin)
        for leg in (self.left_leg, self.right_leg):
            leg.faz_b_enabled = self.faz_b
            leg.predictive_sensor = self.predictive_sensor
            leg.catch_timing = catch_timing          # None: physics.active_gait.CATCH_TIMING
            leg.hip_torque_max = hip_torque_max      # None: physics.active_gait.HIP_TORQUE_MAX
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

    def system_com_vx(self) -> float:
        """Adim 22: govde + kollar + iki bacagin (pergel ekseni segmentleri)
        kutle-agirlikli yatay hizi. Bacak segmenti i: v = (1-f_i) v_kalca + f_i v_ayak."""
        idx, p, q = self.idx, self.body.points, self.body.prev_points
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
            f_n_effective = THRUST_CAP * load_factor

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
            in_danger = self.risk_monitor.update(real_error)
            # Adim 18 (Faz B): stance bacagi toe_off'ta ve kalca onu asiri
            # hizla geciyorsa (ya da tehlike + toe_off), tek-destek kuralini
            # bozmadan yakalama adimi: havadaki bacak varsa ONUN salinimini
            # sikistir, yoksa toe_off bacagini sikistirilmis salinimla birak.
            fazb_handled = False
            # Adim 19c -- prediktif sensor: bu karenin itkisi prev_points'te,
            # kalcanin bir sonraki konumu Verlet kuraliyla tahmin edilir.
            stance_leg.predict_contact(body.points[hip], body.prev_points[hip])
            front = stance_leg.catch_overrun() if self.faz_b else None
            if self.faz_b and front is None and in_danger:
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

            body.set_pinned_position(anchor, [stance_leg.planted[0], GROUND_Y])
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
            self.shock_pending = None
            self.shock_events.append((f, round(self.shock_rest, 1)))
        if self.shock_rest is not None:
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

        if self.arms is not None:
            hp = body.points[hip]
            leg_angles = {k: float(np.arctan2(leg.chain.points[-1][0] - hp[0], leg.chain.points[-1][1] - hp[1]))
                          for k, leg in (("l", left_leg), ("r", right_leg))}
            reflex = None
            if self.arm_reflex and self.reflex_level > ARM_REFLEX_OFF_LEVEL:
                reflex = self.reflex_dir * ARM_REFLEX_DEG
            catching = any(l.catch_active for l in (left_leg, right_leg))
            drive_on = self.arms_mode == "drive" and not (ARM_PASSIVE_DURING_CATCH and catching)
            if self.arms_mode == "cancel" and reflex is None:
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
        if self.posture_k > 0.0 or self.posture_c > 0.0:
            apply_angular_couple(body.points, body.prev_points, body.masses, hip, idx["shoulder"], UP,
                                 POSTURE_TARGET_DEG, self.posture_k, self.posture_c)

        body.step(dt=1.0)
        # preserve_momentum=True -- bkz. modulun 8. tur notu: varsayilan
        # (False) govde/boyun kelepcesi uzerinden kalcanin KENDI hizini da
        # sessizce sifirlayip dusme esigini maskeliyordu.
        clamp_direction(body.points, body.prev_points, hip, idx["shoulder"], UP, TORSO_MAX_LEAN_DEG,
                        preserve_momentum=self.torso_clamp_mode)
        torso_dir = body.points[idx["shoulder"]] - body.points[hip]
        clamp_direction(body.points, body.prev_points, idx["shoulder"], idx["head"], torso_dir, NECK_MAX_TILT_DEG,
                        preserve_momentum=self.torso_clamp_mode)
        if self.arms is not None:
            self.arms.constrain()

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
            leg.update(hip_pos, hip_vx=ctrl_vx, other_leg_swinging=(other.state == "swing"))
            if was_swing and was_catch and leg.state == "stance":
                # Adim 23: yakalama adiminin gercek suresi (kalkistan inise)
                self.catch_frames_log.append((f, "l" if leg is left_leg else "r",
                                              f - getattr(leg, "catch_start_frame", f)))
                if self.shock_absorb:
                    self.shock_pending = leg
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

        if self.leg_mass is not None:
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
