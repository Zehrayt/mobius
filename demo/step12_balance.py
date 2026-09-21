"""
Adim 12 (baslangic): kutle merkezi (center of mass) dengesi -- denge
kaybinda kollarla counter-balance.

Sinyal: her karede govdenin (kalca+omuz+bas ortalamasi -- basitlestirilmis
bir "ust govde" kutle merkezi vekili, tam kutle-agirlikli bir hesap
DEGIL) yatay konumu (`com_x`) ile o an ZEMINDE DURAN (stance) ayagin/
ayaklarin yatay konumu (`support_x`, "destek tabani") arasindaki fark
hesaplanir: `error = com_x - support_x`. Bu, gercekten olculen bir fiziksel
buyuklik -- ip cambazinin "ne kadar one/geriye egildigimi" hissettigi seyin
basitlestirilmis 2D analogudur.

Tepki: kollar bu hataya ORANTILI (negatif geri besleme) sekilde tepki
verir -- `error` pozitifse (govde destek tabaninin ONUNDE, one dusme
riski) kollar GERIYE ve YUKARI kalkar (gercek insan dengesizlik refleksine
benzer sekilde -- kollari kaldirmak govdenin donme ataletini arttirip
dengeyi kolaylastirir; burada bu sadece gorsel/sezgisel bir jest, gercek
ters-dinamik/rigid-body coupling hesaplanmiyor). Bu, mevcut contralateral
kol sallanmasinin (adim 4) UZERINE eklenen AYRI bir terimdir.

DURUST BIR SINIRLAMA: bu basit bir sezgisel geri besleme -- kollarin
kutlesi bu iskelette govdenin gercek kutle merkezini fiziksel olarak
GERI CEKECEK kadar buyuk/baglantili degil (yani kollar "dogru yone"
hareket ediyor ama govdeyi gercekten dengeye GERI GETIRMIYOR, sadece
dengesizligi GORSEL OLARAK ifade ediyor). Bu, oyun animasyonlarinda
yaygin bir "kopyalanmis tepki" (cheap reactive gesture) teknigidir --
tam bir ters-dinamik/rigid-body coupling degildir; bu proje bunu boyle
sunuyor.

Tetikleyici: karakter duz zeminde yuruyor, t=3.0s'de bir "sursme" (ör.
ayagin bir seye takilmasi) sonucu kalca-suren nokta (`driver`) ANI bir
ileri sicrama yapiyor -- bu, ayagin zeminde sabit kalirken govdenin one
firlamasina (gercek bir tokezleme/kayma anina) benzer bir CoM-destek
farki yaratiyor. Kollarin buna orantili tepkisi sayisal olarak asagida
dogrulanmistir.

Cikti: outputs/step12_balance.mp4

7. TUR EKI (kullanici talebi -- "Aktif Denge ve Refleks / Center of Mass
Recovery"): yukaridaki t=3.0s'lik "surtme" (55px) kucuk bir tokezlemedir --
DIAGNOSTIC OLCUM (bkz. commit mesaji / README) bunun destek poligonunun
SADECE ~27px disina ciktigini gosterdi, yani orantili kol tepkisi zaten
yeterli (BAL_MAX_ERR=80px'te DOYMUYOR). Kullanicinin sikayeti DAHA BUYUK,
gercek bir "dusme tehlikesi" icin: sistem bunu "fark etmiyor" (adim
zamanlamasi degismiyor, kol tepkisi erken doyup buyumuyor). Bunu
gostermek/dogrulamak icin t=5.5s'de cok daha buyuk (220px) IKINCI bir kalca
itkisi eklendi (govde-sadece bir darbe DENENDI ama bu sahnenin sert govde-
kenetlemesi yuzunden yeterli buyuklukte bir tehlike yaratamadi -- bkz.
BIG_PUSH_KICK_PX sabitinin dokstring'i ve README'deki durust bulgu). Bu itkide `physics.balance.FallRiskMonitor` "tehlike" durumuna
giriyor ve iki yeni refleks devreye giriyor: (1) kollar
`emergency_counter_balance_offset` ile COK DAHA BUYUK bir genlikte "ters
yone savruluyor" (2) o an ZEMINDE DURAN bacak, normal adim ritmini
BEKLEMEDEN, kutle merkezinin dustugu yonde erken/hizli bir "yakalama
adimi" atiyor (`FootPlantingLeg.trigger_emergency_step`). Ilk (55px)
tokezleme BILINCLI OLARAK degistirilmedi -- eski kanarya sayilari
(tokezleme oncesi/sonrasi |error|, korelasyon) hala gecerli, cunku
FALL_RISK_ENTER_PX esigi bu kucuk tokezlemenin GERCEK aralik-disi
mesafesinin (~27px) UZERINDE secildi.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from physics.verlet import VerletSystem, clamp_direction
from physics.gait import FootPlantingLeg
from physics.balance import (
    upper_body_com_x, support_x, counter_balance_offset,
    support_interval, outside_interval_error, FallRiskMonitor,
    emergency_counter_balance_offset,
)

W, H = 640, 400
FPS = 30
DURATION_S = 8
N_FRAMES = FPS * DURATION_S

HIP_Y = 170.0
GROUND_Y = 330.0
TORSO_LEN = 55.0
HEAD_STICK_LEN = 30.0
HEAD_RADIUS = 15
ARM_SEG_LEN = 32.0
SHOULDER_WIDTH = 10.0

LEG_SEGMENT_LEN = 92.0
STRIDE_RELEASE = 18.0
STRIDE_AHEAD = 28.0
SWING_DURATION_FRAMES = 10
LIFT_HEIGHT = 22.0
KNEE_LIMITS = (8.0, 150.0)
KNEE_BEND_SIGN = -1.0

UP = np.array([0.0, -1.0])
TORSO_MAX_LEAN_DEG = 12.0
NECK_MAX_TILT_DEG = 18.0

TUNED_GRAVITY = np.array([0.0, 0.065])
TUNED_FRICTION = 0.045
ARM_COUNTER_SWING_PX = 16.0
WALK_SPEED = 60.0

STUMBLE_T = 3.0
STUMBLE_KICK_PX = 55.0  # aniden takilma/kayma -- bir kerelik disaridan itki

# -- Dengeleme geri beslemesi (bkz. modul dokstring'i) ------------------
BAL_GAIN_X = 0.55
BAL_GAIN_Y = 0.12
BAL_MAX_ERR = 80.0

# -- 7. tur eki: destek-poligonu tabanli tehlike tespiti + buyutulmus
#    refleks (bkz. modul dokstring'i + physics/balance.py) -----------------
FALL_RISK_ENTER_PX = 45.0   # bkz. dokstring -- 55px'lik ilk tokezlemenin
FALL_RISK_EXIT_PX = 20.0    # GERCEK aralik-disi mesafesinin (~27px) UZERINDE
EMERGENCY_GAIN_X = 0.9
EMERGENCY_MAX_ERR = 220.0
EMERGENCY_STEP_LEAD_PX = 15.0
EMERGENCY_SWING_SPEEDUP = 2.5

BIG_PUSH_T = 5.5
BIG_PUSH_KICK_PX = 220.0  # gercek bir "dusme tehlikesi" -- bkz. dokstring'i.
# DENENDI AMA VAZGECILDI: govdeye/basa SADECE bir hiz-darbesi (impulse,
# hip'e DOKUNMADAN) uygulamak -- boylece "kalcanin kendisi kinematik
# stride_release esigini KAZARA tetiklemesin" istendi. Ama bu SENARYODA
# (step12) govde/boyun HER KAREDE TORSO_MAX_LEAN_DEG=12 derecelik SERT bir
# `clamp_direction()` ile kalcaya kenetli -- bu, gövde/başın kalçadan ne
# kadar uzaklasabilecegini yapisal olarak ~+-25px'le sinirliyor (govde
# uzunlugu * sin(12derece) mertebesinde), impuls buyuklugu ne olursa olsun
# (80'den 320px/kareye kadar denendi, PIK hep ~25-28px'de SABIT kaldi --
# bkz. commit mesaji/README). Yani bu SPESIFIK sahnede (kalca=kinematik
# pin, govde=sert-kenetli) gercek bir "destek poligonu disi" tehlike SADECE
# kalcanin kendisini (driver'i) hareket ettiren bir itki ile yaratilabiliyor
# -- bu da ayni zamanda bacagin KENDI stride_release (18px) esigini de
# ANINDA tetikliyor (bkz. asagidaki DURUST BULGU). Bu, cozulmemis degil,
# BILINCLI OLARAK belgelenen bir mimari sinirlama -- bkz. README "7. tur".


def build_body() -> tuple[VerletSystem, dict]:
    sys_ = VerletSystem.empty()
    sys_.gravity = TUNED_GRAVITY.copy()
    sys_.friction = TUNED_FRICTION
    idx: dict = {}

    idx["driver"] = sys_.add_point([0.0, HIP_Y], pinned=True)
    idx["hip"] = sys_.add_point([0.0, HIP_Y])
    sys_.add_stick(idx["driver"], idx["hip"], length=3.0)

    idx["shoulder"] = sys_.add_point([0.0, HIP_Y - TORSO_LEN])
    sys_.add_stick(idx["hip"], idx["shoulder"], length=TORSO_LEN)

    idx["head"] = sys_.add_point([0.0, HIP_Y - TORSO_LEN - HEAD_STICK_LEN])
    sys_.add_stick(idx["shoulder"], idx["head"], length=HEAD_STICK_LEN)

    for side, x_off in (("l", -1.0), ("r", 1.0)):
        shoulder_pos = sys_.points[idx["shoulder"]]
        anchor_pos = shoulder_pos + [x_off * SHOULDER_WIDTH, 0.0]
        anchor = sys_.add_point(anchor_pos, pinned=True)
        idx[f"{side}_anchor"] = anchor
        elbow = sys_.add_point(anchor_pos + [0.0, ARM_SEG_LEN])
        sys_.add_stick(anchor, elbow, length=ARM_SEG_LEN)
        hand = sys_.add_point(anchor_pos + [0.0, ARM_SEG_LEN * 2])
        sys_.add_stick(elbow, hand, length=ARM_SEG_LEN)
        idx[f"{side}_elbow"] = elbow
        idx[f"{side}_hand"] = hand

    return sys_, idx


def make_leg(hip_pos: np.ndarray, initial_planted_offset: float) -> FootPlantingLeg:
    return FootPlantingLeg(
        hip_pos,
        segment_lengths=[LEG_SEGMENT_LEN, LEG_SEGMENT_LEN],
        ground_y=GROUND_Y,
        stride_release=STRIDE_RELEASE,
        stride_ahead=STRIDE_AHEAD,
        swing_duration_frames=SWING_DURATION_FRAMES,
        lift_height=LIFT_HEIGHT,
        initial_planted_offset=initial_planted_offset,
        knee_limits=KNEE_LIMITS,
        knee_bend_sign=KNEE_BEND_SIGN,
    )


def leg_swing_push(leg: FootPlantingLeg) -> float:
    if leg.state != "swing":
        return 0.0
    return float(np.sin(np.pi * min(leg.swing_t, 1.0)))


def leg_support_x(left_leg: FootPlantingLeg, right_leg: FootPlantingLeg) -> float:
    """Bu sahnenin iki bacağından `physics/balance.support_x()`'in
    beklediği `stance_x`/`fallback_x` listelerini üretip çağıran ince bir
    sarmalayıcı (wrapper)."""
    stance = [leg.planted[0] for leg in (left_leg, right_leg) if leg.state == "stance"]
    fallback = [leg.swing_target[0] for leg in (left_leg, right_leg)]
    return support_x(stance, fallback)


def draw_frame(body: VerletSystem, idx: dict, legs: list[FootPlantingLeg],
               camera_offset: float, error: float, bal_x: float,
               interval: tuple[float, float] | None = None,
               in_danger: bool = False, real_error: float = 0.0) -> np.ndarray:
    frame = np.full((H, W, 3), 22, dtype=np.uint8)
    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (85, 85, 85), 2)

    def to_screen(p):
        return (int(p[0] + camera_offset), int(p[1]))

    body_color = (90, 220, 90)
    leg_colors = [(90, 220, 90), (230, 160, 60)]
    arm_colors = {"l": (150, 230, 150), "r": (60, 170, 230)}

    cv2.line(frame, to_screen(body.points[idx["hip"]]), to_screen(body.points[idx["shoulder"]]), body_color, 4, cv2.LINE_AA)
    cv2.line(frame, to_screen(body.points[idx["shoulder"]]), to_screen(body.points[idx["head"]]), body_color, 3, cv2.LINE_AA)
    cv2.circle(frame, to_screen(body.points[idx["head"]]), HEAD_RADIUS, body_color, 2, cv2.LINE_AA)

    for side in ("l", "r"):
        a = to_screen(body.points[idx[f"{side}_anchor"]])
        e = to_screen(body.points[idx[f"{side}_elbow"]])
        h = to_screen(body.points[idx[f"{side}_hand"]])
        c = arm_colors[side]
        cv2.line(frame, a, e, c, 3, cv2.LINE_AA)
        cv2.line(frame, e, h, c, 3, cv2.LINE_AA)
        cv2.circle(frame, h, 4, (255, 255, 255), -1, cv2.LINE_AA)

    for leg, color in zip(legs, leg_colors):
        pts = leg.chain.points
        for i in range(len(pts) - 1):
            cv2.line(frame, to_screen(pts[i]), to_screen(pts[i + 1]), color, 4, cv2.LINE_AA)
        for p in pts:
            cv2.circle(frame, to_screen(p), 5, (255, 255, 255), -1, cv2.LINE_AA)

    # CoM (beyaz X) ve destek tabani (sari cizgi) gorsellestirmesi.
    com_x = upper_body_com_x(body.points, [idx["hip"], idx["shoulder"], idx["head"]])
    com_screen = to_screen([com_x, HIP_Y - TORSO_LEN - 45])
    cv2.drawMarker(frame, com_screen, (255, 255, 255), cv2.MARKER_CROSS, 10, 2)
    base_screen_x = int(com_x - error + camera_offset)
    cv2.line(frame, (base_screen_x, int(GROUND_Y) - 4), (base_screen_x, int(GROUND_Y) + 4), (80, 220, 240), 3)

    # 7. tur eki: gercek destek araligi (yatay cizgi -- eski tek NOKTA
    # yerine) + tehlike durumu gorsellestirmesi.
    if interval is not None:
        lo, hi = interval
        y_line = int(GROUND_Y) + 14
        cv2.line(frame, (int(lo + camera_offset), y_line), (int(hi + camera_offset), y_line),
                 (60, 220, 255) if not in_danger else (60, 60, 255), 3)
    danger_txt = f"  [TEHLIKE! aralik-disi: {real_error:+.1f}px]" if in_danger else ""
    bar_x, bar_y = 12, 40
    cv2.putText(frame, f"CoM - destek farki (error): {error:+.1f}px   kol tepkisi: {bal_x:+.1f}px{danger_txt}",
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                (210, 210, 210) if not in_danger else (110, 110, 255), 1, cv2.LINE_AA)
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + 160, bar_y + 12), (70, 70, 70), 1)
    mid = bar_x + 80
    cv2.line(frame, (mid, bar_y), (mid, bar_y + 12), (120, 120, 120), 1)
    filled = int(np.clip(error, -BAL_MAX_ERR, BAL_MAX_ERR) / BAL_MAX_ERR * 80)
    if filled >= 0:
        cv2.rectangle(frame, (mid, bar_y), (mid + filled, bar_y + 12), (80, 220, 240), -1)
    else:
        cv2.rectangle(frame, (mid + filled, bar_y), (mid, bar_y + 12), (80, 220, 240), -1)

    cv2.putText(frame, "Adim 12 (baslangic): kutle merkezi - destek tabani farkina orantili kol dengeleme",
                (12, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    body, idx = build_body()
    half_stride = (STRIDE_RELEASE + STRIDE_AHEAD) / 2.0
    left_leg = make_leg(body.points[idx["hip"]].copy(), 0.0)
    right_leg = make_leg(body.points[idx["hip"]].copy(), -half_stride)

    driver_x = 0.0
    dt = 1.0 / FPS
    kicked = False
    big_pushed = False
    risk_monitor = FallRiskMonitor(FALL_RISK_ENTER_PX, FALL_RISK_EXIT_PX)
    emergency_step_frames = []  # (frame, hangi_bacak) -- tani/rapor icin

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step12_balance.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    err_log = []
    offset_log = []
    real_err_log = []
    danger_log = []

    for f in range(N_FRAMES):
        t = f * dt
        driver_x += WALK_SPEED * dt
        if not kicked and t >= STUMBLE_T:
            driver_x += STUMBLE_KICK_PX  # bkz. modul dokstring'i -- ani "tokezleme" itkisi
            kicked = True
        if not big_pushed and t >= BIG_PUSH_T:
            driver_x += BIG_PUSH_KICK_PX  # 7. tur eki -- bkz. yukaridaki sabit dokstring'i
            big_pushed = True
        body.set_pinned_position(idx["driver"], [driver_x, HIP_Y])

        com_x = upper_body_com_x(body.points, [idx["hip"], idx["shoulder"], idx["head"]])
        base_x = leg_support_x(left_leg, right_leg)
        error = com_x - base_x
        err_log.append(error)

        # -- 7. tur eki: gercek destek araligi + aralik-disi mesafe + -------
        #    histerezisli tehlike tespiti (bkz. modul dokstring'i)
        stance_xs = [leg.planted[0] for leg in (left_leg, right_leg) if leg.state == "stance"]
        fallback_xs = [left_leg.swing_target[0], right_leg.swing_target[0]]
        interval = support_interval(stance_xs, fallback_x=fallback_xs)
        real_error = outside_interval_error(com_x, interval)
        in_danger = risk_monitor.update(real_error)
        real_err_log.append(real_error)
        danger_log.append(in_danger)

        if in_danger:
            bal_x, bal_y = emergency_counter_balance_offset(real_error, EMERGENCY_GAIN_X, BAL_GAIN_Y, EMERGENCY_MAX_ERR)
            # dusme reflek'i: o an zeminde duran bacagi kutle merkezinin
            # dustugu yonde erken/hizli bir "yakalama adimi"na yonlendir.
            target_x = com_x + np.sign(real_error) * EMERGENCY_STEP_LEAD_PX
            for name, leg in (("l", left_leg), ("r", right_leg)):
                if leg.trigger_emergency_step(target_x, speedup=EMERGENCY_SWING_SPEEDUP):
                    emergency_step_frames.append((f, name))
                    break
        else:
            bal_x, bal_y = counter_balance_offset(error, BAL_GAIN_X, BAL_GAIN_Y, BAL_MAX_ERR)
        offset_log.append(bal_x)

        shoulder_pos = body.points[idx["shoulder"]]
        left_push = leg_swing_push(right_leg) * ARM_COUNTER_SWING_PX
        right_push = -leg_swing_push(left_leg) * ARM_COUNTER_SWING_PX
        body.set_pinned_position(idx["l_anchor"], shoulder_pos + [-SHOULDER_WIDTH + left_push + bal_x, bal_y])
        body.set_pinned_position(idx["r_anchor"], shoulder_pos + [SHOULDER_WIDTH + right_push + bal_x, bal_y])

        body.step(dt=1.0)
        clamp_direction(body.points, body.prev_points, idx["hip"], idx["shoulder"], UP, TORSO_MAX_LEAN_DEG)
        torso_dir = body.points[idx["shoulder"]] - body.points[idx["hip"]]
        clamp_direction(body.points, body.prev_points, idx["shoulder"], idx["head"], torso_dir, NECK_MAX_TILT_DEG)

        hip_pos = body.points[idx["hip"]]
        left_leg.update(hip_pos)
        right_leg.update(hip_pos)

        camera_offset = W / 2 - hip_pos[0]
        frame = draw_frame(body, idx, [left_leg, right_leg], camera_offset, error, bal_x,
                            interval=interval, in_danger=in_danger, real_error=real_error)
        writer.write(frame)

    writer.release()
    print(f"wrote {out_path}")

    err_log = np.array(err_log)
    offset_log = np.array(offset_log)
    kt = int(STUMBLE_T * FPS)
    print(f"tokezleme oncesi ort. |error| (dogal yuruyus salinimi): {np.abs(err_log[:kt]).mean():.2f}px")
    peak_i = kt + int(np.abs(err_log[kt:kt + 15]).argmax())
    print(f"tokezleme sonrasi PIK |error|: {abs(err_log[peak_i]):.2f}px @ frame {peak_i} (kol tepkisi: {offset_log[peak_i]:+.2f}px)")
    # NOT (7. tur eki): bu korelasyon METRIGI orijinalinde SADECE normal
    # yuruyus-salinimi kaynakli, ORANTILI (counter_balance_offset) tepkiyi
    # olcuyordu. Yeni buyuk-itki (t=5.5s) sahnesi acil-durum penceresinde
    # (bkz. asagi) BILINCLI OLARAK FARKLI bir formul (emergency_counter_
    # balance_offset, farkli girdi: real_error) kullaniyor -- bu ikisini
    # AYNI korelasyonda karistirmak yanlis olur (0.9072 gibi yapay-dusuk
    # bir sayi cikarir). Bu yuzden ORIJINAL kanarya, orijinal olcum
    # penceresiyle (buyuk itkiden ONCE, t<BIG_PUSH_T) BIREBIR AYNI sekilde
    # yeniden hesaplaniyor -- geriye donuk uyumluluk boylece korunuyor.
    pre_push_n = int(BIG_PUSH_T * FPS)
    corr = float(np.corrcoef(offset_log[:pre_push_n],
                              -np.clip(err_log[:pre_push_n], -BAL_MAX_ERR, BAL_MAX_ERR))[0, 1])
    print(f"kol-ofseti ile -error(clipped) korelasyonu (t<{BIG_PUSH_T}s, ORIJINAL kanarya penceresi): "
          f"{corr:.4f} (1.0 = tam orantili tepki)")

    # -- 7. tur eki: buyuk itki (t=5.5s) sonrasi tehlike/kurtarma raporu ----
    real_err_log = np.array(real_err_log)
    danger_log = np.array(danger_log)
    bpt = int(BIG_PUSH_T * FPS)
    peak_real_i = bpt + int(np.abs(real_err_log[bpt:bpt + 30]).argmax())
    print()
    print(f"=== 7. tur: buyuk itki ({BIG_PUSH_KICK_PX:.0f}px @ t={BIG_PUSH_T}s) sonrasi ===")
    print(f"PIK gercek aralik-disi mesafe: {real_err_log[peak_real_i]:+.2f}px @ frame {peak_real_i}")
    print(f"tehlike durumuna girildi mi: {'EVET' if danger_log[bpt:].any() else 'HAYIR'}")
    print(f"tetiklenen acil adim(lar): {emergency_step_frames}")
    if danger_log[bpt:].any():
        recovered = np.where(~danger_log[bpt:])[0]
        # bpt'den SONRAKI ilk 'tehlike disi' karesi (bpt+recovered[0] -- ama
        # danger_log[bpt] zaten True olabilir, ilk KEZ tehlikeye girdikten
        # SONRAKI ilk cikisi ariyoruz)
        entered = np.where(danger_log[bpt:])[0]
        if len(entered) > 0:
            enter_i = bpt + entered[0]
            after_enter = np.where(~danger_log[enter_i:])[0]
            if len(after_enter) > 0:
                recovery_frame = enter_i + after_enter[0]
                print(f"tehlikeye giris: frame {enter_i} (t={enter_i/FPS:.2f}s) -> "
                      f"kurtulma: frame {recovery_frame} (t={recovery_frame/FPS:.2f}s) "
                      f"[{recovery_frame - enter_i} kare surdu]")


if __name__ == "__main__":
    main()
