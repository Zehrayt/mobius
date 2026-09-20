"""
Adim 9 (baslangic): aktif/pasif ragdoll harmani -- IK guedumlu "aktif"
durum ile tamamen yercekimine teslim "pasif ragdoll" durumu arasinda
puruzsuz gecis.

Mekanizma
---------
Bacaklar artik SADECE FabrikChain2D ile cozulmuyor -- kalca/diz/ayak
noktalari AYNI ZAMANDA ana `VerletSystem`'in normal (rijit, compliance=0)
noktalari/cubuklari olarak da var oluyor. Her karede iki konum hesaplanir:
  - "pasif" konum: `body.step()` + `collide_ground()`'un DOGAL SONUCU
    (sadece yercekimi + cubuk kisitlari + zemin -- hicbir IK etkisi yok).
  - "aktif" konum: `FootPlantingLeg`/`FabrikChain2D.solve()`'in urettigi
    IK hedefi (eskisi gibi).
Bir `blend` katsayisi (1.0 = tam aktif/IK, 0.0 = tam pasif/ragdoll) bu
ikisi arasinda lineer olarak karisir: `blend*aktif + (1-blend)*pasif`.
`blend=1` iken sonuc onceki adimlardaki (step3-step8) yuruyusle BIREBIR
ayni davranir (IK tam kontrolde); `blend=0` iken bacaklar/govde tamamen
serbest birakilir.

Govde/boyun `clamp_direction()`'i da ayni `blend` ile "yumusatiliyor":
izin verilen maksimum sapma acisi `blend=1`'de her zamanki gibi dar
(12 derece, kontrollu yuruyus) `blend=0`'da pratik olarak sinirsiz (180
derece, serbest dusme) -- yani govde/boyun stabilizasyonu da IK ile
BIRLIKTE devre disi kaliyor, cunku "aktif" durum zaten govdeyi kontrol
altinda tutmanin bir parcasi.

`driver` (kalca pin'i) de aritik `blend`'e gore hedefleniyor: `blend=1`'de
her zamanki gibi yuruyus hizinin belirledigi konuma, `blend=0`'da ise
kalcanin BIR ONCEKI karedeki KENDI fizik-cozumlu konumuna -- yani pratikte
kalcayi de serbest birakiyor (cubuk sifir mesafeye yakinsadigi icin
pratikte cekme kuvveti kalmiyor). Boylece pasif modda TUM govde (kalca
dahil) yercekimine/`collide_ground()`'a teslim oluyor.

Tetikleyici olay: karakter 3 saniye normal yuruyor (blend=1), sonra bir
"darbe" (ör. bir engele carpma) 0.6 saniyede blend'i 1'den 0'a indiriyor
-- bu andan itibaren karakter govdeyi/bacaklari kontrol eden IK/clamp
tamamen devre disi kalip cigrindan cikmis (double-pendulum tarzi, bkz.
`physics/verlet.py`dostringindeki ayni fenomen -- orada `clamp_direction`
TAM DA bunu onlemek icin eklenmisti; burada BILEREK devre disi birakiliyor)
kaotik bir cokme/yuvarlanma sergiliyor.

DURUST BIR NOT: pasif modda govde/bacaklar arasinda ek bir eklem sonumlemesi
(joint damping) YOK -- sadece `VerletSystem.friction` (pasifte biraz
artiriliyor) var. Bu yuzden cokme cok kisa surede tamamen durup dumduz
yatmiyor; birkac saniye boyunca (gercek bir kontrolsuz cift-sarkac gibi)
salinip yavas yavas enerji kaybediyor -- bu, projenin daha once "aktif"
modda cozdugu ayni kaosun, "pasif" modda BILEREK geri getirilmis hali.
Sayisal olarak: aktif->pasif gecisinde IK'nin onerdigi hedef ile fizigin
kendi basina urettigi konum arasindaki fark (`|aktif - pasif|`) aktif
fazda ort. birkac piksel iken pasif fazda >100px'e cikiyor -- yani IK
kontrolu birakildiginda govde gercekten "kendi basina" hareket ediyor.

Cikti: outputs/step9_ragdoll_blend.mp4
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from physics.verlet import VerletSystem, clamp_direction
from physics.gait import FootPlantingLeg
from physics.fabrik import clamp_joint_angle_points
from physics.collision import collide_ground
from physics.self_collision import push_points_off_segment
from physics.environment import Terrain
from physics.balance import reach_pulldown_offset
from physics.ragdoll import (
    blend_point,
    blend_prev_points,
    blended_max_angle,
    blended_friction,
    driver_follow_target,
    transition_impulse_vector,
    apply_impulse,
)

W, H = 640, 400
FPS = 30
DURATION_S = 13
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
# DUZELTME (3. tur kullanici geri bildirimi -- "ragdoll'da omurga
# cokmesi"): pasif (blend=0) modda `blended_max_angle()` daha once
# varsayilan `passive_max_deg=180.0` (pratik olarak SINIRSIZ) kullaniyordu
# -- sayisal tani (diag_round3.py) bunun GERCEK etkisini olctu: knockdown
# sonrasi govde acisi birkac saniye icinde -12 derece civarindan 120
# dereceyi ASAN degerlere savruluyordu (t=7.6s: 24.4 deg -> t=10.0s: 120.7
# deg) -- gercek bir omurga bu kadar katlanamaz. Kullanicinin onerdigi
# "Verlet yaylarina sertlik atanmasi" fikri, bu mimaride en dogal karsiligi
# `blend_direction()`'in sinirini 180'den, gevsek ama SONLU bir degere
# indirmekte buluyor -- pasifte hala aktiften cok daha serbest (ragdoll
# hissi korunuyor) ama fiziksel olarak imkansiz tam-katlanmaya izin
# vermiyor.
PASSIVE_TORSO_MAX_DEG = 75.0
PASSIVE_NECK_MAX_DEG = 85.0
# DUZELTME (3. tur kullanici geri bildirimi -- "omuz ve dirseklerin vucut
# icinden gecmesi"): eskiden kollarin omuza gore ACISAL hicbir siniri
# yoktu (sadece sabit uzunluklu cubuklarla baglıydı) -- sayisal tani kol/
# govde en yakin mesafesinin AKTIF yurumede bile sag kolda 210 karenin
# 151'inde 10px'in altina dustugunu olctu (bazen 0px -- tam govde
# uzerinde). Iki katmanli duzeltme: (1) "Ulasim Konisi" -- omuz->dirsek
# ve dirsek->el yonleri govde eksenine gore `clamp_direction()` ile
# sinirlaniyor (zaten govde/boyun icin kullanilan AYNI genel fonksiyon);
# (2) `push_points_off_segment()` ile (pelerin icin kullanilanla AYNI
# mekanizma) dirsek/el, govde (kalca-omuz, omuz-kafa) segmentlerinden
# fiziksel olarak itiliyor.
ARM_CONE_ACTIVE_DEG = 45.0
ARM_CONE_PASSIVE_DEG = 100.0
ELBOW_CONE_ACTIVE_DEG = 55.0
ELBOW_CONE_PASSIVE_DEG = 120.0
ARM_SELF_COLLISION_DIST = 11.0

ACTIVE_GRAVITY = np.array([0.0, 0.065])
ACTIVE_FRICTION = 0.045
RAGDOLL_FRICTION = 0.30  # pasif modda enerjinin makul surede sonumlenmesi icin arttirildi

ARM_COUNTER_SWING_PX = 16.0
WALK_SPEED = 65.0

KNOCKDOWN_T = 3.0
BLEND_DOWN_DURATION = 0.6

# DUZELTME (kullanici geri bildirimi -- "momentum aktarilmiyor"): gecis
# aninda kalca/govde/kafaya, karakterin O ANKI vektorel hizinin
# `KNOCKDOWN_VELOCITY_GAIN` katina esit + kucuk bir yukari "kalkis"
# (`KNOCKDOWN_UP_KICK`, negatif = yukari) darbesi enjekte edilir --
# bkz. `physics.ragdoll.transition_impulse_vector`.
KNOCKDOWN_VELOCITY_GAIN = 4.0
KNOCKDOWN_UP_KICK = -5.0

# DUZELTME (kullanici geri bildirimi -- "kemik esnemesi / kutle merkezi
# baglantisizligi"): bacak hedefi kendi menzilini astiginda (bkz.
# FootPlantingLeg.last_overrun_px), bacagi germek yerine kalcayi
# asagi+ileri egerek hedefi bir sonraki karede gercekten erisilebilir
# yapmaya calisir -- bkz. physics.balance.reach_pulldown_offset.
PULLDOWN_GAIN_X = 0.35
PULLDOWN_GAIN_Y = 0.55
PULLDOWN_MAX_OFFSET = 40.0


def build_body() -> tuple[VerletSystem, dict]:
    sys_ = VerletSystem.empty()
    sys_.gravity = ACTIVE_GRAVITY.copy()
    sys_.friction = ACTIVE_FRICTION
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

    # YENI: her bacagin diz/ayak noktalari ARTIK VerletSystem'in de bir
    # parcasi (rijit sticklerle) -- pasif modda serbestce sarkabilsinler
    # diye. Aktif modda bu noktalar her karede IK cozumune "cekilir"
    # (asagida `main()` icinde, blend ile).
    for side in ("l", "r"):
        hip_pos = sys_.points[idx["hip"]]
        knee = sys_.add_point(hip_pos + [0.0, LEG_SEGMENT_LEN])
        foot = sys_.add_point(hip_pos + [0.0, LEG_SEGMENT_LEN * 2])
        sys_.add_stick(idx["hip"], knee, length=LEG_SEGMENT_LEN)
        sys_.add_stick(knee, foot, length=LEG_SEGMENT_LEN)
        idx[f"{side}_knee"] = knee
        idx[f"{side}_foot"] = foot

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


TERRAIN = Terrain(ground_y=GROUND_Y, default_friction=0.3)


def blend_at(t: float) -> float:
    """1.0 = tam aktif (IK kontrolunde), 0.0 = tam pasif (ragdoll).
    t=KNOCKDOWN_T'de bir "darbe" ile BLEND_DOWN_DURATION suresinde
    lineer olarak 1'den 0'a iner ve orada kalir (bu demoda geri kalkma
    yok -- olasi bir sonraki adim, README'de not edildi)."""
    if t < KNOCKDOWN_T:
        return 1.0
    if t < KNOCKDOWN_T + BLEND_DOWN_DURATION:
        return 1.0 - (t - KNOCKDOWN_T) / BLEND_DOWN_DURATION
    return 0.0


def draw_frame(body: VerletSystem, idx: dict, camera_offset: float, blend: float, t: float) -> np.ndarray:
    frame = np.full((H, W, 3), 22, dtype=np.uint8)
    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (85, 85, 85), 2)

    def to_screen(p):
        return (int(p[0] + camera_offset), int(p[1]))

    state_color = (90, 220, 90) if blend > 0.5 else (230, 90, 90)

    cv2.line(frame, to_screen(body.points[idx["hip"]]), to_screen(body.points[idx["shoulder"]]), state_color, 4, cv2.LINE_AA)
    cv2.line(frame, to_screen(body.points[idx["shoulder"]]), to_screen(body.points[idx["head"]]), state_color, 3, cv2.LINE_AA)
    cv2.circle(frame, to_screen(body.points[idx["head"]]), HEAD_RADIUS, state_color, 2, cv2.LINE_AA)

    arm_colors = {"l": (150, 230, 150), "r": (60, 170, 230)}
    for side in ("l", "r"):
        a = to_screen(body.points[idx[f"{side}_anchor"]])
        e = to_screen(body.points[idx[f"{side}_elbow"]])
        h = to_screen(body.points[idx[f"{side}_hand"]])
        c = arm_colors[side]
        cv2.line(frame, a, e, c, 3, cv2.LINE_AA)
        cv2.line(frame, e, h, c, 3, cv2.LINE_AA)
        cv2.circle(frame, h, 4, (255, 255, 255), -1, cv2.LINE_AA)

    leg_colors = [(90, 220, 90) if blend > 0.5 else (230, 160, 90),
                  (230, 160, 60) if blend > 0.5 else (230, 120, 60)]
    for side, color in zip(("l", "r"), leg_colors):
        hip_p = to_screen(body.points[idx["hip"]])
        knee_p = to_screen(body.points[idx[f"{side}_knee"]])
        foot_p = to_screen(body.points[idx[f"{side}_foot"]])
        cv2.line(frame, hip_p, knee_p, color, 4, cv2.LINE_AA)
        cv2.line(frame, knee_p, foot_p, color, 4, cv2.LINE_AA)
        cv2.circle(frame, foot_p, 5, (255, 255, 255), -1, cv2.LINE_AA)

    label = "AKTIF (IK kontrolunde)" if blend > 0.99 else ("PASIF (ragdoll)" if blend < 0.01 else f"GECIS (blend={blend:.2f})")
    cv2.putText(frame, f"t={t:4.1f}s  durum: {label}", (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 210, 210), 1, cv2.LINE_AA)
    bar_x = 12
    cv2.rectangle(frame, (bar_x, 40), (bar_x + 150, 52), (70, 70, 70), 1)
    cv2.rectangle(frame, (bar_x, 40), (bar_x + int(150 * blend), 52), state_color, -1)

    cv2.putText(frame, "Adim 9 (baslangic): aktif (IK) / pasif (ragdoll) fizik harmani",
                (12, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    body, idx = build_body()
    half_stride = (STRIDE_RELEASE + STRIDE_AHEAD) / 2.0
    left_leg = make_leg(body.points[idx["hip"]].copy(), 0.0)
    right_leg = make_leg(body.points[idx["hip"]].copy(), -half_stride)

    driver_x = 0.0
    dt = 1.0 / FPS

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step9_ragdoll_blend.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    control_err_log = []
    torso_angle_log = []
    knockdown_kicked = False
    pulldown_x, pulldown_y = 0.0, 0.0
    max_overrun_seen = 0.0

    for f in range(N_FRAMES):
        t = f * dt
        blend = blend_at(t)

        if blend > 0.0:
            driver_x += WALK_SPEED * dt * blend  # pasifte artik ilerlemiyor

        walk_driver_pos = np.array([driver_x + pulldown_x, HIP_Y + pulldown_y])
        hip_last_pos = body.points[idx["hip"]].copy()
        driver_target = driver_follow_target(hip_last_pos, walk_driver_pos, blend)
        body.set_pinned_position(idx["driver"], driver_target)
        body.friction = blended_friction(blend, ACTIVE_FRICTION, RAGDOLL_FRICTION)

        shoulder_pos = body.points[idx["shoulder"]]
        left_push = leg_swing_push(right_leg) * ARM_COUNTER_SWING_PX * blend
        right_push = -leg_swing_push(left_leg) * ARM_COUNTER_SWING_PX * blend
        body.set_pinned_position(idx["l_anchor"], shoulder_pos + [-SHOULDER_WIDTH + left_push, 0.0])
        body.set_pinned_position(idx["r_anchor"], shoulder_pos + [SHOULDER_WIDTH + right_push, 0.0])

        # DUZELTME (kullanici geri bildirimi -- "momentum aktarilmiyor"):
        # gecisin TAM BASLADIGI karede (blend ilk kez 1.0'in altina
        # dustugunde), karakterin O ANDAKI kendi vektorel hizini
        # buyuterek + kucuk bir yukari kalkisla govdeye (kalca/omuz/kafa)
        # BIR KEZ enjekte et -- boylece ragdoll, "oldugu yerde comelmek"
        # yerine mevcut hareket yonunde/hiziyla firlatilmis gibi baslar.
        if not knockdown_kicked and blend < 1.0:
            impulse_indices = [idx["hip"], idx["shoulder"], idx["head"]]
            current_vel = body.points[idx["hip"]] - body.prev_points[idx["hip"]]
            impulse = transition_impulse_vector(current_vel, KNOCKDOWN_VELOCITY_GAIN, KNOCKDOWN_UP_KICK)
            apply_impulse(body.points, body.prev_points, impulse_indices, impulse)
            knockdown_kicked = True

        body.step(dt=1.0)

        # Govde/boyun stabilizasyonu da blend ile "gevsetiliyor" -- aktifte
        # dar (12/18 derece), pasifte GEVSEK AMA SONLU (3. tur duzeltmesi --
        # bkz. PASSIVE_TORSO_MAX_DEG/PASSIVE_NECK_MAX_DEG tanimi).
        max_lean = blended_max_angle(blend, TORSO_MAX_LEAN_DEG, PASSIVE_TORSO_MAX_DEG)
        max_neck = blended_max_angle(blend, NECK_MAX_TILT_DEG, PASSIVE_NECK_MAX_DEG)
        clamp_direction(body.points, body.prev_points, idx["hip"], idx["shoulder"], UP, max_lean)
        torso_dir = body.points[idx["shoulder"]] - body.points[idx["hip"]]
        clamp_direction(body.points, body.prev_points, idx["shoulder"], idx["head"], torso_dir, max_neck)

        # DUZELTME (3. tur -- "omuz/dirsek govde icinden geciyor"): "Ulasim
        # Konisi" -- omuz->dirsek govde eksenine (torso_dir'in tersi, yani
        # asagi/kola dogru), dirsek->el de ust-kol yonune gore sinirlaniyor.
        arm_cone = blended_max_angle(blend, ARM_CONE_ACTIVE_DEG, ARM_CONE_PASSIVE_DEG)
        elbow_cone = blended_max_angle(blend, ELBOW_CONE_ACTIVE_DEG, ELBOW_CONE_PASSIVE_DEG)
        arm_hang_dir = -torso_dir
        for side in ("l", "r"):
            clamp_direction(body.points, body.prev_points, idx[f"{side}_anchor"], idx[f"{side}_elbow"], arm_hang_dir, arm_cone)
            upper_arm_dir = body.points[idx[f"{side}_elbow"]] - body.points[idx[f"{side}_anchor"]]
            clamp_direction(body.points, body.prev_points, idx[f"{side}_elbow"], idx[f"{side}_hand"], upper_arm_dir, elbow_cone)

        collide_ground(body, TERRAIN.floor_fn, TERRAIN.friction_fn)

        # DUZELTME (3. tur -- devam): koni tek basina dirsegin govdeye COK
        # YAKIN durmasini engellemiyor (genis bir koni acisinda bile kol
        # govdeye deginebilir) -- pelerinde kullanilanla AYNI nokta-vs-
        # segment itme mekanizmasi kol/govde icin de uygulaniyor.
        for side in ("l", "r"):
            push_points_off_segment(body.points, body.prev_points, [idx[f"{side}_elbow"], idx[f"{side}_hand"]],
                                     idx["hip"], idx["shoulder"], ARM_SELF_COLLISION_DIST)
            push_points_off_segment(body.points, body.prev_points, [idx[f"{side}_elbow"], idx[f"{side}_hand"]],
                                     idx["shoulder"], idx["head"], ARM_SELF_COLLISION_DIST)

        # DUZELTME (kullanici geri bildirimi -- "anatomik butunluk / IK
        # dagilmasi"): pasif (ragdoll) diz/ayak temsiline de -- IK'nin
        # kendi zincirinde zaten var olan -- AYNI diz aci sinirini uygula.
        # Aksi halde bu "pasif" konum (asagida yakalanan) hicbir aci
        # kisiti olmadan olculmustu (0.3-142.6 derece arasi serbest --
        # dizin kendi uzerine katlanmasi ya da tersine bukulmesi dahil).
        # Bu bir eklemin fiziksel hareket acikligi bilinc disinda (ragdoll)
        # da bilincli (yururken) oldugundan farkli olmadigi icin HER ZAMAN
        # (blend'den bagimsiz) uygulaniyor.
        for side in ("l", "r"):
            clamp_joint_angle_points(
                body.points, body.prev_points,
                idx["hip"], idx[f"{side}_knee"], idx[f"{side}_foot"],
                *KNEE_LIMITS, bend_sign=KNEE_BEND_SIGN,
            )

        hip_pos = body.points[idx["hip"]]
        # "Pasif" (fizigin kendi basina urettigi) diz/ayak konumlarini,
        # IK ile karistirmadan ONCE yakala.
        passive = {}
        for side in ("l", "r"):
            passive[f"{side}_knee"] = body.points[idx[f"{side}_knee"]].copy()
            passive[f"{side}_foot"] = body.points[idx[f"{side}_foot"]].copy()

        left_leg.update(hip_pos)
        right_leg.update(hip_pos)

        # DUZELTME (kullanici geri bildirimi -- devam): bu karede bacaklardan
        # biri erisemedi mi (overrun>0) diye bak, bir sonraki karenin
        # `walk_driver_pos`'una uygulanacak asagi+ileri kalca ofsetini
        # guncelle. Overrun temizlenince ofset de otomatik sifira doner.
        overrun = max(left_leg.last_overrun_px, right_leg.last_overrun_px)
        max_overrun_seen = max(max_overrun_seen, overrun)
        pulldown_x, pulldown_y = reach_pulldown_offset(
            overrun, PULLDOWN_GAIN_X, PULLDOWN_GAIN_Y, PULLDOWN_MAX_OFFSET, travel_dir=1.0,
        )

        err_accum = 0.0
        for side, leg in (("l", left_leg), ("r", right_leg)):
            active_knee = leg.chain.points[1]
            active_foot = leg.chain.points[2]
            for key, active_pt in ((f"{side}_knee", active_knee), (f"{side}_foot", active_foot)):
                pidx = idx[key]
                pas = passive[key]
                blended = blend_point(pas, active_pt, blend)
                err_accum += float(np.linalg.norm(pas - active_pt))
                body.points[pidx] = blended
                body.prev_points[pidx] = blend_prev_points(body.prev_points[pidx], blended, blend)

        # DUZELTME (kullanici geri bildirimi -- "zemin carpisma ihlalleri"):
        # yukaridaki blend-overwrite, diz/ayagi collide_ground()'dan SONRA
        # degistiriyordu, yani kismen IK'dan gelen bir ayak pozisyonu o kare
        # icin hic zemine karsi kontrol edilmeden ekrana ciziliyordu (r_foot
        # olcumde 17 karede zemin altina sizdigi dogrulandi). Ayni kontrolu
        # blend sonrasinda BIR KEZ DAHA calistirmak, o karenin de zemin
        # kurallarina uymasini saglar.
        collide_ground(body, TERRAIN.floor_fn, TERRAIN.friction_fn)

        control_err_log.append(err_accum / 4.0)
        vec = body.points[idx["shoulder"]] - body.points[idx["hip"]]
        torso_angle_log.append(float(np.degrees(np.arctan2(vec[0], -vec[1]))))

        camera_offset = W / 2 - hip_pos[0]
        frame = draw_frame(body, idx, camera_offset, blend, t)
        writer.write(frame)

    writer.release()
    print(f"wrote {out_path}")

    control_err_log = np.array(control_err_log)
    torso_angle_log = np.array(torso_angle_log)
    active_mask_n = int(KNOCKDOWN_T * FPS)
    passive_start_n = int((KNOCKDOWN_T + BLEND_DOWN_DURATION + 1.0) * FPS)
    print(f"mean |aktif-pasif fark| (aktif fazda, t<{KNOCKDOWN_T}s): {control_err_log[:active_mask_n].mean():.2f}px")
    print(f"mean |aktif-pasif fark| (pasif fazda, gecisten 1s sonra): {control_err_log[passive_start_n:].mean():.2f}px")
    print(f"govde acisi (derece) -- t=0: {torso_angle_log[0]:.1f}, t={KNOCKDOWN_T-0.1}s: {torso_angle_log[active_mask_n-3]:.1f}, "
          f"t={DURATION_S-0.1}s: {torso_angle_log[-1]:.1f}")
    final_y = {k: float(body.points[idx[k]][1]) for k in ("hip", "shoulder", "head", "l_foot", "r_foot")}
    print(f"son kareki y konumlari (zemin={GROUND_Y}): {final_y}")


if __name__ == "__main__":
    main()
