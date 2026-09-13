"""
Adım 8 + 11 (başlangıç, birleşik): zemin çarpışması + bölgesel ("dinamik")
sürtünme -- kullanıcının önerdiği sıradaki adım ("ardından çarpışma ve
sürtünme katsayılarına geçebilirsin").

Neden birlikte? Çünkü aynı yeni mekanizmayı (`VerletSystem.collide_ground`)
kullanıyorlar: çarpışma olmadan "bölgesel sürtünme"nin gösterecek bir
teması yok, sürtünme olmadan da çarpışma "hep aynı şekilde durur" gibi tek
düze kalırdı. İkisi birlikte anlamlı bir tek demo oluşturuyor.

Sahne: Adım 10'daki pelerin artık çok daha UZUN (12 segment, toplam ~180px
erişim) -- omuzdan asılı haldeyken serbestçe sarkarsa zemin seviyesini
(GROUND_Y) geçer. Rüzgar bu demoda KAPALI (mekanizmayı izole etmek için,
step1'in "tek değişkeni izole et" yaklaşımının aynısı) -- yani pelerin
sadece kendi ağırlığıyla aşağı sarkıp karakterin arkasında YERDE SÜRÜKLENİR.
`VerletSystem.collide_ground()` OLMADAN bu, pelerin noktalarının zeminin
altına (negatif "derinliğe") sızması demek olurdu -- fizik motorunda hiçbir
şey bunu önlemiyordu çünkü bacaklar zemini sadece kendi FABRIK hedefleri
üzerinden "biliyor", serbest verlet noktaları zemin kavramından tamamen
habersiz.

Zeminde iki bölge var: normal zemin (yüksek temas sürtünmesi) ve buzlu bir
şerit (ekranda açık mavi/beyaz, neredeyse sıfır temas sürtünmesi).

DÜRÜST BİR BULGU: pelerinin kendisi bu sürtünme farkını neredeyse HİÇ
göstermiyor -- sayısal olarak ölçüldü (kalça-uç mesafesi buz/normalde
~91.4px'e karşı ~91.8px, yani ayırt edilemez). Neden: pelerin çubuklarla
(stick) birbirine sıkı sıkıya bağlı KATI bir zincir; `_satisfy_sticks()`
her karede noktayı komşusuna göre tamamen GEOMETRİK olarak yeniden
konumlandırıyor ve bunu yaparken sürtünmenin azalttığı hıza hiç bakmıyor
-- yani sürtünme, çubuğun çekişi karşısında neredeyse anlamsız kalıyor
(bkz. `physics/verlet.py`'deki `collide_ground()` dokstring'i). Bu yüzden
sürtünmeyi (Adım 11) AYRI ve dürüst bir şekilde göstermek için sahneye
HİÇBİR çubuğa bağlı olmayan iki serbest "taş" (puck) eklendi: karakter
durduğu anda (t=3s) ikisi de aynı hızla fırlatılıyor, biri normal zeminde
biri buzda. Çubuğa bağlı olmadıkları için sürtünme onları GERÇEKTEN
farklı şekilde yavaşlatıyor -- normal zeminde ~13px kayıp anında duruyor,
buzda ~117px kayıp çok daha yavaş duruyor (sayısal olarak doğrulandı,
~9 kat fark).

Özetle bu tek demo iki farklı dersi ayrı ayrı ve dürüstçe gösteriyor:
  1. Çarpışma (Adım 8): uzun, katı bir zincir (pelerin) zemine gömülmeden
     üzerinde duruyor/sürükleniyor.
  2. Sürtünme (Adım 11): sadece GERÇEKTEN serbest bir parçacıkta (taş)
     anlamlı bir fark yaratıyor -- katı bir zincirde neredeyse etkisiz.

Çıktı: outputs/step8_collision_friction.mp4
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from physics.verlet import VerletSystem, clamp_direction
from physics.gait import FootPlantingLeg

W, H = 640, 400
FPS = 30
DURATION_S = 15
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


STOP_START_T = 3.0
STOP_END_T = 5.0     # ~2s durma -- pelerinin serbestçe yere sarkıp yerleşmesi
                     # için yeterli (sayısal olarak doğrulandı, bkz. modül notu)
RESUME_SPEED = 15.0  # bkz. asağıdaki "kite-lift eşiği" notu


def driver_speed(t: float) -> float:
    """Karakter önce normal hızda (70px/s) yürüyor, sonra ~2 saniye
    DURUYOR (bu sırada pelerin hiçbir yatay çekme kuvveti olmadan sadece
    kendi ağırlığıyla aşağı sarkıp `collide_ground()` sayesinde yerde
    "yerleşiyor" -- bkz. modül dokstring'i), sonra tekrar yürüyüşe devam
    edip yerleşmiş pelerini önce normal zeminde, sonra buzlu şeritte
    SÜRÜKLÜYOR.

    ÖNEMLİ sayısal bulgu -- "kite-lift eşiği": durmadan SONRA eski 70px/s
    hızıyla devam edilirse pelerin neredeyse anında yerden kalkıp havada
    (adeta bir uçurtma gibi) sürükleniyor -- towing'in verdiği momentum,
    yerçekiminin sarkıtma etkisinden daha güçlü çıkıyor (480 karelik bir
    denemede sadece 1 kare temas). Hız kademeli olarak düşürülüp test
    edildi: 70/40/25px/s'de hâlâ ~1 kare temas, 15px/s'de 480 karenin
    222'sinde temas, 8px/s'de 281'inde. Bu yüzden pelerin gerçekten
    "sürüklensin" (uçmasın) diye durmadan sonra yürüyüş YAVAŞLATILDI
    (`RESUME_SPEED`) -- gerçek hayattaki ağır bir kuyruk/etek/pelerinin
    de hızlı yürüyüşte havaya kalkıp yavaş yürüyüşte yerde sürünmesine
    benzer bir eşik davranışı."""
    if STOP_START_T <= t < STOP_END_T:
        return 0.0
    if t < STOP_START_T:
        return 70.0
    return RESUME_SPEED


# -- Pelerin: Adım 10'dakinden çok daha uzun (12 segment) -- omuzdan
# serbest sarktığında zemin seviyesini geçer, bu yüzden collide_ground()
# olmadan yere "gömülür".
CAPE_SEGMENTS = 16
CAPE_SEG_LEN = 15.0
CAPE_WIND_SCALES = [0.3 + 0.15 * i for i in range(CAPE_SEGMENTS)]  # ilerideki rüzgar denemeleri için hazır, bu demoda rüzgar=0

# -- Zemin sürtünme bölgeleri (dünya koordinatında x aralığı) -----------
ICE_X0, ICE_X1 = 140.0, 320.0
NORMAL_CONTACT_FRICTION = 0.35   # yüksek -- temas eden nokta hızla durur
ICE_CONTACT_FRICTION = 0.02      # neredeyse sıfır -- kayıp gitmeye devam eder

# -- İki serbest "taş" (puck) -- HİÇBİR çubuğa bağlı değiller, sürtünme
# farkını (pelerinin aksine) gerçekten gösterebiliyorlar.
PUCK_LAUNCH_T = STOP_START_T  # karakter durduğu anda ikisi de fırlatılıyor
PUCK_LAUNCH_VX = 8.0          # px/kare, ikisi de AYNI hızla başlıyor
PUCK_NORMAL_X0 = 60.0         # normal zeminde -- hızla durması beklenir
PUCK_ICE_X0 = 150.0           # buzlu şeritte -- çok daha uzağa kaymalı


def floor_fn(x: float) -> float:
    return GROUND_Y  # düz zemin -- bu demoda yükseklik değil, SÜRTÜNME değişiyor


def friction_fn(x: float) -> float:
    if ICE_X0 <= x <= ICE_X1:
        return ICE_CONTACT_FRICTION
    return NORMAL_CONTACT_FRICTION


def build_body() -> tuple[VerletSystem, dict[str, int]]:
    sys_ = VerletSystem.empty()
    sys_.gravity = TUNED_GRAVITY.copy()
    sys_.friction = TUNED_FRICTION
    idx: dict[str, int] = {}

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

    # Uzun pelerin: omzun biraz arkasına PINNED bir kökten sarkıyor.
    shoulder_pos = sys_.points[idx["shoulder"]]
    cape_anchor_pos = shoulder_pos + [-4.0, -6.0]
    idx["cape_anchor"] = sys_.add_point(cape_anchor_pos, pinned=True)

    prev = idx["cape_anchor"]
    prev_pos = cape_anchor_pos
    idx["cape_points"] = []
    for i in range(CAPE_SEGMENTS):
        seg_pos = prev_pos + [-2.0, CAPE_SEG_LEN]
        p = sys_.add_point(seg_pos, wind_scale=CAPE_WIND_SCALES[i])
        sys_.add_stick(prev, p, length=CAPE_SEG_LEN)
        idx["cape_points"].append(p)
        prev, prev_pos = p, seg_pos

    # İki serbest taş (puck) -- hiçbir stick'e bağlı değiller. Başlangıçta
    # ekranın çok dışında/pasif duruyorlar; `main()` içinde PUCK_LAUNCH_T
    # anında gerçek konum + hıza "ışınlanıyorlar" (aynı `set_pinned_position`
    # tekniğinin pinned OLMAYAN bir noktaya elle uygulanmış hali).
    idx["puck_normal"] = sys_.add_point([-5000.0, GROUND_Y])
    idx["puck_ice"] = sys_.add_point([-5000.0, GROUND_Y])

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


def draw_frame(body: VerletSystem, idx: dict[str, int], legs: list[FootPlantingLeg],
               camera_offset: float) -> np.ndarray:
    frame = np.full((H, W, 3), 22, dtype=np.uint8)

    # Zemin -- buzlu şeridi ayrı bir renkle boyayıp etiketliyoruz ki
    # kamerada hangi bölgede olduğumuz görsel olarak da belli olsun.
    for wx in range(-4000, 4000, 40):
        sx = int(wx + camera_offset)
        if -10 <= sx <= W + 10:
            in_ice = ICE_X0 <= wx <= ICE_X1
            color = (200, 220, 230) if in_ice else (60, 60, 60)
            cv2.line(frame, (sx, int(GROUND_Y)), (sx, int(GROUND_Y) + 6), color, 1)

    ice_start_screen = int(ICE_X0 + camera_offset)
    ice_end_screen = int(ICE_X1 + camera_offset)
    if ice_end_screen > 0 and ice_start_screen < W:
        cv2.rectangle(frame, (max(ice_start_screen, 0), int(GROUND_Y)),
                      (min(ice_end_screen, W), int(GROUND_Y) + 3), (210, 235, 245), -1)
        label_x = max(ice_start_screen, 10)
        if label_x < W - 20:
            cv2.putText(frame, "BUZ (dusuk temas surtunmesi)", (label_x, int(GROUND_Y) + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 235, 245), 1, cv2.LINE_AA)

    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (85, 85, 85), 2)

    def to_screen(p):
        return (int(p[0] + camera_offset), int(p[1]))

    body_color = (90, 220, 90)
    leg_colors = [(90, 220, 90), (230, 160, 60)]
    arm_colors = {"l": (150, 230, 150), "r": (60, 170, 230)}
    cape_color = (210, 110, 220)

    cape_pts = [body.points[idx["cape_anchor"]]] + [body.points[p] for p in idx["cape_points"]]
    for i in range(len(cape_pts) - 1):
        cv2.line(frame, to_screen(cape_pts[i]), to_screen(cape_pts[i + 1]), cape_color, 5, cv2.LINE_AA)
    for p in cape_pts:
        cv2.circle(frame, to_screen(p), 3, (240, 200, 245), -1, cv2.LINE_AA)

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

    # İki serbest taş -- sürtünme farkının GERÇEKTEN görüldüğü yer burası
    # (pelerinin aksine, bkz. modül dokstring'i).
    puck_n = to_screen(body.points[idx["puck_normal"]])
    puck_i = to_screen(body.points[idx["puck_ice"]])
    cv2.circle(frame, puck_n, 7, (140, 140, 240), -1, cv2.LINE_AA)
    cv2.circle(frame, puck_i, 7, (245, 200, 90), -1, cv2.LINE_AA)
    if 0 <= puck_n[0] <= W:
        cv2.putText(frame, "normal tas", (puck_n[0] - 30, puck_n[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 140, 240), 1, cv2.LINE_AA)
    if 0 <= puck_i[0] <= W:
        cv2.putText(frame, "buz tasi", (puck_i[0] - 25, puck_i[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (245, 200, 90), 1, cv2.LINE_AA)

    cv2.putText(frame, "Adim 8+11 (baslangic): zemin carpismasi (pelerin) + bolgesel surtunme (tas)",
                (12, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    body, idx = build_body()
    half_stride = (STRIDE_RELEASE + STRIDE_AHEAD) / 2.0
    left_leg = make_leg(body.points[idx["hip"]].copy(), initial_planted_offset=0.0)
    right_leg = make_leg(body.points[idx["hip"]].copy(), initial_planted_offset=-half_stride)

    driver_x = 0.0
    dt = 1.0 / FPS
    pucks_launched = False

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step8_collision_friction.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    for f in range(N_FRAMES):
        t = f * dt
        driver_x += driver_speed(t) * dt
        body.set_pinned_position(idx["driver"], [driver_x, HIP_Y])

        if not pucks_launched and t >= PUCK_LAUNCH_T:
            # Pinned OLMAYAN bir noktaya "elle" konum + hız vermenin yolu:
            # points'i hedef konuma, prev_points'i de bir kare öncesindeki
            # (istenen hızı ima eden) konuma eşitlemek -- tıpkı
            # `set_pinned_position`'ın pinned noktalar için yaptığı gibi,
            # ama burada nokta serbest kalmaya devam ediyor (fiziğe tabi).
            for key, x0 in (("puck_normal", PUCK_NORMAL_X0), ("puck_ice", PUCK_ICE_X0)):
                pidx = idx[key]
                body.points[pidx] = [x0, GROUND_Y]
                body.prev_points[pidx] = [x0 - PUCK_LAUNCH_VX, GROUND_Y]
            pucks_launched = True

        shoulder_pos = body.points[idx["shoulder"]]
        left_push = leg_swing_push(right_leg) * ARM_COUNTER_SWING_PX
        right_push = -leg_swing_push(left_leg) * ARM_COUNTER_SWING_PX
        body.set_pinned_position(idx["l_anchor"], shoulder_pos + [-SHOULDER_WIDTH + left_push, 0.0])
        body.set_pinned_position(idx["r_anchor"], shoulder_pos + [SHOULDER_WIDTH + right_push, 0.0])
        body.set_pinned_position(idx["cape_anchor"], shoulder_pos + [-4.0, -6.0])

        # Bu demoda rüzgar KAPALI -- amaç sadece çarpışma/sürtünmeyi izole
        # etmek (bkz. modül dokstring'i). `wind_scale` alanları yine de
        # ileride rüzgarla birleştirilebilsin diye set edilmiş durumda.
        body.wind = np.zeros(2)

        body.step(dt=1.0)
        clamp_direction(body.points, body.prev_points, idx["hip"], idx["shoulder"], UP, TORSO_MAX_LEAN_DEG)
        torso_dir = body.points[idx["shoulder"]] - body.points[idx["hip"]]
        clamp_direction(body.points, body.prev_points, idx["shoulder"], idx["head"], torso_dir, NECK_MAX_TILT_DEG)

        # YENİ: pelerin zeminle çarpışıyor + bölgeye göre farklı temas
        # sürtünmesi yaşıyor. clamp_direction gibi step()'ten SONRA çağrılır.
        body.collide_ground(floor_fn, friction_fn)

        hip_pos = body.points[idx["hip"]]
        left_leg.update(hip_pos)
        right_leg.update(hip_pos)

        camera_offset = W / 2 - hip_pos[0]
        frame = draw_frame(body, idx, [left_leg, right_leg], camera_offset)
        writer.write(frame)

    writer.release()
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
