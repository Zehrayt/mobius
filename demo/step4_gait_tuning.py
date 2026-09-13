"""
Step 4: Yürüyüş döngüsü ince ayarı -- karşı-bacak (contralateral) kol
sallanması + sürtünme/yerçekimi sabitlerinin gözle kalibrasyonu.

Step 3'teki sorun: iki kol da aynı "shoulder" noktasından sarktığı ve
fiziği neredeyse özdeş olduğu için görsel olarak üst üste biniyorlardı.

Gerçek insan yürüyüşünde koldaki sallanma, karşı bacağın adımına tepki
olarak gövdenin hafif ters-dönmesinden (counter-rotation) doğar: sağ
bacak ileri sallanırken sol omuz hafifçe ileri, sağ omuz hafifçe geri
gider. Bunu KLON bir sin() formülüyle "dikte etmek" yerine (bu tam da
projenin başında reddettiğimiz yaklaşım), her omuz-tutamağını gerçek
bacak durumuna (FootPlantingLeg.state / swing_t) bağlı bir "sürüş"
(driver) noktası yapıyoruz: bacak salınım (swing) fazındayken karşı
omuz tutamağı öne doğru kayar, bu da kola pasif bir itki verir ve kol
oradan itibaren yine tamamen verlet fiziğiyle (yerçekimi + sürtünme +
momentum) serbestçe tepki verir. Yani "ne zaman" sinyali gerçek bir
olaydan (bacağın kalkıp inmesi) geliyor, "nasıl hareket edeceği" hâlâ
fizikten geliyor -- salt periyodik bir saat fonksiyonu değil.

Ayrıca physics.verlet.GRAVITY / FRICTION varsayılanları yerine bu
karakter için elle kalibre edilmiş (görsel olarak denenip seçilmiş)
gravity/friction değerleri kullanılıyor -- bkz. TUNED_GRAVITY / TUNED_FRICTION.

Çıktı: outputs/step4_gait_tuning.mp4
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
DURATION_S = 6
N_FRAMES = FPS * DURATION_S

HIP_Y = 170.0
GROUND_Y = 330.0
TORSO_LEN = 55.0
HEAD_STICK_LEN = 30.0
HEAD_RADIUS = 15
ARM_SEG_LEN = 32.0
SHOULDER_WIDTH = 10.0

LEG_SEGMENT_LEN = 75.0
STRIDE_RELEASE = 18.0
STRIDE_AHEAD = 28.0
SWING_DURATION_FRAMES = 10
LIFT_HEIGHT = 22.0

KNEE_LIMITS = (8.0, 150.0)
KNEE_BEND_SIGN = -1.0

UP = np.array([0.0, -1.0])
TORSO_MAX_LEAN_DEG = 12.0
NECK_MAX_TILT_DEG = 18.0

# -- İnce ayar: elle kalibre edilmiş fizik sabitleri --------------------
# Step 1-3'te physics/verlet.py'nin modül-varsayılanları (gravity=0.05,
# friction=0.02) kullanılmıştı -- bunlar genel amaçlı, hiçbir karaktere
# özel ayarlanmamış sabitlerdi. Burada tam bu karakter/ölçek için
# görsel olarak denenip seçilmiş değerler kullanılıyor:
#   - gravity biraz artırıldı (0.05 -> 0.065): kollar çok "tüy gibi"
#     hafif kalıp gerçekçiliği bozuyordu, biraz daha "ağırlık" hissi.
#   - friction artırıldı (0.02 -> 0.045): varsayılan değerde kollar
#     hız değişiminde gereğinden fazla salınıp geç sönümleniyordu
#     ("sarhoş" görünüm); daha yüksek sürtünme birkaç salınımda
#     yerine oturmasını sağlıyor.
TUNED_GRAVITY = np.array([0.0, 0.065])
TUNED_FRICTION = 0.045

# Karşı-bacak kol itkisi: bir bacak swing fazındayken KARŞI omzun
# tutamağı bu kadar piksel öne kayar (bacağın swing ilerlemesiyle
# orantılı, sin(pi*t) eğrisiyle -- ayağın kalkıp inme eğrisiyle aynı
# şekil, keyfi bir saat fonksiyonu değil).
ARM_COUNTER_SWING_PX = 16.0


def driver_speed(t: float) -> float:
    base = 55.0
    if 2.2 <= t <= 3.2:
        return base * 1.9
    return base


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

    # Her kol artık ortak "shoulder" noktasından değil, kendi PINNED omuz
    # tutamağından (l_anchor / r_anchor) sarkıyor. Bu tutamaklar her karede
    # dışarıdan (shoulder pozisyonu + karşı-bacak itkisi ile) güncellenecek.
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
    """Bacak swing fazındaysa 0..1..0 arası sin(pi*t) eğrisiyle bir itki
    değeri döndürür (ayağın kalkıp inme eğrisiyle birebir aynı zamanlama --
    keyfi bir periyodik sin(zaman) DEĞİL, doğrudan bacağın kendi swing_t
    ilerlemesinden okunuyor)."""
    if leg.state != "swing":
        return 0.0
    return float(np.sin(np.pi * min(leg.swing_t, 1.0)))


def draw_frame(body: VerletSystem, idx: dict[str, int], legs: list[FootPlantingLeg],
               camera_offset: float) -> np.ndarray:
    frame = np.full((H, W, 3), 24, dtype=np.uint8)

    for wx in range(-3000, 3000, 40):
        sx = int(wx + camera_offset)
        if -10 <= sx <= W + 10:
            cv2.line(frame, (sx, int(GROUND_Y)), (sx, int(GROUND_Y) + 6), (70, 70, 70), 1)
    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (90, 90, 90), 2)

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

    cv2.putText(frame, "Adim 4: karsi-bacak kol sallanmasi + kalibre edilmis fizik",
                (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    body, idx = build_body()
    half_stride = (STRIDE_RELEASE + STRIDE_AHEAD) / 2.0
    left_leg = make_leg(body.points[idx["hip"]].copy(), initial_planted_offset=0.0)
    right_leg = make_leg(body.points[idx["hip"]].copy(), initial_planted_offset=-half_stride)

    driver_x = 0.0
    dt = 1.0 / FPS

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step4_gait_tuning.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    for f in range(N_FRAMES):
        t = f * dt
        driver_x += driver_speed(t) * dt
        body.set_pinned_position(idx["driver"], [driver_x, HIP_Y])

        # Karşı-bacak kol itkisi: sağ bacak swing'teyken sol omuz öne,
        # sol bacak swing'teyken sağ omuz öne kayar.
        shoulder_pos = body.points[idx["shoulder"]]
        left_push = leg_swing_push(right_leg) * ARM_COUNTER_SWING_PX
        right_push = -leg_swing_push(left_leg) * ARM_COUNTER_SWING_PX
        body.set_pinned_position(idx["l_anchor"], shoulder_pos + [-SHOULDER_WIDTH + left_push, 0.0])
        body.set_pinned_position(idx["r_anchor"], shoulder_pos + [SHOULDER_WIDTH + right_push, 0.0])

        body.step(dt=1.0)
        clamp_direction(body.points, body.prev_points, idx["hip"], idx["shoulder"], UP, TORSO_MAX_LEAN_DEG)
        torso_dir = body.points[idx["shoulder"]] - body.points[idx["hip"]]
        clamp_direction(body.points, body.prev_points, idx["shoulder"], idx["head"], torso_dir, NECK_MAX_TILT_DEG)

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
