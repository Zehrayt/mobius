"""
Step 5: Parallax arka plan katmanları.

Step 4'teki karakter/fizik aynen korunuyor; bu adımda eklenen tek şey
sahneye derinlik katmanları (Z-index) eklemek. Her katmanın bir "depth"
(derinlik) çarpanı var:
  - depth < 1.0  -> karakterden daha YAVAŞ kayar (uzaktaki dağlar, oba)
  - depth == 1.0 -> karakterle aynı hızda kayar (zemin/karakter düzlemi)
  - depth > 1.0  -> karakterden daha HIZLI kayar (öndeki çalılar)

Parallax formülü: screen_x = world_x * depth + (W/2 - hip_x * depth).
Yani "kamera ofseti" de katmanın derinliğine göre ölçekleniyor -- bu,
uzak nesnelerin ekranda neredeyse sabit kalması, yakın nesnelerin ise
hızla kayıp gitmesi hissini verir (gerçek parallax'ın matematiği budur).

Şekiller (dağ/oba/çalı) elle çizilmiş görsel varlıklar (asset) YERİNE,
basit geometrik ilkellerle prosedürel olarak üretiliyor ve `tile_width`
periyoduyla sonsuza tekrarlanıyor (kamera hangi yöne giderse gitsin
katman hiç bitmiyor). Gerçek bir üründe bu şekillerin yerine sanat
ekibinin ürettiği .png sprite'lar konur -- mimari (derinlik/kayma
matematiği) aynı kalır.

Çıktı: outputs/step5_parallax.mp4
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

# Yürüyüşü Adım 4'ten daha uzun ve daha hızlı tuttuk ki parallax kayması
# birkaç tam periyot boyunca gözle görülsün.
def driver_speed(t: float) -> float:
    return 70.0


# -- Parallax katman tanımları -------------------------------------------
# depth: kayma çarpanı (bkz. modül dokstring'i). tile: periyot (px, dünya
# koordinatında). y_base: şeklin zeminden ne kadar yukarıda oturduğu.
PARALLAX_LAYERS = [
    dict(name="mountains", depth=0.15, tile=260, y_base=GROUND_Y - 120, color=(70, 55, 40), shape="mountain", size=110),
    dict(name="camp", depth=0.45, tile=160, y_base=GROUND_Y, color=(50, 75, 95), shape="tent", size=46),
]
FOREGROUND_LAYER = dict(name="bushes", depth=1.7, tile=95, y_base=GROUND_Y + 8, color=(25, 65, 25), shape="bush", size=30)


def draw_parallax_layer(frame: np.ndarray, hip_x: float, layer: dict) -> None:
    depth = layer["depth"]
    tile = layer["tile"]
    y_base = layer["y_base"]
    color = layer["color"]
    size = layer["size"]
    shape = layer["shape"]

    layer_offset = W / 2 - hip_x * depth
    # Ekranda görünebilecek dünya-x aralığını bul (bir tile payı taşırarak).
    x_min_world = (-tile - layer_offset) / depth
    x_max_world = (W + tile - layer_offset) / depth
    i_start = int(np.floor(x_min_world / tile))
    i_end = int(np.ceil(x_max_world / tile))

    for i in range(i_start, i_end + 1):
        # Her tile'a deterministik ama çeşitli bir yükseklik/ofset ver
        # (gerçek rastgelelik yerine i'ye bağlı bir sin/cos karışımı --
        # her render'da aynı sahneyi üretir, "seed" saklamaya gerek yok).
        jitter_y = np.sin(i * 12.9898) * 0.5 + 0.5  # 0..1 deterministik
        jitter_size = 0.8 + 0.4 * (np.cos(i * 7.233) * 0.5 + 0.5)

        world_x = i * tile
        sx = int(world_x * depth + layer_offset)
        if sx < -size * 2 or sx > W + size * 2:
            continue
        s = size * jitter_size

        if shape == "mountain":
            peak_y = int(y_base - s * 0.9 * jitter_y)
            pts = np.array([[sx - int(s), int(y_base)], [sx, peak_y], [sx + int(s), int(y_base)]], dtype=np.int32)
            cv2.fillPoly(frame, [pts], color, lineType=cv2.LINE_AA)
        elif shape == "tent":
            base_y = int(y_base)
            top_y = int(y_base - s)
            pts = np.array([[sx - int(s * 0.6), base_y], [sx, top_y], [sx + int(s * 0.6), base_y]], dtype=np.int32)
            cv2.fillPoly(frame, [pts], color, lineType=cv2.LINE_AA)
            cv2.line(frame, (sx, top_y), (sx, base_y), (color[0] // 2, color[1] // 2, color[2] // 2), 1, cv2.LINE_AA)
        elif shape == "bush":
            base_y = int(y_base)
            r = max(3, int(s * 0.5 * jitter_size))
            cv2.circle(frame, (sx - r // 2, base_y - r // 2), r, color, -1, cv2.LINE_AA)
            cv2.circle(frame, (sx + r // 2, base_y - r // 2), r, color, -1, cv2.LINE_AA)
            cv2.circle(frame, (sx, base_y - r), r, color, -1, cv2.LINE_AA)


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
               hip_x: float, camera_offset: float) -> np.ndarray:
    # Gökyüzü: basit dikey gradyan (statik -- en uzak "katman", hiç kaymaz).
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    top = np.array([35, 24, 18], dtype=np.float32)      # BGR, koyu lacivert-kahve
    bottom = np.array([90, 70, 45], dtype=np.float32)    # ufuk çizgisine yakın ton
    for y in range(0, int(GROUND_Y)):
        t = y / GROUND_Y
        frame[y, :] = (top * (1 - t) + bottom * t).astype(np.uint8)
    frame[int(GROUND_Y):, :] = (30, 26, 22)

    for layer in PARALLAX_LAYERS:
        draw_parallax_layer(frame, hip_x, layer)

    for wx in range(-4000, 4000, 40):
        sx = int(wx + camera_offset)
        if -10 <= sx <= W + 10:
            cv2.line(frame, (sx, int(GROUND_Y)), (sx, int(GROUND_Y) + 6), (55, 50, 45), 1)
    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (75, 68, 60), 2)

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

    # En ön katman (çalılar) karakterin ÜSTÜNE çiziliyor -- depth>1 olduğu
    # için kameradan daha hızlı kayıp karakterin önünden geçme hissi verir.
    draw_parallax_layer(frame, hip_x, FOREGROUND_LAYER)

    cv2.putText(frame, "Adim 5: parallax arka plan katmanlari (derinlige gore farkli kayma hizi)",
                (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    body, idx = build_body()
    half_stride = (STRIDE_RELEASE + STRIDE_AHEAD) / 2.0
    left_leg = make_leg(body.points[idx["hip"]].copy(), initial_planted_offset=0.0)
    right_leg = make_leg(body.points[idx["hip"]].copy(), initial_planted_offset=-half_stride)

    driver_x = 0.0
    dt = 1.0 / FPS

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step5_parallax.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    for f in range(N_FRAMES):
        t = f * dt
        driver_x += driver_speed(t) * dt
        body.set_pinned_position(idx["driver"], [driver_x, HIP_Y])

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
        frame = draw_frame(body, idx, [left_leg, right_leg], float(hip_pos[0]), camera_offset)
        writer.write(frame)

    writer.release()
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
