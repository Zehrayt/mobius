"""
Adım 10 (ön çalışma): İkincil fizik nesneleri -- rüzgarda/hava direncinde
salınan bir pelerin (cape), hedefe bağlı OLMAYAN saf bir verlet zinciriyle.

Bu, roadmap'teki "ileri seviye organik fizik" maddelerinden 10 numaralıyı
(secondary animation) ve 7 numaralının (squash & stretch) da üzerine
kurulacağı altyapıyı (physics/verlet.py'deki yeni `wind`/`wind_scale`)
tanıtıyor. Kullanıcının önerisi doğrultusunda ilk somut adım bu oldu:
"önce verlet.py'ye hava direncini ekleyip rüzgarda salınan bir kuyruk/
kumaş simülasyonu elde et".

Mimari -- ÖNEMLİ fark:
  Kollar/bacaklar gibi HEDEFE ulaşmaya çalışan (FABRIK/IK) ya da
  `clamp_direction()` ile yarı-rijit tutulan parçaların aksine, pelerin
  HİÇBİR HEDEFE bağlanmaz ve hiçbir açı/yön kısıtlaması almaz -- sadece:
    - ana karakterin hareketinden (pinned "cape_anchor" omuza bağlı),
    - `VerletSystem.gravity`'den,
    - ve şimdi yeni eklenen `VerletSystem.wind` / `wind_scale`'den
  etkilenen saf bir verlet zinciridir. Bu yüzden "secondary animation"
  olarak adlandırılıyor: ana hareketi pasif olarak, gecikmeli ve
  kendi ataletiyle takip eder.

Rüzgar tasarımı:
  `wind_x(t)` tek bir periyodik sin(t) DEĞİL -- birbirine asal olmayan
  birkaç frekansın toplamı (bkz. fonksiyonun kendisi), böylece "gust"lar
  (esinti dalgalanmaları) düzenli/robotik bir metronom gibi değil, gerçek
  rüzgar gibi düzensiz hissettiriyor. Bu, projenin "karakterin KENDİ
  hareketini bir saat fonksiyonuyla dikte etme" kuralına aykırı değil --
  rüzgar karakterin kendi kinematiği değil, dışarıdan gelen bir ÇEVRE
  kuvveti (tıpkı `driver_speed()`'teki senaryo hızlanması gibi).

  İlk ~3 saniye rüzgar KAPALI (pelerin sadece kendi ağırlığı ve karakterin
  hareketiyle sallanıyor) -- sonra ~1.5 saniyede rüzgar açılıp esintili
  şekilde devam ediyor. Bu, "rüzgarsız" ve "rüzgarlı" durumu aynı klipte
  yan yana karşılaştırabilmek için (step1'deki organik-vs-robotik
  karşılaştırma tekniğinin aynısı).

  `wind_scale` pelerinin kökünden (anchor'a yakın, az tepki) ucuna doğru
  (uzak, çok tepki) artıyor -- gerçek bir kumaşın ucunun köküne göre çok
  daha fazla savrulmasının basit bir yaklaşıklaması (kaldıraç etkisi).

Çıktı: outputs/step10_secondary_wind.mp4
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from physics.verlet import VerletSystem, clamp_direction
from physics.gait import FootPlantingLeg
from physics.environment import GustWind

W, H = 640, 400
FPS = 30
DURATION_S = 10
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


def driver_speed(t: float) -> float:
    return 70.0


# -- Pelerin (cape) -- saf ikincil fizik, hedefsiz, kısıtlamasız --------
CAPE_SEGMENTS = 8
CAPE_SEG_LEN = 15.0
# Kökten uca doğru artan rüzgar duyarlılığı (kaldıraç yaklaşıklaması).
CAPE_WIND_SCALES = [0.4, 0.6, 0.85, 1.1, 1.4, 1.7, 2.05, 2.4]

WIND_START_T = 3.0     # bu saniyeye kadar rüzgar kapalı
WIND_RAMP_T = 1.5      # bu kadar sürede 0 -> tam güce çıkar

# Refactor notu: bu, daha önce burada hardcode bir `wind_x(t)` fonksiyonuydu
# -- artık `physics/environment.py`'deki genel `GustWind` yardımcısı
# kullanılıyor (aynı formül, birebir aynı sayısal davranış, bkz. o modülün
# dokstring'i). Tek periyodik sin(t) değil -- birbirine asal olmayan üç
# frekansın toplamı, düzensiz "gust" (esinti) hissi versin diye. Karakter
# +x yönünde yürüdüğü için negatif (ters yönden esen) rüzgar, pelerinin
# karakterin ARKASINA doğru savrulmasını sağlıyor. Büyüklük
# `VerletSystem.gravity` (~0.065) ile aynı mertebede tutuldu (~0.12 taban +
# ~0.16 gust) -- ilk denemede taban çok daha büyüktü (0.55) ve pelerin
# gerçekte savrulmuyor, sadece dümdüz gerilip kalıyordu (rüzgar tüm iç
# fiziği bastırıyordu); bu değer gerçek bir "flutter" hissi için sayısal
# olarak doğrulandı.
WIND = GustWind(
    base=-0.12,
    components=[(0.085, 0.18, 0.7), (0.045, 0.47, 2.1), (0.025, 0.83, 0.0)],
    start_t=WIND_START_T,
    ramp_t=WIND_RAMP_T,
)


def wind_x(t: float) -> float:
    return WIND.value(t)


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

    # Pelerin: omzun biraz arkasına (ve yukarısına) PINNED bir kök
    # noktasından sarkıyor. Kendisi hiçbir hedefe uzanmaz, hiçbir açı
    # sınırı almaz -- saf ikincil fizik.
    shoulder_pos = sys_.points[idx["shoulder"]]
    cape_anchor_pos = shoulder_pos + [-4.0, -6.0]
    idx["cape_anchor"] = sys_.add_point(cape_anchor_pos, pinned=True)

    prev = idx["cape_anchor"]
    prev_pos = cape_anchor_pos
    idx["cape_points"] = []
    for i in range(CAPE_SEGMENTS):
        # Başlangıç dinlenme pozu: aşağı ve hafifçe geriye (-x) doğru.
        seg_pos = prev_pos + [-3.0, CAPE_SEG_LEN]
        p = sys_.add_point(seg_pos, wind_scale=CAPE_WIND_SCALES[i])
        sys_.add_stick(prev, p, length=CAPE_SEG_LEN)
        idx["cape_points"].append(p)
        prev, prev_pos = p, seg_pos

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


def draw_wind_indicator(frame: np.ndarray, wind_val: float) -> None:
    """Sol üstte rüzgarın anlık yönünü/gücünü gösteren basit bir ok."""
    cx, cy = 90, 60
    cv2.circle(frame, (cx, cy), 34, (60, 60, 60), 1, cv2.LINE_AA)
    length = float(np.clip(wind_val * 40.0, -32.0, 32.0))
    end = (int(cx + length), cy)
    color = (120, 200, 255) if abs(wind_val) > 1e-6 else (90, 90, 90)
    cv2.arrowedLine(frame, (cx, cy), end if end != (cx, cy) else (cx - 1, cy),
                     color, 2, cv2.LINE_AA, tipLength=0.35)
    label = f"ruzgar: {wind_val:+.2f}" if abs(wind_val) > 1e-3 else "ruzgar: yok"
    cv2.putText(frame, label, (cx - 55, cy + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (200, 200, 200), 1, cv2.LINE_AA)


def draw_frame(body: VerletSystem, idx: dict[str, int], legs: list[FootPlantingLeg],
               camera_offset: float, wind_val: float) -> np.ndarray:
    frame = np.full((H, W, 3), 22, dtype=np.uint8)

    for wx in range(-4000, 4000, 40):
        sx = int(wx + camera_offset)
        if -10 <= sx <= W + 10:
            cv2.line(frame, (sx, int(GROUND_Y)), (sx, int(GROUND_Y) + 6), (60, 60, 60), 1)
    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (85, 85, 85), 2)

    def to_screen(p):
        return (int(p[0] + camera_offset), int(p[1]))

    body_color = (90, 220, 90)
    leg_colors = [(90, 220, 90), (230, 160, 60)]
    arm_colors = {"l": (150, 230, 150), "r": (60, 170, 230)}
    cape_color = (210, 110, 220)

    # Pelerin -- gövdenin ARKASINDA çizilsin diye önce o çiziliyor.
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

    draw_wind_indicator(frame, wind_val)
    cv2.putText(frame, "Adim 10 (on calisma): pelerin = ikincil fizik (IK YOK, sadece verlet+ruzgar)",
                (12, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    body, idx = build_body()
    half_stride = (STRIDE_RELEASE + STRIDE_AHEAD) / 2.0
    left_leg = make_leg(body.points[idx["hip"]].copy(), initial_planted_offset=0.0)
    right_leg = make_leg(body.points[idx["hip"]].copy(), initial_planted_offset=-half_stride)

    driver_x = 0.0
    dt = 1.0 / FPS

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step10_secondary_wind.mp4")
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
        body.set_pinned_position(idx["cape_anchor"], shoulder_pos + [-4.0, -6.0])

        w = wind_x(t)
        body.wind = np.array([w, 0.0])

        body.step(dt=1.0)
        clamp_direction(body.points, body.prev_points, idx["hip"], idx["shoulder"], UP, TORSO_MAX_LEAN_DEG)
        torso_dir = body.points[idx["shoulder"]] - body.points[idx["hip"]]
        clamp_direction(body.points, body.prev_points, idx["shoulder"], idx["head"], torso_dir, NECK_MAX_TILT_DEG)
        # NOT: pelerine KASITLI olarak clamp_direction() uygulanmıyor --
        # ikincil fizik nesnelerinin bütün amacı serbest/kısıtlamasız kalıp
        # ana hareketi kendi ataletiyle takip etmesidir (bkz. modül dokstring'i).

        hip_pos = body.points[idx["hip"]]
        left_leg.update(hip_pos)
        right_leg.update(hip_pos)

        camera_offset = W / 2 - hip_pos[0]
        frame = draw_frame(body, idx, [left_leg, right_leg], camera_offset, w)
        writer.write(frame)

    writer.release()
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
