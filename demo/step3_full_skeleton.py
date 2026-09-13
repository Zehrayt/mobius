"""
Step 3: Tam iskelet -- gövde + baş + 2 kol + 2 bacak, diz açı sınırıyla.

Mimari:
  Tek bir physics.verlet.VerletSystem grafiği; şunları içerir:
    driver (pinned, sabit yürüyüş hızıyla ilerler)
      -- hip (dinamik, gevşek çubuk -> step 1/2'deki gibi hafif sallanma)
      -- shoulder (dinamik, hip'e bağlı -> gövdenin kendi bounce/lean'i)
         -- head (dinamik, shoulder'a bağlı -> baş da hafifçe sallanıyor)
         -- sol_dirsek -- sol_el   (pasif sarkan verlet zinciri, step 1'deki
                                     "kuyruk" tekniğinin aynısı)
         -- sag_dirsek -- sag_el
  Bacaklar bu grafiğin DIŞINDA: her biri physics.gait.FootPlantingLeg (FABRIK
  + foot-planting state machine), hip'e her karede yeniden bağlanıyor.
  Diz eklemine physics.fabrik.FabrikChain2D.clamp_joint_angles() ile açı
  sınırı konuyor -- diz olması gereken yönün tersine bükülemiyor.

Yani: gövde/kollar/baş = "pasif fizik" (verlet), bacaklar = "hedef güdümlü"
(FABRIK). İkisi de aynı VerletSystem'in ürettiği kalça konumunu paylaşıyor.

Çıktı: outputs/step3_full_skeleton.mp4
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

LEG_SEGMENT_LEN = 92.0
STRIDE_RELEASE = 18.0
STRIDE_AHEAD = 28.0
SWING_DURATION_FRAMES = 10
LIFT_HEIGHT = 22.0

# Diz sadece "geriye" bükülsün (insan dizi gibi) -- yön işareti (bend_sign)
# görsel olarak doğrulanıp bu şekilde bırakıldı (bkz. README "Ayarlanabilir
# sabitler" notu). Açı 0 derece = düz bacak, 180'e yaklaştıkça tam bükülü.
KNEE_LIMITS = (8.0, 150.0)
KNEE_BEND_SIGN = -1.0

# Gövde (hip->shoulder) ve boyun (shoulder->head) serbest bırakılırsa
# "çift sarkaç" gibi kaotik döner (bkz. physics/verlet.py clamp_direction
# dokstring'i). Gövdeyi sabit bir global "yukarı" yönüne göre, boynu da
# (artık stabilize edilmiş) gövde yönüne göre küçük bir aralıkla sınırlayıp
# yarı-rijit bir gövde/boyun hissi veriyoruz.
UP = np.array([0.0, -1.0])
TORSO_MAX_LEAN_DEG = 12.0
NECK_MAX_TILT_DEG = 18.0


def driver_speed(t: float) -> float:
    base = 55.0
    if 2.2 <= t <= 3.2:
        return base * 1.9
    return base


def build_body() -> tuple[VerletSystem, dict[str, int]]:
    sys_ = VerletSystem.empty()
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
        elbow = sys_.add_point(shoulder_pos + [x_off * 10, ARM_SEG_LEN])
        sys_.add_stick(idx["shoulder"], elbow, length=ARM_SEG_LEN)
        hand = sys_.add_point(shoulder_pos + [x_off * 4, ARM_SEG_LEN * 2])
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

    # gövde (hip-shoulder), boyun (shoulder-head)
    cv2.line(frame, to_screen(body.points[idx["hip"]]), to_screen(body.points[idx["shoulder"]]), body_color, 4, cv2.LINE_AA)
    cv2.line(frame, to_screen(body.points[idx["shoulder"]]), to_screen(body.points[idx["head"]]), body_color, 3, cv2.LINE_AA)
    cv2.circle(frame, to_screen(body.points[idx["head"]]), HEAD_RADIUS, body_color, 2, cv2.LINE_AA)

    # kollar (pasif sarkan verlet zinciri -- step 1'deki "kuyruk" tekniği)
    arm_colors = {"l": (150, 230, 150), "r": (90, 220, 90)}
    for side in ("l", "r"):
        s = to_screen(body.points[idx["shoulder"]])
        e = to_screen(body.points[idx[f"{side}_elbow"]])
        h = to_screen(body.points[idx[f"{side}_hand"]])
        c = arm_colors[side]
        cv2.line(frame, s, e, c, 3, cv2.LINE_AA)
        cv2.line(frame, e, h, c, 3, cv2.LINE_AA)
        cv2.circle(frame, h, 4, (255, 255, 255), -1, cv2.LINE_AA)

    # bacaklar (FABRIK)
    for leg, color in zip(legs, leg_colors):
        pts = leg.chain.points
        for i in range(len(pts) - 1):
            cv2.line(frame, to_screen(pts[i]), to_screen(pts[i + 1]), color, 4, cv2.LINE_AA)
        for p in pts:
            cv2.circle(frame, to_screen(p), 5, (255, 255, 255), -1, cv2.LINE_AA)

    cv2.putText(frame, "Tam iskelet: verlet govde/kol/bas + FABRIK bacak (diz acisi sinirli)",
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
                             "outputs", "step3_full_skeleton.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    for f in range(N_FRAMES):
        t = f * dt
        driver_x += driver_speed(t) * dt
        body.set_pinned_position(idx["driver"], [driver_x, HIP_Y])
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
