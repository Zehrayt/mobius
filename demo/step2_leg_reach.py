"""
Step 2: FABRIK ile hedefe uzanma (foot-planting yürüyüş döngüsü).

Step 1'deki verlet zinciri (bkz. demo/step1_verlet_chain.py) tek başına bir
"kuyruğun" doğal sarkmasını gösteriyordu. Bu adımda iki sistemi birleştiriyoruz:

  - Kalça (hip) noktası, bir "driver" (sabit hızda yürüyen kinematik nokta)
    ile yumuşak bir Verlet çubuğuyla bağlı -- yön/hız değiştiğinde hafifçe
    sallanıp toparlanıyor (step 1'deki organik his).
  - Her bacak, physics.fabrik.FabrikChain2D ile bir "ayak hedefi"ne uzanıyor.
    Ayaklar bir state machine ile "yerde sabit dur" (stance) / "ileri taşı"
    (swing) fazları arasında geçiş yapıyor -- klasik IK foot-planting yürüyüş
    tekniği. Hiçbir kare elle çizilmedi / keyframe kullanılmadı; tüm hareket
    prosedürel olarak hesaplanıyor.

Çıktı: outputs/step2_leg_reach.mp4
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from physics.verlet import VerletSystem
from physics.gait import FootPlantingLeg

W, H = 640, 360
FPS = 30
DURATION_S = 6
N_FRAMES = FPS * DURATION_S

HIP_Y = 190.0
GROUND_Y = 300.0
HEAD_OFFSET = np.array([0.0, -70.0])

LEG_SEGMENT_LEN = 65.0
STRIDE_RELEASE = 18.0   # ayak kalçanın bu kadar gerisinde kalınca kalkar
STRIDE_AHEAD = 28.0     # yeni basış noktası kalçanın bu kadar ilerisine konur
SWING_DURATION_FRAMES = 10
LIFT_HEIGHT = 22.0


def driver_speed(t: float) -> float:
    """Sabit yürüyüş hızı + t=2.2-3.2s arası kısa bir hızlanma (momentum/
    ivme farkını göstermek için)."""
    base = 55.0
    if 2.2 <= t <= 3.2:
        return base * 1.9
    return base


def make_leg(hip_pos: np.ndarray, initial_planted_offset: float = 0.0) -> FootPlantingLeg:
    """initial_planted_offset: iki bacağı zıt fazda başlatmak için ayak
    basış noktasını kalçaya göre öteler (ör. sol bacak 0, sağ bacak
    -yarım adım -- böylece kalça ilerledikçe biri diğerinden önce ayağını
    kaldırır ve gerçek bir alternatif yürüyüş elde edilir)."""
    return FootPlantingLeg(
        hip_pos,
        segment_lengths=[LEG_SEGMENT_LEN, LEG_SEGMENT_LEN],
        ground_y=GROUND_Y,
        stride_release=STRIDE_RELEASE,
        stride_ahead=STRIDE_AHEAD,
        swing_duration_frames=SWING_DURATION_FRAMES,
        lift_height=LIFT_HEIGHT,
        initial_planted_offset=initial_planted_offset,
    )


def build_hip_system() -> tuple[VerletSystem, int, int]:
    sys_ = VerletSystem.empty()
    driver_idx = sys_.add_point([0.0, HIP_Y], pinned=True)
    hip_idx = sys_.add_point([0.0, HIP_Y], pinned=False)
    sys_.add_stick(driver_idx, hip_idx, length=3.0)  # gevşek bağ -> hafif sallanma
    return sys_, driver_idx, hip_idx


def draw_frame(hip_world: np.ndarray, head_world: np.ndarray,
               legs_feet_world: list[np.ndarray], legs: list[FootPlantingLeg],
               camera_offset: float) -> np.ndarray:
    frame = np.full((H, W, 3), 24, dtype=np.uint8)

    # zemin: dünya koordinatındaki tik işaretleri, kamerayla birlikte kayar
    for wx in range(-2000, 2000, 40):
        sx = int(wx + camera_offset)
        if -10 <= sx <= W + 10:
            cv2.line(frame, (sx, int(GROUND_Y)), (sx, int(GROUND_Y) + 6), (70, 70, 70), 1)
    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (90, 90, 90), 2)

    def to_screen(p):
        return (int(p[0] + camera_offset), int(p[1]))

    hip_s = to_screen(hip_world)
    head_s = to_screen(head_world)
    cv2.line(frame, hip_s, head_s, (90, 220, 90), 3, cv2.LINE_AA)
    cv2.circle(frame, head_s, 14, (90, 220, 90), 2, cv2.LINE_AA)

    leg_colors = [(90, 220, 90), (230, 160, 60)]  # sol: yeşil, sağ: mavi-turkuaz (BGR)
    for leg, color in zip(legs, leg_colors):
        pts = leg.chain.points
        for i in range(len(pts) - 1):
            p1 = to_screen(pts[i])
            p2 = to_screen(pts[i + 1])
            cv2.line(frame, p1, p2, color, 3, cv2.LINE_AA)
        for p in pts:
            cv2.circle(frame, to_screen(p), 4, (255, 255, 255), -1, cv2.LINE_AA)

    cv2.putText(frame, "FABRIK foot-planting + verlet hip bounce (keyframe yok)",
                (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    verlet_sys, driver_idx, hip_idx = build_hip_system()
    half_stride = (STRIDE_RELEASE + STRIDE_AHEAD) / 2.0
    left_leg = make_leg(verlet_sys.points[hip_idx].copy(), initial_planted_offset=0.0)
    right_leg = make_leg(verlet_sys.points[hip_idx].copy(), initial_planted_offset=-half_stride)

    driver_x = 0.0
    dt = 1.0 / FPS

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step2_leg_reach.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    for f in range(N_FRAMES):
        t = f * dt
        driver_x += driver_speed(t) * dt
        verlet_sys.set_pinned_position(driver_idx, [driver_x, HIP_Y])
        verlet_sys.step(dt=1.0)

        hip_pos = verlet_sys.points[hip_idx]
        head_pos = hip_pos + HEAD_OFFSET

        left_foot = left_leg.update(hip_pos)
        right_foot = right_leg.update(hip_pos)

        camera_offset = W / 2 - hip_pos[0]
        frame = draw_frame(hip_pos, head_pos, [left_foot, right_foot],
                            [left_leg, right_leg], camera_offset)
        writer.write(frame)

    writer.release()
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
