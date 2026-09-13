"""
Adim 7 (baslangic): momentum ve esneme (squash & stretch) -- verlet
zincirlerinin hiza/dusmeye tepki olarak uzayip buzusmesi.

Bu, kullanicinin Rain World analizindeki ilk mekanizmayla ilgili: "organik"
hissin buyuk kismi karakterin katilarmis gibi degil, esnek/kutleli bir
seyler tasiyormus gibi hareket etmesinden geliyor.

Yaklasim -- neden ayri bir "top" demosu:
  Yuruyen iskelet karakterin gövde/bacak çubuklari BILEREK rijit tutuluyor
  (compliance=0) -- cunku squash&stretch'i orada denemek digger denge/IK
  mekanizmalarini (FABRIK, clamp_direction) bozardi ve etkiyi izole etmek
  zorlasirdi (bu projenin "step1: tek degiskeni izole et" ilkesiyle ayni
  gerekce). Onun yerine `physics/verlet.py`'ye eklenen YENI genel mekanizma
  -- `add_stick(..., compliance=...)` -- kendi basina, halka + capraz
  "jant" (spoke) topolojisiyle kurulmus basit bir "yumusak top" ile
  gosteriliyor. Bu top, `VerletSystem`'in ayni cekirdek kod yolunu
  (ayni `_satisfy_sticks`, ayni `collide_ground`) kullaniyor -- yani
  gelecekte karakterin govdesine/uzuvlarina da (ör. bir dususten sonra
  govdenin hafifce sikismasi) ayni compliance parametresiyle uygulanabilir;
  bu, README'de sonraki adim olarak not edildi.

DURUST BIR NOT: tekduze bir yercekimi altinda serbest dususte (hicbir dis
kuvvet/surtunme farki yokken) bir katiligi-esnek halkanin noktalari HEPSI
AYNI ivmeyi aldigi icin sekil bozulmaz (butun noktalar ayni hizla duser,
aralarindaki mesafe degismez) -- yani "havadayken gercekten uzama" burada
FIZIKSEL OLARAK gerceklesmiyor (bu, dogru: uniform yercekimi hicbir
deformasyon yaratmaz). Gercek deformasyon SADECE zeminle CARPISMA aninda
ortaya cikiyor: topun alt noktalari `collide_ground()` ile aniden
durdurulurken ust/yan noktalar bir-iki kare daha yercekimiyle asagi/yana
hareket etmeye devam ediyor -- bu ANLIK ASIMETRI halkayi yassiltiyor
(squash), ardindan esnek jant/halka cubuklari (`compliance>0`) topu
birkac kare icinde kismen geri yuvarlaklastiriyor (stretch/rebound), ve
son olarak sonlanan (damped) bir salinimla dinleniyor. Bu, sayisal olarak
asagida dogrulandi (bkz. calistirma ciktisi + modul sonu ozet).

Cikti: outputs/step7_squash_stretch.mp4
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from physics.verlet import VerletSystem
from physics.collision import collide_ground
from physics.environment import Terrain

W, H = 640, 400
FPS = 30
DURATION_S = 6
N_FRAMES = FPS * DURATION_S

GROUND_Y = 330.0
N_PTS = 10
BALL_RADIUS = 32.0
CENTER0 = np.array([180.0, 120.0])

# Ayar dersi (bkz. modul sonu ozet + tune script): halka cubuklari orta
# derecede esnek (RING_COMPLIANCE), capraz "jant" cubuklari daha rijit
# (SPOKE_COMPLIANCE) tutulup topun tamamen collapse olmasi engellendi.
RING_COMPLIANCE = 0.55
SPOKE_COMPLIANCE = 0.15
BALL_GRAVITY = np.array([0.0, 0.35])
BALL_FRICTION = 0.02
GROUND_CONTACT_FRICTION = 0.15
INITIAL_DRIFT_PX = 1.2  # hafif yatay surukleme -- topun sadece dikey duser gibi durmamasi icin


def build_ball() -> tuple[VerletSystem, list[int]]:
    sys_ = VerletSystem.empty()
    sys_.gravity = BALL_GRAVITY.copy()
    sys_.friction = BALL_FRICTION
    idx = []
    for k in range(N_PTS):
        ang = 2 * np.pi * k / N_PTS
        pos = CENTER0 + BALL_RADIUS * np.array([np.cos(ang), np.sin(ang)])
        idx.append(sys_.add_point(pos))
    for k in range(N_PTS):
        sys_.add_stick(idx[k], idx[(k + 1) % N_PTS], compliance=RING_COMPLIANCE)
    half = N_PTS // 2
    for k in range(half):
        sys_.add_stick(idx[k], idx[k + half], compliance=SPOKE_COMPLIANCE)
    sys_.prev_points[:, 0] -= INITIAL_DRIFT_PX
    return sys_, idx


TERRAIN = Terrain(ground_y=GROUND_Y, default_friction=GROUND_CONTACT_FRICTION)


def bbox(points: np.ndarray) -> tuple[float, float, float, float]:
    xs, ys = points[:, 0], points[:, 1]
    return float(xs.min()), float(xs.max()), float(ys.min()), float(ys.max())


def draw_frame(points: np.ndarray, ratio: float, frame_no: int) -> np.ndarray:
    frame = np.full((H, W, 3), 22, dtype=np.uint8)
    cv2.line(frame, (0, int(GROUND_Y)), (W, int(GROUND_Y)), (85, 85, 85), 2)

    poly = points.astype(np.int32).reshape(-1, 1, 2)
    cv2.fillPoly(frame, [poly], (60, 130, 220))
    cv2.polylines(frame, [poly], True, (140, 200, 255), 2, cv2.LINE_AA)
    centroid = points.mean(axis=0)
    cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, (255, 255, 255), -1, cv2.LINE_AA)

    x0, x1, y0, y1 = bbox(points)
    cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (90, 90, 90), 1)

    cv2.putText(frame, f"squash orani (h/w): {ratio:.3f}  (1.0 = dinlenme sekli)",
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 210, 210), 1, cv2.LINE_AA)
    bar_x = 12
    bar_w = int(np.clip(ratio, 0.5, 1.3) * 150)
    cv2.rectangle(frame, (bar_x, 40), (bar_x + 150, 52), (70, 70, 70), 1)
    cv2.rectangle(frame, (bar_x, 40), (bar_x + bar_w, 52), (140, 200, 255), -1)

    cv2.putText(frame, "Adim 7 (baslangic): sicramali yumusak top -- carpma anindaki gercek squash/stretch",
                (12, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    body, idx = build_ball()

    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "outputs", "step7_squash_stretch.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    ratios = []
    for f in range(N_FRAMES):
        body.step(dt=1.0)
        collide_ground(body, TERRAIN.floor_fn, TERRAIN.friction_fn)

        x0, x1, y0, y1 = bbox(body.points)
        w = x1 - x0
        h = y1 - y0
        ratio = h / w if w > 1e-6 else 1.0
        ratios.append(ratio)

        frame = draw_frame(body.points, ratio, f)
        writer.write(frame)

    writer.release()
    print(f"wrote {out_path}")

    ratios = np.array(ratios)
    print(f"rest (frame 0) ratio: {ratios[0]:.3f}")
    print(f"min ratio (max squash): {ratios.min():.3f} @ frame {int(ratios.argmin())}")
    print(f"max ratio (max stretch/rebound): {ratios.max():.3f} @ frame {int(ratios.argmax())}")
    print(f"last 20 frame ratio mean/std: {ratios[-20:].mean():.3f} / {ratios[-20:].std():.5f}")


if __name__ == "__main__":
    main()
