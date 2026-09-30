"""
Adim 17 -- Bilge'nin 16 parcali derisi GERCEK fizik iskeletine giydirildi
(Faz A kinematik kontak-faz sensoru ile birlikte).

    python3 demo/step17_bilge_physics_skin.py            # normal video
    python3 demo/step17_bilge_physics_skin.py --debug    # iskelet + faz serit
    python3 demo/step17_bilge_physics_skin.py --preview-only

KAYNAK / SAHIPLIK: tum eklem konumlari `demo/step14_active_biped.
ActiveBipedSim`'den (step14'un fizigi, refactor sonrasi bit-bit ayni) gelir:
  * kalca, omuz, bas  -> serbest Verlet noktalari (`VerletSystem`)
  * diz, ayak         -> `ActiveFootPlantingLeg` (capture-point stride
                         karari + Bezier swing + FABRIK), stance'ta ayak
                         `planted`'e (anchor ile ayni nokta) kilitli
  * ayak egimi        -> Faz A sensoru (`contact_phase` + `leg_angle_deg`)
Arkadasin `bilge_walk_validation.py`'sindeki kinematik kalca egrisi
(`pelvis_at`) burada HIC kullanilmiyor; o dosya render REFERANSI olarak
yerinde duruyor.

GORSEL EŞLEME (fizige yazilmayan, sadece cizim-uzayinda yapilan donusumler --
hepsi durustce listelenmistir):
  1. Govde/boyun YONU fizikten, cizim UZUNLUGU rig'den: step14'un govde
     cubugu 55px (yetiskin-cubuk oranlari), Bilge'nin gövde parcasi ~86px
     bekliyor. `chest = hip + yon(omuz-kalca) * 86`. COM/denge hesaplari
     GERCEK 55px'lik noktalari kullanmaya devam eder.
  2. Kollar step14'te YOK. Kol salinimi, iki ayagin FIZIKSEL x-farkindan
     (karsi-bacak) turetilen bir FABRIK hedefidir -- bagimsiz bir zaman
     cizelgesi degil ama fiziksel bir kol da degil (Prosedurel Kol Salinimi /
     Angular Momentum Cancellation hala yol haritasinda, baslanmadi).
  3. Ayakkabi egimi Faz A fazina gore: heel_strike -> TOPUK etrafinda
     parmak ucu yukari; toe_off -> PARMAK UCU etrafinda topuk yukari;
     flat_foot -> 0. Pivot noktasi dunyada sabit kalir (yere gomulme/kayma
     yok). Egim acisi = |bacak acisi| - olu bant, ust sinirli.
  4. Egim gorsel bilegi tasidigi icin diz, AYNI kemik uzunluklariyla
     kalca->gorsel bilek arasinda analitik 2-kemik IK ile yeniden cozulur
     (fizik dizinin bukulme TARAFI korunur). Duz basista ve swing'de
     sonuc fizik diziyle ayni noktadir; sapma raporda olculur.
"""
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo import step14_active_biped as s14
from demo.bilge_walk_skinned import BilgeSkin, ORDER, overlap_pixels
from physics.active_gait import (CONTACT_DEADBAND_DEG, classify_contact_phase, PHASE_SWING, PHASE_HEEL_STRIKE,
                                 PHASE_FLAT_FOOT, PHASE_TOE_OFF)
from physics.fabrik import FabrikChain2D
from scene.export import render_video
from scene.skinning import rigid_matrix, transform_point, composite_cutout

WIDTH, HEIGHT, FPS = 1280, 720, s14.FPS
DURATION_S = s14.DURATION_S
SCREEN_GROUND_Y = 565.0
SCREEN_ANCHOR_X = 560.0          # kamera kalcayi bu ekran x'inde tutar
CAMERA_SMOOTH = 0.12             # sadece kamera takibi (fizige etkisi yok)
RENDER_TORSO_LEN = 86.0          # Bilge gövde parcasinin beklenen uzunlugu
RENDER_HEAD_OFFSET = 43.0
SHOULDER_HALF_WIDTH = 14.0
UPPER_ARM, FOREARM = 54.0, 50.0
HEEL_STRIKE_MAX_PITCH_DEG = 20.0
TOE_OFF_MAX_PITCH_DEG = 35.0
PITCH_GAIN = 1.0
# Adim 18: 60/-15 yuvarlanma geometrisi artik step14'un VARSAYILANI. Eski
# "topuk yuruyusu" (6/12) karsilastirma icin --legacy-heel-gait ile.
LEGACY_GAIT = dict(support_margin=s14.LEGACY_SUPPORT_MARGIN, swing_lead_margin=s14.LEGACY_SWING_LEAD_MARGIN)
LANDING_BLEND_START = 0.6        # swing_t bu degerden sonra inis egimine yumusak gecis
THIGH_LEN = s14.LEG_SEGMENT_LEN
SIDES = (("left", "left_leg"), ("right", "right_leg"))
PHASE_COLORS = {PHASE_SWING: (190, 190, 190), PHASE_HEEL_STRIKE: (40, 140, 245),
                PHASE_FLAT_FOOT: (70, 170, 60), PHASE_TOE_OFF: (200, 110, 40)}
PHASE_SHORT = {PHASE_SWING: "SW", PHASE_HEEL_STRIKE: "HS", PHASE_FLAT_FOOT: "FF", PHASE_TOE_OFF: "TO"}


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.array([0.0, -1.0])


def simulate(n_frames: int | None = None, **sim_kwargs) -> tuple[list[dict], s14.ActiveBipedSim]:
    """step14 fizigini kosar; her karenin DUNYA-uzayi anlik goruntusunu toplar."""
    sim = s14.ActiveBipedSim(**sim_kwargs)
    n = n_frames or s14.N_FRAMES
    idx = sim.idx
    frames = []
    slip_frames = set()
    for f in range(n):
        hip_vx = sim.step()
        pts = sim.body.points
        snap = dict(frame=f, time=f / FPS, hip_vx=float(hip_vx), fell=sim.fell,
                    in_danger=sim.last_in_danger,
                    hip=pts[idx["hip"]].copy(), shoulder=pts[idx["shoulder"]].copy(),
                    head=pts[idx["head"]].copy(), legs={})
        for side, attr in SIDES:
            leg = getattr(sim, attr)
            snap["legs"][side] = dict(
                state=leg.state, phase=leg.contact_phase, angle=float(leg.leg_angle_deg),
                chain=leg.chain.points.copy(), planted=leg.planted.copy(),
                swing_t=float(leg.swing_t) if leg.state == "swing" else None,
                swing_target=leg.swing_target.copy(), foot_target=leg.foot_target.copy(),
                slipping=bool(leg.is_slipping))
        frames.append(snap)
        if sim.nan:
            break
    for f, *_ in sim.slip_events:
        slip_frames.add(f)
    for snap in frames:
        snap["slip_frame"] = snap["frame"] in slip_frames
    return frames, sim


def stance_pitch_deg(phase: str, angle: float) -> float:
    """Faz A -> ayakkabi egimi (derece). Negatif: parmak ucu yukari (topuk
    pivotu), pozitif: topuk yukari (parmak ucu pivotu)."""
    over = max(0.0, abs(angle) - CONTACT_DEADBAND_DEG) * PITCH_GAIN
    if phase == PHASE_HEEL_STRIKE:
        return -min(HEEL_STRIKE_MAX_PITCH_DEG, over)
    if phase == PHASE_TOE_OFF:
        return min(TOE_OFF_MAX_PITCH_DEG, over)
    return 0.0


def two_bone_knee(hip, ankle, l1, l2, side_ref):
    """Analitik 2-kemik IK; `side_ref` (fizik dizi) hangi tarafa bukulecegini belirler."""
    d_vec = ankle - hip
    d = float(np.linalg.norm(d_vec))
    reach_clamped = False
    if d >= l1 + l2 - 1e-6:
        reach_clamped = True
        return hip + _unit(d_vec) * l1, reach_clamped
    d = max(d, abs(l1 - l2) + 1e-6)
    a = (l1 * l1 - l2 * l2 + d * d) / (2 * d)
    h = float(np.sqrt(max(0.0, l1 * l1 - a * a)))
    u = _unit(d_vec)
    perp = np.array([-u[1], u[0]])
    base = hip + u * a
    c1, c2 = base + perp * h, base - perp * h
    cross = lambda p: (d_vec[0] * (p - hip)[1] - d_vec[1] * (p - hip)[0])
    ref = np.sign(cross(side_ref)) or 1.0
    return (c1 if np.sign(cross(c1)) == ref else c2), reach_clamped


class PhysicsBilgeRig:
    """Dunya-uzayi fizik anlik goruntusunu Bilge parca matrislerine cevirir."""

    def __init__(self, skin: BilgeSkin | None = None):
        self.skin = skin or BilgeSkin()
        self.arms = {}
        self.arm_swing = 0.0
        self.cam_x = None
        self.shoe_local = {}
        for side in ("left", "right"):
            part = self.skin.parts[f"{side}_shoe"]
            hull = part.hull.astype(float)
            bottom = hull[hull[:, 1] >= hull[:, 1].max() - 2.0]
            self.shoe_local[side] = dict(heel=bottom[bottom[:, 0].argmin()].copy(),
                                         toe=bottom[bottom[:, 0].argmax()].copy())
        # gorsel incik = fizik incik boyu - duz basistaki bilek yuksekligi
        self.shin_visual = {}
        for side in ("left", "right"):
            m = self._flat_shoe(side, np.array([0.0, 0.0]))
            ankle_h = -transform_point(m, self.skin.anchor(f"{side}_shoe"))[1]
            self.shin_visual[side] = s14.LEG_SEGMENT_LEN - ankle_h

    def to_screen(self, p):
        return np.array([p[0] - self.cam_x + SCREEN_ANCHOR_X, p[1] - s14.GROUND_Y + SCREEN_GROUND_Y])

    def update_camera(self, hip_x):
        if self.cam_x is None:
            self.cam_x = float(hip_x)
        self.cam_x += (float(hip_x) - self.cam_x) * CAMERA_SMOOTH

    def _flat_shoe(self, side, foot):
        name = f"{side}_shoe"
        m = rigid_matrix(self.skin.anchor(name), foot, self.skin.scale(name), 0.0)
        hull = self.skin.parts[name].hull @ m[:, :2].T + m[:, 2]
        m[1, 2] += foot[1] - .55 - hull[:, 1].max()
        return m

    @staticmethod
    def _rotate_about(m, pivot, angle_rad):
        co, si = np.cos(angle_rad), np.sin(angle_rad)
        r = np.array([[co, -si], [si, co]])
        out = m.copy()
        out[:, :2] = r @ m[:, :2]
        out[:, 2] = r @ (m[:, 2] - pivot) + pivot
        return out

    def pose(self, snap: dict) -> dict:
        """Ekran-uzayi eklemler + ayakkabi matrisleri + olcum bilgisi."""
        self.update_camera(snap["hip"][0])
        S = self.to_screen
        hip = S(snap["hip"])
        chest = hip + _unit(snap["shoulder"] - snap["hip"]) * RENDER_TORSO_LEN
        head = chest + _unit(snap["head"] - snap["shoulder"]) * RENDER_HEAD_OFFSET
        p = {"pelvis": hip, "chest": chest, "head": head}
        shoes, info = {}, {}
        for side, _ in SIDES:
            leg = snap["legs"][side]
            chain = [S(q) for q in leg["chain"]]
            # ayak = gait'in KENDI hedefi (stance: planted, swing: Bezier noktasi).
            # FABRIK zincir ucu, hedef bacak boyunu astiginda (asiri uzanma)
            # hedeften sapabiliyor -- ayakkabi her zaman fiziksel temas/yorunge
            # noktasina konur, sapma raporda olculur.
            knee_phys = chain[1]
            foot = S(leg["planted"]) if leg["state"] == "stance" else S(leg["foot_target"])
            flat = self._flat_shoe(side, foot)
            local = self.shoe_local[side]
            pitch = 0.0
            if leg["state"] == "stance":
                pitch = stance_pitch_deg(leg["phase"], leg["angle"])
                if pitch < 0:
                    pivot = transform_point(flat, local["heel"])
                elif pitch > 0:
                    pivot = transform_point(flat, local["toe"])
                else:
                    pivot = foot
                m = self._rotate_about(flat, pivot, np.radians(pitch))
            else:
                # swing: salinimin ilk yarisi arkadasin ucu-kaldirmasi; son
                # %40'inda, ayagin INECEGI noktaya (swing_target) gore Faz A'nin
                # ongordugu inis egimine yumusakca yaklasir -- boylece inis
                # karesinde egim sicramaz (gorsel interpolasyon, fizige yazilmaz).
                st = min(1.0, leg["swing_t"])
                pitch = -9.0 * np.sin(np.pi * st) ** 2
                target = leg["swing_target"]
                pred = float(np.degrees(np.arctan2(snap["hip"][0] - target[0], -(snap["hip"][1] - target[1]))))
                land = stance_pitch_deg(classify_contact_phase(pred), pred)
                w = float(np.clip((st - LANDING_BLEND_START) / (1.0 - LANDING_BLEND_START), 0.0, 1.0))
                w = w * w * (3 - 2 * w)
                pitch = (1 - w) * pitch + w * land
                m = rigid_matrix(self.skin.anchor(f"{side}_shoe"), foot,
                                 self.skin.scale(f"{side}_shoe"), np.radians(pitch))
                pivot = foot
            # taban temasi: donmus kabugun EN ALT noktasi zemine/ayak y'sine
            # oturtulur (yuvarlanan temas -- gomulme yok)
            hull = self.skin.parts[f"{side}_shoe"].hull @ m[:, :2].T + m[:, 2]
            m[1, 2] += foot[1] - .55 - hull[:, 1].max()
            ankle = transform_point(m, self.skin.anchor(f"{side}_shoe"))
            if leg["state"] == "stance" and pitch < 0:
                pivot = transform_point(m, local["heel"])
            elif leg["state"] == "stance" and pitch > 0:
                pivot = transform_point(m, local["toe"])
            # diz: kalca -> gorsel bilek, sabit kemik boylari, ILERI bukulme
            forward_ref = hip + (ankle - hip) * 0.5 + np.array([1.0, 0.0])
            knee, clamped = two_bone_knee(hip, ankle, THIGH_LEN, self.shin_visual[side], forward_ref)
            p[f"{side}_hip"], p[f"{side}_knee"], p[f"{side}_foot"] = hip, knee, foot
            p[f"{side}_ankle"] = ankle
            shoes[side] = m
            shin_stretch = float(np.linalg.norm(ankle - knee)) - self.shin_visual[side]
            info[side] = dict(pitch=pitch, pivot=pivot, knee_dev=float(np.linalg.norm(knee - knee_phys)),
                              shin_stretch=shin_stretch, chain_end_error=float(np.linalg.norm(chain[2] - foot)),
                              reach_clamped=clamped, knee_phys=knee_phys,
                              heel=transform_point(m, local["heel"]), toe=transform_point(m, local["toe"]))
        # kollar: karsi-bacak x-farkindan (fiziksel ayak konumlari) turetilen FABRIK hedefi
        separation = p["right_foot"][0] - p["left_foot"][0]
        desired = float(np.clip(0.62 * separation, -40, 40))
        self.arm_swing += 0.35 * (desired - self.arm_swing)
        for side, sign in (("left", -1), ("right", 1)):
            shoulder = chest + [sign * SHOULDER_HALF_WIDTH, 0.0]
            if side not in self.arms:
                arm = FabrikChain2D(shoulder.copy(), [UPPER_ARM, FOREARM], 1e-9)
                arm.points[1] = shoulder + [25.0, UPPER_ARM * 0.8]
                arm.points[2] = shoulder + [0.0, 93.0]
                self.arms[side] = arm
            arm = self.arms[side]
            hand_x = -sign * self.arm_swing
            arm.set_base(shoulder.copy())
            arm.solve(shoulder + [hand_x, 94.0 - abs(hand_x) * 0.1], max_iterations=256)
            p[f"{side}_shoulder"], p[f"{side}_elbow"], p[f"{side}_hand"] = arm.points.copy()
        return dict(points=p, shoes=shoes, info=info, snap=snap)

    def matrices(self, pose: dict) -> dict:
        sk, p = self.skin, pose["points"]
        mats = {}
        for side in ("right", "left"):
            mats[f"{side}_shoe"] = pose["shoes"][side]
            mats[f"{side}_shin"] = sk.bind_bone(f"{side}_shin", p[f"{side}_knee"], p[f"{side}_ankle"])
            mats[f"{side}_thigh"] = sk.bind_bone(f"{side}_thigh", p[f"{side}_hip"], p[f"{side}_knee"])
            mats[f"{side}_upper_arm"] = sk.bind_bone(f"{side}_upper_arm", p[f"{side}_shoulder"], p[f"{side}_elbow"])
            mats[f"{side}_forearm"] = sk.bind_bone(f"{side}_forearm", p[f"{side}_elbow"], p[f"{side}_hand"])
            d = p[f"{side}_hand"] - p[f"{side}_elbow"]
            mats[f"{side}_hand"] = rigid_matrix(sk.anchor(f"{side}_hand"), p[f"{side}_hand"],
                                                sk.scale(f"{side}_hand"), np.arctan2(-d[0], d[1]))
        axis = p["chest"] - p["pelvis"]
        body_angle = np.arctan2(axis[0], -axis[1])
        mats["torso"] = sk.bind_bone("torso", p["chest"], p["pelvis"] + [0., 6.])
        mats["pelvis"] = rigid_matrix(sk.anchor("pelvis"), p["pelvis"] + [0., 8.], sk.scale("pelvis"), body_angle)
        mats["head"] = rigid_matrix(sk.anchor("head"), p["head"], sk.scale("head"), body_angle)
        braid_root = transform_point(mats["head"], sk.parts["head"].local([95 / 435, 341 / 438]))
        mats["braid"] = sk.bind_bone("braid", braid_root, p["pelvis"] + [-19., -12.])
        return mats

    def layers(self, pose):
        mats = self.matrices(pose)
        return {name: self.skin.parts[name].warp(mats[name]) for name in ORDER}


def background(rig: PhysicsBilgeRig) -> np.ndarray:
    image = np.full((HEIGHT, WIDTH, 3), (237, 244, 249), np.uint8)
    floor = round(SCREEN_GROUND_Y)
    image[floor + 1:] = (226, 235, 241)
    cv2.line(image, (0, floor + 1), (WIDTH - 1, floor + 1), (199, 212, 222), 1, cv2.LINE_AA)
    # dunyaya sabit zemin isaretleri: kamera takip ederken ilerleme gorunsun
    x0 = rig.cam_x - SCREEN_ANCHOR_X
    for wx in range(int(x0 // 60) * 60, int(x0 + WIDTH) + 60, 60):
        sx = int(round(wx - x0))
        cv2.line(image, (sx, floor + 8), (sx, floor + 14), (199, 212, 222), 1)
    # buz bolgesi (step14 ICE_ZONES) -- gorsel ipucu
    for zx0, zx1, _ in s14.ICE_ZONES:
        a, b = int(round(zx0 - x0)), int(round(zx1 - x0))
        if b > 0 and a < WIDTH:
            cv2.rectangle(image, (max(a, 0), floor + 2), (min(b, WIDTH - 1), floor + 6), (243, 226, 196), -1)
    return image


def draw_timeline(image, frames, cursor):
    """Debug: her bacak icin faz seridi (SW gri / HS turuncu / FF yesil / TO mavi)."""
    x0, x1, y = 40, WIDTH - 40, 640
    n = len(frames)
    for row, side in enumerate(("left", "right")):
        yy = y + row * 16
        for i, snap in enumerate(frames):
            xa = x0 + (x1 - x0) * i // n
            xb = x0 + (x1 - x0) * (i + 1) // n
            cv2.rectangle(image, (xa, yy), (max(xa, xb - 1), yy + 11), PHASE_COLORS[snap["legs"][side]["phase"]], -1)
        cv2.putText(image, side[0].upper(), (x0 - 18, yy + 10), cv2.FONT_HERSHEY_SIMPLEX, .4, (60, 70, 75), 1, cv2.LINE_AA)
    cx = x0 + (x1 - x0) * cursor // n
    cv2.line(image, (cx, y - 4), (cx, y + 31), (30, 30, 30), 2)
    lx = x0
    for ph in (PHASE_SWING, PHASE_HEEL_STRIKE, PHASE_FLAT_FOOT, PHASE_TOE_OFF):
        cv2.rectangle(image, (lx, 682), (lx + 12, 692), PHASE_COLORS[ph], -1)
        cv2.putText(image, ph, (lx + 16, 692), cv2.FONT_HERSHEY_SIMPLEX, .42, (60, 70, 75), 1, cv2.LINE_AA)
        lx += 140


def render(pose, rig, frames=None, debug=False) -> np.ndarray:
    snap = pose["snap"]
    image = background(rig)
    for side in ("right", "left"):
        foot = pose["points"][f"{side}_foot"]
        opacity = .14 * np.exp(-max(0, SCREEN_GROUND_Y - foot[1]) / 12.)
        shadow = image.copy()
        cv2.ellipse(shadow, (round(foot[0] + 8), round(SCREEN_GROUND_Y + 2)), (24, 4), 0, 0, 360,
                    (117, 131, 145), -1, cv2.LINE_AA)
        cv2.addWeighted(shadow, opacity, image, 1 - opacity, 0, image)
    for name, (warped, origin) in rig.layers(pose).items():
        composite_cutout(image, warped, origin)
    status = "DUSTU" if snap["fell"] else ("DENGE TEHLIKESI" if snap["in_danger"] else "yuruyor")
    cv2.putText(image, f"Adim 17: Bilge derisi step14 fizigi uzerinde  t={snap['time']:.2f}s  "
                f"hip_vx={snap['hip_vx']:+.2f}  {status}",
                (40, 40), cv2.FONT_HERSHEY_SIMPLEX, .55, (65, 76, 89), 1, cv2.LINE_AA)
    if debug:
        p = pose["points"]
        xy = lambda a: tuple(np.rint(a).astype(int))
        for side, color in (("left", (210, 170, 0)), ("right", (10, 130, 240))):
            inf = pose["info"][side]
            cv2.line(image, xy(p[f"{side}_hip"]), xy(inf["knee_phys"]), color, 1, cv2.LINE_AA)
            cv2.line(image, xy(inf["knee_phys"]), xy(p[f"{side}_foot"]), color, 1, cv2.LINE_AA)
            cv2.circle(image, xy(inf["knee_phys"]), 3, color, -1, cv2.LINE_AA)
            cv2.drawMarker(image, xy(p[f"{side}_ankle"]), (180, 50, 180), cv2.MARKER_CROSS, 8, 1)
            cv2.circle(image, xy(inf["pivot"]), 4, (40, 40, 200), 1, cv2.LINE_AA)
            leg = snap["legs"][side]
            label = f"{side[0].upper()}:{PHASE_SHORT[leg['phase']]} {leg['angle']:+.1f}deg"
            cv2.putText(image, label, (round(p[f'{side}_foot'][0]) - 40, round(SCREEN_GROUND_Y) + 32 + (16 if side == 'right' else 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
        cv2.line(image, xy(p["pelvis"]), xy(p["chest"]), (30, 100, 30), 1)
        if frames is not None:
            draw_timeline(image, frames, snap["frame"])
    return image


def inspect(frames, rig_factory=PhysicsBilgeRig) -> dict:
    """Parca baglantilari, taban temasi, pivot sabitligi, diz sapmasi, faz dizileri."""
    rig = rig_factory()
    min_overlap = {}
    max_pen = 0.0
    max_pivot_drift = 0.0
    max_knee_dev = 0.0
    reach_clamped = 0
    shin_stretch = []
    chain_err = []
    max_pitch_step = 0.0
    touchdown_jumps = []
    prev_pitch, prev_state = {}, {}
    pivot_origin = {}
    for snap in frames:
        pose = rig.pose(snap)
        layers = rig.layers(pose)
        for side in ("left", "right"):
            for a, b in ((f"{side}_upper_arm", f"{side}_forearm"), (f"{side}_forearm", f"{side}_hand"),
                         (f"{side}_thigh", f"{side}_shin"), (f"{side}_shin", f"{side}_shoe"),
                         ("torso", f"{side}_upper_arm"), ("pelvis", f"{side}_thigh")):
                key = a + "/" + b
                min_overlap[key] = min(min_overlap.get(key, 10 ** 9), overlap_pixels(layers[a], layers[b]))
            warped, origin = layers[f"{side}_shoe"]
            ys, _ = np.nonzero(warped[..., 3] > .5)
            max_pen = max(max_pen, float(ys.max() + origin[1]) - SCREEN_GROUND_Y)
            inf = pose["info"][side]
            leg = snap["legs"][side]
            max_knee_dev = max(max_knee_dev, inf["knee_dev"])
            reach_clamped += int(inf["reach_clamped"])
            shin_stretch.append(max(0.0, inf["shin_stretch"]))
            chain_err.append(inf["chain_end_error"])
            if leg["state"] == "stance":
                # pivot: topuk/parmak ucu/ayak -- ekran-uzayindan DUNYAYA geri cevir
                world = inf["pivot"] + [rig.cam_x - SCREEN_ANCHOR_X, 0.0]
                key = (side, leg["phase"])
                if prev_state.get(side) != "stance" or key not in pivot_origin or snap["slip_frame"]:
                    pivot_origin = {k: v for k, v in pivot_origin.items() if k[0] != side}
                    pivot_origin[key] = world.copy()
                max_pivot_drift = max(max_pivot_drift, float(np.linalg.norm(world - pivot_origin[key])))
                if prev_state.get(side) == "swing":
                    touchdown_jumps.append(abs(inf["pitch"] - prev_pitch.get(side, 0.0)))
            else:
                pivot_origin = {k: v for k, v in pivot_origin.items() if k[0] != side}
            if side in prev_pitch and prev_state.get(side) == leg["state"]:
                max_pitch_step = max(max_pitch_step, abs(inf["pitch"] - prev_pitch[side]))
            prev_pitch[side], prev_state[side] = inf["pitch"], leg["state"]
        for a, b in (("torso", "head"), ("torso", "pelvis"), ("head", "braid")):
            key = a + "/" + b
            min_overlap[key] = min(min_overlap.get(key, 10 ** 9), overlap_pixels(layers[a], layers[b]))
    return dict(minimum_pair_overlap_pixels=min_overlap,
                max_visible_sole_penetration_px=round(max_pen, 3),
                max_stance_pivot_drift_px_excluding_physics_slip=round(max_pivot_drift, 4),
                max_visual_knee_deviation_from_physics_knee_px=round(max_knee_dev, 3),
                reach_clamped_leg_frames=reach_clamped,
                shin_stretch_px=dict(p95=round(float(np.percentile(shin_stretch, 95)), 2), max=round(float(max(shin_stretch)), 2)),
                fabrik_end_vs_gait_target_px=dict(median=round(float(np.median(chain_err)), 3),
                                                  frames_over_5px=int(sum(e > 5 for e in chain_err)),
                                                  max=round(float(max(chain_err)), 2),
                                                  note="asiri uzanma (hedef > 184px bacak boyu) -- fizik geometrisi"),
                max_within_state_pitch_step_deg=round(max_pitch_step, 3),
                touchdown_pitch_jumps_deg=dict(count=len(touchdown_jumps),
                                               mean=round(float(np.mean(touchdown_jumps)), 3) if touchdown_jumps else None,
                                               max=round(float(np.max(touchdown_jumps)), 3) if touchdown_jumps else None))


def phase_report(frames) -> dict:
    """Faz A sensorunun dizileri: her stance'in faz dizisi, monotonluk, dagilim."""
    order = {PHASE_HEEL_STRIKE: 0, PHASE_FLAT_FOOT: 1, PHASE_TOE_OFF: 2}
    stances, cur = [], {"left": [], "right": []}
    counts = {PHASE_HEEL_STRIKE: 0, PHASE_FLAT_FOOT: 0, PHASE_TOE_OFF: 0}
    for snap in frames + [None]:
        for side in ("left", "right"):
            leg = None if snap is None else snap["legs"][side]
            if leg is not None and leg["state"] == "stance":
                cur[side].append((snap["frame"], leg["phase"]))
                counts[leg["phase"]] += 1
            elif cur[side]:
                seq = [k for k, _ in itertools.groupby(p for _, p in cur[side])]
                stances.append(dict(side=side, start=cur[side][0][0], frames=len(cur[side]),
                                    sequence="".join(PHASE_SHORT[s] + ">" for s in seq)[:-1],
                                    monotonic=all(order[a] < order[b] for a, b in zip(seq, seq[1:]))))
                cur[side] = []
    multi = [s for s in stances if s["frames"] >= 5]
    full_roll = [s for s in multi if s["sequence"].startswith("HS") and s["sequence"].endswith("TO")]
    return dict(deadband_deg=CONTACT_DEADBAND_DEG, stance_count=len(stances),
                stances_ge5_frames=len(multi), non_monotonic=sum(not s["monotonic"] for s in stances),
                full_heel_to_toe_rolls=len(full_roll), stance_frame_phase_counts=counts,
                one_frame_stances=sum(s["frames"] == 1 for s in stances), stances=stances)


def montage(images: dict, frames, labels) -> np.ndarray:
    canvas = np.full((720, 1280, 3), (237, 244, 249), np.uint8)
    for panel, (i, label) in enumerate(zip(images, labels)):
        img, cx = images[i]
        cx = int(np.clip(round(cx), 130, WIDTH - 130))
        crop = img[245:605, cx - 130:cx + 130]
        tile = cv2.resize(crop, (300, 415), interpolation=cv2.INTER_CUBIC)
        canvas[95:510, panel * 320 + 10:panel * 320 + 310] = tile
        cv2.putText(canvas, label, (panel * 320 + 20, 40), cv2.FONT_HERSHEY_SIMPLEX, .6, (65, 76, 89), 2, cv2.LINE_AA)
        cv2.putText(canvas, f"{frames[i]['time']:.2f} s", (panel * 320 + 20, 68), cv2.FONT_HERSHEY_SIMPLEX, .5, (110, 120, 130), 1, cv2.LINE_AA)
    return canvas


def preview_picks(frames) -> tuple[list[int], list[str]]:
    def first(pred, lo=0):
        for s in frames[lo:]:
            if pred(s):
                return s["frame"]
        return None
    stance = lambda s, ph: any(l["state"] == "stance" and l["phase"] == ph for l in s["legs"].values())
    picks = [(first(lambda s: s["frame"] >= 40 and stance(s, PHASE_HEEL_STRIKE)), "HEEL STRIKE"),
             (first(lambda s: stance(s, PHASE_FLAT_FOOT)), "FLAT FOOT"),
             (first(lambda s: s["frame"] >= 110 and stance(s, PHASE_TOE_OFF)), "TOE OFF"),
             (max(frames[round(s14.BIG_PUSH_T * FPS):], key=lambda s: s["hip"][1])["frame"]
              if len(frames) > round(s14.BIG_PUSH_T * FPS) else None, "ITKI: EN ALCAK KALCA")]
    picks = [(i, l) for i, l in picks if i is not None]
    return [i for i, _ in picks], [l for _, l in picks]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--preview-only", action="store_true")
    ap.add_argument("--legacy-heel-gait", action="store_true",
                    help="Adim 17 oncesi 6/12 capture-point marjlari (topuk yuruyusu) -- karsilastirma")
    ap.add_argument("--no-faz-b", action="store_true",
                    help="Faz B yakalama adimini kapat (Adim 17 acil adim davranisi) -- karsilastirma")
    args = ap.parse_args()
    kw = dict(LEGACY_GAIT) if args.legacy_heel_gait else {}
    if args.no_faz_b:
        kw["faz_b"] = False
    frames, sim = simulate(**kw)
    report = dict(source="demo.step14_active_biped.ActiveBipedSim (fizik degismedi)",
                  fell=sim.fell, fall_frame=sim.fall_frame, steps=len(sim.step_events),
                  emergency_steps=len(sim.emergency_step_events), slip_frames=len(sim.slip_events),
                  faz_b=sim.faz_b, faz_b_events=sim.fazb_events,
                  min_hip_height_after_push=round(s14.GROUND_Y - max(sim.hip_y_log[round(s14.BIG_PUSH_T * FPS):]), 1),
                  double_swing_frames=sum(all(l["state"] == "swing" for l in fr["legs"].values()) for fr in frames),
                  frames=len(frames))
    report["phases"] = phase_report(frames)
    report["skin"] = inspect(frames)
    out = ROOT / "outputs"
    out.mkdir(exist_ok=True)
    stem = "step17_bilge_physics_skin" + ("_legacy" if args.legacy_heel_gait else "") + ("_nofazb" if args.no_faz_b else "") + ("_debug" if args.debug else "")
    report["gait_params"] = LEGACY_GAIT if args.legacy_heel_gait else dict(
        support_margin=s14.SUPPORT_MARGIN, swing_lead_margin=s14.SWING_LEAD_MARGIN)
    picks, labels = preview_picks(frames)
    rig = PhysicsBilgeRig()   # kamera takibi sirali: her kare sirayla poz'lanir
    images = {}
    if args.preview_only:
        for snap in frames:
            pose = rig.pose(snap)
            if snap["frame"] in picks:
                images[snap["frame"]] = (render(pose, rig, frames, args.debug), pose["points"]["pelvis"][0])
    else:
        def frame_at(t):
            i = round(t * FPS)
            pose = rig.pose(frames[i])
            img = render(pose, rig, frames, args.debug)
            if i in picks:
                images[i] = (img, pose["points"]["pelvis"][0])
            return img
        render_video(frame_at, out / f"{stem}.mp4", size=(WIDTH, HEIGHT), fps=FPS, duration=len(frames) / FPS)
    cv2.imwrite(str(out / f"{stem}_preview.png"), montage(images, frames, labels))
    brief = {k: v for k, v in report.items()}
    brief["phases"] = {k: v for k, v in report["phases"].items() if k != "stances"}
    (out / f"{stem}_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(brief, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
