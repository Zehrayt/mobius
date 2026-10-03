"""
arms.py -- Adim 20: fiziksel (Verlet) kollar.

Her kol, omuz Verlet noktasina asili iki kutleli noktadan (dirsek, el) ve iki
rijit cubuktan olusur. Kollar `VerletSystem`'in kendi kisit cozucusunde
kalan govdeyle AYNI anda cozulur -- yani kolun hareketi omuz uzerinden
govdeye/kalcaya GERI ETKI EDER (ters-kutle agirlikli PBD, bkz. verlet.py
"kutle hiyerarsisi").

KUTLE (anatomik): Bu motorda govde = kalca + omuz + bas, her biri 1.0 kutle.
Anatomik referansla (Winter, "Biomechanics and Motor Control of Human
Movement" segment tablolari, yuvarlanmis): govde+bas ~%58, her bacak ~%16,
her kol ~%5. Govde 3.0 birim = %58 ise toplam ~5.17, bir kol ~0.26 birim:
dirsek noktasi 0.16 (ust kol + on kolun yarisi), el noktasi 0.10.

UC MEKANIZMA (hepsi ayri ayri kapatilabilir, olculebilsin diye):
  1. PASIF: yercekimi + koni sinirlari (omuz/dirsek) + dirsek yay-sonumu.
  2. MOMENTUM DENGELEME (`drive`): ayni taraftaki bacagin kalca-ayak
     acisina ve acisal hizina TERS bir PD omuz torku. Tork, dirsek
     noktasina tanjantiyel bir hiz degisimi olarak uygulanir; ESIT ve TERS
     dogrusal momentum omuza verilir (Newton 3) -- kol govdeye tork aktarir.
  3. REFLEKS (`reflex_target_deg`): tehlike aninda hedef aci bacaktan
     bagimsiz, sabit bir "denge arama" acisina kayar.

DURUST SINIR: bacaklar bu motorda kutlesiz (FABRIK kinematigi) -- yani
bacagin "gercek" acisal momentumu motorun icinde YOK. Kol torku bacak
kinematiginden okunur ve govdeye gercek bir reaksiyon uygular; "iptal"
olcumu (bkz. `angular_momentum_about`) bacaklara anatomik sanal kutleler
atanarak yapilir.

ADIM 22 GUNCELLEMESI: bacaklar artik kutleli (physics/leg_mass.py, ters
dinamik). Olcum: sagittal duzlemde kollarin momentum iptali ~%0 -- iki kol
birbirine ters salinir, katkilari birbirini goturur. Iptalin gercek ekseni
dikey eksen (yaw): `drive_cancel` kollari bacaklarin yaw momentumunu
sifirlayacak hiza surer (physics/trunk_yaw.py, README "Adim 22").
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from physics.verlet import VerletSystem, clamp_direction, apply_angular_spring, apply_angular_couple
from physics.fabrik import clamp_joint_angle_points

DOWN = np.array([0.0, 1.0])

UPPER_ARM_LEN = 34.0     # step14 olceginde (govde 55 px); render rig'de 54/50
FOREARM_LEN = 32.0
ELBOW_MASS = 0.16
HAND_MASS = 0.10
SHOULDER_CONE_DEG = 100.0   # asagi yondan +/- (yukari kaldirma refleksi icin genis)
ELBOW_MIN_BEND_DEG = 2.0    # tek yonlu dirsek menteşesi: hiperekstansiyon yok
ELBOW_MAX_BEND_DEG = 140.0
ELBOW_BEND_SIGN = -1.0      # on kol one dogru bukulur (bu cizim duzleminde negatif)
ELBOW_REST_DEG = -12.0      # hafif dirsek bukumu: on kol one dogru (apply_angular_spring isaretinde ileri = negatif)
ELBOW_STIFFNESS = 0.04
ELBOW_DAMPING = 0.25
ELBOW_BRACE_REST_DEG = -15.0   # Adim 30: koruyucu refleks -- dirsek hafif bukuk kilitlenir


def _angle_from_down(vec: np.ndarray) -> float:
    """Asagi yone gore isaretli aci (radyan); + = ileri (+x)."""
    return float(np.arctan2(vec[0], vec[1]))


@dataclass
class ArmGains:
    swing_gain: float = 1.5        # hedef kol acisi = -swing_gain * bacak acisi
    stiffness: float = 0.3         # PD: aci hatasi -> acisal ivme
    damping: float = 0.8           # PD: (hedef hiz - GERCEK kol hizi) -> acisal ivme
    max_accel: float = 0.15        # rad/kare^2 -- tork tavani (savrulma emniyeti)
    reflex_max_accel: float = 0.4  # refleks sirasindaki tork tavani (0.15 ile etki ~1 px, bkz. README Adim 20)
    cancel_damping: float = 0.8    # Adim 22 'cancel': hedef acisal hiza izleme kazanci (<=1)
    cancel_center: float = 0.04    # Adim 22 'cancel': kolu dikeye ceken zayif yay (kayma olmasin)
    cancel_max_accel: float = 0.3  # Adim 22 'cancel': tork tavani
    max_leg_rate: float = 0.0      # bacak acisal hizi ileri beslemesi (rad/kare tavani; 0 = kapali).
                                   # Acik iken izleme KOTULESTI (corr -0.86 -> -0.71): FABRIK zincir
                                   # ucunun acisi inis karelerinde sicriyor, hiz hedefi o sicramayi
                                   # kola tork olarak tasiyordu. Hiz hedefi = 0, sonum kolun kendi hizina.


class PhysicalArms:
    """Iki fiziksel kol. `sides` sirasi: ('l', 'r')."""

    def __init__(self, body: VerletSystem, shoulder_idx: int, gains: ArmGains | None = None):
        self.body = body
        self.shoulder = shoulder_idx
        self.gains = gains or ArmGains()
        self.idx = {}
        s = body.points[shoulder_idx].copy()
        for side in ("l", "r"):
            e = body.add_point(s + [0.0, UPPER_ARM_LEN], mass=ELBOW_MASS)
            h = body.add_point(s + [0.0, UPPER_ARM_LEN + FOREARM_LEN], mass=HAND_MASS)
            body.add_stick(shoulder_idx, e, length=UPPER_ARM_LEN)
            body.add_stick(e, h, length=FOREARM_LEN)
            self.idx[side] = (e, h)
        self.prev_leg_angle = {"l": None, "r": None}
        self.last_target = {"l": 0.0, "r": 0.0}
        self.last_torque = {"l": 0.0, "r": 0.0}

    def arm_angle(self, side: str) -> float:
        e, _ = self.idx[side]
        p = self.body.points
        return _angle_from_down(p[e] - p[self.shoulder])

    def arm_angular_velocity(self, side: str) -> float:
        """Kolun acisal hizi -- govdeyle AYNI zaman cizelgesinden (points /
        prev_points). Adim 20'de bu hiz ayri bir gecmisten okunuyordu, cunku
        govde kelepcesi omzun prev_points'inde duvara dogru "hayalet hiz"
        sakliyordu; Adim 21'de kok neden (esnek olmayan kelepce) duzeltildi
        ve ayri gecmis kaldirildi."""
        e, _ = self.idx[side]
        p, q = self.body.points, self.body.prev_points
        a_now = _angle_from_down(p[e] - p[self.shoulder])
        a_prev = _angle_from_down(q[e] - q[self.shoulder])
        return float(np.arctan2(np.sin(a_now - a_prev), np.cos(a_now - a_prev)))

    def drive(self, leg_angles: dict, enabled: bool = True, reflex_target_deg: float | None = None,
              reflex_weight: float = 1.0, exit_damping: float = 0.0, passive_base: bool = False,
              skip: tuple = ()) -> None:
        """Her kol icin PD omuz torku. `leg_angles[side]`: o taraftaki bacagin
        kalca->ayak acisi (radyan, asagiya gore, + ileri). Refleks verilirse
        hedef, bacaktan bagimsiz sabit bir aciya doner (hiz hedefi 0).
        Adim 30: `passive_base` -- refleks yuruyus hedefine degil SIFIR torka
        karisir (yigilmis govdede kollar yuruyus salinimina geri surulmez);
        `skip` -- o kollar surulmez (eli yere dayali, yuk alan kol)."""
        g = self.gains
        p, q = self.body.points, self.body.prev_points
        masses = self.body.masses
        for side in ("l", "r"):
            leg_a = leg_angles[side]
            prev = self.prev_leg_angle[side]
            leg_w = 0.0 if prev is None else leg_a - prev
            self.prev_leg_angle[side] = leg_a
            if not enabled or side in skip:
                self.last_torque[side] = 0.0
                continue
            lw = float(np.clip(leg_w, -g.max_leg_rate, g.max_leg_rate))
            target, target_w = -g.swing_gain * leg_a, -g.swing_gain * lw
            if reflex_target_deg is not None:
                # Adim 21: refleks hedefi, yuruyus hedefine reflex_weight ile
                # karisir; agirlik ustel sondugunde kol yumusakca geri doner.
                wr = float(np.clip(reflex_weight, 0.0, 1.0))
                if passive_base:
                    target, target_w = np.radians(reflex_target_deg), 0.0
                else:
                    target = wr * np.radians(reflex_target_deg) + (1.0 - wr) * target
                    target_w = (1.0 - wr) * target_w
            self.last_target[side] = target
            a = self.arm_angle(side)
            w = self.arm_angular_velocity(side)
            alpha = g.stiffness * (target - a) + (g.damping + exit_damping) * (target_w - w)
            cap = g.max_accel if exit_damping == 0.0 else g.reflex_max_accel
            if reflex_target_deg is not None:
                cap = g.max_accel + float(np.clip(reflex_weight, 0.0, 1.0)) * (g.reflex_max_accel - g.max_accel)
            alpha = float(np.clip(alpha, -cap, cap))
            if reflex_target_deg is not None and passive_base:
                alpha *= float(np.clip(reflex_weight, 0.0, 1.0))
            self.last_torque[side] = alpha
            e, _ = self.idx[side]
            r = float(np.linalg.norm(p[e] - p[self.shoulder]))
            tangent = np.array([np.cos(a), -np.sin(a)])      # d/da (sin a, cos a)
            dv = tangent * (alpha * r)
            q[e] = q[e] - dv                                  # dirsege hiz degisimi
            # Newton 3: esit ve ters dogrusal momentum omuza
            q[self.shoulder] = q[self.shoulder] + dv * (masses[e] / masses[self.shoulder])

    def _apply_alpha(self, side: str, alpha: float) -> None:
        p, q = self.body.points, self.body.prev_points
        masses = self.body.masses
        e, _ = self.idx[side]
        a = self.arm_angle(side)
        r = float(np.linalg.norm(p[e] - p[self.shoulder]))
        tangent = np.array([np.cos(a), -np.sin(a)])
        dv = tangent * (alpha * r)
        q[e] = q[e] - dv
        q[self.shoulder] = q[self.shoulder] + dv * (masses[e] / masses[self.shoulder])

    def yaw_lever(self, side: str) -> float:
        """d(L_yaw)/d(omega) bu kol icin: W * sum m_i d_i cos(a) (dirsek/el)."""
        from physics.trunk_yaw import SHOULDER_HALF_WIDTH
        e, h = self.idx[side]
        p = self.body.points
        m = self.body.masses
        s = p[self.shoulder]
        a = self.arm_angle(side)
        lever = m[e] * float(np.linalg.norm(p[e] - s)) + m[h] * float(np.linalg.norm(p[h] - s))
        return SHOULDER_HALF_WIDTH * lever * max(np.cos(a), 0.2)

    def drive_cancel(self, L_leg_yaw: float) -> None:
        """Adim 22 -- momentum iptali: kollar, bacaklarin dikey eksen (yaw)
        momentumunu sifirlayacak acisal hizi hedefler. Sol kol (+W) ve sag
        kol (-W) ters yonde doner (capraz salinim kendiliginden cikar):
            W_l*K*w_l - W*K*w_r = -L_bacak  ->  w_l = -w_r = -L_bacak / (2 K)
        Tork, `drive` ile ayni Newton-3 yoluyla uygulanir (sagittal tepki omza)."""
        g = self.gains
        for side in ("l", "r"):
            from physics.trunk_yaw import side_sign
            target_w = -side_sign(side) * L_leg_yaw / (2.0 * self.yaw_lever(side) + 1e-9)
            a = self.arm_angle(side)
            w = self.arm_angular_velocity(side)
            alpha = g.cancel_damping * (target_w - w) - g.cancel_center * a
            alpha = float(np.clip(alpha, -g.cancel_max_accel, g.cancel_max_accel))
            self.last_target[side] = target_w
            self.last_torque[side] = alpha
            self._apply_alpha(side, alpha)

    def constrain(self, fallen: bool = False, brace_k: float = 0.0, brace_c: float = 0.0,
                  skip: tuple = ()) -> None:
        """body.step()'ten SONRA: koni sinirlari + dirsek yay-sonumu.
        Adim 30: `fallen` iken dirsek menteşesi ve pasif yay burada UYGULANMAZ
        (fabrik.clamp_joint_angle_points yalnizca eli oynatir; zemine dayanmis
        elde momentum sizdirip surunme uretir -- Adim 29 diz bulgusu). Menteşe
        step14._fallen_constraints'te momentum koruyarak cozulur. `brace_k/c`:
        koruyucu refleksteki dirsek ekstansor tonusu -- momentum koruyan kuvvet
        cifti (apply_angular_couple)."""
        p, q = self.body.points, self.body.prev_points
        for side in ("l", "r"):
            e, h = self.idx[side]
            # Govde/boyun kelepceleri body.step()'ten SONRA omzu tasiyabiliyor;
            # kol cubuklari o zaman acik kalir (itkide 36-87 px "omuzdan kopma"
            # olculdu). Kol noktalari omza yeniden baglanir; ayni kaydirma
            # prev_points'e de uygulanir -- kolun kendi hizi korunur, kelepcenin
            # fiziksel olmayan sicramasi kola enerji olarak gecmez.
            for a_i, b_i, length in ((self.shoulder, e, UPPER_ARM_LEN), (e, h, FOREARM_LEN)):
                d = p[b_i] - p[a_i]
                n = float(np.linalg.norm(d))
                if n > 1e-9:
                    shift = p[a_i] + d / n * length - p[b_i]
                    p[b_i] = p[b_i] + shift
                    q[b_i] = q[b_i] + shift
                    if b_i == e:   # dirsek kayinca el de ayni miktar kayar
                        p[h] = p[h] + shift
                        q[h] = q[h] + shift
            clamp_direction(p, q, self.shoulder, e, DOWN, SHOULDER_CONE_DEG, preserve_momentum="inelastic")
            upper_dir = p[e] - p[self.shoulder]
            if fallen:
                if (brace_k > 0.0 or brace_c > 0.0) and side not in skip:
                    apply_angular_couple(p, q, self.body.masses, e, h, upper_dir, ELBOW_BRACE_REST_DEG,
                                         brace_k, brace_c)
                continue
            apply_angular_spring(p, q, e, h, upper_dir, ELBOW_REST_DEG, ELBOW_STIFFNESS, ELBOW_DAMPING)
            clamp_joint_angle_points(p, q, self.shoulder, e, h, ELBOW_MIN_BEND_DEG, ELBOW_MAX_BEND_DEG,
                                     bend_sign=ELBOW_BEND_SIGN, inelastic=True)

    def points_for(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        e, h = self.idx[side]
        return self.body.points[e].copy(), self.body.points[h].copy()


def angular_momentum_about(center: np.ndarray, v_center: np.ndarray,
                           pts: list[np.ndarray], vels: list[np.ndarray], masses: list[float]) -> float:
    """2B acisal momentum (z), `center`'e gore ve `center`'in hizina GORELI."""
    L = 0.0
    for r, v, m in zip(pts, vels, masses):
        rr = r - center
        vv = v - v_center
        L += m * (rr[0] * vv[1] - rr[1] * vv[0])
    return float(L)
