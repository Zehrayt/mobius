"""
leg_mass.py -- Adim 22: salinim bacaginin anatomik kutlesi ve govdeye tepkisi.

Bu motorda bacaklar kinematiktir (capture-point karari + Bezier salinim +
FABRIK). Kutlesiz oldugu icin, salinimdaki bacagin hizlanip yavaslamasi
govdeye HICBIR tepki vermiyordu; kollarin "bacak momentumunu iptal" edecegi
gercek bir momentum yoktu (Adim 20/21).

Bu modul bacak segmentlerine anatomik kutle atar ve kalca ekleminin bacagi
kinematik yorungesinde surdurmek icin uygulamasi GEREKEN kuvvet/momenti
hesaplar (ters dinamik, Newton-Euler), sonra bunun TEPKISINI ust govdeye
uygular:

    F   = sum m_i (a_i - g)                 kalcanin bacaga uyguladigi kuvvet
    tau = sum r_i x m_i (a_i - g)           kalcanin bacaga uyguladigi moment
                                            (r_i: kalcaya gore)

    ust govde: kalca noktasina -F (itki, prev_points uzerinden) -- Verlet
               cubuklari bunu govdeye/kollara dagitir; dogrusal momentum
               korunur.
    govde:     -tau, kalca-omuz eksenine dik bir KUVVET CIFTI olarak (omuza
               +f, kalcaya -f; net kuvvet 0, sadece moment).

SEGMENT KONUMU -- PERGEL (compass-gait) EKSENI: segment kutle merkezleri
kalca -> ayak hedefi (`leg.foot_target`, Bezier yorungesi) ekseni uzerinde
alinir (uyluk %21.5, incik+ayak %75). FABRIK diz noktasi KULLANILMAZ:
olcum -- bacak neredeyse tam gergin (kalca-ayak 184 px = 92+92), bu
durumda kalca yuksekligindeki 0.5 px'lik degisimler dizi tek karede
+/-12 px yana sicratiyor (kotu kosullu cozum); ikinci farki 5-20 px/kare^2
sahte ivme uretiyordu (pergel ekseninde ayni kareler: <=0.1).

KUTLELER (Winter segment tablolari, yuvarlanmis): govde+bas %58 = bu
motordaki 3.0 birim -> toplam ~5.17. Uyluk %10 -> 0.52, incik+ayak %6 ->
0.31.

DURUST SINIRLAR: (1) Sadece SALINIM bacagi hesaplanir; stance bacaginin
kutlesini zemin tasir (anchor). (2) Bacak yorungesi hala kinematik --
tepki govdeyi etkiler ama bacak hareketini geri etkilemez (ters dinamik,
ileri dinamik degil). (3) Ivme, uc karelik merkezi farktan gelir -> bir
karelik gecikme. (4) Diz bukulmesinin momentum katkisi yok (pergel ekseni).
"""
from __future__ import annotations

import numpy as np

THIGH_MASS = 0.52
SHANK_MASS = 0.31
THIGH_AXIS_FRAC = 0.215   # 0.43 * 92 / 184
SHANK_AXIS_FRAC = 0.75    # (92 + 0.5 * 92) / 184
LEG_MASS = THIGH_MASS + SHANK_MASS


def _cross(a, b) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


class LegMassModel:
    def __init__(self, gravity: np.ndarray):
        self.g = np.asarray(gravity, dtype=float)
        self.hist = {}            # bacak id -> [(foot, swing?), ...] son 3 kare
        self.last_force = np.zeros(2)
        self.last_torque = 0.0
        self.log = []             # (F, tau, n_salinim) her kare

    @staticmethod
    def segment_coms(hip: np.ndarray, foot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        d = foot - hip
        return hip + THIGH_AXIS_FRAC * d, hip + SHANK_AXIS_FRAC * d

    def observe(self, legs) -> None:
        """Her karenin SONUNDA (bacaklar guncellendikten sonra) cagrilir."""
        for leg in legs:
            h = self.hist.setdefault(id(leg), [])
            h.append((np.asarray(leg.foot_target, dtype=float).copy(), leg.state == "swing"))
            if len(h) > 3:
                h.pop(0)

    def swinging(self, legs) -> list:
        """Ivme ornegi (t-1) VEYA en son ornekte salinimdaki bacaklar --
        inis karesindeki yavaslama da dahil."""
        out = []
        for leg in legs:
            h = self.hist.get(id(leg), [])
            if len(h) == 3 and (h[1][1] or h[2][1]):
                out.append(leg)
        return out

    def hip_load(self, legs, hip_pos: np.ndarray) -> tuple[np.ndarray, float, int]:
        """ACIK (explicit) kalca yuku: ayak programinin payi.

            F   = sum m_i f_i (a_ayak - g)
            tau = sum r_i x m_i (f_i a_ayak - g),   r_i = f_i (ayak - kalca)

        Kalca payi sum m_i (1 - f_i) ORTUK (implicit): kalca noktasinin
        kutlesine eklenir (bkz. `implicit_hip_mass`), Verlet cozer --
        gecikmeli acik geri besleme dongusu yok."""
        F = np.zeros(2)
        tau = 0.0
        sw = self.swinging(legs)
        for leg in sw:
            h = self.hist[id(leg)]
            a_foot = h[2][0] - 2.0 * h[1][0] + h[0][0]
            d = h[1][0] - hip_pos
            for frac, m in ((THIGH_AXIS_FRAC, THIGH_MASS), (SHANK_AXIS_FRAC, SHANK_MASS)):
                F += m * frac * (a_foot - self.g)
                tau += _cross(frac * d, m * (frac * a_foot - self.g))
        self.last_force, self.last_torque = F, tau
        self.log.append((F.copy(), tau, len(sw)))
        return F, tau, len(sw)

    @staticmethod
    def implicit_hip_mass() -> float:
        return THIGH_MASS * (1.0 - THIGH_AXIS_FRAC) + SHANK_MASS * (1.0 - SHANK_AXIS_FRAC)

    @staticmethod
    def apply_reaction(points, prev_points, masses, hip: int, shoulder: int,
                       F: np.ndarray, tau: float) -> None:
        """-F kalcaya itki; -tau kalca-omuz eksenine dik kuvvet cifti."""
        prev_points[hip] = prev_points[hip] + F / float(masses[hip])
        r = points[shoulder] - points[hip]
        d2 = float(np.dot(r, r))
        if d2 <= 1e-9:
            return
        # -tau momentini (z) uretecek cift: omuza f, kalcaya -f; f = (-tau/|r|^2) * z x r
        f = (-tau / d2) * np.array([-r[1], r[0]])
        prev_points[shoulder] = prev_points[shoulder] - f / float(masses[shoulder])
        prev_points[hip] = prev_points[hip] + f / float(masses[hip])
