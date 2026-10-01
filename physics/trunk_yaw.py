"""
trunk_yaw.py -- Adim 22: govdenin dikey eksen (yaw) donmesi -- 2.5B.

Capraz (kontralateral) kol salinimi, sagittal duzlemde (bu motorun 2B
duzlemi) neredeyse HICBIR momenti iptal etmez: iki kol birbirine ters
salinir, sagittal acisal momentumlari birbirini goturur (olcum: kol/bacak
korelasyonu ~0, iptal ~%0). Gercek insanda kol salinimi, salinim bacaginin
DIKEY EKSEN etrafindaki momentumunu (kalca eklemi govdenin yan tarafinda,
bacak one atilinca govdeyi kendi etrafinda burar) iptal eder.

Bu modul o ekseni TEK serbestlik dereceli bir govde yaw durumu olarak ekler:

    L_toplam = I_govde * w + L_bacaklar + L_kollar
    I_govde * dw/dt = -(dL_bacaklar + dL_kollar)/dt + tau_zemin
    tau_zemin = -k * theta - c * w        (stance ayaginin zemine "serbest
                                           momenti" + kalca rotatorlari)

L_segment = m * yanal_ofset * (v_x - v_x_kalca): sol taraf +W, sag taraf -W
(kalca yari genisligi 18 px, omuz yari genisligi 38 px; 184 px bacak =
0.9 m olcegiyle ~9 cm / ~19 cm).

DURUST SINIRLAR: (1) Yaw durumu 2B sagittal fizige GERI ETKI ETMEZ (tek
yonlu: bacak/kol hareketi -> govde burulmasi). (2) Ofsetler sabit
(segmentlerin yanal hareketi yok). (3) I_govde, k, c secilmis parametreler
(bkz. sabitler); olcumler bu secime gore raporlanir.
"""
from __future__ import annotations

import numpy as np

HIP_HALF_WIDTH = 18.0
SHOULDER_HALF_WIDTH = 38.0
TRUNK_YAW_INERTIA = 3.0 * 20.0 ** 2      # govde kutlesi 3.0, yaw donme yaricapi ~20 px (~10 cm)
GROUND_YAW_STIFFNESS = TRUNK_YAW_INERTIA * 0.2 ** 2   # dogal frekans 0.2 rad/kare (~31 kare periyot)
GROUND_YAW_DAMPING = 2.0 * 0.7 * 0.2 * TRUNK_YAW_INERTIA


def side_sign(side: str) -> float:
    return 1.0 if side == "l" else -1.0


def yaw_momentum(items) -> float:
    """items: (yanal_ofset, kutle, v_x_goreli) uclulerinin listesi."""
    return float(sum(lat * m * vx for lat, m, vx in items))


class TrunkYaw:
    def __init__(self, inertia: float = TRUNK_YAW_INERTIA, k: float = GROUND_YAW_STIFFNESS,
                 c: float = GROUND_YAW_DAMPING):
        self.I = inertia
        self.k = k
        self.c = c
        self.theta = 0.0
        self.omega = 0.0
        self.prev_L = None
        self.log = []    # (theta, omega, L_bacak, L_kol)

    def update(self, L_legs: float, L_arms: float) -> float:
        L = L_legs + L_arms
        dL = 0.0 if self.prev_L is None else L - self.prev_L
        self.prev_L = L
        # yari-ortuk Euler: once hiz, sonra aci (dt = 1 kare)
        self.omega += (-dL - self.k * self.theta - self.c * self.omega) / self.I
        self.theta += self.omega
        self.log.append((self.theta, self.omega, L_legs, L_arms))
        return self.theta
