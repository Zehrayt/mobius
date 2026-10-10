"""Two-segment fall-only torso, with a mass-weighted passive waist hinge.

Units match the 30 FPS point-mass solver. Limits are animation constraints,
not a validated anatomical model. Walking keeps its original rigid torso.
"""
from __future__ import annotations
import numpy as np

MIN_BEND = np.radians(-30.0)
MAX_BEND = np.radians(75.0)
WAIST_RADIUS = 12.0
STIFFNESS = 0.008
DAMPING = 0.25


def cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


class FallenSpine:
    def __init__(self, body, hip, shoulder):
        self.body = body
        self.hip, self.shoulder = hip, shoulder
        p, q = body.points, body.prev_points
        endpoints = [hip, shoulder]
        old_m = body.masses[endpoints].copy()
        old_p, old_v = p[endpoints].copy(), (p-q)[endpoints].copy()
        center = np.average(old_p, axis=0, weights=old_m)
        angular_before = float(np.sum(old_m * cross(old_p-center, old_v)))
        kinetic_before = float(np.sum(old_m[:, None] * old_v**2) / 2)
        # Equal mass is transferred from both endpoints to their midpoint.
        # This preserves total mass and COM, unlike adding a fresh torso mass.
        transfer = 0.1 * min(old_m)
        waist = (p[hip] + p[shoulder]) / 2
        waist_v = (old_v[0] + old_v[1]) / 2
        self.waist = body.add_point(waist, mass=2*transfer)
        body.prev_points[self.waist] = waist-waist_v
        body.masses[endpoints] -= transfer
        self.ids = [hip, self.waist, shoulder]
        p, q, m = body.points, body.prev_points, body.masses
        # Repartitioning changes rotational inertia. Restore angular as well
        # as linear momentum with a zero-net-impulse rotational correction.
        r = p[self.ids]-center
        v = (p-q)[self.ids]
        angular_after = float(np.sum(m[self.ids]*cross(r, v)))
        inertia = float(np.sum(m[self.ids, None]*r*r))
        dw = (angular_before-angular_after)/max(inertia, 1e-9)
        q[self.ids] -= dw*np.column_stack((-r[:, 1], r[:, 0]))
        self.transition = dict(
            mass_before=float(old_m.sum()), mass_after=float(m[self.ids].sum()),
            angular_before=angular_before,
            angular_after=float(np.sum(m[self.ids]*cross(r, (p-q)[self.ids]))),
            kinetic_before=kinetic_before,
            kinetic_after=float(np.sum(m[self.ids, None]*(p-q)[self.ids]**2)/2))
        for index, (i, j, length, compliance) in enumerate(body.sticks):
            if {i, j} == {hip, shoulder}:
                body.sticks[index] = (hip, self.waist, length/2, compliance)
                body.add_stick(self.waist, shoulder, length=length/2, compliance=compliance)
                self.segment_length = length/2
                break
        else:
            raise ValueError('Rigid torso stick not found')

    def geometry(self):
        p = self.body.points
        a = p[self.waist]-p[self.hip]
        b = p[self.shoulder]-p[self.waist]
        a2, b2 = float(a@a), float(b@b)
        if min(a2,b2) < 1e-9:
            return 0.0, np.zeros((3,2))
        theta = float(np.arctan2(cross(a,b), a@b))
        ga = np.array([-a[1],a[0]])/a2
        gb = np.array([-b[1],b[0]])/b2
        return theta, np.array([ga,-ga-gb,gb])

    def drive(self):
        theta, gradient = self.geometry()
        p,q,m = self.body.points,self.body.prev_points,self.body.masses
        velocity = (p-q)[self.ids]
        omega = float(np.sum(gradient*velocity))
        inverse_mass = 1/m[self.ids]
        effective = float(np.sum(inverse_mass[:,None]*gradient**2))
        alpha = float(np.clip(-STIFFNESS*theta-DAMPING*omega,-.08,.08))
        impulse = alpha/max(effective,1e-9)
        q[self.ids] -= impulse*inverse_mass[:,None]*gradient

    def constrain(self):
        theta, gradient = self.geometry()
        error = theta-float(np.clip(theta,MIN_BEND,MAX_BEND))
        inverse_mass = 1/self.body.masses[self.ids]
        effective = float(np.sum(inverse_mass[:,None]*gradient**2))
        correction = -float(np.clip(error,-.2,.2))/max(effective,1e-9)
        self.body.points[self.ids] += correction*inverse_mass[:,None]*gradient


def hinge_drive(body, ids, stiffness, damping, rest=0.0, limit=.08,
                end_damping=0.0, end_start=None, end_limit=None):
    """Uc noktali mentese (ids = [kok, eklem, uc]) icin acisal yay/sonum.
    FallenSpine.drive ile ayni bicim: gradyan boyunca esit/zit hiz itkileri,
    dogrusal momentum korunur. Boyun icin: [bel ya da kalca, omuz, bas]."""
    p, q, m = body.points, body.prev_points, body.masses
    a = p[ids[1]]-p[ids[0]]
    b = p[ids[2]]-p[ids[1]]
    a2, b2 = float(a@a), float(b@b)
    if min(a2, b2) < 1e-9:
        return 0.0
    theta = float(np.arctan2(cross(a, b), a@b))
    ga = np.array([-a[1], a[0]])/a2
    gb = np.array([-b[1], b[0]])/b2
    gradient = np.array([ga, -ga-gb, gb])
    omega = float(np.sum(gradient*(p-q)[ids]))
    inverse_mass = 1/m[ids]
    effective = float(np.sum(inverse_mass[:, None]*gradient**2))
    c = damping
    if end_damping > 0.0 and end_start is not None and end_limit is not None:
        # ilerleyici sonum: hareket siniri yaklastikca artar (sinirin ortasinda serbest)
        u = (abs(theta-rest)-end_start)/max(end_limit-end_start, 1e-9)
        c += end_damping*float(np.clip(u, 0.0, 1.0))**2
    alpha = float(np.clip(-stiffness*(theta-rest)-c*omega, -limit, limit))
    q[ids] -= alpha/max(effective, 1e-9)*inverse_mass[:, None]*gradient
    return alpha


class SoftHinge:
    """XPBD (Macklin ve dig. 2016) sonumlu acisal kisit, cozucu DONGUSUNUN ICINDE.

    Uc nokta [kok, eklem, uc]; C = theta - rest. Her iterasyonda:
        dlam = (-C - a*lam - g * gradC.(x - x_n)) / ((1 + g) * sum(w |gradC|^2) + a)
    a = esneklik (compliance, dt = 1 kare), g = a * beta (sonum). Sonum terimi karenin
    basindan (x_n = onceki kare konumlari) beri olan yer degisimine bakar: zemin
    projeksiyonu gogsu dongu icinde durdurdugunda basin goreli donusu de ayni dongude
    frenlenir. Adimlar arasi (Verlet hizina) uygulanan sonum buna yetisemiyordu.
    Dogrusal momentum korunur (gradyanin kutle agirlikli toplami sifir)."""

    def __init__(self, compliance, beta):
        self.compliance, self.beta = compliance, beta
        self.lam = 0.0

    def begin_frame(self):
        self.lam = 0.0

    def project(self, p, prev, m, ids, rest=0.0):
        a = p[ids[1]]-p[ids[0]]
        b = p[ids[2]]-p[ids[1]]
        a2, b2 = float(a@a), float(b@b)
        if min(a2, b2) < 1e-9:
            return
        theta = float(np.arctan2(cross(a, b), a@b))
        ga = np.array([-a[1], a[0]])/a2
        gb = np.array([-b[1], b[0]])/b2
        grad = np.array([ga, -ga-gb, gb])
        w = 1.0/m[ids]
        core = float(np.sum(w[:, None]*grad**2))
        alpha = self.compliance
        gamma = alpha*self.beta
        dC = float(np.sum(grad*(p[ids]-prev[ids])))
        C = theta-rest
        dlam = (-C-alpha*self.lam-gamma*dC)/((1.0+gamma)*core+alpha)
        p[ids] += w[:, None]*grad*dlam
        self.lam += dlam

