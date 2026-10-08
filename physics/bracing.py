"""Fall-only arm reach and compliant support, in the engine's frame units.

This is a bounded controller for the existing point-mass arms, not a
biomechanical injury model. It never pins a hand or moves a body point.
"""
from __future__ import annotations

import numpy as np

from physics.arms import UPPER_ARM_LEN, FOREARM_LEN
from physics.hill import force_velocity


def constrain_fallen_elbows(arms):
    """Mass-weighted elbow limits, solved with bones and ground contacts.

    The walking arm constraint treats the shoulder as an anchor and moves
    only the hand. Doing that on the floor can inject sliding into a ragdoll.
    Here all three points share the angular correction.
    """
    p, masses = arms.body.points, arms.body.masses
    for elbow, hand in arms.idx.values():
        ids = [arms.shoulder, elbow, hand]
        a, b = p[elbow] - p[arms.shoulder], p[hand] - p[elbow]
        a2, b2 = float(a @ a), float(b @ b)
        if min(a2, b2) < 1e-9:
            continue
        angle = float(np.arctan2(a[0] * b[1] - a[1] * b[0], a @ b))
        error = angle - float(np.clip(angle, np.radians(-140), np.radians(-2)))
        if abs(error) < 1e-9:
            continue
        ga = np.array([-a[1], a[0]]) / a2
        gb = np.array([-b[1], b[0]]) / b2
        gradients = np.array([ga, -ga - gb, gb])
        weights = 1.0 / masses[ids]
        denominator = float(np.sum(weights[:, None] * gradients ** 2))
        # Bound each iteration near a fully folded singular configuration.
        correction = -np.clip(error, -0.25, 0.25) / max(denominator, 1e-9)
        p[ids] += correction * weights[:, None] * gradients


REACH_STIFFNESS = 0.35
REACH_DAMPING = 0.65
EXTENSION_STIFFNESS = 0.7
EXTENSION_DAMPING = 0.65
MAX_EXTENSION_FORCE = 7.0


class FallBracing:
    def __init__(self):
        self.state = "idle"
        self.start_frame = None
        self.contact_frame = None
        self.direction = 1.0
        self.target = None
        self.support_impulse = 0.0

    @property
    def active(self):
        return self.state in ("reach", "absorb")

    def drive(self, arms, head_idx, ground_y, gravity, frame):
        body = arms.body
        p, q, m = body.points, body.prev_points, body.masses
        sh = arms.shoulder
        v = p[sh] - q[sh]
        clearance = max(ground_y - 14.0 - p[sh, 1], 0.0)
        # Ballistic time to chest contact, including acceleration even when
        # the torso is momentarily moving up at collapse.
        ttc = (-v[1] + np.sqrt(v[1] ** 2 + 2 * gravity * clearance)) / max(gravity, 1e-9)
        if self.state == "idle":
            if ttc > 12.0:
                return
            self.state = "reach"
            self.start_frame = frame
            self.direction = 1.0 if v[0] >= 0 else -1.0
        if self.state == "released":
            return
        if (p[head_idx, 1] >= ground_y - 17.0 or
                frame - self.start_frame >= 36):
            self.state = "released"
            return
        touching = any(p[h, 1] >= ground_y - 6.0 for _, h in arms.idx.values())
        if touching and self.contact_frame is None:
            self.contact_frame = frame
        self.state = "absorb" if touching else "reach"
        # Intercept the ground in front of the falling shoulder. Limited lead
        # keeps the hand underneath the load rather than far beyond reach.
        lead = self.direction * min(24.0, max(10.0, abs(v[0]) * min(ttc, 3.0)))
        self.target = np.array([p[sh, 0] + lead, ground_y - 5.0])
        target_angle = float(np.arctan2(lead, max(self.target[1] - p[sh, 1], 60.0))) - 0.35
        for side, (_, h) in arms.idx.items():
            angle = arms.arm_angle(side)
            error = float(np.arctan2(np.sin(target_angle - angle), np.cos(target_angle - angle)))
            alpha = float(np.clip(REACH_STIFFNESS * error - REACH_DAMPING * arms.arm_angular_velocity(side), -0.22, 0.22))
            arms._apply_alpha(side, alpha)
            # Shoulder-hand extension acts through the two rigid bones. Under
            # contact the elbow yields; a finite, Hill-scaled force resists
            # compression. Equal/opposite impulses conserve linear momentum.
            delta = p[h] - p[sh]
            length = float(np.linalg.norm(delta))
            if length < 1e-6:
                continue
            axis = delta / length
            rate = float(np.dot((p[h] - q[h]) - (p[sh] - q[sh]), axis))
            rest = 0.94 * (UPPER_ARM_LEN + FOREARM_LEN)
            reduced_mass = 1.0 / (1.0 / m[h] + 1.0 / m[sh])
            force = max(0.0, (EXTENSION_STIFFNESS * (rest - length) - EXTENSION_DAMPING * rate) * reduced_mass)
            cap = MAX_EXTENSION_FORCE * force_velocity(rate / (UPPER_ARM_LEN + FOREARM_LEN))
            force = min(force, cap)
            impulse = axis * force
            q[h] -= impulse / m[h]
            q[sh] += impulse / m[sh]
            if touching:
                self.support_impulse += max(0.0, float(impulse[1]))
