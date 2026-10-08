"""Raise the torso and unload the arms while retaining foot/knee support.

This extends the bounded world-reference motor approximation of FootTransfer;
it is not an angular-momentum-conserving muscle model.
"""
import numpy as np
from physics.foot_transfer import FootTransfer, LEG_EXTENSION, LEG_FORCE_CAP
from physics.ground_recovery import wrap, ATTEMPT_TIMEOUT

RAISE_STATES = ('torso_raising', 'upright_kneeling')
PREP_FRAMES = 120
TORSO_DELAY = 60
TORSO_FRAMES = 240
QUIET_FRAMES = 30
QUIET_SPEED = .02
MOTION_LOSS_FRAMES = 5


class KneelRise(FootTransfer):
    def __init__(self):
        super().__init__()
        self.raise_frame = None
        self.upright_frame = None
        self.upright_quiet_frames = 0
        self.motion_loss_frames = 0
        self.lower_contacts = 0
        self.lower_margin = None
        self.arm_contacts = 0
        self.min_hand_clearance = None
        self.torso_tilt = None
        self.hip_height_before_raise = None

    @property
    def constraint_iterations_multiplier(self):
        # Two support points leave the coupled ground/hinge solve more sensitive
        # to residual projection drift than the earlier four-point support.
        return 2 if self.state in RAISE_STATES else 1

    def rise_pairs(self, sim):
        rear = next(k for side, (k, _) in sim.fallen_legs.items() if side != self.front_side)
        hands = [h for _, h in sim.arms.idx.values()]
        for a, b, degrees, cap in self.transfer_pairs(sim):
            if (a, b) in ((sim.idx['hip'], sim.idx['shoulder']),
                          (sim.idx['shoulder'], sim.idx['head'])) or a in hands:
                degrees = 0.
            if a == rear and b == sim.idx['hip']:
                degrees = 5.
            yield a, b, degrees, cap

    def _begin_raise(self, sim, continuing=False):
        pairs = list(self.rise_pairs(sim) if continuing else self.transfer_pairs(sim))
        self._enter(sim, 'torso_raising')
        # Keep the previous commanded motor targets, rather than removing
        # their support torque by resetting targets to measured joint angles.
        for a, b, degrees, _ in pairs:
            self.initial_angles[a, b] = np.radians(degrees)
        self.upright_quiet_frames = 0
        self.motion_loss_frames = 0

    def drive(self, sim):
        if self.state not in RAISE_STATES:
            return super().drive(sim)
        elapsed = sim.frame-self.phase_frame
        hands = [h for _, h in sim.arms.idx.values()]
        for a, b, degrees, cap in self.rise_pairs(sim):
            upper = a in (sim.idx['hip'], sim.idx['shoulder']) or a in hands
            t = float(np.clip((elapsed-TORSO_DELAY)/TORSO_FRAMES if upper else elapsed/PREP_FRAMES, 0, 1))
            blend = t*t*(3-2*t)
            initial = self.initial_angles[a, b]
            self._motor(sim.body, a, b, initial+blend*wrap(np.radians(degrees)-initial), cap, 1.)
        for hand in hands:
            self._extend(sim.body, hand, sim.idx['shoulder'], 60., 16., 1.)
        self._extend(sim.body, sim.idx['hip'], sim.fallen_legs[self.front_side][1],
                     LEG_EXTENSION, LEG_FORCE_CAP, 1.)

    def observe(self, sim, ground_y):
        super().observe(sim, ground_y)
        if not sim.collapsed or not sim.fallen_legs:
            return
        self._measure_rise(sim, ground_y)
        if self.state == 'half_kneeling' and sim.frame-self.half_kneel_frame >= 60:
            self.raise_frame = sim.frame
            self.hip_height_before_raise = float(ground_y-sim.body.points[sim.idx['hip'], 1])
            self._begin_raise(sim)
        elif self.state in RAISE_STATES:
            p = sim.body.points
            hip_height = ground_y-p[sim.idx['hip'], 1]
            chest_height = ground_y-p[sim.idx['shoulder'], 1]
            supported = (self.lower_contacts == 2 and self.lower_margin >= 5. and
                      self.arm_contacts == 0 and self.min_hand_clearance > 20. and
                      self.front_knee_clearance > 25. and self.foot_share >= .2 and
                      hip_height > self.hip_height_before_raise+5. and chest_height > 120. and
                      chest_height-hip_height > 40. and abs(self.torso_tilt) < 20. and
                      sim.frame-self.phase_frame >= TORSO_DELAY+TORSO_FRAMES)
            calm = self.speed < QUIET_SPEED
            stable = supported and calm
            self.upright_quiet_frames = self.upright_quiet_frames+1 if stable else 0
            self.motion_loss_frames = self.motion_loss_frames+1 if not calm else 0
            if self.state == 'torso_raising' and self.upright_quiet_frames >= QUIET_FRAMES:
                self.state = 'upright_kneeling'
                self.upright_frame = sim.frame
                self.events.append((sim.frame, 'upright_kneeling'))
            elif self.state == 'upright_kneeling' and (not supported or self.motion_loss_frames >= MOTION_LOSS_FRAMES):
                self._begin_raise(sim, continuing=True)
                self.events.append((sim.frame, 'upright_support_lost'))
            if self.state == 'torso_raising' and sim.frame-self.phase_frame >= ATTEMPT_TIMEOUT:
                self.state = 'failed'
                self.failure_reason = 'torso_raise_timeout'
                self.events.append((sim.frame, 'failed'))

    def _measure_rise(self, sim, ground_y):
        p = sim.body.points
        _, foot = sim.fallen_legs[self.front_side]
        rear = next(k for side, (k, _) in sim.fallen_legs.items() if side != self.front_side)
        contacts = set()
        for frame, point, projection in reversed(sim.ground_projection_impulses):
            if frame != sim.frame:
                break
            if projection > 1e-8:
                contacts.add(point)
        supports = [i for i, radius in ((foot, 6.), (rear, 12.))
                    if i in contacts and ground_y-p[i, 1] <= radius+1.]
        self.lower_contacts = len(supports)
        ids = [i for i in range(len(p)) if i not in sim.body.pinned]
        com = float(np.average(p[ids, 0], weights=sim.body.masses[ids]))
        self.lower_margin = (float(min(com-min(p[supports, 0]), max(p[supports, 0])-com))
                             if supports else None)
        self.arm_contacts = sum(i in contacts for arm in sim.arms.idx.values() for i in arm)
        self.min_hand_clearance = float(min(ground_y-p[h, 1]-5. for _, h in sim.arms.idx.values()))
        d = p[sim.idx['shoulder']]-p[sim.idx['hip']]
        self.torso_tilt = float(np.degrees(np.arctan2(d[0], -d[1])))
        if self.state in RAISE_STATES:
            self.support_contacts = self.lower_contacts
            self.com_margin = self.lower_margin
