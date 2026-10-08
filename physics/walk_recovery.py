"""Slow alternating steps on the existing massive recovery legs.

Vector hip/foot servos are bounded internal force pairs (linear momentum is
preserved). Like the inherited world-reference posture motors, they are a
simplified actuator model, not an angular-momentum-conserving muscle model.
No topology, mass, gravity, pin or point-position changes occur at handover.
"""
import numpy as np
from physics.stand_recovery import StandRecovery

WALK_STATES = ('walk_prepare', 'walk_shift', 'walk_swing', 'walk_land')
PREP_FRAMES = 180
SHIFT_FRAMES = 120
SWING_FRAMES = 120
LAND_FRAMES = 80
STEP_LENGTH = 45.
FOOT_LIFT = 25.
FORCE_MIN = np.array([-4., -14.])
FORCE_MAX = np.array([4., 8.])


def smooth(t):
    t = float(np.clip(t, 0., 1.))
    return t*t*(3-2*t)


class WalkRecovery(StandRecovery):
    def __init__(self):
        super().__init__()
        self.walk_start_frame = None
        self.walk_start_x = None
        self.stance_side = None
        self.swing_side = None
        self.foot_origins = {}
        self.shift_start_x = None
        self.completed_steps = []
        self.step_clearance = 0.
        self.step_stance_travel = 0.
        self.last_stance_x = None
        self.landing_frames = 0
        self.unsupported_frames = 0
        self.max_walk_force_ratio = 0.

    @property
    def constraint_iterations_multiplier(self):
        return 2 if self.state in WALK_STATES else super().constraint_iterations_multiplier

    def _phase(self, sim, state):
        self.state = state
        self.phase_frame = sim.frame
        self.events.append((sim.frame, state))

    def _begin_shift(self, sim, state='walk_shift'):
        p = sim.body.points
        self.foot_origins = {side: np.array([p[f, 0], self.ground_y-6.])
                             for side, (_, f) in sim.fallen_legs.items()}
        self.shift_start_x = float(p[sim.idx['hip'], 0])
        self.step_clearance = 0.
        self.step_stance_travel = 0.
        self.last_stance_x = float(p[sim.fallen_legs[self.stance_side][1], 0])
        self.landing_frames = 0
        self._phase(sim, state)

    def _pair_force(self, sim, side, desired, share):
        b = sim.body; p, q, m = b.points, b.prev_points, b.masses
        hip = sim.idx['hip']; foot = sim.fallen_legs[side][1]
        delta = p[hip]-p[foot]
        velocity = (p[hip]-q[hip])-(p[foot]-q[foot])
        weight = sum(m[i] for i in range(len(m)) if i not in b.pinned)*b.gravity[1]
        force = (1/(1/m[hip]+1/m[foot]))*(.5*(desired-delta)-1.2*velocity)
        force[1] -= weight*share
        demand = force.copy()
        force = np.clip(force, FORCE_MIN, FORCE_MAX)
        # 2B cift: eksen disi bilesen kalca-ayak cizgisine dik bir kuvvet ciftidir
        self.force_log.append((self.telemetry_frame, self.state, foot, hip,
                               float(np.linalg.norm(force)), float(np.linalg.norm(demand)),
                               float(np.max(np.maximum(-FORCE_MIN, FORCE_MAX))), force.copy(), delta.copy()))
        q[hip] -= force/m[hip]
        q[foot] += force/m[foot]
        ratio = float(np.max(np.abs(force)/np.where(force < 0, -FORCE_MIN, FORCE_MAX)))
        self.max_walk_force_ratio = max(self.max_walk_force_ratio, ratio)
        self.max_force_ratio = max(self.max_force_ratio, ratio)

    def _walk_drive(self, sim):
        t = sim.frame-self.phase_frame
        feet = {side: pos.copy() for side, pos in self.foot_origins.items()}
        hx = self.shift_start_x
        load = 0.
        if self.state != 'walk_prepare':
            blend = smooth(t/SHIFT_FRAMES) if self.state == 'walk_shift' else 1.
            hx += (feet[self.stance_side][0]-hx)*blend
            load = .5*blend
        if self.state in ('walk_swing', 'walk_land'):
            a = min(t/SWING_FRAMES, 1.) if self.state == 'walk_swing' else 1.
            feet[self.swing_side][0] += (feet[self.stance_side][0]+STEP_LENGTH-feet[self.swing_side][0])*smooth(a)
            feet[self.swing_side][1] -= FOOT_LIFT*np.sin(np.pi*a)**2
            if self.state == 'walk_land':
                load *= 1-smooth(t/LAND_FRAMES)
        hip = np.array([hx, self.ground_y-184.])
        for side in sim.fallen_legs:
            self._pair_force(sim, side, hip-feet[side], .5+load if side == self.stance_side else .5-load)
        knees = {k for k, _ in sim.fallen_legs.values()}
        for a, b, degrees, cap in self.stand_pairs(sim):
            if a not in knees:
                self._motor(sim.body, a, b, np.radians(degrees), cap, 1.)
        for _, hand in sim.arms.idx.values():
            self._extend(sim.body, hand, sim.idx['shoulder'], 60., 16., 1.)

    def drive(self, sim):
        if self.state not in WALK_STATES:
            return super().drive(sim)
        if self.state == 'walk_prepare':
            # Cross-fade actuator impulses; do not reset position or velocity.
            q = sim.body.prev_points.copy()
            self.state = 'standing'
            continuing = self.continuing_stand
            self.continuing_stand = True
            super().drive(sim)
            self.continuing_stand = continuing
            old = sim.body.prev_points.copy()
            sim.body.prev_points[:] = q
            self.state = 'walk_prepare'
            self._walk_drive(sim)
            blend = smooth((sim.frame-self.phase_frame)/PREP_FRAMES)
            sim.body.prev_points[:] = old*(1-blend)+sim.body.prev_points*blend
        else:
            self._walk_drive(sim)

    def _fail(self, sim, reason):
        self.failure_reason = reason
        self._phase(sim, 'failed')

    def observe(self, sim, ground_y):
        super().observe(sim, ground_y)
        if self.state == 'standing' and sim.frame-self.standing_frame >= 60:
            self.walk_start_frame = sim.frame
            self.walk_start_x = float(sim.body.points[sim.idx['hip'], 0])
            self.stance_side = max(sim.fallen_legs, key=lambda side: sim.body.points[sim.fallen_legs[side][1], 0])
            self.swing_side = 'r' if self.stance_side == 'l' else 'l'
            self._begin_shift(sim, state='walk_prepare')
        if self.state not in WALK_STATES:
            return
        p = sim.body.points; t = sim.frame-self.phase_frame
        self.support_contacts = self.stand_contacts
        self.com_margin = self.stand_margin
        self.unsupported_frames = self.unsupported_frames+1 if self.stand_contacts == 0 else 0
        if (self.stand_other_contacts or ground_y-p[sim.idx['hip'], 1] < 150. or
                abs(self.torso_tilt) > 25. or self.unsupported_frames >= 3):
            self._fail(sim, 'walking_support_lost')
            return
        if self.state == 'walk_prepare':
            if t >= PREP_FRAMES and self.stand_contacts == 2:
                self._begin_shift(sim)
            elif t >= PREP_FRAMES+240:
                self._fail(sim, 'walking_prepare_timeout')
            return
        foot = sim.fallen_legs[self.swing_side][1]
        stance = sim.fallen_legs[self.stance_side][1]
        self.step_clearance = max(self.step_clearance, float(ground_y-p[foot, 1]-6.))
        self.step_stance_travel += abs(float(p[stance, 0])-self.last_stance_x)
        self.last_stance_x = float(p[stance, 0])
        if self.state == 'walk_shift':
            if t >= SHIFT_FRAMES and self.stand_foot_contacts[self.stance_side] and abs(p[sim.idx['hip'], 0]-p[stance, 0]) < 8.:
                self._phase(sim, 'walk_swing')
            elif t >= SHIFT_FRAMES+240:
                self._fail(sim, 'walking_shift_timeout')
        elif self.state == 'walk_swing' and t >= SWING_FRAMES:
            self._phase(sim, 'walk_land')
        elif self.state == 'walk_land':
            landed = (self.stand_contacts == 2 and self.stand_min_foot_share >= .1 and
                      self.step_clearance >= 8. and p[foot, 0]-self.foot_origins[self.swing_side][0] >= 25.)
            self.landing_frames = self.landing_frames+1 if landed else 0
            if t >= LAND_FRAMES and self.landing_frames >= 5:
                self.completed_steps.append(dict(frame=sim.frame, side=self.swing_side,
                    clearance_px=self.step_clearance, stance_travel_px=self.step_stance_travel,
                    advance_px=float(p[foot, 0]-self.foot_origins[self.swing_side][0])))
                self.stance_side, self.swing_side = self.swing_side, self.stance_side
                self._begin_shift(sim)
            elif t >= LAND_FRAMES+240:
                self._fail(sim, 'walking_landing_timeout')
