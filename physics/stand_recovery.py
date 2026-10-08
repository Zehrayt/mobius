"""Bounded leg extension from upright kneeling to a two-foot standing hold.

The new axial impulses conserve linear/angular momentum within each pair;
the inherited world-reference posture motors remain a simplified model.
"""
import numpy as np
from physics.kneel_rise import KneelRise
from physics.ground_recovery import wrap, ATTEMPT_TIMEOUT

STAND_STATES = ('standing_rising', 'standing')
EXTEND_FRAMES = 300
STAND_LENGTH = 180.
LEG_FORCE_CAP = 14.
QUIET_SPEED = .02
QUIET_FRAMES = 30
MOTION_LOSS_FRAMES = 5


class StandRecovery(KneelRise):
    def __init__(self):
        super().__init__()
        self.stand_start_frame = None
        self.standing_frame = None
        self.stand_quiet_frames = 0
        self.stand_motion_frames = 0
        self.initial_leg_lengths = {}
        self.stand_contacts = 0
        self.stand_margin = None
        self.stand_width = None
        self.stand_min_knee_clearance = None
        self.stand_other_contacts = 0
        self.stand_min_foot_share = 0.
        self.stand_foot_contacts = {'l': False, 'r': False}
        self.all_ground_margin = None
        self.max_stand_force_ratio = 0.
        self.stand_force_peak = 0.
        self.continuing_stand = False

    @property
    def constraint_iterations_multiplier(self):
        return 2 if self.state in STAND_STATES else super().constraint_iterations_multiplier

    def stand_pairs(self, sim):
        for a, b, degrees, cap in self.rise_pairs(sim):
            for side, (knee, foot) in sim.fallen_legs.items():
                if (a, b) == (knee, sim.idx['hip']):
                    degrees = -12. if side == self.front_side else 5.
                if (a, b) == (knee, foot):
                    degrees = -177. if side == self.front_side else -170.
            yield a, b, degrees, cap

    def _begin_stand(self, sim, continuing=False):
        pairs = list(self.stand_pairs(sim) if continuing else self.rise_pairs(sim))
        self._enter(sim, 'standing_rising')
        self.continuing_stand = continuing
        for a, b, degrees, _ in pairs:
            self.initial_angles[a, b] = np.radians(degrees)
        self.initial_leg_lengths = {
            side: float(np.linalg.norm(sim.body.points[sim.idx['hip']]-sim.body.points[foot]))
            for side, (_, foot) in sim.fallen_legs.items()}
        self.stand_quiet_frames = 0
        self.stand_motion_frames = 0

    def _support_leg(self, sim, side, blend):
        body = sim.body
        p, q, m = body.points, body.prev_points, body.masses
        hip = sim.idx['hip'];foot = sim.fallen_legs[side][1]
        delta = p[hip]-p[foot]
        length = max(float(np.linalg.norm(delta)), 1e-6)
        axis = delta/length
        rate = float(np.dot((p[hip]-q[hip])-(p[foot]-q[foot]), axis))
        total_mass = sum(m[i] for i in range(len(m)) if i not in body.pinned)
        # Feed-forward support accounts for gravity; the PD term extends the
        # leg. Forces remain internal pairs, and floor reactions supply support.
        weight = total_mass*float(body.gravity[1])
        initial = self.initial_leg_lengths[side]
        target = initial+blend*(STAND_LENGTH-initial)
        reduced_mass = 1/(1/m[hip]+1/m[foot])
        force = weight*.5/max(.3, -axis[1])+reduced_mass*(1.5*(target-length)-1.2*rate)
        force = float(np.clip(force, 0., LEG_FORCE_CAP))
        previous = 2. if side == self.front_side else 0.
        force = previous*(1-blend)+force*blend
        impulse = axis*force
        q[hip] -= impulse/m[hip]
        q[foot] += impulse/m[foot]
        self.stand_force_peak = max(self.stand_force_peak, force)
        self.max_stand_force_ratio = max(self.max_stand_force_ratio, force/LEG_FORCE_CAP)
        self.max_force_ratio = max(self.max_force_ratio, force/LEG_FORCE_CAP)

    def drive(self, sim):
        if self.state not in STAND_STATES:
            return super().drive(sim)
        t = float(np.clip((sim.frame-self.phase_frame)/EXTEND_FRAMES, 0., 1.))
        blend = t*t*(3-2*t)
        if self.continuing_stand:
            blend = 1.  # retain support rather than unloading after a loss
        for a, b, degrees, cap in self.stand_pairs(sim):
            initial = self.initial_angles[a, b]
            self._motor(sim.body, a, b, initial+blend*wrap(np.radians(degrees)-initial), cap, 1.)
        for _, hand in sim.arms.idx.values():
            self._extend(sim.body, hand, sim.idx['shoulder'], 60., 16., 1.)
        for side in sim.fallen_legs:
            self._support_leg(sim, side, blend)

    def observe(self, sim, ground_y):
        super().observe(sim, ground_y)
        if not sim.collapsed or not sim.fallen_legs:
            return
        self._measure_standing(sim, ground_y)
        if self.state == 'upright_kneeling' and sim.frame-self.upright_frame >= 60:
            self.stand_start_frame = sim.frame
            self._begin_stand(sim)
        elif self.state in STAND_STATES:
            p = sim.body.points
            hip_height = ground_y-p[sim.idx['hip'], 1]
            chest_height = ground_y-p[sim.idx['shoulder'], 1]
            supported = (self.stand_contacts == 2 and self.stand_margin >= 8. and
                25. <= self.stand_width <= 100. and self.stand_other_contacts == 0 and
                self.stand_min_knee_clearance > 60. and self.stand_min_foot_share >= .15 and
                hip_height > 170. and chest_height > 210. and chest_height-hip_height > 40. and
                abs(self.torso_tilt) < 15. and sim.frame-self.phase_frame >= EXTEND_FRAMES)
            calm = self.speed < QUIET_SPEED
            self.stand_quiet_frames = self.stand_quiet_frames+1 if supported and calm else 0
            self.stand_motion_frames = self.stand_motion_frames+1 if not calm else 0
            if self.state == 'standing_rising' and self.stand_quiet_frames >= QUIET_FRAMES:
                self.state = 'standing'
                self.standing_frame = sim.frame
                self.events.append((sim.frame, 'standing'))
            elif self.state == 'standing' and (not supported or self.stand_motion_frames >= MOTION_LOSS_FRAMES):
                self._begin_stand(sim, continuing=True)
                self.events.append((sim.frame, 'standing_support_lost'))
            if self.state == 'standing_rising' and sim.frame-self.phase_frame >= ATTEMPT_TIMEOUT:
                self.state = 'failed';self.failure_reason = 'standing_timeout'
                self.events.append((sim.frame, 'failed'))

    def _measure_standing(self, sim, ground_y):
        p = sim.body.points
        projections = {}
        for frame, point, value in reversed(sim.ground_projection_impulses):
            if frame != sim.frame:
                break
            if value > 1e-8:
                projections[point] = projections.get(point, 0.)+value
        feet = [foot for _, foot in sim.fallen_legs.values()]
        supports = [foot for foot in feet if foot in projections and ground_y-p[foot, 1] <= 7.]
        self.stand_foot_contacts = {side: foot in supports for side, (_, foot) in sim.fallen_legs.items()}
        self.stand_contacts = len(supports)
        self.stand_width = float(np.ptp(p[feet, 0]))
        self.stand_other_contacts = sum(i not in feet for i in projections)
        self.stand_min_knee_clearance = float(min(ground_y-p[knee, 1]-12. for knee, _ in sim.fallen_legs.values()))
        self.stand_min_foot_share = min(projections.get(foot, 0.) for foot in feet)/max(sum(projections.values()), 1e-9)
        ids = [i for i in range(len(p)) if i not in sim.body.pinned]
        com = float(np.average(p[ids, 0], weights=sim.body.masses[ids]))
        self.stand_margin = (float(min(com-min(p[supports, 0]), max(p[supports, 0])-com)) if supports else None)
        ground_ids = list(projections)
        self.all_ground_margin = (float(min(com-min(p[ground_ids, 0]), max(p[ground_ids, 0])-com)) if ground_ids else None)
        if self.state in STAND_STATES:
            self.support_contacts = self.stand_contacts
            self.com_margin = self.stand_margin
