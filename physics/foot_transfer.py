"""Hands-assisted half kneel, after verified four-point ground support.

Uses bounded pair impulses. Contact-projection shares are solver diagnostics,
not physical force measurements or a percentage of body weight.
"""
import numpy as np
from physics.ground_recovery import GroundRecovery, wrap, ATTEMPT_TIMEOUT

PLACE_FRAMES = 240
TRANSFER_FRAMES = 120
HOLD_FRAMES = 30
QUIET_SPEED = .15
LEG_EXTENSION = 115.
LEG_FORCE_CAP = 2.
ACTIVE_STATES = ('foot_placing', 'transferring', 'half_kneeling')


class FootTransfer(GroundRecovery):
    def __init__(self):
        super().__init__(reposition=True)
        self.front_side = 'l'
        self.placement_frame = None
        self.transfer_frame = None
        self.half_kneel_frame = None
        self.transfer_quiet_frames = 0
        self.foot_contact = False
        self.rear_knee_contact = False
        self.hand_contacts = 0
        self.foot_share = 0.
        self.before_share = None
        self.placed_share = None
        self.com_to_foot = None
        self.front_knee_clearance = None
        self.transfer_contacts = 0
        self.transfer_margin = None

    def transfer_pairs(self, sim):
        hip, sh, head = [sim.idx[k] for k in ('hip', 'shoulder', 'head')]
        pairs = [(hip, sh, 125., 480.), (sh, head, 55., 100.)]
        for side, (knee, foot) in sim.fallen_legs.items():
            front = side == self.front_side
            pairs.extend([(knee, hip, -75. if front else 20., 480.),
                          (knee, foot, -145. if front else -86., 120.)])
        pairs.extend((hand, sh, -10., 300.) for _, hand in sim.arms.idx.values())
        return pairs

    def drive(self, sim):
        if self.state not in ACTIVE_STATES:
            return super().drive(sim)
        elapsed = sim.frame-self.phase_frame
        duration = PLACE_FRAMES if self.state == 'foot_placing' else TRANSFER_FRAMES
        t = float(np.clip(elapsed/duration, 0, 1))
        blend = t*t*(3-2*t)
        for a, b, degrees, cap in self.transfer_pairs(sim):
            initial = self.initial_angles[a, b]
            target = initial+blend*wrap(np.radians(degrees)-initial)
            self._motor(sim.body, a, b, target, cap, 1.)
        for _, hand in sim.arms.idx.values():
            self._extend(sim.body, hand, sim.idx['shoulder'], 60., 16., 1.)
        if self.state != 'foot_placing':
            # Closed-chain leg extension presses the planted foot down and
            # applies the equal opposite impulse to the hip, without pins.
            foot = sim.fallen_legs[self.front_side][1]
            self._extend(sim.body, sim.idx['hip'], foot, LEG_EXTENSION, LEG_FORCE_CAP, blend)

    def observe(self, sim, ground_y):
        super().observe(sim, ground_y)
        if not sim.collapsed or not sim.fallen_legs:
            return
        self._measure_transfer(sim, ground_y)
        if self.state == 'supported' and sim.frame-self.support_frame >= 60:
            self.before_share = self.foot_share
            self.placement_frame = sim.frame
            self._enter(sim, 'foot_placing')
        elif self.state in ACTIVE_STATES:
            elapsed = sim.frame-self.phase_frame
            p = sim.body.points
            hip = sim.idx['hip']
            foot = sim.fallen_legs[self.front_side][1]
            geometry = (self.transfer_contacts == 4 and self.transfer_margin >= 0 and
                        self.front_knee_clearance > 25 and ground_y-p[hip, 1] > 65 and
                        ground_y-p[sim.idx['shoulder'], 1] > 30 and
                        abs(p[foot, 0]-p[hip, 0]) < 50 and abs(self.com_to_foot) < 20)
            quiet = geometry and self.speed < QUIET_SPEED
            if self.state == 'foot_placing':
                quiet = quiet and elapsed >= PLACE_FRAMES
            else:
                quiet = (quiet and self.foot_share >= .25 and
                         self.foot_share >= self.placed_share+.08 and elapsed >= TRANSFER_FRAMES)
            self.transfer_quiet_frames = self.transfer_quiet_frames+1 if quiet else 0
            if self.state == 'foot_placing' and self.transfer_quiet_frames >= HOLD_FRAMES:
                self.placed_share = self.foot_share
                self.transfer_frame = sim.frame
                self.transfer_quiet_frames = 0
                self._enter(sim, 'transferring')
            elif self.state == 'transferring' and self.transfer_quiet_frames >= HOLD_FRAMES:
                self.state = 'half_kneeling'
                self.half_kneel_frame = sim.frame
                self.events.append((sim.frame, 'half_kneeling'))
            elif self.state == 'half_kneeling' and not quiet:
                self.transfer_quiet_frames = 0
                self._enter(sim, 'transferring')
                self.events.append((sim.frame, 'transfer_support_lost'))
            if self.state in ('foot_placing', 'transferring') and sim.frame-self.phase_frame >= ATTEMPT_TIMEOUT:
                self.failure_reason = self.state+'_timeout'
                self.state = 'failed'
                self.events.append((sim.frame, 'failed'))

    def _measure_transfer(self, sim, ground_y):
        p = sim.body.points
        knee, foot = sim.fallen_legs[self.front_side]
        rear = next(k for side, (k, _) in sim.fallen_legs.items() if side != self.front_side)
        hands = [h for _, h in sim.arms.idx.values()]
        projections = {}
        for frame, point, value in reversed(sim.ground_projection_impulses):
            if frame != sim.frame:
                break
            if value > 1e-8:
                projections[point] = projections.get(point, 0.)+value
        radii = {foot: 6., rear: 12., **{h: 5. for h in hands}}
        supports = [i for i, radius in radii.items() if i in projections and
                    ground_y-p[i, 1] <= radius+1.]
        self.foot_contact = foot in supports
        self.rear_knee_contact = rear in supports
        self.hand_contacts = sum(h in supports for h in hands)
        self.transfer_contacts = len(supports)
        self.foot_share = projections.get(foot, 0.)/max(sum(projections.values()), 1e-9)
        ids = [i for i in range(len(p)) if i not in sim.body.pinned]
        com = float(np.average(p[ids, 0], weights=sim.body.masses[ids]))
        self.com_to_foot = float(com-p[foot, 0])
        self.front_knee_clearance = float(ground_y-p[knee, 1]-12.)
        self.transfer_margin = (float(min(com-min(p[supports, 0]), max(p[supports, 0])-com))
                                if supports else None)
        if self.state in ACTIVE_STATES:
            self.support_contacts = self.transfer_contacts
            self.com_margin = self.transfer_margin
