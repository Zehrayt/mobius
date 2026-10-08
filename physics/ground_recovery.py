"""Opt-in first recovery stage: settle, raise onto hands/knees, hold.

Bounded, world-referenced joint-pair motors (same approximation as posture
PD), not a full muscle model. Motors apply equal/opposite velocity impulses;
contacts and the existing constraints supply support. No pose teleport/pins.
"""
import numpy as np

CALM_SPEED = .15
CALM_FRAMES = 30
MIN_FALL_AGE = 60
RISE_FRAMES = 120
SUPPORT_SPEED = .3
SUPPORT_FRAMES = 30
ATTEMPT_TIMEOUT = 600

def wrap(angle):
    return (angle+np.pi)%(2*np.pi)-np.pi


class GroundRecovery:
    def __init__(self, reposition=False):
        self.reposition = reposition
        self.pending_legs = []
        self.moving_leg = None
        self.reposition_count = 0
        self.state='waiting'
        self.calm_frames=0
        self.support_frames=0
        self.start_frame=None
        self.support_frame=None
        self.events=[]
        self.initial_angles={}
        self.last_positions=None
        self.speed=float('inf')
        self.max_torque_ratio=0.
        self.max_force_ratio=0.
        self.support_contacts=0
        self.com_margin=None
        self.failure_reason=None
        self.phase_frame=None
        self.ground_y=None

    def _motor(self,body,a,b,target,cap,gain):
        p,q,m=body.points,body.prev_points,body.masses
        d=p[b]-p[a];length=max(float(np.linalg.norm(d)),1e-6)
        direction=d/length; tangent=np.array([-direction[1],direction[0]])
        angle=np.arctan2(d[0],-d[1])
        omega=float(np.dot((p[b]-q[b])-(p[a]-q[a]),tangent))/length
        inertia=length**2/(1/m[a]+1/m[b])
        torque=float(np.clip(inertia*(-.5*wrap(angle-target)-.9*omega),-cap,cap))*gain
        impulse=tangent*torque/length
        q[b]-=impulse/m[b];q[a]+=impulse/m[a]
        self.max_torque_ratio=max(self.max_torque_ratio,abs(torque)/cap)

    def _extend(self,body,a,b,target,cap,gain):
        p,q,m=body.points,body.prev_points,body.masses
        d=p[b]-p[a];length=max(float(np.linalg.norm(d)),1e-6);axis=d/length
        rate=float(np.dot((p[b]-q[b])-(p[a]-q[a]),axis))
        reduced=1/(1/m[a]+1/m[b])
        force=float(np.clip(reduced*(.4*(target-length)-.8*rate),-cap,cap))*gain
        impulse=axis*force
        q[b]-=impulse/m[b];q[a]+=impulse/m[a]
        self.max_force_ratio=max(self.max_force_ratio,abs(force)/cap)

    def pairs(self,sim):
        hip,sh,head=[sim.idx[k] for k in ('hip','shoulder','head')]
        pairs=[(hip,sh,110.,480.),(sh,head,55.,100.)]
        for knee,foot in sim.fallen_legs.values():
            pairs.extend([(knee,hip,30.,480.),(knee,foot,-86.,120.)])
        for elbow,hand in sim.arms.idx.values():
            pairs.append((hand,sh,-10.,300.))
        return pairs

    def drive(self,sim):
        if self.state not in ('repositioning','rising','supported'):return
        elapsed=sim.frame-self.phase_frame
        duration=240 if self.state=='repositioning' else RISE_FRAMES
        t=float(np.clip(elapsed/duration,0,1)); blend=t*t*(3-2*t)
        gain=min(1.,elapsed/30.)
        for a,b,degrees,cap in self.pairs(sim):
            initial=self.initial_angles[a,b]
            if self.reposition and self.state in ('rising','supported') and b==sim.idx['hip']:
                # Lower a hovering knee by bringing its thigh closer to vertical.
                gap=max(0.,self.ground_y-sim.body.points[a,1]-12.)
                degrees-=min(25.,gap*15.)
            if self.state=='repositioning':
                if (a,b)==(sim.idx['hip'],sim.idx['shoulder']):degrees=65.
                if (a,b)==(sim.idx['shoulder'],sim.idx['head']):degrees=15.
                for knee,foot in sim.fallen_legs.values():
                    if (a,b) in ((knee,sim.idx['hip']),(knee,foot)) and knee!=self.moving_leg:
                        degrees=np.degrees(initial)
            delta=wrap(np.radians(degrees)-initial)
            if self.state=='repositioning' and a==self.moving_leg:
                # Swing the leg through the upper half-plane. The short arc
                # would drive a straight, forward leg through the ground.
                delta=np.radians(degrees)-initial
                while delta>0:delta-=2*np.pi
            if a==sim.idx['hip'] and b==sim.idx['shoulder'] and initial<0:
                # A left-leaning torso must turn through the upper half-plane,
                # not take the shorter angular path through the ground.
                delta=np.radians(degrees)-initial
            target=initial+blend*delta
            self._motor(sim.body,a,b,target,cap,gain)
        for elbow,hand in sim.arms.idx.values():
            self._extend(sim.body,hand,sim.idx['shoulder'],60.,16.,gain)

    def _enter(self,sim,state):
        self.state=state;self.phase_frame=sim.frame
        if self.start_frame is None:self.start_frame=sim.frame
        for a,b,_,_ in self.pairs(sim):
            d=sim.body.points[b]-sim.body.points[a]
            self.initial_angles[a,b]=float(np.arctan2(d[0],-d[1]))
        self.events.append((sim.frame,state))

    def observe(self,sim,ground_y):
        if not sim.collapsed or sim.arms is None or sim.spine is None:return
        self.ground_y=ground_y
        p=sim.body.points
        ids=[i for i in range(len(p)) if i not in sim.body.pinned]
        self.speed=(float(np.linalg.norm(p[ids]-self.last_positions,axis=1).max())
                    if self.last_positions is not None else float('inf'))
        self.last_positions=p[ids].copy()
        if self.state=='waiting':
            calm=(self.speed<CALM_SPEED and sim.frame-sim.collapse_frame>=MIN_FALL_AGE and
                  (sim.bracing is None or not sim.bracing.active))
            self.calm_frames=self.calm_frames+1 if calm else 0
            if self.calm_frames>=CALM_FRAMES:
                if self.reposition:
                    self.pending_legs=[k for k,f in sim.fallen_legs.values() if p[f,0]>p[k,0]]
                    if self.pending_legs:
                        self.moving_leg=self.pending_legs.pop(0)
                        self.reposition_count+=1
                        self._enter(sim,'repositioning')
                    else:self._enter(sim,'rising')
                else:
                    self._start_legacy(sim,p)
        elif (self.state=='repositioning' and sim.frame-self.phase_frame>=240
              and self.speed<CALM_SPEED and all(p[f,0]<p[k,0] for k,f in sim.fallen_legs.values() if k==self.moving_leg)):
            if self.pending_legs:
                self.moving_leg=self.pending_legs.pop(0)
                self.reposition_count+=1
                self._enter(sim,'repositioning')
            else:
                self.moving_leg=None
                self._enter(sim,'rising')
        if self.state=='repositioning' and sim.frame-self.phase_frame>=ATTEMPT_TIMEOUT:
            self.state='failed';self.failure_reason='leg_placement_timeout'
            self.events.append((sim.frame,'failed'))
        self._observe_support(sim,ground_y,p,ids)

    def _start_legacy(self,sim,p):
        legs_forward=np.mean([p[f,0]-p[k,0] for k,f in sim.fallen_legs.values()])>0
        if legs_forward:
            self.state='needs_roll';self.failure_reason='legs_forward'
            self.events.append((sim.frame,'needs_roll'))
        else:
            self._enter(sim,'rising')

    def _observe_support(self,sim,ground_y,p,ids):
        knees=[k for k,_ in sim.fallen_legs.values()]
        hands=[h for _,h in sim.arms.idx.values()]
        contact_ids=set()
        for frame,point,projection in reversed(sim.ground_projection_impulses):
            if frame != sim.frame:break
            if projection > 1e-8:contact_ids.add(point)
        supports=[i for i in knees+hands if i in contact_ids and
                  ground_y-p[i,1] <= (12. if i in knees else 5.)+1.]
        self.support_contacts=len(supports)
        com=float(np.average(p[ids,0],weights=sim.body.masses[ids]))
        self.com_margin=(min(com-min(p[supports,0]),max(p[supports,0])-com) if supports else None)
        lifted=ground_y-p[sim.idx['hip'],1]>45 and ground_y-p[sim.idx['shoulder'],1]>40
        stable=(lifted and self.support_contacts==4 and self.com_margin>=0 and self.speed<SUPPORT_SPEED)
        if self.state in ('rising','supported'):
            self.support_frames=self.support_frames+1 if stable else 0
            if self.support_frames>=SUPPORT_FRAMES and self.state!='supported':
                self.state='supported';self.support_frame=sim.frame
                self.events.append((sim.frame,'supported'))
            elif self.state=='supported' and not stable:
                self.state='rising';self.events.append((sim.frame,'support_lost'))
            if self.state=='rising' and sim.frame-(self.phase_frame if self.reposition else self.start_frame)>=ATTEMPT_TIMEOUT:
                self.state='failed';self.failure_reason='support_timeout'
                self.events.append((sim.frame,'failed'))
