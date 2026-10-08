"""Single-gravity migration: walking, force limits, catching and falling.

python3 demo/step32_gravity.py --sweep
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from demo.step14_active_biped import ActiveBipedSim, GROUND_Y, TARGET_VX
from demo.step17_bilge_physics_skin import simulate
from demo.step30_bracing import measure, write_video
from physics.gravity import REAL_GRAVITY


def compare(push=150.,phase=0):
    report,snapshots={},{}
    for mode in ('legacy','unified'):
        frames,sim=simulate(420,gravity_mode=mode,big_push_kick_px=push,big_push_t=7+phase/30,
                            balance_recovery=False,upright_head=False)
        r=measure(frames,sim)
        r.update(gravity_values=sorted(set(f['gravity_y'] for f in frames)),
                 collapse_reason=sim.collapse_reason,
                 step_count=len(sim.step_events),slip_frames=len(sim.slip_events),
                 mean_pre_push_speed=float(np.mean(sim.hip_vx_log[30:210])),
                 catch_gravity=sim.left_leg.gravity_y,leg_load_gravity=float(sim.leg_mass.g[1]),
                 muscle_gravity=sim.gravity_policy.muscle_load,
                 capture_frequency=sim.left_leg.omega0)
        report[mode],snapshots[mode]=r,frames
    report.update(push_px=push,phase_offset_frames=phase)
    return report,snapshots


def walk_report(mode='unified',frames=1800,ice=True):
    sim=ActiveBipedSim(gravity_mode=mode,stumble_kick_px=0,big_push_kick_px=0,
                       balance_recovery=False,upright_head=False)
    if not ice:
        sim.terrain_static.zones=[]
        sim.terrain_kinetic.zones=[]
    minimum_height=float('inf')
    phases=set()
    gravity=set()
    for _ in range(frames):
        sim.step()
        minimum_height=min(minimum_height,GROUND_Y-sim.body.points[sim.idx['hip'],1])
        gravity.add(float(sim.body.gravity[1]))
        phases.update(leg.contact_phase for leg in sim.legs)
    speeds=np.asarray(sim.hip_vx_log[frames//2:])
    return dict(frames=frames,seconds=frames/30,ice=ice,collapsed=sim.collapsed,fell=sim.fell,
                finite=not sim.nan,mean_speed=float(speeds.mean()),speed_std=float(speeds.std()),
                target_speed=TARGET_VX,steps=len(sim.step_events),slip_frames=len(sim.slip_events),
                min_hip_height=float(minimum_height),phases=sorted(phases),gravity_values=sorted(gravity))


def mild_push_report():
    rows=[]
    for dv in (.25,-.25,.5,-.5,1.,-1.):
        for phase in range(0,30,5):
            sim=ActiveBipedSim(big_push_kick_px=dv*5.18*184/.9/30,big_push_t=7+phase/30,
                               balance_recovery=False,upright_head=False)
            for _ in range(900):
                sim.step()
            rows.append(dict(push_ms=dv,phase=phase,collapsed=sim.collapsed,
                             collapse_frame=sim.collapse_frame,finite=not sim.nan))
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep',action='store_true')
    parser.add_argument('--no-video',action='store_true')
    args=parser.parse_args()
    report,snapshots=compare()
    report['walking']={mode:walk_report(mode) for mode in ('legacy','unified')}
    report['walking_dry_ground']=walk_report(ice=False)
    report['metric_notes']=[
        'Unified gravity is 9.81 m/s^2 = 2.2284444444444444 px/frame^2 at 184 px/0.9 m and 30 Hz.',
        'Legacy reproduces step 31: 0.065 walking, real-scale muscle load and falling; it is not the default.',
        'Both runs use protective arms and articulated spine. Gravity, capture margins and gravity-based traction differ.',
        'The support-height failure detector is enabled in unified mode to enter ragdoll before ground penetration, even without a catch landing.',
        'The engine remains a hybrid IK/Verlet model. Normal gait and support-length control are not a full force-driven musculoskeletal model.',
        'Force/velocity and impact diagnostic limits remain those documented in steps 30 and 31.',
    ]
    if args.sweep:
        report['sweep']=[]
        kick=5.18*184/.9/30
        for push in (2*kick,-2*kick,2.5*kick,-2.5*kick,150.,-150.):
            for phase in range(0,30,5):
                row,_=compare(push,phase)
                report['sweep'].append(row)
        collapsed=[r for r in report['sweep'] if r['unified']['collapse_frame'] is not None]
        report['sweep_summary']=dict(
            cases=len(report['sweep']),legacy_collapsed=sum(r['legacy']['collapse_frame'] is not None for r in report['sweep']),
            unified_collapsed=len(collapsed),
            all_finite=all(r['unified']['finite'] for r in report['sweep']),
            max_penetration=max(r['unified']['max_ground_penetration_px'] for r in report['sweep']),
            max_tail_speed=max((r['unified']['tail_max_speed_px_frame'] for r in collapsed),default=0),
            max_tail_hip_drift=max((r['unified']['tail_hip_drift_px'] for r in collapsed),default=0),
            gravity_constant=all(r['unified']['gravity_values']==[REAL_GRAVITY] for r in report['sweep']))
        print(json.dumps(report['sweep_summary'],indent=2),flush=True)
        report['mild_pushes']=mild_push_report()
    out=ROOT/'outputs';out.mkdir(exist_ok=True)
    (out/'step32_gravity_report.json').write_text(json.dumps(report,indent=2)+'\n')
    if not args.no_video:
        write_video(snapshots,report,out/'step32_gravity_comparison.mp4',
                    labels={'legacy':'ADIM 31 / IKI YERCEKIMI','unified':'ADIM 32 / TEK YERCEKIMI'})

if __name__=='__main__':main()
