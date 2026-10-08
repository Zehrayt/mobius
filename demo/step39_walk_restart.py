"""Measured slow walking restart after full standing recovery."""
from pathlib import Path
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from demo.step17_bilge_physics_skin import simulate
from demo.step34_ground_support import measure, video
from demo.step33_balance_head import KICK_PER_MS
from physics.walk_recovery import WALK_STATES


def run_case(push=150., phase=0, enabled=True, count=3700):
    frames, sim = simulate(count, big_push_kick_px=push, big_push_t=7+phase/30,
        ground_recovery=True, recovery_reposition=True, recovery_transfer=True,
        recovery_rise=True, recovery_stand=True, recovery_walk=enabled)
    report = measure(frames, sim)
    report.update(push_px=push, phase=phase, frames=len(frames))
    if enabled:
        r = sim.recovery
        active = [f for f in frames if r.walk_start_frame is not None and f['frame'] >= r.walk_start_frame]
        hips = np.array([f['hip'] for f in active])
        steps = r.completed_steps
        report.update(walk_start_frame=r.walk_start_frame, steps=steps, completed_steps=len(steps),
            alternating=all(a['side'] != b['side'] for a,b in zip(steps,steps[1:])),
            distance_px=float(hips[-1,0]-hips[0,0]) if len(hips) else 0.,
            min_foot_contacts=min((f['stand_contacts'] for f in active),default=0),
            max_other_contacts=max((f['stand_other_contacts'] for f in active),default=0),
            min_hip_height_px=float(330-hips[:,1].max()) if len(hips) else None,
            max_hip_delta_px=float(np.linalg.norm(np.diff(hips,axis=0),axis=1).max()) if len(hips)>1 else None,
            max_vertical_hip_delta_px=float(np.abs(np.diff(hips[:,1])).max()) if len(hips)>1 else None,
            max_stance_travel_px=max((s['stance_travel_px'] for s in steps),default=None),
            min_step_clearance_px=min((s['clearance_px'] for s in steps),default=None),
            max_abs_torso_tilt_deg=max((abs(f['rise_torso_tilt']) for f in active),default=None),
            max_walk_force_ratio=r.max_walk_force_ratio,
            tail_all_walking=all(f['recovery_state'] in WALK_STATES for f in frames[-60:]))
        report['passed'] = bool(report['finite'] and report['max_penetration_px']==0 and
            report['tail_all_walking'] and len(steps)>=3 and report['alternating'] and
            report['min_foot_contacts']>=1 and report['max_other_contacts']==0 and
            report['distance_px']>120 and report['min_hip_height_px']>150 and
            report['max_hip_delta_px']<3 and report['max_stance_travel_px']<1 and
            report['min_step_clearance_px']>=8 and report['max_walk_force_ratio']<=1)
    return report, frames, sim


def sweep_case(case):
    row, _, _ = run_case(*case)
    print(row['push_px'],row['phase'],row['state'],row['completed_steps'],row['passed'],flush=True)
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep',action='store_true')
    parser.add_argument('--no-video',action='store_true')
    args=parser.parse_args()
    report,frames={},{}
    for name,enabled in [('previous',False),('recovery',True)]:
        report[name],frames[name],_=run_case(enabled=enabled)
    report['notes']=[
        'Opt-in recovery_walk requires recovery_stand and all previous stages; old defaults remain unchanged.',
        '180-frame force cross-fade after 60 standing frames; 120-frame load transfer, 120-frame swing, at least 80-frame landing.',
        'Slow alternating physical steps, not the former pinned-anchor IK gait or normal walking speed.',
        'Hip-foot vector impulses conserve linear momentum, but are simplified world-reference servos, not an angular-momentum-conserving muscle model.',
        'No point-position, mass, pin, gravity or topology reset; historical collapsed/fell flags retain the explicit-body solver.',
        'Step completion requires measured clearance, forward travel and both feet loaded for five frames; timeout or loss of support fails the attempt.',
        'Projection contact shares are numerical diagnostics, not physical body-weight percentages.',
        'Existing earlier torso-raise COM margin deficit is unchanged. Walking balance is dynamic; single-foot COM interval margin is not a static success criterion.',
        '36 original push/phase cases; 3700 frames each; video simulation seconds 58-100.'
    ]
    if args.sweep:
        cases=[(push,phase) for push in (2*KICK_PER_MS,-2*KICK_PER_MS,2.5*KICK_PER_MS,-2.5*KICK_PER_MS,150.,-150.) for phase in range(0,30,5)]
        with ProcessPoolExecutor(max_workers=3) as pool:
            rows=list(pool.map(sweep_case,cases))
        report['sweep']=rows
        report['summary']=dict(cases=len(rows),passed=sum(r['passed'] for r in rows),
            min_completed_steps=min(r['completed_steps'] for r in rows),
            min_distance_px=min(r['distance_px'] for r in rows),
            max_stance_travel_px=max(r['max_stance_travel_px'] or 0 for r in rows),
            max_hip_delta_px=max(r['max_hip_delta_px'] or 0 for r in rows),
            max_vertical_hip_delta_px=max(r['max_vertical_hip_delta_px'] or 0 for r in rows),
            min_step_clearance_px=min(r['min_step_clearance_px'] or 0 for r in rows),
            all_finite=all(r['finite'] for r in rows),
            max_penetration_px=max(r['max_penetration_px'] for r in rows))
        print(json.dumps(report['summary'],indent=2),flush=True)
    out=ROOT/'outputs';out.mkdir(exist_ok=True)
    data=json.dumps(report,indent=2)+'\n'
    (out/'step39_walk_restart_report.json').write_text(data)
    (ROOT/'docs/validation/step39_walk_restart_report.json').write_text(data)
    if not args.no_video:
        video(frames,out/'step39_walk_restart_comparison.mp4',
            labels={'previous':'ADIM 38 / AYAKTA BEKLEME','recovery':'ADIM 39 / YAVAS YURUYUSE DONUS'},
            duration=42,start_frame=1740,preview_frame=2180,
            caption='Ayakta durus > agirlik aktarimi > ilk adim > donusumlu yavas yuruyus')


if __name__=='__main__':main()
