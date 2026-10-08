"""Recover from upright kneeling to a measured two-foot standing hold."""
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


def run_case(push=150., phase=0, enabled=True, count=2700):
    frames, sim = simulate(count, big_push_kick_px=push, big_push_t=7+phase/30,
        ground_recovery=True, recovery_reposition=True, recovery_transfer=True,
        recovery_rise=True, recovery_stand=enabled)
    report = measure(frames, sim)
    report.update(push_px=push, phase=phase, frames=count)
    earlier = [f['rise_margin'] for f in frames
        if f['recovery_state'] in ('torso_raising','upright_kneeling') and
        f['rise_arm_contacts']==0 and f['rise_hand_clearance']>5 and f['rise_margin'] is not None]
    report['earlier_raise_min_margin_px'] = min(earlier, default=None)
    if enabled:
        r = sim.recovery;tail = frames[-60:]
        active = [f for f in frames if r.stand_start_frame is not None and f['frame']>=r.stand_start_frame]
        two_feet = [f for f in active if f['stand_contacts']==2 and f['stand_other_contacts']==0]
        travels = {}
        for side in ('left','right'):
            feet = np.array([f['legs'][side]['chain'][2] for f in tail])
            travels[side] = float(np.abs(np.diff(feet[:,0])).sum())
        report.update(stand_start_frame=r.stand_start_frame, standing_frame=r.standing_frame,
            tail_all_standing=all(f['recovery_state']=='standing' for f in tail),
            min_foot_contacts=min(f['stand_contacts'] for f in tail),
            max_other_contacts=max(f['stand_other_contacts'] for f in tail),
            min_knee_clearance_px=min(f['stand_knee_clearance'] for f in tail),
            min_standing_margin_px=min((f['stand_margin'] for f in tail if f['stand_margin'] is not None), default=None),
            min_stand_transition_support_margin_px=min((f['stand_all_ground_margin'] for f in active if f['stand_all_ground_margin'] is not None), default=None),
            min_two_foot_transition_margin_px=min((f['stand_margin'] for f in two_feet if f['stand_margin'] is not None), default=None),
            min_foot_share=min(f['stand_min_foot_share'] for f in tail),
            foot_width_px=r.stand_width,tail_foot_travel_px=travels,
            max_abs_torso_tilt_deg=max(abs(f['rise_torso_tilt']) for f in tail),
            max_stand_force_ratio=r.max_stand_force_ratio,stand_force_peak=r.stand_force_peak)
    return report, frames, sim


def sweep_case(case):
    row, _, _ = run_case(*case)
    print(row['push_px'],row['phase'],row['state'],flush=True)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep',action='store_true')
    parser.add_argument('--no-video',action='store_true')
    args = parser.parse_args()
    report, frames = {}, {}
    for name, enabled in [('previous',False),('recovery',True)]:
        report[name],frames[name],_=run_case(enabled=enabled)
    report['notes'] = [
        'Opt-in recovery_stand=True requires all earlier recovery stages.',
        'Waits 60 frames in upright kneeling, then extends legs over 300 frames, moving the rear foot forward along the floor.',
        'Axial hip-foot impulses include gravity feed-forward plus length/rate feedback, capped at 14 solver force units per leg.',
        'Success requires both feet loaded, no other ground support, raised knees, near-full hip height and COM inside the foot interval.',
        'Entry needs 30 quiet frames; geometric/contact loss revokes success immediately and excess motion must persist for 5 frames.',
        'No new pins, direct pose changes or gravity changes. The old walking anchor remains released; walking is not restarted.',
        'Contact-projection shares are solver diagnostics, not measured physical weight percentages.',
        'Earlier torso-raising transition retains its known transient COM margin deficit; this is reported separately.',
        'Same 36 push/phase cases, now 2700 frames (90 seconds). Video focuses on reference simulation seconds 38-66.',
    ]
    if args.sweep:
        cases=[(push,phase) for push in (2*KICK_PER_MS,-2*KICK_PER_MS,2.5*KICK_PER_MS,-2.5*KICK_PER_MS,150.,-150.) for phase in range(0,30,5)]
        with ProcessPoolExecutor(max_workers=3) as pool:
            rows=list(pool.map(sweep_case,cases))
        report['sweep']=rows
        report['summary']=dict(cases=len(rows),
            states={state:sum(r['state']==state for r in rows) for state in sorted(set(r['state'] for r in rows))},
            all_finite=all(r['finite'] for r in rows),all_standing_hold=all(r['tail_all_standing'] for r in rows),
            max_penetration_px=max(r['max_penetration_px'] for r in rows),
            max_torque_ratio=max(r['torque_limit_ratio'] for r in rows),max_force_ratio=max(r['force_limit_ratio'] for r in rows),
            max_stand_force_ratio=max(r['max_stand_force_ratio'] for r in rows),
            max_tail_speed_px_frame=max(r['tail_max_speed_px_frame'] for r in rows),
            max_tail_foot_travel_px=max(max(r['tail_foot_travel_px'].values()) for r in rows))
        print(json.dumps(report['summary'],indent=2),flush=True)
    out=ROOT/'outputs';out.mkdir(exist_ok=True)
    data=json.dumps(report,indent=2)+'\n'
    (out/'step38_standing_report.json').write_text(data)
    (ROOT/'docs/validation/step38_standing_report.json').write_text(data)
    if not args.no_video:
        video(frames,out/'step38_standing_comparison.mp4',
            labels={'previous':'ADIM 37 / AYAK-DIZ DESTEGI','recovery':'ADIM 38 / IKI AYAKTA DURUS'},
            duration=28,start_frame=1140,preview_frame=1900,
            caption='Diz destegini birakma > arka ayagi yaklastirma > iki ayak uzerinde dogrulma')


if __name__ == '__main__':main()
