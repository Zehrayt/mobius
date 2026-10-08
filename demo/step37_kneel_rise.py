"""Raise the torso and release hand support, stopping on one foot and one knee."""
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


def run_case(push=150., phase=0, enabled=True, count=2100):
    frames, sim = simulate(count, big_push_kick_px=push, big_push_t=7+phase/30,
        ground_recovery=True, recovery_reposition=True, recovery_transfer=True, recovery_rise=enabled)
    report = measure(frames, sim)
    report.update(push_px=push, phase=phase, frames=count)
    if enabled:
        r = sim.recovery
        tail = frames[-60:]
        foot = np.array([f['legs']['left']['chain'][2] for f in tail])
        knee = np.array([f['legs']['right']['chain'][1] for f in tail])
        active = [f for f in frames if r.raise_frame is not None and f['frame'] >= r.raise_frame]
        free = [f for f in active if f['rise_arm_contacts']==0 and f['rise_hand_clearance']>5.]
        report.update(raise_frame=r.raise_frame, upright_frame=r.upright_frame,
            hip_height_before_raise=r.hip_height_before_raise,
            tail_all_upright=all(f['recovery_state']=='upright_kneeling' for f in tail),
            min_lower_contacts=min(f['rise_lower_contacts'] for f in tail),
            max_arm_contacts=max(f['rise_arm_contacts'] for f in tail),
            min_hand_clearance_px=min(f['rise_hand_clearance'] for f in tail),
            max_abs_torso_tilt_deg=max(abs(f['rise_torso_tilt']) for f in tail),
            min_lower_margin_px=min((f['rise_margin'] for f in tail if f['rise_margin'] is not None), default=None),
            min_unassisted_transition_margin_px=min((f['rise_margin'] for f in free if f['rise_margin'] is not None), default=None),
            foot_share_final=r.foot_share,
            tail_foot_travel_px=float(np.abs(np.diff(foot[:,0])).sum()),
            tail_knee_travel_px=float(np.abs(np.diff(knee[:,0])).sum()))
    return report, frames, sim


def sweep_case(case):
    row, _, _ = run_case(*case)
    print(row['push_px'], row['phase'], row['state'], flush=True)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep', action='store_true')
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    report, frames = {}, {}
    for name, enabled in [('previous', False), ('recovery', True)]:
        report[name], frames[name], _ = run_case(enabled=enabled)
    report['notes'] = [
        'Opt-in recovery_rise=True requires all earlier recovery stages.',
        'Keeps previous motor commands; prepares rear leg over 120 frames and raises torso over 240 frames after a 60-frame delay.',
        'Success requires front foot/rear knee contact, no hand/elbow contact, raised hands, upright torso and COM inside their support interval.',
        'Rear foot may touch passively; it is excluded from the two-point balance test.',
        'Entry needs 30 quiet frames. Contact/geometry loss revokes success immediately; excess motion must persist for 5 frames.',
        'Contact projections are numerical diagnostics, not physical force measurements.',
        'Same motor limits and gravity. No new pins, teleport or walking restart.',
        'Only the new raise/hold states use 128 rather than 64 coupled constraint iterations to reduce projection drift.',
        'Same 36 push/phase cases; 2100 frames (70 seconds). Stops in upright kneeling, not standing.',
    ]
    if args.sweep:
        cases = [(push,phase) for push in (2*KICK_PER_MS,-2*KICK_PER_MS,2.5*KICK_PER_MS,-2.5*KICK_PER_MS,150.,-150.) for phase in range(0,30,5)]
        with ProcessPoolExecutor(max_workers=3) as pool:
            rows = list(pool.map(sweep_case,cases))
        report['sweep'] = rows
        report['summary'] = dict(cases=len(rows),
            states={state:sum(r['state']==state for r in rows) for state in sorted(set(r['state'] for r in rows))},
            all_finite=all(r['finite'] for r in rows),all_upright_hold=all(r['tail_all_upright'] for r in rows),
            max_penetration_px=max(r['max_penetration_px'] for r in rows),
            max_torque_ratio=max(r['torque_limit_ratio'] for r in rows),max_force_ratio=max(r['force_limit_ratio'] for r in rows),
            max_tail_speed_px_frame=max(r['tail_max_speed_px_frame'] for r in rows),
            max_tail_foot_travel_px=max(r['tail_foot_travel_px'] for r in rows),
            max_tail_knee_travel_px=max(r['tail_knee_travel_px'] for r in rows))
        print(json.dumps(report['summary'],indent=2),flush=True)
    out = ROOT/'outputs';out.mkdir(exist_ok=True)
    data = json.dumps(report,indent=2)+'\n'
    (out/'step37_kneel_rise_report.json').write_text(data)
    (ROOT/'docs/validation/step37_kneel_rise_report.json').write_text(data)
    if not args.no_video:
        video(frames,out/'step37_kneel_rise_comparison.mp4',
            labels={'previous':'ADIM 36 / ELLER DESTEKTE','recovery':'ADIM 37 / GOVDEYI DOGRULTMA'},
            duration=42,caption='Bacak destegi > govdeyi yukseltme > elleri ayirma > ayak-diz dengesi',preview_frame=1400)


if __name__ == '__main__':main()
