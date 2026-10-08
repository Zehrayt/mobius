"""Move one foot under the body, transfer load and hold with hands/rear knee."""
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


def run_case(push=150., phase=0, enabled=True, count=1800):
    frames, sim = simulate(count, big_push_kick_px=push, big_push_t=7+phase/30,
                          ground_recovery=True, recovery_reposition=True, recovery_transfer=enabled)
    report = measure(frames, sim)
    report.update(push_px=push, phase=phase, frames=count)
    if enabled:
        r = sim.recovery
        tail = frames[-60:]
        feet = np.array([f['legs']['left']['chain'][2] for f in tail])
        report.update(placement_frame=r.placement_frame, transfer_frame=r.transfer_frame,
                      half_kneel_frame=r.half_kneel_frame, foot_share_before=r.before_share,
                      foot_share_placed=r.placed_share, foot_share_final=r.foot_share,
                      com_to_foot_px=r.com_to_foot, front_knee_clearance_px=r.front_knee_clearance,
                      tail_all_half_kneeling=all(f['recovery_state']=='half_kneeling' for f in tail),
                      min_transfer_contacts=min(f['transfer_contacts'] for f in tail),
                      min_tail_foot_share=min(f['transfer_foot_share'] for f in tail),
                      tail_foot_travel_px=float(np.abs(np.diff(feet[:, 0])).sum()),
                      max_tail_com_to_foot_px=max(abs(f['transfer_com_to_foot']) for f in tail))
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
        'Opt-in recovery_transfer=True requires ground_recovery=True and recovery_reposition=True.',
        'Waits 60 frames in confirmed four-point support, then moves the left foot under the body.',
        'Placement and transfer require both hands, rear knee and front foot in actual ground contact, raised front knee and COM inside support.',
        'Closed-chain hip-foot extension applies equal/opposite impulses, capped at 2 solver force units.',
        'Contact projection shares are numerical solver diagnostics, NOT measured force or percentage of body weight.',
        'No new pins, teleport, change of gravity or walking restart. Stops in a hands-assisted half kneel.',
        'Same 36 pushes/phases as step35; 1800 frames (60 seconds) to include transfer and hold.',
    ]
    if args.sweep:
        cases = [(push, phase) for push in (2*KICK_PER_MS, -2*KICK_PER_MS, 2.5*KICK_PER_MS, -2.5*KICK_PER_MS, 150., -150.) for phase in range(0,30,5)]
        with ProcessPoolExecutor(max_workers=3) as pool:
            rows = list(pool.map(sweep_case, cases))
        report['sweep'] = rows
        report['summary'] = dict(cases=len(rows),
            states={state:sum(r['state']==state for r in rows) for state in sorted(set(r['state'] for r in rows))},
            all_finite=all(r['finite'] for r in rows),
            max_penetration_px=max(r['max_penetration_px'] for r in rows),
            max_torque_ratio=max(r['torque_limit_ratio'] for r in rows),
            max_force_ratio=max(r['force_limit_ratio'] for r in rows),
            all_half_kneeling_hold=all(r['tail_all_half_kneeling'] for r in rows),
            min_tail_foot_share=min(r['min_tail_foot_share'] for r in rows),
            max_tail_speed_px_frame=max(r['tail_max_speed_px_frame'] for r in rows),
            max_tail_foot_travel_px=max(r['tail_foot_travel_px'] for r in rows))
        print(json.dumps(report['summary'], indent=2), flush=True)
    out = ROOT/'outputs';out.mkdir(exist_ok=True)
    data = json.dumps(report, indent=2)+'\n'
    (out/'step36_foot_transfer_report.json').write_text(data)
    (ROOT/'docs/validation/step36_foot_transfer_report.json').write_text(data)
    if not args.no_video:
        video(frames, out/'step36_foot_transfer_comparison.mp4',
              labels={'previous':'ADIM 35 / EL-DIZ DESTEGI', 'recovery':'ADIM 36 / AYAGA YUK AKTARMA'},
              duration=34, caption='El-diz destegi > ayagi yerlestirme > kontrollu yuk aktarma > bekleme',
              preview_frame=1150)


if __name__ == '__main__':
    main()
