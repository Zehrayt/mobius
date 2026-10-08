"""Place forward legs behind the body and close hovering knee contacts."""
from pathlib import Path
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from demo.step17_bilge_physics_skin import simulate
from demo.step34_ground_support import measure,video
from demo.step33_balance_head import KICK_PER_MS


def run_case(push=-150.,phase=0,enabled=True,count=1500):
    frames,sim=simulate(count,big_push_kick_px=push,big_push_t=7+phase/30,
                        ground_recovery=True,recovery_reposition=enabled)
    report=measure(frames,sim)
    report.update(push_px=push,phase=phase,frames=count,reposition_count=sim.recovery.reposition_count)
    return report,frames,sim


def sweep_case(case):
    push,phase=case
    row,_,_=run_case(push,phase)
    print(push,phase,row['state'],flush=True)
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
        'Opt-in recovery_reposition=True, requires ground_recovery=True.',
        'Sequential upper-arc leg motion; next leg waits for rearward shin placement and low speed.',
        'Hovering knees receive a bounded thigh-angle correction. Support still requires four actual contacts.',
        'No teleport, extra pins or gravity changes. World-referenced bounded motors are not a full muscle model.',
        'Each placement and rise attempt has a 600-frame timeout. Stops at hands/knees, not standing.',
        'Same 36 pushes/phases as step34, extended observation from 1000 to 1500 frames to accommodate leg placement.',
    ]
    if args.sweep:
        cases=[(push,phase) for push in (2*KICK_PER_MS,-2*KICK_PER_MS,2.5*KICK_PER_MS,-2.5*KICK_PER_MS,150.,-150.) for phase in range(0,30,5)]
        with ProcessPoolExecutor(max_workers=3) as pool:rows=list(pool.map(sweep_case,cases))
        report['sweep']=rows
        report['summary']=dict(cases=len(rows),states={state:sum(r['state']==state for r in rows) for state in sorted(set(r['state'] for r in rows))},
            all_finite=all(r['finite'] for r in rows),max_penetration_px=max(r['max_penetration_px'] for r in rows),
            max_torque_ratio=max(r['torque_limit_ratio'] for r in rows),max_force_ratio=max(r['force_limit_ratio'] for r in rows),
            all_supported_hold=all(r['tail_all_supported'] for r in rows if r['state']=='supported'))
        print(json.dumps(report['summary'],indent=2),flush=True)
    out=ROOT/'outputs';out.mkdir(exist_ok=True)
    data=json.dumps(report,indent=2)+'\n'
    (out/'step35_support_placement_report.json').write_text(data)
    (ROOT/'docs/validation/step35_support_placement_report.json').write_text(data)
    if not args.no_video:
        video(frames,out/'step35_support_placement_comparison.mp4',
              labels={'previous':'ADIM 34 / DONUS BEKLIYOR','recovery':'ADIM 35 / BACAK YERLESTIRME'},
              duration=34,caption='Durulma > bacaklari sirayla arkaya alma > dort temasla destek',preview_frame=1150)


if __name__=='__main__':main()
