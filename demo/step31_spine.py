"""Compare rigid step-30 torso with a fall-only articulated spine.

python3 demo/step31_spine.py --sweep
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from demo.step17_bilge_physics_skin import simulate
from demo.step30_bracing import measure, write_video


def compare(push=150., phase=0):
    report,snapshots = {},{}
    for name, enabled in (('rigid',False),('articulated',True)):
        fs,sim = simulate(420,gravity_mode="legacy",bracing=True,articulated_spine=enabled,
                          big_push_kick_px=push,big_push_t=7+phase/30)
        result = measure(fs,sim)
        bends = [f['spine_bend_deg'] for f in fs if 'waist' in f]
        lengths = [abs(np.linalg.norm(f[b]-f[a])-sim.spine.segment_length)
                   for f in fs if 'waist' in f
                   for a,b in (('hip','waist'),('waist','shoulder'))]
        result.update(spine_min_deg=min(bends,default=0),spine_max_deg=max(bends,default=0),
                      final_spine_deg=bends[-1] if bends else 0,
                      max_segment_error_px=max(lengths,default=0),
                      transition=sim.spine.transition if sim.spine else None)
        report[name],snapshots[name] = result,fs
    report.update(push_px=push,phase_offset_frames=phase)
    return report,snapshots


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep',action='store_true')
    parser.add_argument('--no-video',action='store_true')
    args=parser.parse_args()
    report,snapshots=compare()
    report['metric_notes']=[
        'Both conditions include step-30 protective arms; only the articulated spine path differs.',
        'The waist hinge and added segment use 64 fallen-solver iterations; rigid torso keeps 16.',
        'Contact metrics use the same 45-frame window and PBD diagnostics as step 30.',
        'Repartitioning torso mass preserves mass, COM and both momenta; rotational inertia and kinetic energy can change. Transition energy is recorded.',
        'Upper panels show split shirt skinning with existing visual floor correction; lower panels show actual physical joints.',
    ]
    if args.sweep:
        report['sweep']=[]
        kick=5.18*184/.9/30
        for push in (2*kick,-2*kick,2.5*kick,-2.5*kick,150.,-150.):
            for phase in range(0,30,5):
                row,_=compare(push,phase)
                report['sweep'].append(row)
        collapsed=[r for r in report['sweep'] if r['articulated']['collapse_frame'] is not None]
        report['sweep_summary']=dict(
            cases=36,collapsed=len(collapsed),
            max_tail_speed=max(r['articulated']['tail_max_speed_px_frame'] for r in collapsed),
            max_tail_hip_drift=max(r['articulated']['tail_hip_drift_px'] for r in collapsed),
            min_bend=min(r['articulated']['spine_min_deg'] for r in collapsed),
            max_bend=max(r['articulated']['spine_max_deg'] for r in collapsed),
            max_segment_error=max(r['articulated']['max_segment_error_px'] for r in collapsed),
            head_improved=sum(r['articulated']['head']['peak_input_speed_px_frame'] < r['rigid']['head']['peak_input_speed_px_frame'] for r in collapsed),
            chest_improved=sum(r['articulated']['chest']['peak_input_speed_px_frame'] < r['rigid']['chest']['peak_input_speed_px_frame'] for r in collapsed))
        print(json.dumps(report['sweep_summary'],indent=2),flush=True)
    out=ROOT/'outputs'
    out.mkdir(exist_ok=True)
    (out/'step31_spine_report.json').write_text(json.dumps(report,indent=2)+'\n')
    if not args.no_video:
        write_video(snapshots,report,out/'step31_spine_comparison.mp4',
                    labels={'rigid':'ADIM 30 / RIJIT GOVDE','articulated':'ADIM 31 / ESNEK OMURGA'})

if __name__=='__main__':
    main()
