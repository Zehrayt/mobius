"""Step 32/33 comparison: bidirectional recovery and calibrated head posture."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from demo.step14_active_biped import ActiveBipedSim, GROUND_Y
from demo.step17_bilge_physics_skin import simulate, PhysicsBilgeRig, render
from demo.step30_bracing import measure, write_video
from physics.gravity import REAL_GRAVITY

KICK_PER_MS = 5.18 * 184 / .9 / 30
LABELS = {'before': 'ADIM 32 / ONCE', 'after': 'ADIM 33 / DENGE + BAS'}


def options(enabled):
    return dict(balance_recovery=enabled, upright_head=enabled)


def compare(push=KICK_PER_MS, phase=0):
    report, snapshots = {}, {}
    for name, enabled in (('before', False), ('after', True)):
        frames, sim = simulate(420, big_push_kick_px=push, big_push_t=7+phase/30, **options(enabled))
        report[name] = measure(frames, sim)
        report[name]['collapse_reason'] = sim.collapse_reason
        snapshots[name] = frames
    report.update(push_px=push, phase=phase)
    return report, snapshots


def run_case(enabled, push, phase, frames=900, ice=True):
    sim = ActiveBipedSim(big_push_kick_px=push, big_push_t=7+phase/30,
                         stumble_kick_px=0 if push == 0 else None, **options(enabled))
    if not ice:
        sim.terrain_static.zones = []
        sim.terrain_kinetic.zones = []
    angles, gravity, penetration = [], set(), 0.
    positions = []
    minimum_height = float('inf')
    for f in range(frames):
        sim.step()
        p = sim.body.points
        gravity.add(float(sim.body.gravity[1]))
        ids = [i for i in range(len(p)) if i not in sim.body.pinned]
        penetration = max(penetration, float(p[ids, 1].max()-GROUND_Y))
        minimum_height = min(minimum_height, GROUND_Y-p[sim.idx['hip'], 1])
        if f >= frames-60:
            positions.append(p[ids].copy())
        if 30 <= f < 210:
            a, b = ('head', 'shoulder') if enabled else ('shoulder', 'hip')
            d = p[sim.idx[a]]-p[sim.idx[b]]
            # Source eye-line angle was added to torso rotation in step 32.
            angles.append(float(np.degrees(np.arctan2(d[0], -d[1]))) +
                          (0 if enabled else float(np.degrees(np.arctan2(42, 110)))))
    tail = np.asarray(positions)
    return dict(push_px=push, phase=phase, frames=frames, ice=ice,
                collapsed=sim.collapsed, collapse_frame=sim.collapse_frame, fell=sim.fell,
                finite=not sim.nan, gravity_values=sorted(gravity),
                max_penetration_px=penetration,
                mean_tail_speed=float(np.mean(sim.hip_vx_log[-300:])),
                steps=len(sim.step_events), min_hip_height=float(minimum_height),
                mean_eye_tilt_deg=float(np.mean(angles)),
                max_abs_eye_tilt_deg=float(np.max(np.abs(angles))),
                tail_point_speed=float(np.linalg.norm(np.diff(tail, axis=0), axis=2).max()),
                tail_hip_drift=float(abs(tail[-1, 0, 0]-tail[0, 0, 0])))


def sweep(pushes, frames):
    rows = []
    for push in pushes:
        for phase in range(0, 30, 5):
            row = {name: run_case(enabled, push, phase, frames)
                   for name, enabled in (('before', False), ('after', True))}
            rows.append(row)
        print(f'Finished push {push:.3f}', flush=True)
    return rows


def summary(rows):
    result = {}
    for name in LABELS:
        cases = [r[name] for r in rows]
        fallen = [r for r in cases if r['collapsed']]
        result[name] = dict(cases=len(cases), collapsed=len(fallen),
                            finite=all(r['finite'] for r in cases),
                            constant_gravity=all(r['gravity_values'] == [REAL_GRAVITY] for r in cases),
                            max_penetration_px=max(r['max_penetration_px'] for r in cases),
                            max_fallen_tail_speed=max((r['tail_point_speed'] for r in fallen), default=0),
                            max_fallen_tail_drift=max((r['tail_hip_drift'] for r in fallen), default=0))
    return result


def extended_settling(rows):
    """Keep the original 14 s comparison, then inspect slower settling cases."""
    results = []
    for row in rows:
        for name, enabled in (('before',False), ('after',True)):
            case = row[name]
            if case['collapsed'] and (case['tail_point_speed'] > .2 or case['tail_hip_drift'] > 1.):
                results.append(dict(condition=name, original=case,
                                    followups=[run_case(enabled,case['push_px'],case['phase'],n)
                                               for n in (480,900)]))
    return results


def head_preview(snapshots, path):
    panels = []
    for name in LABELS:
        rig = PhysicsBilgeRig(corrected_head=name == 'after')
        pose = rig.pose(snapshots[name][120])
        frame = render(pose, rig)
        x, y = np.round(pose['points']['head']).astype(int)
        crop = frame[max(0,y-85):y+125, x-115:x+115]
        crop = cv2.resize(crop, (460, 420))
        cv2.putText(crop, LABELS[name], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (30,30,30), 2)
        panels.append(crop)
    cv2.imwrite(str(path), np.hstack(panels))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep', action='store_true')
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    report, snapshots = compare()
    report['walking'] = {name: {surface: run_case(enabled, 0, 0, 1800, ice)
                                for surface, ice in (('dry',False), ('ice',True))}
                         for name, enabled in (('before',False), ('after',True))}
    report['notes'] = [
        'Both conditions use the same real gravity, leg torque limits and initial impulses.',
        'Impulse labels in m/s are nominal momentum conversion, not measured COM velocity changes.',
        'Step 33 senses the current impulse, permits backward capture steps and commits catch foot placement.',
        'Head source tilt is calibrated with eye landmarks. Upright-reference neck PD is disabled after collapse.',
        'Knee bend-side constraints are mass-weighted and solved together with ground contacts in step 33.',
        'Neck/torso PD and IK support remain simplified controllers, not a complete muscle simulation.',
        'Eye tilt is measured before the large push. Tail drift/speed describe settling only for collapsed cases.',
    ]
    if args.sweep:
        report['mild'] = sweep([v*KICK_PER_MS for v in (.25,-.25,.5,-.5,1.,-1.)],900)
        report['strong'] = sweep([2*KICK_PER_MS,-2*KICK_PER_MS,2.5*KICK_PER_MS,-2.5*KICK_PER_MS,150.,-150.],420)
        for key in ('mild','strong'):
            report[key+'_summary'] = summary(report[key])
            print(key, json.dumps(report[key+'_summary']), flush=True)
        report['extended_settling'] = extended_settling(report['strong'])
    out = ROOT/'outputs'
    out.mkdir(exist_ok=True)
    data = json.dumps(report, indent=2)+'\n'
    (out/'step33_balance_head_report.json').write_text(data)
    (ROOT/'docs/validation/step33_balance_head_report.json').write_text(data)
    head_preview(snapshots, out/'step33_head_comparison.png')
    if not args.no_video:
        write_video(snapshots, report, out/'step33_balance_head_comparison.mp4', LABELS,
                    rig_options={'before': {'corrected_head': False}})


if __name__ == '__main__':
    main()
