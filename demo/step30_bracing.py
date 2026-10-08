"""Reproducible step-30 comparison: passive step 29 versus protective arms.

python3 demo/step30_bracing.py                 # report + side-by-side video
python3 demo/step30_bracing.py --sweep --no-video  # 36 paired push scenarios
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from demo import step14_active_biped as s14
from demo.step17_bilge_physics_skin import simulate, PhysicsBilgeRig, render
from scene.export import find_ffmpeg, render_video


def measure(frames, sim):
    """Use identical observation windows and definitions for both conditions."""
    start = sim.collapse_frame
    window_end = start + 45 if start is not None else 0
    contacts = [row for row in sim.ground_contacts if row[0] < window_end]
    def impact(name):
        events = [(f, v) for f, i, v in contacts if i == sim.idx[name]]
        return dict(first_frame=events[0][0] if events else None,
                    peak_input_speed_px_frame=max((v for _, v in events), default=0.0))
    hands = {h for _, h in sim.arms.idx.values()} if sim.arms else set()
    hand_events = [(f, v) for f, i, v in contacts if i in hands]
    projected = [row for row in sim.ground_projection_impulses if row[0] < window_end]
    total = sum(j for _, _, j in projected)
    hand_total = sum(j for _, i, j in projected if i in hands)
    tail = frames[-60:]
    def points(snap):
        rows = [snap[k] for k in ('hip', 'shoulder', 'head')]
        if 'waist' in snap:
            rows.append(snap['waist'])
        for leg in snap['legs'].values():
            rows.extend(leg['chain'][1:])
        for arm in snap.get('arms', {}).values():
            rows.extend(arm)
        return np.array(rows)
    tail_points = np.array([points(snap) for snap in tail])
    collapsed_points = [points(snap) for snap in frames if snap['collapsed']]
    return dict(collapse_frame=start, head=impact('head'), chest=impact('shoulder'),
                first_hand_contact_frame=hand_events[0][0] if hand_events else None,
                brace_start_frame=sim.bracing.start_frame if sim.bracing else None,
                brace_final_state=sim.bracing.state if sim.bracing else 'off',
                hand_ground_projection_share_pct=100.0 * hand_total / total if total else None,
                shoulder_actuator_upward_impulse=sim.bracing.support_impulse if sim.bracing else 0.0,
                tail_max_speed_px_frame=float(np.linalg.norm(np.diff(tail_points, axis=0), axis=2).max()),
                tail_hip_drift_px=float(abs(tail[-1]['hip'][0] - tail[0]['hip'][0])),
                max_ground_penetration_px=max(0.0, max((float(p[:, 1].max()) - s14.GROUND_Y
                                                      for p in collapsed_points), default=0.0)),
                finite=not sim.nan)


def compare(push=150.0, phase=0):
    results, snapshots = {}, {}
    for name, enabled in (('passive', False), ('bracing', True)):
        frames, sim = simulate(420, gravity_mode="legacy", bracing=enabled, articulated_spine=False, big_push_kick_px=push,
                               big_push_t=7.0 + phase / 30.0)
        results[name] = measure(frames, sim)
        snapshots[name] = frames
    results['push_px'] = push
    results['phase_offset_frames'] = phase
    return results, snapshots


def skeleton(snap):
    canvas = np.full((360, 640, 3), (29, 25, 23), dtype=np.uint8)
    offset = 310.0 - snap['hip'][0]
    def point(p):
        return round(p[0] + offset), round(p[1] - 25)
    def line(a, b, color, width=3):
        cv2.line(canvas, point(a), point(b), color, width, cv2.LINE_AA)
    cv2.line(canvas, (0, 305), (640, 305), (140, 135, 130), 2)
    if 'waist' in snap:
        line(snap['hip'], snap['waist'], (200, 200, 200), 4)
        line(snap['waist'], snap['shoulder'], (200, 200, 200), 4)
        cv2.circle(canvas, point(snap['waist']), 5, (70, 200, 255), -1, cv2.LINE_AA)
        cv2.putText(canvas, f"Bel acisi: {snap['spine_bend_deg']:+.1f} derece", (20, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (70, 200, 255), 1, cv2.LINE_AA)
    else:
        line(snap['hip'], snap['shoulder'], (200, 200, 200), 4)
    line(snap['shoulder'], snap['head'], (200, 200, 200))
    cv2.circle(canvas, point(snap['head']), 16, (200, 200, 200), 2, cv2.LINE_AA)
    for side, color in (('left', (140, 210, 110)), ('right', (240, 180, 90))):
        chain = snap['legs'][side]['chain']
        for a, b in zip(chain, chain[1:]):
            line(a, b, color, 4)
        e, h = snap['arms'][side[0]]
        line(snap['shoulder'], e, color)
        line(e, h, color)
        cv2.circle(canvas, point(h), 5, color, -1, cv2.LINE_AA)
    cv2.putText(canvas, f"{snap['time']:.2f} s | frame {snap['frame']} | {snap['bracing_state']}",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .55, (230, 230, 230), 1, cv2.LINE_AA)
    if 'gravity_y' in snap:
        cv2.putText(canvas, f"g: {snap['gravity_y']:.3f} px/kare^2", (20, 82),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (200, 200, 200), 1, cv2.LINE_AA)
    return canvas


def make_video(snapshots, report, labels, rig_options=None):
    rigs = {name: PhysicsBilgeRig(**(rig_options or {}).get(name, {})) for name in snapshots}
    cache = {}
    # Pose/camera updates must happen in temporal order, once per sim frame.
    for f in range(360):
        panels = []
        for name in labels:
            snap = snapshots[name][f]
            pose = rigs[name].pose(snap)
            dressed = render(pose, rigs[name], snapshots[name], False)
            dressed = cv2.resize(dressed, (640, 360), interpolation=cv2.INTER_AREA)
            panels.append(np.vstack((dressed, skeleton(snap))))
        canvas = np.full((820, 1280, 3), (29, 25, 23), dtype=np.uint8)
        canvas[50:770] = np.hstack(panels)
        for x, (name, title) in zip((20,660), labels.items()):
            cv2.putText(canvas, title, (x, 33), cv2.FONT_HERSHEY_SIMPLEX, .7, (240, 235, 230), 2, cv2.LINE_AA)
            m = report[name]
            caption = (f"Bas: {m['head']['peak_input_speed_px_frame']:.2f} | "
                       f"Gogus: {m['chest']['peak_input_speed_px_frame']:.2f} px/kare")
            cv2.putText(canvas, caption, (x, 800), cv2.FONT_HERSHEY_SIMPLEX, .52,
                        (225, 225, 225), 1, cv2.LINE_AA)
        cv2.line(canvas, (640, 0), (640, 820), (100, 100, 100), 1)
        # Only replay frames need to survive after the first render pass.
        if 210 <= f < 256:
            cache[f] = canvas.copy()
        yield f, canvas, cache


def write_video(snapshots, report, output, labels=None, rig_options=None):
    labels = labels or {'passive': 'ADIM 29 / PASIF', 'bracing': 'ADIM 30 / KORUYUCU KOLLAR'}
    frames = make_video(snapshots, report, labels, rig_options)
    cache = {}
    def frame_at(t):
        index = round(t * 30)
        if index < 360:
            _, canvas, replay = next(frames)
            if index == 359:
                cache.update(replay)
            return canvas
        source = 210 + (index - 360) // 3
        canvas = cache[source].copy()
        cv2.rectangle(canvas, (0, 770), (1280, 820), (29, 25, 23), -1)
        cv2.putText(canvas, 'YAVAS TEKRAR / 3x - Ust: karakter gorunumu | Alt: gercek fizik eklemleri',
                    (24, 802), cv2.FONT_HERSHEY_SIMPLEX, .65, (240, 235, 230), 1, cv2.LINE_AA)
        return canvas
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='bracing-', dir=output.parent) as tmp:
        raw = Path(tmp) / 'raw.mp4'
        render_video(frame_at, raw, size=(1280, 820), fps=30, duration=498/30)
        ffmpeg = find_ffmpeg()
        if ffmpeg:
            subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-i', str(raw),
                            '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p',
                            '-movflags', '+faststart', str(output)], check=True)
        else:
            raw.replace(output)
    cv2.imwrite(str(output.with_suffix('.png')), cache[232])
    print(f'Video: {output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep', action='store_true')
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    out = ROOT / 'outputs'
    out.mkdir(exist_ok=True)
    report, snapshots = compare()
    report['metric_notes'] = [
        'Impact speed is positive vertical solver input speed at a ground-contact frame, px/frame.',
        'Impact window: collapse frame through collapse+44; no contact is reported separately from zero speed.',
        'Ground projection share is sum(mass * upward PBD ground correction), dt=1. It is a solver proxy, not a measured shoulder impulse or injury metric.',
        'Passive restores step-29 arm constraints; bracing includes the mass-weighted fallen elbow solver (16 iterations). This compares complete implementations, not the actuator alone.',
        'The upper character rendering retains existing ground-clamped skin geometry. The lower view shows actual physical joints.',
    ]
    if args.sweep:
        kick_per_ms = 5.18 * 184.0 / 0.9 / 30.0
        report['sweep'] = []
        for push in (2*kick_per_ms, -2*kick_per_ms, 2.5*kick_per_ms, -2.5*kick_per_ms, 150.0, -150.0):
            for phase in range(0, 30, 5):
                row, _ = compare(push, phase)
                report['sweep'].append(row)
        collapsed = [row for row in report['sweep'] if row['bracing']['collapse_frame'] is not None]
        report['sweep_summary'] = dict(
            cases=36, collapsed=len(collapsed),
            head_improved=sum(row['bracing']['head']['peak_input_speed_px_frame'] < row['passive']['head']['peak_input_speed_px_frame'] for row in collapsed),
            chest_improved=sum(row['bracing']['chest']['peak_input_speed_px_frame'] < row['passive']['chest']['peak_input_speed_px_frame'] for row in collapsed),
            max_tail_speed=max(row['bracing']['tail_max_speed_px_frame'] for row in collapsed),
            max_tail_hip_drift=max(row['bracing']['tail_hip_drift_px'] for row in collapsed))
        print(json.dumps(report['sweep_summary'], indent=2))
    (out / 'step30_bracing_report.json').write_text(json.dumps(report, indent=2) + '\n')
    if not args.no_video:
        write_video(snapshots, report, out / 'step30_bracing_comparison.mp4')


if __name__ == '__main__':
    main()
