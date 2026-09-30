"""Bilge's isolated 30 Hz skeleton/foot-planting validation (no sprite assets).

Run: python3 demo/bilge_walk_validation.py

Reuses physics.gait.FootPlantingLeg for ALL foot trajectories and
physics.fabrik.FabrikChain2D for legs and arms. A smooth pelvis driver,
as in step3/step4, supplies travel; planted feet remain in world space.
This is a kinematic gait test, not a dynamic balance simulation. No
step15/step16 machinery is imported. Configuration is in WalkConfig.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from physics.fabrik import FabrikChain2D
from physics.gait import FootPlantingLeg
from scene.export import render_video


@dataclass(frozen=True)
class WalkConfig:
    width: int = 1280
    height: int = 720
    fps: int = 30  # FootPlantingLeg is frame based: this test runs at 30 Hz.
    duration: float = 8.0
    ground_y: float = 565.0
    start_x: float = 350.0
    speed: float = 90.0
    start_time: float = 1.0
    acceleration_time: float = 0.65
    brake_time: float = 5.2
    stop_time: float = 6.0
    standing_height: float = 182.0
    walking_crouch: float = 4.0
    thigh: float = 93.0
    shin: float = 94.0
    hip_half_width: float = 6.0
    rest_foot_offset: float = 12.0
    torso_length: float = 86.0
    shoulder_half_width: float = 14.0
    head_offset: float = 43.0
    head_radius: int = 32
    upper_arm: float = 54.0
    forearm: float = 50.0
    stride_release: float = 18.0
    stride_ahead: float = 35.0
    swing_frames: int = 10
    foot_lift: float = 30.0
    ik_tolerance: float = 1e-9
    ik_iterations: int = 256


def smooth(u: float) -> float:
    u = float(np.clip(u, 0.0, 1.0))
    return u * u * (3.0 - 2.0 * u)


def ramp_integral(t: float, start: float, duration: float) -> float:
    """Integral of smoothstep speed; constant speed after the ramp."""
    elapsed = max(0.0, t - start)
    u = min(1.0, elapsed / duration)
    return duration * (u**3 - 0.5 * u**4) + max(0.0, elapsed - duration)


def pelvis_at(t: float, c: WalkConfig) -> tuple[np.ndarray, float]:
    drive = smooth((t - c.start_time) / c.acceleration_time)
    brake = smooth((t - c.brake_time) / (c.stop_time - c.brake_time))
    activity = drive - brake
    distance = c.speed * (
        ramp_integral(t, c.start_time, c.acceleration_time)
        - ramp_integral(t, c.brake_time, c.stop_time - c.brake_time)
    )
    return np.array([c.start_x + distance,
                     c.ground_y - c.standing_height + c.walking_crouch * activity]), activity


def seed_chain(chain: FabrikChain2D, target: np.ndarray, forward: float) -> None:
    """Select the forward knee/elbow branch once; FABRIK handles all motion."""
    chain.points[1] = chain.base + [forward, chain.lengths[0] * 0.8]
    chain.points[2] = target + [0.0, -1.0]  # ensure first solve actually iterates


def simulate(c: WalkConfig = WalkConfig()) -> list[dict]:
    pelvis, _ = pelvis_at(0, c)
    legs, arms = {}, {}
    signs = {"left": -1, "right": 1}
    for side, sign in signs.items():
        hip = pelvis + [sign * c.hip_half_width, 0.0]
        leg = FootPlantingLeg(
            hip.copy(), [c.thigh, c.shin], c.ground_y,
            stride_release=c.stride_release, stride_ahead=c.stride_ahead,
            swing_duration_frames=c.swing_frames, lift_height=c.foot_lift,
            initial_planted_offset=sign * c.rest_foot_offset,
            knee_limits=None,
        )
        # The legacy clamp-then-pin path can change the last bone's length.
        # Instead use a reachable, continuously bent branch and converge IK.
        # Do not overwrite its endpoint or change physics.gait for other demos.
        leg.chain.margin_of_error = c.ik_tolerance
        seed_chain(leg.chain, leg.planted, 40.0)
        leg.chain.solve(leg.planted, max_iterations=c.ik_iterations)
        legs[side] = leg
        shoulder = pelvis + [sign * c.shoulder_half_width, -c.torso_length]
        arm = FabrikChain2D(shoulder.copy(), [c.upper_arm, c.forearm], c.ik_tolerance)
        seed_chain(arm, shoulder + [0, 94], 25.0)
        arms[side] = arm

    frames = []
    next_side = "left"
    arm_swing = 0.0
    final_pelvis, _ = pelvis_at(c.stop_time, c)
    for i in range(round(c.duration * c.fps)):
        t = i / c.fps
        pelvis, activity = pelvis_at(t, c)
        old_states = {side: leg.state for side, leg in legs.items()}
        airborne = any(state == "swing" for state in old_states.values())
        # At rest, finish the current swing and bring each foot under its hip.
        # Reuse the existing explicitly targeted step API at NORMAL speed.
        # The API's name is historical: here it is a settling step, not a fall.
        if t >= c.stop_time and not airborne:
            for side, sign in signs.items():
                target_x = pelvis[0] + sign * (c.hip_half_width + c.rest_foot_offset)
                if abs(legs[side].planted[0] - target_x) > 1.0:
                    legs[side].trigger_emergency_step(target_x, speedup=1.0)
                    airborne = True
                    break

        points = {"pelvis": pelvis.copy()}
        states, targets, phases = {}, {}, {}
        # Eligibility is snapshotted: iteration order cannot launch two feet.
        eligible = next_side if not airborne and c.start_time <= t < c.stop_time else None
        for side, sign in signs.items():
            hip = pelvis + [sign * c.hip_half_width, 0.0]
            leg = legs[side]
            final_foot_x = final_pelvis[0] + sign * (c.hip_half_width + c.rest_foot_offset)
            # Shorten only NEW steps when braking. An airborne foot retains
            # its original target, so there is no mid-swing trajectory jump.
            if t >= c.brake_time:
                leg.stride_ahead = min(c.stride_ahead, final_foot_x - hip[0])
            already_home = t >= c.brake_time and abs(leg.planted[0] - final_foot_x) < 1.0
            foot = leg.update(hip.copy(), hold_release=(side != eligible or already_home))
            leg.chain.solve(foot, max_iterations=c.ik_iterations)
            if old_states[side] == "stance" and leg.state == "swing":
                next_side = "right" if side == "left" else "left"
            points[f"{side}_hip"], points[f"{side}_knee"], points[f"{side}_foot"] = leg.chain.points.copy()
            states[side] = leg.state
            targets[side] = foot.copy()
            phases[side] = leg.swing_t if leg.state == "swing" else None

        # Opposite arm follows actual leg separation, not an independent clock.
        separation = points["right_foot"][0] - points["left_foot"][0]
        envelope = smooth((t - c.start_time) / c.acceleration_time) * (1 - smooth(t - c.stop_time))
        desired_swing = float(np.clip(0.62 * separation, -40, 40)) * envelope
        arm_swing += 0.35 * (desired_swing - arm_swing)
        lean = 3.0 * activity
        torso_dy = np.sqrt(c.torso_length**2 - lean**2)
        points["chest"] = pelvis + [lean, -torso_dy]
        points["head"] = points["chest"] + [0.0, -c.head_offset]
        for side, sign in signs.items():
            shoulder = points["chest"] + [sign * c.shoulder_half_width, 0.0]
            hand_x = -sign * arm_swing
            target = shoulder + [hand_x, 94.0 - abs(hand_x) * 0.1]
            arm = arms[side]
            arm.set_base(shoulder.copy())
            arm.solve(target, max_iterations=c.ik_iterations)
            points[f"{side}_shoulder"], points[f"{side}_elbow"], points[f"{side}_hand"] = arm.points.copy()
        frames.append(dict(time=t, points=points, states=states, targets=targets, swing_phase=phases))
    return frames


def measure(frames: list[dict], c: WalkConfig) -> dict:
    """Measure the SOLVED/rendered joints, never substitute commanded targets."""
    slip, penetration, bone_error, target_error, overreach = 0., 0., 0., 0., 0.
    knee_angles, joint_steps = [], []
    stance_origins = {}
    steps = {"left": 0, "right": 0}
    max_lift = {"left": 0., "right": 0.}
    double_swing = 0
    for i, frame in enumerate(frames):
        p = frame["points"]
        double_swing += all(s == "swing" for s in frame["states"].values())
        for side in steps:
            foot, hip = p[f"{side}_foot"], p[f"{side}_hip"]
            stance = frame["states"][side] == "stance"
            if stance:
                if side not in stance_origins:
                    stance_origins[side] = float(foot[0])
                slip = max(slip, abs(float(foot[0]) - stance_origins[side]))
            else:
                if i == 0 or frames[i-1]["states"][side] == "stance":
                    steps[side] += 1
                stance_origins.pop(side, None)
            penetration = max(penetration, float(foot[1] - c.ground_y))
            max_lift[side] = max(max_lift[side], float(c.ground_y - foot[1]))
            target_error = max(target_error, float(np.linalg.norm(foot - frame["targets"][side])))
            overreach = max(overreach, float(np.linalg.norm(frame["targets"][side] - hip) - c.thigh - c.shin))
            for names, lengths in ((('hip', 'knee', 'foot'), (c.thigh, c.shin)),
                                   (('shoulder', 'elbow', 'hand'), (c.upper_arm, c.forearm))):
                a, b, d = [p[f"{side}_{name}"] for name in names]
                bone_error = max(bone_error, abs(float(np.linalg.norm(b-a))-lengths[0]),
                                 abs(float(np.linalg.norm(d-b))-lengths[1]))
            thigh = p[f"{side}_knee"] - hip
            shin = foot - p[f"{side}_knee"]
            knee_angles.append(float(np.degrees(np.arctan2(thigh[0]*shin[1]-thigh[1]*shin[0], np.dot(thigh, shin)))))
        if i:
            joint_steps.append(max(float(np.linalg.norm(pos - frames[i-1]["points"][name])) for name, pos in p.items()))
    all_points = np.array([list(f["points"].values()) for f in frames])
    boundary_jumps = {}
    for t in (c.start_time, c.stop_time, 7.0):
        i = round(t*c.fps)
        boundary_jumps[str(t)] = max(joint_steps[max(0, i-2):i+1])
    return dict(
        resolution=[c.width, c.height], fps=c.fps, frames=len(frames), duration_s=len(frames)/c.fps,
        max_stance_horizontal_slip_px=slip, max_ground_penetration_px=penetration,
        max_bone_length_error_px=bone_error, max_foot_target_error_px=target_error,
        max_target_overreach_px=overreach, knee_bend_deg=[min(knee_angles), max(knee_angles)],
        steps_including_settling=steps, max_foot_lift_px=max_lift, double_swing_frames=int(double_swing),
        max_joint_displacement_per_frame_px=max(joint_steps), transition_displacements_px=boundary_jumps,
        all_joints_finite=bool(np.isfinite(all_points).all()),
        all_joints_in_frame=bool((all_points[..., 0] > c.head_radius).all()
                                 and (all_points[..., 0] < c.width-c.head_radius).all()
                                 and (all_points[..., 1] > c.head_radius).all()
                                 and (all_points[..., 1] < c.height).all()),
        measurement='World-space solved joints before rasterization; stance measured relative to each touchdown. '
                    'Feet are point contacts (no shoe/sole geometry).',
    )


def validate(report: dict) -> None:
    for key in ('max_stance_horizontal_slip_px', 'max_ground_penetration_px',
                'max_bone_length_error_px', 'max_foot_target_error_px', 'max_target_overreach_px'):
        if report[key] > 1e-5:
            raise RuntimeError(f'{key}: {report[key]:.9f}')
    lo, hi = report['knee_bend_deg']
    if not 3 < lo < hi < 140:
        raise RuntimeError(f'Invalid knee branch/range: {lo}, {hi}')
    if report['double_swing_frames'] or not report['all_joints_in_frame'] or not report['all_joints_finite']:
        raise RuntimeError('Support or framing validation failed')
    if min(report['steps_including_settling'].values()) < 2:
        raise RuntimeError('Both legs must take repeated steps')


# OpenCV colors are BGR. Both limbs remain visible in a slight oblique view.
COLORS = {'left': (211, 195, 63), 'right': (103, 151, 245)}
INK = (226, 233, 243)


def render_frame(frame: dict, c: WalkConfig) -> np.ndarray:
    image = np.full((c.height, c.width, 3), (34, 28, 24), np.uint8)
    floor = round(c.ground_y)
    image[floor+1:] = (41, 35, 30)
    cv2.line(image, (65, floor), (c.width-65, floor), (118, 113, 104), 2, cv2.LINE_AA)
    for x in range(100, c.width-60, 50):
        cv2.line(image, (x, floor+10), (x, floor+16), (85, 80, 73), 1)
    p = frame['points']
    xy = lambda point: tuple(np.rint(point).astype(int))

    def line(a, b, color, width=4):
        cv2.line(image, xy(p[a]), xy(p[b]), color, width, cv2.LINE_AA)

    for side in ('right', 'left'):
        color = COLORS[side]
        for a, b in (('hip', 'knee'), ('knee', 'foot'), ('shoulder', 'elbow'), ('elbow', 'hand')):
            line(f'{side}_{a}', f'{side}_{b}', color)
    for a, b in (('pelvis', 'chest'), ('chest', 'head'),
                 ('left_hip', 'right_hip'), ('left_shoulder', 'right_shoulder')):
        line(a, b, INK)
    for name, pos in p.items():
        if name == 'head':
            continue
        color = COLORS.get(name.split('_')[0], INK)
        cv2.circle(image, xy(pos), 5, color, -1, cv2.LINE_AA)
        cv2.circle(image, xy(pos), 2, (45, 40, 35), -1, cv2.LINE_AA)
    cv2.circle(image, xy(p['head']), c.head_radius, INK, 3, cv2.LINE_AA)
    # Small orientation marker, still purely skeletal (no face or clothing).
    h = xy(p['head'])
    cv2.line(image, (h[0]+c.head_radius-2, h[1]), (h[0]+c.head_radius+7, h[1]), INK, 3, cv2.LINE_AA)
    for side in ('left', 'right'):
        x, y = xy(p[f'{side}_foot'])
        cv2.line(image, (x-7, y-2), (x+10, y-2), COLORS[side], 3, cv2.LINE_AA)
        if frame['states'][side] == 'stance':
            cv2.line(image, (x-11, floor+7), (x+11, floor+7), COLORS[side], 2, cv2.LINE_AA)
    t = frame['time']
    phase = 'BEKLEME' if t < c.start_time else 'YURUYUS' if t < c.stop_time else 'DURUS / BEKLEME'
    cv2.putText(image, 'BILGE / ISKELET YURUYUS TESTI', (65, 62), cv2.FONT_HERSHEY_SIMPLEX, .85, INK, 2, cv2.LINE_AA)
    cv2.putText(image, f'{t:04.2f} s   {phase}', (65, 98), cv2.FONT_HERSHEY_SIMPLEX, .65, (169, 169, 166), 1, cv2.LINE_AA)
    for side, label, x in (('left', 'SOL', 65), ('right', 'SAG', 350)):
        state = 'BASMA' if frame['states'][side] == 'stance' else 'SALINIM'
        cv2.putText(image, f'{label}: {state}', (x, 636), cv2.FONT_HERSHEY_SIMPLEX, .65, COLORS[side], 2, cv2.LINE_AA)
    cv2.putText(image, 'Sabit kamera  |  Dunya koordinatlari  |  30 FPS', (65, 679), cv2.FONT_HERSHEY_SIMPLEX, .5, (156, 153, 147), 1, cv2.LINE_AA)
    return image


def preview_indices(frames: list[dict], c: WalkConfig) -> list[int]:
    result = [round(.5*c.fps)]
    for side in ('left', 'right'):
        candidates = [i for i, f in enumerate(frames)
                      if 2 < f['time'] < 5 and f['swing_phase'][side] is not None]
        result.append(min(candidates, key=lambda i: abs(frames[i]['swing_phase'][side]-.5)))
    return result + [len(frames)-1]


def main() -> None:
    c = WalkConfig()
    frames = simulate(c)
    report = measure(frames, c)
    validate(report)
    out = ROOT / 'outputs'
    out.mkdir(exist_ok=True)
    path = out / 'bilge_walk_validation.mp4'
    render_video(lambda t: render_frame(frames[round(t*c.fps)], c), path,
                 size=(c.width, c.height), fps=c.fps, duration=c.duration)
    # Build the preview from DECODED video frames, and verify every frame.
    indices = preview_indices(frames, c)
    decoded = {}
    cap = cv2.VideoCapture(str(path))
    count = 0
    try:
        while True:
            ok, image = cap.read()
            if not ok:
                break
            if image.shape != (c.height, c.width, 3):
                raise RuntimeError('Encoded frame has wrong dimensions')
            if count in indices:
                decoded[count] = image
            count += 1
    finally:
        cap.release()
    if count != len(frames):
        raise RuntimeError(f'Decoded only {count}/{len(frames)} frames')
    montage = np.zeros((c.height, c.width, 3), np.uint8)
    labels = ('BEKLEME', 'SOL ADIM', 'SAG ADIM', 'SON DURUS')
    for panel, (idx, label) in enumerate(zip(indices, labels)):
        tile = cv2.resize(decoded[idx], (c.width//2, c.height//2), interpolation=cv2.INTER_AREA)
        cv2.rectangle(tile, (0, 0), (c.width//2, 55), (34, 28, 24), -1)
        cv2.putText(tile, f'{label} / {frames[idx]["time"]:.2f} s', (18, 21), cv2.FONT_HERSHEY_SIMPLEX, .55, INK, 1, cv2.LINE_AA)
        y, x = (panel//2)*(c.height//2), (panel%2)*(c.width//2)
        montage[y:y+c.height//2, x:x+c.width//2] = tile
    preview = out / 'bilge_walk_validation_preview.png'
    if not cv2.imwrite(str(preview), montage):
        raise RuntimeError('Could not write preview')
    report['decoded_video_frames'] = count
    report['preview_frame_indices'] = indices
    report_path = out / 'bilge_walk_validation_report.json'
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f'Preview: {preview}')


if __name__ == '__main__':
    main()
