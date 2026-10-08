"""First get-up stage: settle, push onto hands and knees, hold for review."""
from pathlib import Path
import argparse
import json
import subprocess
import sys
import tempfile
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from demo.step14_active_biped import ActiveBipedSim,GROUND_Y
from demo.step17_bilge_physics_skin import simulate,PhysicsBilgeRig,render,SCREEN_GROUND_Y
from demo.step30_bracing import skeleton
from demo.step33_balance_head import KICK_PER_MS
from scene.export import render_video,find_ffmpeg

LABELS={'passive':'ONCE / YERDE BEKLEME','recovery':'ADIM 34 / EL-DIZ DESTEGI'}
STATES={'waiting':'DURULMAYI BEKLIYOR','rising':'DESTEK ALIYOR','supported':'DESTEK SABIT',
        'needs_roll':'ONCE DONME GEREKIYOR','failed':'DENEME SONLANDI','off':'PASIF','repositioning':'BACAK YERLESTIRILIYOR',
        'foot_placing':'AYAK YERLESTIRILIYOR','transferring':'AGIRLIK AKTARILIYOR',
        'half_kneeling':'AYAK DESTEGI SABIT',
        'torso_raising':'GOVDE YUKSELIYOR','upright_kneeling':'ELLER SERBEST / DENGEDE',
        'standing_rising':'AYAGA KALKIYOR','standing':'AYAKTA / DENGEDE',
        'walk_prepare':'YURUYUSE HAZIRLANIYOR','walk_shift':'AGIRLIK AKTARIYOR',
        'walk_swing':'ADIM ATIYOR','walk_land':'AYAGA BASIYOR'}


def measure(frames,sim):
    r=sim.recovery
    tail=frames[-60:]
    pts=lambda f: np.array([f[k] for k in ('hip','shoulder','head','waist')]+
                           [p for leg in f['legs'].values() for p in leg['chain'][1:]]+
                           [p for arm in f['arms'].values() for p in arm])
    settled=[f for f in frames if 'waist' in f]
    tail_points=np.array([pts(f) for f in tail]) if 'waist' in tail[0] else None
    start=r.start_frame if r else None
    active=[f for f in frames if start is not None and f['frame']>=start]
    active_points=np.array([pts(f) for f in active]) if active else None
    return dict(collapse_frame=sim.collapse_frame,
                state=r.state if r else 'off',start_frame=start,
                support_frame=r.support_frame if r else None,
                events=r.events if r else [],reason=r.failure_reason if r else None,
                support_contacts=r.support_contacts if r else None,
                com_margin_px=float(r.com_margin) if r and r.com_margin is not None else None,
                hip_height_px=float(GROUND_Y-frames[-1]['hip'][1]),
                chest_height_px=float(GROUND_Y-frames[-1]['shoulder'][1]),
                head_height_px=float(GROUND_Y-frames[-1]['head'][1]),
                tail_max_speed_px_frame=float(np.linalg.norm(np.diff(tail_points,axis=0),axis=2).max()) if tail_points is not None else None,
                max_recovery_speed_px_frame=float(np.linalg.norm(np.diff(active_points,axis=0),axis=2).max()) if active_points is not None and len(active)>1 else None,
                min_supported_contacts=min((f['recovery_contacts'] for f in tail),default=0) if r else None,
                tail_all_supported=all(f.get('recovery_state')=='supported' for f in tail),
                torque_limit_ratio=r.max_torque_ratio if r else 0.,
                force_limit_ratio=r.max_force_ratio if r else 0.,
                gravity_values=sorted(set(f['gravity_y'] for f in frames)),
                max_penetration_px=max(0.,max((pts(f)[:,1].max()-GROUND_Y for f in settled),default=0.)),
                finite=not sim.nan)


def run_case(push=150.,phase=0,enabled=True,count=1000):
    frames,sim=simulate(count,big_push_kick_px=push,big_push_t=7+phase/30,ground_recovery=enabled)
    report=measure(frames,sim)
    report.update(push_px=push,phase=phase)
    return report,frames,sim


def video(frames,out,labels=None,duration=16,caption=None,preview_frame=550,start_frame=180):
    labels=LABELS if labels is None else labels
    caption=caption or 'Ilk asama: durulma > eller ve dizlerle destek > bekleme'
    rigs={name:PhysicsBilgeRig() for name in frames}
    def frame_at(t):
        f=start_frame+round(t*30)
        panels=[]
        for name in labels:
            snap=frames[name][f];rig=rigs[name]
            pose=rig.pose(snap)
            dressed=render(pose,rig)
            x0=int(np.clip(round(pose['points']['pelvis'][0])-360,0,dressed.shape[1]-720))
            y0=round(SCREEN_GROUND_Y)-380
            dressed=dressed[y0:y0+405,x0:x0+720]
            dressed=cv2.resize(dressed,(640,360),interpolation=cv2.INTER_AREA)
            bones=skeleton(snap)
            state=snap.get('recovery_state','off')
            cv2.putText(bones,STATES[state],(20,110),cv2.FONT_HERSHEY_SIMPLEX,.55,(130,240,150),1,cv2.LINE_AA)
            if name=='recovery':
                cv2.putText(bones,f"Destek temasi: {snap['recovery_contacts']}/{2 if state in ('torso_raising','upright_kneeling','standing_rising','standing','walk_prepare','walk_shift','walk_swing','walk_land') else 4}",(20,140),0,.55,(230,230,230),1,cv2.LINE_AA)
            if 'transfer_foot_share' in snap and state in ('foot_placing','transferring','half_kneeling'):
                cv2.putText(bones,f"Ayak temas payi (gosterge): %{100*snap['transfer_foot_share']:.1f}",(20,170),0,.48,(230,230,230),1,cv2.LINE_AA)
                cv2.putText(bones,f"Kutle merkezi - ayak: {snap['transfer_com_to_foot']:+.1f} px",(20,195),0,.48,(230,230,230),1,cv2.LINE_AA)
            if state in ('torso_raising','upright_kneeling'):
                cv2.putText(bones,f"Kol temasi: {snap['rise_arm_contacts']} | El acikligi: {snap['rise_hand_clearance']:.1f} px",(20,170),0,.48,(230,230,230),1,cv2.LINE_AA)
                margin=snap['rise_margin']
                cv2.putText(bones,f"Destek payi: {margin:.1f} px" if margin is not None else 'Destek yok',(20,195),0,.48,(230,230,230),1,cv2.LINE_AA)
            if state in ('standing_rising','standing'):
                cv2.putText(bones,f"Diger temas: {snap['stand_other_contacts']} | Diz acikligi: {snap['stand_knee_clearance']:.1f} px",(20,170),0,.48,(230,230,230),1,cv2.LINE_AA)
                margin=snap['stand_margin']
                cv2.putText(bones,f"Iki ayak denge payi: {margin:.1f} px" if margin is not None else 'Ayak destegi yok',(20,195),0,.48,(230,230,230),1,cv2.LINE_AA)
            if state in ('walk_prepare','walk_shift','walk_swing','walk_land'):
                cv2.putText(bones,f"Tamamlanan adim: {snap['walk_steps']} | Ilerleme: {snap['walk_distance']:.1f} px",(20,170),0,.48,(230,230,230),1,cv2.LINE_AA)
                cv2.putText(bones,f"Diger zemin temasi: {snap['stand_other_contacts']} | Ayak acikligi: {snap['walk_clearance']:.1f} px",(20,195),0,.48,(230,230,230),1,cv2.LINE_AA)
            panels.append(np.vstack((dressed,bones)))
        canvas=np.full((820,1280,3),(29,25,23),np.uint8)
        canvas[50:770]=np.hstack(panels)
        for x,title in zip((20,660),labels.values()):
            cv2.putText(canvas,title,(x,33),0,.65,(240,240,240),2,cv2.LINE_AA)
        cv2.putText(canvas,caption,(25,802),0,.65,(240,240,240),1,cv2.LINE_AA)
        if f==preview_frame:cv2.imwrite(str(out.with_suffix('.png')),canvas)
        return canvas
    with tempfile.TemporaryDirectory(prefix='support-',dir=out.parent) as tmp:
        raw=Path(tmp)/'raw.mp4'
        render_video(frame_at,raw,size=(1280,820),fps=30,duration=duration)
        ffmpeg=find_ffmpeg()
        if not ffmpeg:raise RuntimeError('ffmpeg required for H.264 preview')
        subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-y','-i',str(raw),
                        '-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],check=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep',action='store_true')
    parser.add_argument('--no-video',action='store_true')
    args=parser.parse_args()
    report,frames={},{}
    for name,enabled in [('passive',False),('recovery',True)]:
        report[name],frames[name],_=run_case(enabled=enabled)
    report['notes']=[
        'Opt-in ground_recovery=True; normal walking and passive historical demos stay unchanged.',
        'This stage stops on hands and knees. It does not stand up or restart walking.',
        'Legs lying forward require a separate roll/reposition stage, reported as needs_roll; no recovery motors run there.',
        'World-reference PD motor pairs conserve linear momentum but are not a full angular-momentum-conserving muscle model.',
        'All motors are force/torque limited. No points are pinned or directly relocated by recovery.',
        'Support requires both hands and knees in actual ground-contact projections, raised hip/chest, COM within support, and 30 quiet frames.',
        'Contact projections are numerical constraint diagnostics, not measured ground forces.',
    ]
    if args.sweep:
        rows=[]
        for push in [2*KICK_PER_MS,-2*KICK_PER_MS,2.5*KICK_PER_MS,-2.5*KICK_PER_MS,150.,-150.]:
            for phase in range(0,30,5):
                row,_,_=run_case(push,phase);rows.append(row)
            print('Finished push',push,flush=True)
        report['sweep']=rows
        report['summary']=dict(cases=len(rows),states={state:sum(r['state']==state for r in rows) for state in STATES},
                               all_finite=all(r['finite'] for r in rows),
                               max_penetration_px=max(r['max_penetration_px'] for r in rows),
                               max_torque_ratio=max(r['torque_limit_ratio'] for r in rows),
                               max_force_ratio=max(r['force_limit_ratio'] for r in rows),
                               all_supported_hold=all(r['tail_all_supported'] for r in rows if r['state']=='supported'))
        print(json.dumps(report['summary'],indent=2),flush=True)
    out=ROOT/'outputs';out.mkdir(exist_ok=True)
    data=json.dumps(report,indent=2)+'\n'
    (out/'step34_ground_support_report.json').write_text(data)
    (ROOT/'docs/validation/step34_ground_support_report.json').write_text(data)
    if not args.no_video:video(frames,out/'step34_ground_support_comparison.mp4')


if __name__=='__main__':main()
