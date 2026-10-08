"""Close-up of the same simulated walk with face-centred / neck-root binding."""
from pathlib import Path
import json
import sys
import tempfile
import subprocess
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from demo.step17_bilge_physics_skin import simulate, PhysicsBilgeRig, render
from demo.bilge_walk_skinned import overlap_pixels
from scene.skinning import transform_point, bone_matrix
from scene.export import render_video, find_ffmpeg


def socket_matrix(rig, pose, matrices):
    sk,p=rig.skin,pose['points']
    if 'waist' not in p:return matrices['torso']
    a,b=sk.anchor('torso'),sk.anchor('torso','end')
    return bone_matrix(a,(a+b)/2,p['chest'],p['waist'],sk.scale('torso'))


def attachment_report(frames):
    report={}
    for name,attached in [('before',False),('after',True)]:
        rig=PhysicsBilgeRig(attached_head=attached)
        errors=[];overlaps=[];walking=[]
        for snap in frames:
            pose=rig.pose(snap);m=rig.matrices(pose);sk=rig.skin
            socket=transform_point(socket_matrix(rig,pose,m),sk.anchor('torso','neck_socket'))
            base=transform_point(m['head'],sk.anchor('head','neck_base'))
            error=float(np.linalg.norm(base-socket));errors.append(error)
            if not snap['collapsed']:walking.append(error)
            layers=rig.layers(pose)
            overlaps.append(overlap_pixels(layers['head'],layers['torso']))
        report[name]=dict(max_neck_socket_error_px=max(errors),
                          max_walking_socket_error_px=max(walking),
                          minimum_head_torso_overlap_pixels=min(overlaps))
    report['frames']=len(frames)
    report['notes']=['Both renderings use identical physical snapshots.',
                     'The head and its existing neck are rigidly attached at the neck root to the collar.',
                     'Articulated falls attach to the actual upper shirt segment; the braid follows the head.',
                     'Source image pixels and physical controllers are unchanged by this visual correction.']
    return report


def main():
    frames,_=simulate(360)
    out=ROOT/'outputs';out.mkdir(exist_ok=True)
    report=attachment_report(frames)
    data=json.dumps(report,indent=2)+'\n'
    (out/'neck_attachment_report.json').write_text(data)
    (ROOT/'docs/validation/neck_attachment_report.json').write_text(data)
    rigs=[PhysicsBilgeRig(attached_head=False),PhysicsBilgeRig()]
    def frame_at(t):
        n=round(t*30);panels=[]
        for rig,title in zip(rigs,['ONCE / YUZ MERKEZINDEN','SONRA / BOYUN KOKUNDEN']):
            pose=rig.pose(frames[n]);im=render(pose,rig)
            x,y=np.round(pose['points']['chest']).astype(int)
            crop=cv2.resize(im[y-115:y+85,x-100:x+100],(500,500))
            cv2.putText(crop,title,(12,30),cv2.FONT_HERSHEY_SIMPLEX,.58,(25,25,25),2,cv2.LINE_AA)
            panels.append(crop)
        canvas=np.hstack(panels)
        if n==120:cv2.imwrite(str(out/'neck_attachment_comparison.png'),canvas)
        return canvas
    target=out/'neck_attachment_comparison.mp4'
    with tempfile.TemporaryDirectory(prefix='neck-',dir=out) as tmp:
        raw=Path(tmp)/'raw.mp4'
        render_video(frame_at,raw,size=(1000,500),fps=30,duration=6)
        ffmpeg=find_ffmpeg()
        if not ffmpeg:raise RuntimeError('ffmpeg is required for the H.264 preview')
        subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-y','-i',str(raw),
                        '-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(target)],check=True)
    print(json.dumps(report,indent=2))
    print(target)


if __name__=='__main__':main()
