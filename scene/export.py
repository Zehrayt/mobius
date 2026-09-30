"""Checked video writing and optional ffmpeg audio muxing."""
from pathlib import Path
import math
import os
import shutil
import subprocess
import tempfile
import warnings
import cv2


def find_ffmpeg():
    """Prefer PATH; allow a project-local installation without global changes."""
    system = shutil.which('ffmpeg')
    if system:
        return system
    local = Path(__file__).resolve().parents[1]/'.venv/bin/ffmpeg'
    return str(local) if local.is_file() and os.access(local, os.X_OK) else None


def mux_audio(silent_path, audio_path, output_path, duration, audio_start=0.0):
    """Preserve silent output on missing ffmpeg, missing audio or mux failure."""
    silent_path, audio_path, output_path = map(Path, (silent_path, audio_path, output_path))
    ffmpeg = find_ffmpeg()
    if not audio_path.is_file():
        print(f'Ses dosyasi yok: {audio_path}. Sessiz MP4 uretildi.')
        shutil.copyfile(silent_path, output_path)
        return False
    if ffmpeg is None:
        print('ffmpeg bulunamadi. Sessiz MP4 uretildi; ses eklemek icin ffmpeg kurun.')
        shutil.copyfile(silent_path, output_path)
        return False
    cmd = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-i', str(silent_path),
           '-ss', str(audio_start), '-i', str(audio_path), '-map', '0:v:0', '-map', '1:a:0',
           '-c:v', 'copy', '-c:a', 'aac', '-af', 'apad', '-t', str(duration),
           '-movflags', '+faststart', str(output_path)]
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True, timeout=120)
        print(f'Ses eklendi: {audio_start:.2f}–{audio_start+duration:.2f} saniye.')
        return True
    except (OSError, subprocess.SubprocessError) as exc:
        detail = getattr(exc, 'stderr', None) or str(exc)
        warnings.warn(f'Ses birlestirilemedi; sessiz MP4 korunuyor. {detail}', RuntimeWarning)
        shutil.copyfile(silent_path, output_path)
        return False


def render_video(render_frame, output_path, *, size=(1280, 720), fps=30,
                 duration=8.0, audio_path=None, audio_start=0.0):
    if not all(math.isfinite(v) for v in (fps, duration, audio_start)) or fps <= 0 or duration <= 0 or audio_start < 0:
        raise ValueError('FPS/duration must be positive; audio_start nonnegative')
    frames = round(fps*duration)
    if frames < 1 or not math.isclose(frames/fps, duration, abs_tol=1e-9):
        raise ValueError('Duration must contain a whole number of frames')
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='mobius-', dir=output_path.parent) as tmp:
        silent = Path(tmp)/'silent.mp4'
        writer = cv2.VideoWriter(str(silent), cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        if not writer.isOpened():
            writer.release()
            raise RuntimeError('OpenCV MP4 writer could not open (mp4v codec)')
        try:
            for i in range(frames):
                frame = render_frame(i/fps)
                if frame.shape != (size[1], size[0], 3) or str(frame.dtype) != 'uint8':
                    raise ValueError('Renderer must return a uint8 BGR frame matching size')
                writer.write(frame)
        finally:
            writer.release()
        capture = cv2.VideoCapture(str(silent))
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        ok, _ = capture.read()
        capture.release()
        if not ok or count != frames:
            raise RuntimeError(f'Video validation failed: {count}/{frames} frames')
        staged = Path(tmp)/'final.mp4'
        if audio_path is None:
            shutil.copyfile(silent, staged)
        else:
            mux_audio(silent, audio_path, staged, duration, audio_start)
        staged.replace(output_path)
    print(f'Video: {output_path} ({size[0]}x{size[1]}, {fps:g} FPS, {duration:g} sn, {frames} kare)')
    return output_path
