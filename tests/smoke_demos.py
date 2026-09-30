"""Run existing demos unchanged, redirecting MP4s into a temporary directory.

Usage: python3 tests/smoke_demos.py
Runs the complete simulations/render loops, including the step16 numeric lab.
"""
from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch
import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    original_writer = cv2.VideoWriter
    paths = sorted(ROOT.glob('demo/step*.py'), key=lambda p: int(p.stem.split('_')[0][4:]))
    with tempfile.TemporaryDirectory(prefix='mobius-demo-smoke-') as tmp:
        for path in paths:
            if path.name.startswith('step6_'):
                continue
            spec = importlib.util.spec_from_file_location(path.stem, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[path.stem] = module
            spec.loader.exec_module(module)
            written = []

            def writer(filename, *args, **kwargs):
                target = Path(tmp)/Path(filename).name
                written.append(target)
                result = original_writer(str(target), *args, **kwargs)
                assert result.isOpened(), target
                return result

            log = io.StringIO()
            with patch('cv2.VideoWriter', side_effect=writer), redirect_stdout(log):
                module.main()
            for target in written:
                capture = cv2.VideoCapture(str(target))
                count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
                expected = getattr(module, 'N_FRAMES', count)
                assert count == expected and count > 0, (path.name, count, expected)
                capture.set(cv2.CAP_PROP_POS_FRAMES, count-1)
                ok, _ = capture.read()
                capture.release()
                assert ok, target
            assert 'UYARI: NaN/inf' not in log.getvalue(), path.name
            print(f'PASS {path.name} ({len(written)} MP4)', flush=True)


if __name__ == '__main__':
    main()
