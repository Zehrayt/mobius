"""The skin must join at the neck root, including the articulated fall."""
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from demo.step17_bilge_physics_skin import simulate
from demo.neck_attachment_preview import attachment_report


class NeckAttachmentTest(unittest.TestCase):
    def test_neck_root_stays_in_collar_during_walk_and_articulated_fall(self):
        frames,_=simulate(360)
        self.assertTrue(any('waist' in frame for frame in frames))
        report=attachment_report(frames)
        self.assertGreater(report['before']['max_walking_socket_error_px'],10.)
        self.assertLess(report['after']['max_neck_socket_error_px'],1e-9)
        self.assertGreaterEqual(report['after']['minimum_head_torso_overlap_pixels'],5)


if __name__=='__main__':unittest.main()
