"""Fall-only spine: transition invariants, hinge forces and full-body stability."""
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from demo import step14_active_biped as s14
from demo.step31_spine import compare
from demo.step17_bilge_physics_skin import PhysicsBilgeRig, SCREEN_GROUND_Y
from physics.spine import FallenSpine, cross, MIN_BEND, MAX_BEND
from physics.verlet import VerletSystem


def invariants(body):
    p,v,m=body.points,body.points-body.prev_points,body.masses
    return (m.sum(),np.sum(m[:,None]*p,axis=0),np.sum(m[:,None]*v,axis=0),
            np.sum(m*cross(p,v)))


class SpineUnitTest(unittest.TestCase):
    def make_body(self):
        b=VerletSystem.empty()
        h=b.add_point([11.,71.],mass=1.3)
        s=b.add_point([14.,16.],mass=.9)
        b.add_stick(h,s,length=55.)
        b.prev_points[h]-=[4.,7.]
        b.prev_points[s]-=[-2.,5.]
        return b,h,s

    def test_split_preserves_mass_positions_com_and_both_momenta(self):
        b,h,s=self.make_body()
        before=invariants(b)
        p=b.points.copy()
        spine=FallenSpine(b,h,s)
        np.testing.assert_array_equal(b.points[:2],p)
        np.testing.assert_array_equal(b.points[spine.waist],p.mean(axis=0))
        for a,z in zip(before,invariants(b)):
            np.testing.assert_allclose(a,z,atol=1e-10)
        self.assertEqual(len(b.sticks),2)
        self.assertNotIn((h,s),[(i,j) for i,j,*_ in b.sticks])

    def test_spring_is_internal_and_damps_bending(self):
        b,h,s=self.make_body()
        spine=FallenSpine(b,h,s)
        b.points[spine.waist]+=[8.,0.]
        b.prev_points=b.points.copy()
        before=invariants(b)
        positions=b.points.copy()
        theta,_=spine.geometry()
        spine.drive()
        _,gradient=spine.geometry()
        omega=np.sum(gradient*(b.points-b.prev_points)[spine.ids])
        self.assertLess(theta*omega,0)
        np.testing.assert_array_equal(b.points,positions)
        after=invariants(b)
        np.testing.assert_allclose(before[2],after[2],atol=1e-10)
        self.assertAlmostEqual(before[3],after[3],places=9)

    def test_hinge_projection_preserves_com_and_enforces_limits(self):
        b,h,s=self.make_body()
        spine=FallenSpine(b,h,s)
        b.points[s]=b.points[spine.waist]+[-20.,20.]
        before=invariants(b)[1]
        for _ in range(60):spine.constrain()
        np.testing.assert_allclose(before,invariants(b)[1],atol=1e-10)
        angle,_=spine.geometry()
        self.assertGreaterEqual(angle,MIN_BEND-1e-6)
        self.assertLessEqual(angle,MAX_BEND+1e-6)


class SpineIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report,cls.frames=compare()

    def test_no_waist_until_collapse_and_identical_walking(self):
        rigid,flex=self.frames['rigid'],self.frames['articulated']
        split=self.report['articulated']['collapse_frame']
        self.assertEqual(split,self.report['rigid']['collapse_frame'])
        for f in range(split):
            self.assertNotIn('waist',flex[f])
            for name in ('hip','shoulder','head'):
                np.testing.assert_array_equal(rigid[f][name],flex[f][name])
            for side in ('left','right'):
                np.testing.assert_array_equal(rigid[f]['legs'][side]['chain'],flex[f]['legs'][side]['chain'])
        self.assertIn('waist',flex[split])

    def test_thirty_seconds_undisturbed_walk_identical(self):
        a=s14.ActiveBipedSim(gravity_mode="legacy", articulated_spine=False,stumble_kick_px=0,big_push_kick_px=0)
        b=s14.ActiveBipedSim(gravity_mode="legacy", articulated_spine=True,stumble_kick_px=0,big_push_kick_px=0)
        for _ in range(900):
            a.step();b.step()
            np.testing.assert_array_equal(a.body.points,b.body.points)
        self.assertIsNone(b.spine)

    def test_bending_lengths_and_settling(self):
        r=self.report['articulated']
        self.assertTrue(r['finite'])
        self.assertLess(r['spine_min_deg'],-10.)
        self.assertGreaterEqual(r['spine_min_deg'],-31.)
        self.assertLessEqual(r['spine_max_deg'],76.)
        self.assertLess(r['max_segment_error_px'],2.)
        self.assertLess(r['tail_max_speed_px_frame'],.2)
        self.assertLess(r['tail_hip_drift_px'],1.)
        self.assertEqual(r['max_ground_penetration_px'],0)
        self.assertIsNotNone(r['brace_start_frame'])

    def test_skin_uses_two_connected_torso_segments(self):
        rig=PhysicsBilgeRig()
        for frame in self.frames['articulated'][::5]:
            pose=rig.pose(frame)
            if 'waist' not in frame:continue
            p=pose['points']
            for name in ('pelvis','waist','chest','head'):
                self.assertLessEqual(p[name][1],SCREEN_GROUND_Y)
            self.assertAlmostEqual(np.linalg.norm(p['waist']-p['pelvis']),43.)
            self.assertAlmostEqual(np.linalg.norm(p['chest']-p['waist']),43.)
            layer,origin=rig.layers(pose)['torso']
            self.assertEqual(layer.shape[2],4)
            self.assertTrue(np.isfinite(layer).all())
            self.assertGreater(float(layer[...,3].max()),.9)
            self.assertTrue(np.all(layer[...,3]<=1.00001))

if __name__=='__main__':unittest.main()
