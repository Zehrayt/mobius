"""Recovery timing, backward stepping and source-calibrated head posture."""
from pathlib import Path
import copy
import sys
import unittest
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from demo.step14_active_biped import ActiveBipedSim
from demo.step17_bilge_physics_skin import PhysicsBilgeRig, simulate
from demo.step33_balance_head import KICK_PER_MS, run_case
from scene.skinning import transform_point


class BalanceRecoveryTest(unittest.TestCase):
    def test_impulse_is_sensed_before_control_on_the_same_frame(self):
        sim = ActiveBipedSim(stumble_kick_px=0, big_push_t=0, big_push_kick_px=35.)
        hip = sim.idx['hip']
        previous = sim.body.prev_points.copy()
        sim.body.prev_points[hip,0] -= 35*sim.hip_base_mass/sim.body.masses[hip]
        expected = sim.system_com_vx()
        sim.body.prev_points[:] = previous
        sim.step()
        self.assertGreater(expected, 0)
        self.assertAlmostEqual(sim.last_ctrl_vx, expected)

    def test_capture_step_can_release_behind_the_body(self):
        sim = ActiveBipedSim()
        leg = sim.left_leg
        hip = sim.body.points[sim.idx['hip']].copy()
        leg.update(hip, hip_vx=-10, other_leg_swinging=False)
        self.assertEqual(leg.state, 'swing')
        self.assertLess(leg.swing_target[0], hip[0])

    def test_committed_catch_keeps_target_without_teleporting_foot(self):
        sim = ActiveBipedSim()
        leg = sim.left_leg
        hip = sim.body.points[sim.idx['hip']].copy()
        leg.launch_catch_step(hip[0], 10.)
        target = leg.swing_target.copy()
        start = leg.servo_pos.copy()
        leg.update(hip+[4.,0.], hip_vx=12.)
        np.testing.assert_array_equal(leg.swing_target, target)
        self.assertLess(np.linalg.norm(leg.servo_pos-start), np.linalg.norm(target-start))
        self.assertEqual(leg.state, 'swing')

    def test_forward_and_backward_recovery_preserve_gravity(self):
        for push, phase in ((KICK_PER_MS,0),(-KICK_PER_MS,10),(-.5*KICK_PER_MS,10)):
            r = run_case(True,push,phase,900)
            self.assertFalse(r['collapsed'] or r['fell'],r)
            self.assertTrue(r['finite'])
            self.assertGreater(r['mean_tail_speed'],1.7)
            self.assertEqual(len(r['gravity_values']),1)

    def test_backward_falls_settle_without_knee_projection_creep(self):
        for push,phase in ((-2*KICK_PER_MS,5),(-2*KICK_PER_MS,20),(-150.,25)):
            r=run_case(True,push,phase,420)
            self.assertTrue(r['collapsed'] and r['finite'],r)
            self.assertEqual(r['max_penetration_px'],0)
            self.assertLess(r['tail_point_speed'],.2,r)
            self.assertLess(r['tail_hip_drift'],1.,r)


class HeadPostureTest(unittest.TestCase):
    def test_eye_line_follows_neck_and_neck_base_stays_in_collar(self):
        frames,_ = simulate(1)
        rig = PhysicsBilgeRig()
        pose = rig.pose(frames[0])
        p = pose['points']
        sk = rig.skin
        for angle in (-55.,0.,25.,85.):
            a = np.radians(angle)
            p['head'] = p['chest']+43*np.array([np.sin(a),-np.cos(a)])
            matrices = rig.matrices(pose)
            matrix = matrices['head']
            eyes = [transform_point(matrix,sk.parts['head'].local(e))
                    for e in sk.settings['head']['eye_line']]
            d = eyes[1]-eyes[0]
            self.assertAlmostEqual(np.degrees(np.arctan2(d[1],d[0])),angle)
            socket = transform_point(matrices['torso'],sk.anchor('torso','neck_socket'))
            np.testing.assert_allclose(transform_point(matrix,sk.anchor('head','neck_base')),socket,atol=1e-10)

    def test_walking_head_stays_upright_and_walk_stays_stable(self):
        for ice in (False,True):
            r=run_case(True,0,0,1800,ice)
            self.assertFalse(r['fell'] or r['collapsed'],r)
            self.assertLess(r['max_abs_eye_tilt_deg'],5.)
            self.assertGreater(r['mean_tail_speed'],1.7)
            self.assertLess(r['mean_tail_speed'],2.2)
            self.assertGreater(r['min_hip_height'],150.)

    def test_neck_controller_is_released_on_collapse(self):
        a=ActiveBipedSim()
        for _ in range(40):a.step()
        a._collapse('test')
        b=copy.deepcopy(a)
        b.upright_head=False
        for _ in range(5):
            a.step();b.step()
            np.testing.assert_array_equal(a.body.points,b.body.points)
            np.testing.assert_array_equal(a.body.prev_points,b.body.prev_points)


if __name__ == '__main__':unittest.main()
