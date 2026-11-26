"""
test_learning.py - Proves the learning parts actually improve accuracy.
Run:  python tests/test_learning.py
"""
import math
import os
import sys
import tempfile
import time
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "server"))
sys.path.insert(0, HERE)

from learning import RLS, DriveModel, PoseFilter, GraspMemory, Learning  # noqa: E402
import test_hardware_protocol as thp  # noqa: E402
MEDS = thp.MEDS
from orchestrator import MissionToken  # noqa: E402
from intents import IntentParser  # noqa: E402


class TestRegression(unittest.TestCase):
    def test_rls_learns_a_line(self):
        r = RLS(2, theta0=[1.0, 0.0])
        rng = np.random.default_rng(0)
        for _ in range(200):
            x = rng.uniform(-300, 300)
            r.update([x, 1.0], 0.8 * x + 12 + rng.normal(0, 5))
        self.assertAlmostEqual(r.theta[0], 0.8, delta=0.02)
        self.assertAlmostEqual(r.theta[1], 12, delta=3)

    def test_drive_model_learns_and_corrects_slow_drifting_car(self):
        """Car really goes 20% slower and pulls to one side - after learning, commands fix it."""
        m = DriveModel()
        rng = np.random.default_rng(1)
        real = lambda v, w: (0.8 * v, 0.85 * w + 0.15 * v / 1000.0)
        for _ in range(150):
            v, w = rng.uniform(-250, 250), rng.uniform(-1.0, 1.0)
            rv, rw = real(v, w)
            m.add_sample(v, w, rv + rng.normal(0, 6), rw + rng.normal(0, 0.02))
        self.assertTrue(m.v_ready and m.w_ready)
        for v_des, w_des in [(200, 0.0), (150, 0.5), (-100, -0.3)]:
            got = real(*m.correct(v_des, w_des))
            self.assertAlmostEqual(got[0], v_des, delta=8)
            self.assertAlmostEqual(got[1], w_des, delta=0.04)
            raw = real(v_des, w_des)
            self.assertLess(abs(got[0] - v_des), abs(raw[0] - v_des))
        # manual "move forward 500 mm" becomes ~625 mm sent -> 500 mm real
        self.assertAlmostEqual(m.step_distance(500) * 0.8, 500, delta=15)

    def test_untrained_model_changes_nothing(self):
        m = DriveModel()
        self.assertEqual(m.correct(200, 0.3), (200.0, 0.3))
        self.assertEqual(m.step_distance(500), 500)


class TestKalman(unittest.TestCase):
    def test_filter_beats_raw_laggy_camera(self):
        """15 fps camera, 8 mm noise, 0.25 s late. Filter must be much closer to the truth NOW."""
        rng = np.random.default_rng(3)
        f = PoseFilter(pos_noise_mm=8.0, latency_s=0.25)
        v, w = 200.0, 0.4
        x, y, th = 0.0, 0.0, 0.0
        dt = 1 / 15
        hist = []
        err_f, err_raw = [], []
        t0 = 1000.0
        for i in range(150):
            t = t0 + i * dt
            x += v * dt * math.cos(th); y += v * dt * math.sin(th); th += w * dt
            hist.append((t, x, y, th))
            # camera shows where we were 0.25 s ago, with noise
            lag = next(h for h in reversed(hist) if h[0] <= t - 0.25 + 1e-6) if t - t0 > 0.25 else hist[0]
            meas = (lag[1] + rng.normal(0, 8), lag[2] + rng.normal(0, 8), lag[3] + rng.normal(0, 0.015))
            f.update(meas, lag[0], v, w)
            p = f.predict_now(t, v, w)
            if i > 30:
                err_f.append(math.hypot(p[0] - x, p[1] - y))
                err_raw.append(math.hypot(meas[0] - x, meas[1] - y))
        self.assertLess(np.mean(err_f), 0.35 * np.mean(err_raw))
        self.assertLess(np.mean(err_f), 15.0)

    def test_filter_bridges_a_short_camera_dropout(self):
        f = PoseFilter(latency_s=0.0, max_predict_s=0.8)
        f.update((0, 0, 0), 100.0)
        p = f.predict_now(100.5, 200.0, 0.0)          # 0.5 s without camera, driving 200 mm/s
        self.assertAlmostEqual(p[0], 100.0, delta=1.0)
        self.assertIsNone(f.predict_now(102.0, 200.0, 0.0))   # too long -> no pose (car stops)


class TestGraspLearning(unittest.TestCase):
    def test_shelf_memory_looks_where_box_was_last_seen(self):
        g = GraspMemory()
        self.assertEqual(g.scan_order(1, 100.0, [100.0, 0.0, -100.0])[0], 100.0)
        g.remember_location(1, -100.0)
        self.assertEqual(g.scan_order(1, 100.0, [100.0, 0.0, -100.0])[0], -100.0)

    def test_offsets_are_averaged(self):
        g = GraspMemory(alpha=0.5)
        g.learn_offset(100, 10, -4)
        g.learn_offset(100, 20, -8)
        self.assertEqual(g.offset_for(100), (15.0, -6.0))
        self.assertEqual(g.offset_for(-100), (0.0, 0.0))

    def test_learning_survives_restart(self):
        d = tempfile.mkdtemp()
        L = Learning(d, {}, sim=True)
        L.learn_grasp(0.0, 7.0, -3.0)
        L.remember_location(2, 0.0)
        L.log_mission({"medicine": "x", "success": True, "duration_s": 30.0})
        L.save(force=True)
        L2 = Learning(d, {}, sim=True)
        self.assertEqual(L2.memory.offset_for(0.0), (7.0, -3.0))
        self.assertEqual(L2.memory.stats()["missions"], 1)


class TestVisualServo(unittest.TestCase):
    def test_camera_lines_up_the_claws_and_the_correction_is_learned(self):
        orch = thp.TestMissionLogic().make()
        orch.learning = Learning(tempfile.mkdtemp(), {}, sim=True)
        true_dy, true_dz = 14.0, -9.0             # the box is really 14 mm left and 9 mm lower
        w_mm = 2 * orch.servo_dist * math.tan(orch.servo_hfov / 2)
        h_mm = 2 * orch.servo_dist * math.tan(orch.servo_vfov / 2)
        state = {"dy": 0.0, "dz": 0.0}

        real_targets = orch._grasp_joint_targets

        def tracking_targets(med, slot_u, dy=0.0, dz=0.0):
            state["dy"], state["dz"] = dy, dz     # where the arm is aiming now
            return real_targets(med, slot_u, dy, dz)
        orch._grasp_joint_targets = tracking_targets

        def fake_locate(med_id, newer_than=0.0):
            # box left of the claws (+y) appears left in the picture (ex < 0.5)
            return {"ex": 0.5 - (true_dy - state["dy"]) / w_mm, "ey": 0.5 - (true_dz - state["dz"]) / h_mm, "source": "test"}
        orch.verifier.locate_target = fake_locate

        orch._grasp_with_check(MissionToken("v1", 2), MEDS[1], MEDS[1], 0.0)
        self.assertTrue(orch.last_servo["aligned"], orch.last_servo)
        self.assertAlmostEqual(orch.last_servo["dy"], true_dy, delta=4.0)
        self.assertAlmostEqual(orch.last_servo["dz"], true_dz, delta=4.0)
        learned = orch.learning.memory.offset_for(0.0)
        self.assertAlmostEqual(learned[0], true_dy, delta=4.0)

        # second time: starts from the learned correction -> already (almost) aligned
        orch.arm.open_gripper(); orch.carried_box = None
        orch.arm.home()
        orch._grasp_with_check(MissionToken("v2", 2), MEDS[1], MEDS[1], 0.0)
        self.assertLessEqual(orch.last_servo["iterations"], 1)


class TestSimDriveLearning(unittest.TestCase):
    def test_learns_the_real_speed_of_the_simulated_car(self):
        """Drive the simulated car around; the learned speed gain must match the car's TRUE
        speed (measured from its internal position) - whatever it is."""
        orch = thp.TestMissionLogic().make()
        L = Learning(tempfile.mkdtemp(), {}, sim=True)
        L.car = orch.car
        orch.tracker.learning = L
        sc = orch.car.sim_car
        true_ratios = []
        for v in [120.0, 180.0, 240.0, 150.0, 210.0, 240.0, 120.0, 180.0, 200.0, 160.0, 230.0, 140.0]:
            sc.x, sc.y, sc.heading = 600.0, 1200.0, 0.0
            L._poses.clear()                                   # teleport is not real motion
            t_seg = time.time()
            x0 = t0 = None
            while time.time() - t_seg < 2.6:
                orch.car.drive_velocity(v, 0)
                orch.tracker.get_pose()
                if x0 is None and time.time() - t_seg > 0.8:     # after speeding up
                    x0, t0 = sc.x, time.time()
                time.sleep(0.05)
            true_ratios.append((sc.x - x0) / (time.time() - t0) / v)
        orch.car.stop()
        s = L.drive.summary()
        true_gain = float(np.mean(true_ratios))
        print(f"\n   learned speed gain {s['speed_gain']}  vs  true {true_gain:.3f}  ({s['speed_samples']} samples)")
        self.assertGreater(s["speed_samples"], 10)
        self.assertAlmostEqual(s["speed_gain"], true_gain, delta=0.04)


class TestVoice(unittest.TestCase):
    def test_fuzzy_fallback(self):
        p = IntentParser()
        self.assertEqual(p.parse("moove forwad").name, "DRIVE_STEP")
        self.assertEqual(p.parse("opn gripper").name, "GRIPPER_OPEN")
        self.assertEqual(p.parse("some random gibberish phrase").name, "UNKNOWN")


if __name__ == "__main__":
    unittest.main(verbosity=2)
