"""
test_hardware_protocol.py - Server <-> ESP32 protocol tests using fake_esp32.py,
plus mission-level tests (shelf scan, obstacle re-planning, full fetch) in simulation.

Run:  python tests/test_hardware_protocol.py
"""
import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "server"))
sys.path.insert(0, HERE)

from fake_esp32 import FakeEsp32  # noqa: E402
from car_client import CarClient  # noqa: E402
from arm_client import ArmClient, HOME_Q  # noqa: E402
from tracker import Tracker  # noqa: E402
from verifier import BoxVerifier  # noqa: E402
from display_client import DisplayClient  # noqa: E402
from audio_in import AudioInput  # noqa: E402
from audio_out import AudioOutput  # noqa: E402
from intents import IntentParser  # noqa: E402
from orchestrator import Orchestrator, MissionToken, SEARCH_Q  # noqa: E402
from events import MissionState  # noqa: E402

MEDS = [
    {"id": 1, "name": "Paracetamol 500mg", "ean13": "8961000000112", "aruco_id": 21, "weight_g": 45, "shelf_id": "S1",
     "slot_u_mm": 100, "grasp_point_arm_frame_mm": [250.0, 0.0, -12.0]},
    {"id": 2, "name": "Cetirizine 10mg", "ean13": "8961000000228", "aruco_id": 22, "weight_g": 40, "shelf_id": "S1",
     "slot_u_mm": 0, "grasp_point_arm_frame_mm": [260.0, 0.0, -12.0]},
    {"id": 3, "name": "Omeprazole 20mg", "ean13": "8961000000334", "aruco_id": 23, "weight_g": 48, "shelf_id": "S1",
     "slot_u_mm": -100, "grasp_point_arm_frame_mm": [245.0, 0.0, -12.0]},
]


def wait_for(cond, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


class TestEspProtocol(unittest.TestCase):
    def setUp(self):
        self.esp = FakeEsp32()
        self.car = CarClient(host="127.0.0.1", port=self.esp.car_port, sim=False)
        self.arm = ArmClient(host="127.0.0.1", port=self.esp.arm_port, sim=False)
        self.assertTrue(self.car.connect())
        self.assertTrue(self.arm.connect())
        self.assertTrue(wait_for(lambda: time.time() - self.arm.last_telemetry < 0.2))

    def tearDown(self):
        self.car.close()
        self.arm.close()
        self.esp.close()

    # ---------------- base ----------------
    def test_telemetry_gives_all_four_distances(self):
        self.esp.dist_mm = [400, 1500, 250, 0xFFFF]
        self.assertTrue(wait_for(lambda: self.car.status()["distances_cm"]["front"] == 40.0))
        d = self.car.status()["distances_cm"]
        self.assertEqual(d, {"front": 40.0, "back": 150.0, "left": 25.0, "right": 999.0})
        self.assertAlmostEqual(self.car.battery_v, 7.4)

    def test_forward_step_is_acked_and_finishes(self):
        self.assertTrue(self.car.drive_forward(300, 60))
        self.assertEqual(self.car.wait_step_done(2.0), "done")
        cmd, payload = [(c, p) for n, c, p in self.esp.received if n == "car" and c == 0x10][-1]
        self.assertEqual(payload, (300).to_bytes(2, "big") + bytes([60]))

    def test_forward_refused_when_obstacle_ahead(self):
        self.esp.dist_mm[0] = 80
        self.assertFalse(self.car.drive_forward(200))
        self.assertIn("obstacle", self.car.last_error)

    def test_esp32_obstacle_stop_reported(self):
        self.assertTrue(self.car.drive_forward(2000))
        self.esp.dist_mm[0] = 90
        self.assertTrue(wait_for(lambda: self.car.status()["obstacle_side"] == "FRONT"))
        self.assertTrue(self.car.wait_step_done(1.0).startswith("failed"))

    def test_docking_threshold_command(self):
        self.assertTrue(self.car.set_obstacle_thresholds(50, 120))
        self.assertEqual(self.esp.front_stop_mm, 50)

    # ---------------- arm ----------------
    def test_arm_move_is_verified_against_telemetry(self):
        res = self.arm.move_joints(SEARCH_Q)
        self.assertTrue(res["ok"], res)
        self.assertTrue(res.get("verified"))
        for got, want in zip(self.arm.q_true, SEARCH_Q):
            self.assertAlmostEqual(got, want, delta=0.2)

    def test_arm_move_fails_when_a_joint_does_not_reach(self):
        self.esp.stall_joint = 2
        res = self.arm.move_joints(SEARCH_Q)
        self.assertFalse(res["ok"])
        self.assertIn("elbow", res["error"])

    def test_arm_move_fails_when_pca_offline(self):
        self.esp.pca_ok = [True, False]
        res = self.arm.move_joints(SEARCH_Q)
        self.assertFalse(res["ok"])
        self.assertIn("PCA9685", res["error"])

    def test_gripper_needs_authorisation(self):
        self.assertFalse(self.arm.close_gripper(mission_id="m1", medicine_id=1)["ok"])
        self.assertTrue(self.arm.authorise_grab("m1", 1))
        res = self.arm.close_gripper(mission_id="m1", medicine_id=1)
        self.assertTrue(res["ok"], res)
        self.assertTrue(self.arm.fw_gripper_closed)
        self.assertTrue(self.arm.open_gripper())

    def test_firmware_rejects_gripper_close_without_server_gate(self):
        # Bypass the server-side gate: the ESP32 must still refuse
        with self.arm._lock:
            self.arm.grab_authorized, self.arm.auth_timestamp = True, time.time()
            self.arm.auth_mission_id, self.arm.auth_medicine_id = "zz", 2
        res = self.arm.close_gripper(mission_id="zz", medicine_id=2)
        self.assertFalse(res["ok"])
        self.assertIn("not authorised", res["error"])


    def test_grip_sensor_flags_reach_the_server(self):
        self.assertFalse(self.arm.get_status()["grip_sensor_present"])
        self.esp.grip_sensor = False
        self.assertTrue(wait_for(lambda: self.arm.get_status()["grip_sensor_present"]))
        self.assertFalse(self.arm.get_status()["grip_sensor_box"])
        self.esp.grip_sensor = True
        self.assertTrue(wait_for(lambda: self.arm.get_status()["grip_sensor_box"]))


class FakeCamera:
    """Stands in for the ESP32-CAM upload: returns whatever frame the test sets."""
    def __init__(self):
        self.frame, self.t = None, 0.0

    def show(self, frame):
        self.frame, self.t = frame, time.time()

    def get_latest_frame(self):
        if self.frame is not None:
            self.t = time.time()            # a new upload every call
        return self.frame, self.t


class TestCameraGripCheck(unittest.TestCase):
    def setUp(self):
        import numpy as np
        rng = np.random.default_rng(1)
        base = rng.integers(0, 255, (60, 80, 3), dtype=np.uint8)
        self.box_view = cv2_resize(base)                          # what the camera sees with a box held
        self.room_view = cv2_resize(rng.integers(0, 255, (60, 80, 3), dtype=np.uint8))
        self.cam = FakeCamera()
        self.v = BoxVerifier({}, [dict(m) for m in MEDS], sim=False, camera_grabber=self.cam)

    def test_box_still_in_view_means_held(self):
        import numpy as np
        noisy = np.clip(self.box_view.astype(int) + np.random.default_rng(2).integers(-12, 12, self.box_view.shape), 0, 255).astype("uint8")
        self.cam.show(noisy)
        res = self.v.check_grip(1, reference_frame=self.box_view)
        self.assertTrue(res["held"], res)

    def test_room_in_view_means_empty(self):
        self.cam.show(self.room_view)
        res = self.v.check_grip(1, reference_frame=self.box_view)
        self.assertFalse(res["held"], res)

    def test_no_camera_is_unsure(self):
        res = self.v.check_grip(1, reference_frame=self.box_view)
        self.assertIsNone(res["held"])


def cv2_resize(img):
    import cv2
    return cv2.resize(img, (640, 480), interpolation=cv2.INTER_NEAREST)


class TestMissionLogic(unittest.TestCase):
    def make(self):
        config = {"robot": {"v_max_mm_s": 250.0}, "vision": {"scan_retries_per_slot": 0}}
        car = CarClient(sim=True)
        arm = ArmClient(sim=True)
        tracker = Tracker(config, sim=True, sim_car=car.sim_car)
        verifier = BoxVerifier(config, [dict(m) for m in MEDS], sim=True)
        display = DisplayClient(mode="sim")
        audio_in = AudioInput(IntentParser(display), sim=True)
        audio_out = AudioOutput(audio_in)
        orch = Orchestrator(car, arm, tracker, verifier, display, audio_in, audio_out,
                            {"room_mm": [3000, 2400], "cell_mm": 20.0}, config=config, medicines=MEDS)
        return orch

    def test_scan_finds_box_in_a_different_slot(self):
        orch = self.make()
        # Boxes were swapped on the shelf: paracetamol now sits where omeprazole was
        for m in orch.verifier.medicines:
            if m["id"] == 1:
                m["slot_u_mm"] = -100
            elif m["id"] == 3:
                m["slot_u_mm"] = 100
        token = MissionToken("t1", 1)
        slot, med = orch._scan_shelf(token, MEDS[0])
        self.assertEqual(slot, -100)
        self.assertEqual(orch.last_scan["slots"][100.0], 3)
        targets = orch._grasp_joint_targets(med, slot)
        self.assertLess(targets["grasp"][0], -15)       # arm turned towards the -100 mm slot

    def test_obstacle_triggers_replan_and_still_arrives(self):
        orch = self.make()
        real_status = orch.car.status
        state = {"t0": time.time(), "fired": False}

        def status_with_obstacle():
            st = real_status()
            if not state["fired"] and time.time() - state["t0"] > 1.0:
                state["fired"] = True
                st["front_obstacle_cm"] = 15.0
                st["distances_cm"]["front"] = 15.0
            return st
        orch.car.status = status_with_obstacle
        token = MissionToken("t2", 0)
        orch._drive_route(token, (1500.0, 1200.0), v_max=250.0, label="test point")
        pose = orch.tracker.get_pose()
        self.assertLess(((pose.x_mm - 1500) ** 2 + (pose.y_mm - 1200) ** 2) ** 0.5, 80)
        self.assertEqual(len(orch.dynamic_obstacles), 1)

    def test_missed_grip_is_detected_and_retried(self):
        orch = self.make()
        orch.arm.sim_grip_failures = 1                  # first grab comes up empty
        token = MissionToken("g1", 2)
        orch._grasp_with_check(token, MEDS[1], MEDS[1], 0.0)
        self.assertTrue(orch.last_grip["held"])
        self.assertIn("try 2", orch.last_grip["stage"])
        self.assertIsNotNone(orch.carried_box)

    def test_gives_up_after_retries(self):
        orch = self.make()
        orch.arm.sim_grip_failures = 5
        with self.assertRaises(RuntimeError) as ctx:
            orch._grasp_with_check(MissionToken("g2", 2), MEDS[1], MEDS[1], 0.0)
        self.assertIn("could not grip", str(ctx.exception))
        self.assertIsNone(orch.carried_box)

    def test_teach_mode_saves_and_reaches_the_taught_point(self):
        orch = self.make()
        self.assertTrue(orch.arm.move_joints([10.0, 35.0, -95.0, 60.0, 0.0])["ok"])
        tip = orch.arm_tip()
        res = orch.teach_grasp(3, update_slot=True)
        self.assertTrue(res["ok"], res)
        self.assertAlmostEqual(res["grasp_point_mm"][0], tip["x"], delta=0.2)
        self.assertAlmostEqual(res["slot_u_mm"], tip["y"], delta=0.2)
        self.assertTrue(orch.arm.home())
        self.assertTrue(orch.test_grasp(3, "grasp")["ok"])
        after = orch.arm_tip()
        for k in ("x", "y", "z"):
            self.assertAlmostEqual(after[k], tip[k], delta=2.0)

    def test_nudge_moves_tip(self):
        orch = self.make()
        self.assertTrue(orch.arm.move_joints(list(SEARCH_Q))["ok"])
        before = orch.arm_tip()
        self.assertTrue(orch.nudge_arm(dz=10)["ok"])
        self.assertAlmostEqual(orch.arm_tip()["z"] - before["z"], 10.0, delta=1.0)

    def test_full_fetch_mission_in_simulation(self):
        orch = self.make()
        res = orch.start_fetch_mission(MEDS[2])           # omeprazole, slot -100
        self.assertTrue(res["ok"])
        self.assertTrue(wait_for(lambda: orch.state in (MissionState.SUCCESS, MissionState.IDLE, MissionState.ERROR), 120))
        self.assertNotEqual(orch.state, MissionState.ERROR)
        self.assertEqual(orch.last_scan.get("found_slot"), -100.0)
        self.assertIsNone(orch.carried_box)
        self.assertEqual(orch.arm.q_cmd[:4], HOME_Q[:4])


if __name__ == "__main__":
    unittest.main(verbosity=2)
