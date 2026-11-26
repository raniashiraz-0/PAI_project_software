"""
test_server.py - Comprehensive Unit & Interlock Test Suite for Pharma Rover.
Runnable completely without hardware (all in simulation mode).
"""
import sys
import os
import time
import math
import socket
import unittest
import cv2
import numpy as np

# Add server directory to path
HERE = os.path.dirname(os.path.abspath(__file__))
SERVER_DIR = os.path.join(HERE, "..", "server")
if SERVER_DIR not in sys.path:
    sys.path.insert(0, SERVER_DIR)

import ik_arm as K
import kin3d as K3
from car_client import CarClient
from arm_client import ArmClient
from tracker import Tracker, Pose
from verifier import BoxVerifier, validate_ean13_checksum
from display_client import DisplayClient
from audio_in import AudioInput
from audio_out import AudioOutput
from events import MissionState
from intents import IntentParser
from orchestrator import Orchestrator, MissionToken
import planner
import navigator

class TestPharmaRover(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = {
            "system": {"mode": "sim", "subsystems": {"car_sim": True, "arm_sim": True, "tracker_sim": True, "verifier_sim": True, "display_sim": True}},
            "arm": {"speed_limits_deg_s": {"shoulder": 30.0, "elbow": 40.0, "yaw": 50.0, "wrist_pitch": 45.0, "wrist_roll": 60.0, "gripper": 60.0}}
        }
        cls.room = {"room_mm": [3000, 2400], "cell_mm": 20.0}
        cls.meds = [
            {"id": 1, "name": "Paracetamol 500mg", "ean13": "8961000000112", "aruco_id": 21, "weight_g": 45, "shelf_id": "S1", "slot_u_mm": 100},
            {"id": 2, "name": "Cetirizine 10mg", "ean13": "8961000000228", "aruco_id": 22, "weight_g": 40, "shelf_id": "S1", "slot_u_mm": 0},
            {"id": 3, "name": "Omeprazole 20mg", "ean13": "8961000000334", "aruco_id": 23, "weight_g": 48, "shelf_id": "S1", "slot_u_mm": -100}
        ]

        cls.car = CarClient(sim=True)
        cls.arm = ArmClient(sim=True)
        cls.tracker = Tracker(cls.config, sim=True, sim_car=cls.car.sim_car)
        cls.verifier = BoxVerifier(cls.config, cls.meds, sim=True)
        cls.display = DisplayClient(mode="sim")
        cls.parser = IntentParser(cls.display)
        cls.audio_in = AudioInput(cls.parser, sim=True)
        cls.audio_out = AudioOutput(cls.audio_in)
        cls.orchestrator = Orchestrator(cls.car, cls.arm, cls.tracker, cls.verifier, cls.display, cls.audio_in, cls.audio_out, cls.room,
                                        config=cls.config, medicines=cls.meds)

    def setUp(self):
        # Reset state between tests
        self.arm.deauthorise_grab()
        self.car.stop()
        self.arm.q_cmd = [0.0, 110.0, -145.0, 60.0, 0.0]
        self.orchestrator.active_mission = None
        self.orchestrator.state = MissionState.IDLE

    # 1. Kinematics Tests
    def test_ik_paracetamol_reproduction(self):
        """Verify closed-form IK matches medicines.json Paracetamol grasp pose."""
        # Paracetamol grasp: r = 250.0, z = -12.0, phi = 0
        q = K.ik_planar(250.0, -12.0, 0.0)
        self.assertIsNotNone(q)
        q1, q2, q3 = [round(x, 2) for x in q]
        self.assertAlmostEqual(q1, 32.16, delta=0.1)
        self.assertAlmostEqual(q2, -90.13, delta=0.1)
        self.assertAlmostEqual(q3, 57.97, delta=0.1)

    def test_torque_calculator_and_overload_detection(self):
        """Verify torque analysis flags overloaded MG995 shoulder and SG90 wrist pitch."""
        grasp_q = [0.0, 32.2, -90.1, 58.0]
        torque = K.calculate_torque(grasp_q, payload_g=45.0)
        # Safe continuous limit for MG995 shoulder is 5.5 kg.cm
        # Grasp requires 6.47 kg.cm -> ~118% (HIGH_LOAD / OVERLOADED)
        self.assertTrue(torque["is_high_load"])
        self.assertGreater(torque["shoulder_kg_cm"], 5.5)
        # Wrist pitch safe limit for SG90 is 1.1 kg.cm -> grasp requires ~1.48 kg.cm
        self.assertGreater(torque["wrist_pitch_kg_cm"], 1.1)

    # 2. Interlock Tests
    def test_interlock_arm_refused_when_car_moving(self):
        """Interlock 1: Arm movement refused while car is moving."""
        self.car.sim_car.is_moving = True
        ok, msg = self.orchestrator.can_move_arm()
        self.assertFalse(ok)
        self.assertIn("Car is currently driving", msg)
        self.car.sim_car.is_moving = False

    def test_interlock_car_drive_refused_when_arm_extended(self):
        """Interlock 2: Car drive refused while arm is extended at non-HOME pose."""
        self.arm.q_cmd = [0.0, 45.0, -90.0, 45.0, 0.0] # Extended pose
        ok, msg = self.orchestrator.can_drive_car()
        self.assertFalse(ok)
        self.assertIn("extended", msg.lower())
        # Restore to HOME
        self.arm.q_cmd = [0.0, 110.0, -145.0, 60.0, 0.0]

    def test_fetch_button_path_refuses_second_mission(self):
        """Direct medicine-card requests cannot bypass the one-mission guard."""
        self.orchestrator.state = MissionState.PLANNING
        result = self.orchestrator.start_fetch_mission({"id": 1, "name": "Paracetamol 500mg"})
        self.assertFalse(result["ok"])
        self.assertIn("busy", result["message"].lower())

    def test_manual_forward_and_reverse_steps(self):
        """D-pad forward and reverse actions request the corresponding motion."""
        forward = self.orchestrator.execute_manual_drive_step("fwd", 200)
        self.assertTrue(forward["ok"])
        self.assertEqual(self.car.sim_car.v_target, 120.0)
        self.car.stop()

        reverse = self.orchestrator.execute_manual_drive_step("back", 200)
        self.assertTrue(reverse["ok"])
        self.assertEqual(self.car.sim_car.v_target, -90.0)
        self.car.stop()

    def test_disconnected_car_does_not_report_successful_motion(self):
        """A disconnected real car link refuses a step and clears its moving flag."""
        self.car.sim = False          # real mode, but no ESP32 link exists
        self.car.link = None

        result = self.orchestrator.execute_manual_drive_step("fwd", 200)

        self.assertFalse(result["ok"])
        self.assertIn("not sent", result["error"])
        self.assertFalse(self.car.is_moving)
        self.car.sim = True

    def test_go_to_start_when_already_at_start(self):
        """GO_START is handled and does not launch a pointless navigation thread."""
        intent = self.parser.parse("go to start")
        result = self.orchestrator.dispatch_intent(intent)
        self.assertEqual(intent.name, "GO_START")
        self.assertTrue(result["ok"])
        self.assertIn("Already at the start", result["message"])

    def test_go_to_configured_start_moves_simulated_rover(self):
        """GO_START navigates to the configured start coordinates in simulation."""
        self.tracker.set_start_pose(600.0, 500.0, 0.0)
        result = self.orchestrator.start_return_to_start()
        self.assertTrue(result["ok"])
        self.orchestrator._mission_thread.join(timeout=5.0)
        self.assertFalse(self.orchestrator._mission_thread.is_alive())
        self.assertEqual(self.orchestrator.state, MissionState.IDLE)
        self.assertLess(math.hypot(self.car.sim_car.x - 600.0, self.car.sim_car.y - 500.0), 50.0)

    def test_interlock_emergency_stop_beats_all(self):
        """Interlock 4: STOP stops car, aborts arm, cancels mission within 200 ms."""
        t_start = time.time()
        self.orchestrator.emergency_stop()
        t_elapsed = (time.time() - t_start) * 1000.0 # ms

        self.assertLess(t_elapsed, 200.0)
        self.assertFalse(self.car.is_moving)
        self.assertFalse(self.arm.grab_authorized)
        self.assertIsNone(self.orchestrator.active_mission)

    def test_grab_authorisation_gate(self):
        """Interlock 5: Gripper close refused without valid server authorisation."""
        self.arm.deauthorise_grab()
        res = self.arm.close_gripper(mission_id="TEST", medicine_id=1)
        self.assertFalse(res.get("ok", False))
        self.assertIn("ERR_NOT_AUTHORISED", res.get("error", ""))

        # Authorise grab with valid token
        self.arm.authorise_grab("TEST", 1)
        res_auth = self.arm.close_gripper(mission_id="TEST", medicine_id=1)
        self.assertTrue(res_auth.get("ok", False))

    # 3. Intent Parser Tests
    def test_intent_parsing_medicines(self):
        """Test fuzzy matching for drug names and common mishearings."""
        i1 = self.parser.parse("bring me paracetmol")
        self.assertEqual(i1.name, "FETCH")
        self.assertEqual(i1.params["medicine_id"], 1)

        i2 = self.parser.parse("fetch panadol please")
        self.assertEqual(i2.name, "FETCH")
        self.assertEqual(i2.params["medicine_id"], 1)

        i3 = self.parser.parse("get the antacid")
        self.assertEqual(i3.name, "FETCH")
        self.assertEqual(i3.params["medicine_id"], 3)

        i4 = self.parser.parse("stop right now")
        self.assertEqual(i4.name, "STOP")

        i5 = self.parser.parse("some random gibberish phrase")
        self.assertEqual(i5.name, "UNKNOWN")

    def test_voice_forward_one_step_dispatches_drive(self):
        """A spoken forward command dispatches one 200 mm simulated step."""
        audio_in = AudioInput(self.parser, sim=True)
        intent = audio_in.process_text_command("go forward one step")
        result = self.orchestrator.dispatch_intent(intent)

        self.assertEqual(intent.name, "DRIVE_STEP")
        self.assertEqual(intent.params, {"dir": "fwd", "dist_mm": 200.0})
        self.assertTrue(result["ok"])
        self.assertEqual(self.car.sim_car.v_target, 120.0)
        self.car.stop()

    def test_audio_transcription_uses_local_model(self):
        """Browser audio bytes are passed to the configured local transcription model."""
        class FakeSegment:
            text = " go forward "

        class FakeModel:
            received = b""

            def transcribe(self, audio_path, **kwargs):
                with open(audio_path, "rb") as audio_file:
                    self.received = audio_file.read()
                return iter([FakeSegment()]), None

        audio_in = AudioInput(self.parser, stt_backend="faster-whisper")
        audio_in._whisper_model = FakeModel()

        transcript = audio_in.transcribe_audio(b"fake audio", "audio/webm")

        self.assertEqual(transcript, "go forward")
        self.assertEqual(audio_in._whisper_model.received, b"fake audio")

    def test_tracker_calibrates_from_floor_markers_and_finds_car(self):
        """Real tracker maps car/start tags into room coordinates using floor references."""
        room_data = {
            "floor_markers": {
                "centres_mm": {
                    "0": [250, 250],
                    "1": [2750, 250],
                    "2": [2750, 2150],
                    "3": [250, 2150]
                }
            }
        }
        canvas = np.full((1100, 1300), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        def place_marker(marker_id, x_mm, y_mm):
            marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 70)
            center_x = round(50 + x_mm * 0.4)
            center_y = round(50 + (2400 - y_mm) * 0.4)
            canvas[center_y - 35:center_y + 35, center_x - 35:center_x + 35] = marker

        for marker_id, point in room_data["floor_markers"]["centres_mm"].items():
            place_marker(int(marker_id), *point)
        place_marker(5, 1500, 1200)
        place_marker(6, 400, 500)

        tracker = Tracker(self.config, sim=False, room_data=room_data)
        pose = tracker.process_frame(cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR), time.time())

        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.x_mm, 1500, delta=20)
        self.assertAlmostEqual(pose.y_mm, 1200, delta=20)
        self.assertAlmostEqual(tracker.start_pose()[0], 400, delta=20)
        self.assertAlmostEqual(tracker.start_pose()[1], 500, delta=20)

    def test_arm_client_sends_authorization_to_dedicated_socket(self):
        """Arm authorization uses its TCP link and the documented framed payload."""
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        client = ArmClient(sim=True)
        client._running = False
        client._worker_thread.join(timeout=1.0)
        client.sim = False
        client.host = "127.0.0.1"
        client.port = listener.getsockname()[1]

        try:
            self.assertTrue(client.connect())
            peer, _ = listener.accept()
            received = []

            def fake_esp32_ack():
                data = peer.recv(64)
                received.append(data)
                peer.sendall(bytes([0xAA, 2, 0x01, 0x31, 2 ^ 0x01 ^ 0x31, 0x55]))   # ACK 0x31

            import threading
            t = threading.Thread(target=fake_esp32_ack)
            t.start()
            self.assertTrue(client.authorise_grab("12345678", 1))
            t.join(timeout=2.0)
            frame = received[0]
            payload = bytes([1]) + b"12345678" + (10000).to_bytes(2, "big")
            checksum = (1 + len(payload)) ^ 0x31
            for value in payload:
                checksum ^= value
            self.assertEqual(frame, bytes([0xAA, 1 + len(payload), 0x31]) + payload + bytes([checksum, 0x55]))
            client.close()
            peer.close()
        finally:
            if client._running:
                client.close()
            listener.close()

    # 4. Checksum & Barcode Verification
    def test_ean13_checksum_validation(self):
        """Validate EAN-13 checksum algorithm."""
        self.assertTrue(validate_ean13_checksum("8961000000112"))
        self.assertTrue(validate_ean13_checksum("8961000000228"))
        self.assertTrue(validate_ean13_checksum("8961000000334"))
        # Corrupt last digit
        self.assertFalse(validate_ean13_checksum("8961000000115"))

    # 5. Navigation & Shortest Path
    def test_astar_route_planning(self):
        """Verify A* finds shortest obstacle-free route."""
        grid = planner.GridMap(3000, 2400, 20.0)
        grid.inflate(165.0, 300.0)
        path = planner.astar(grid, (400, 500), (2000, 1500))
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 5)
        smoothed = planner.smooth_path(grid, path)
        self.assertLessEqual(len(smoothed), len(path))

if __name__ == "__main__":
    unittest.main()
