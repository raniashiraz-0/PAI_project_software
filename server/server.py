"""
server.py - Complete unified web server, mission orchestrator, and real-time telemetry dashboard.
Standard Python library (ThreadingHTTPServer) with zero external server framework dependencies.
Provides REST API, MJPEG video streams for all cameras/displays, and live control interface.
"""
import os
import sys
import json
import time
import math
import yaml
import socket
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import cv2
import numpy as np
from typing import Any, Dict, List, Optional, Tuple

# Add server directory to sys.path
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from events import bus, Event, EventType, MissionState
from car_client import CarClient
from arm_client import ArmClient
from tracker import Tracker
from camera import CameraGrabber
from verifier import BoxVerifier
from display_client import DisplayClient
from audio_in import AudioInput
from audio_out import AudioOutput
from intents import IntentParser
from orchestrator import Orchestrator
from discovery import discovery, resolve_host
from learning import Learning
from selftest import SelfTest
import streamer
import doctor
import torque_report

CONFIG_PATH = os.path.join(HERE, "config.yaml")
ROOM_PATH = os.path.join(HERE, "..", "data", "room.json")
MEDICINES_PATH = os.path.join(HERE, "..", "data", "medicines.json")

def load_yaml_config():
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    return {}

def load_json_file(path):
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}

class RobotServerContext:
    def __init__(self):
        self.config = load_yaml_config()
        self.room_data = load_json_file(ROOM_PATH)
        self.medicines_data = load_json_file(MEDICINES_PATH)
        self.med_list = self.medicines_data.get("medicines", [])

        # Subsystems sim flags
        sub_cfg = self.config.get("system", {}).get("subsystems", {})
        car_sim = sub_cfg.get("car_sim", True)
        arm_sim = sub_cfg.get("arm_sim", True)
        tracker_sim = sub_cfg.get("tracker_sim", True)
        verifier_sim = sub_cfg.get("verifier_sim", True)
        display_sim = sub_cfg.get("display_sim", True)
        audio_sim = sub_cfg.get("audio_sim", True)

        # 0. One main ESP32 runs the base, the arm and the TFT relay.
        #    host "auto" = found automatically on the hotspot (UDP discovery).
        ctrl = self.config.get("devices", {}).get("controller_esp32", {})
        ctrl_host = str(ctrl.get("host", "auto"))
        ctrl_fallback = str(ctrl.get("fallback_ip", ""))
        disc_cfg = self.config.get("discovery", {})
        if disc_cfg.get("enabled", True) and not (car_sim and arm_sim and display_sim):
            discovery.udp_port = int(disc_cfg.get("udp_port", 4210))
            discovery.http_port = int(self.config.get("server", {}).get("port", 8000))
            discovery.start()
        main_host = lambda: resolve_host(ctrl_host, ctrl_fallback, "main")

        # 1. Car Client
        start_pose = tuple(self.room_data.get(
            "start_pose_mm",
            self.config.get("robot", {}).get("start_pose_mm", [400.0, 500.0, 0.0])
        ))
        self.car = CarClient(
            host=ctrl_host,
            port=int(ctrl.get("car_tcp_port", 5000)),
            sim=car_sim,
            initial_pose=start_pose,
            host_resolver=main_host
        )

        # 2. Arm Client
        self.arm = ArmClient(
            host=ctrl_host,
            port=int(ctrl.get("arm_tcp_port", 5001)),
            sim=arm_sim,
            host_resolver=main_host
        )

        # 3. Display Client (relay through the main ESP32 -> Uno D0, or USB serial)
        disp_cfg = self.config.get("devices", {}).get("display_uno", {})
        self.display = DisplayClient(
            mode="sim" if display_sim else disp_cfg.get("mode", "relay"),
            host=main_host,
            tcp_port=int(ctrl.get("display_relay_port", 8080)),
            serial_port=disp_cfg.get("serial_port", "COM4")
        )

        # 4. Cameras
        cam_cfg = self.config.get("devices", {}).get("cameras", {})
        self.overhead_cam = CameraGrabber(
            name="OverheadPhone",
            source=cam_cfg.get("overhead_phone", {}).get("source", 0),
            fps_limit=cam_cfg.get("overhead_phone", {}).get("fps_limit", 15)
        )
        laptop_cam_cfg = cam_cfg.get("laptop_webcam", {})
        self.laptop_cam = CameraGrabber(
            name="LaptopWebcam",
            source=laptop_cam_cfg.get("source", 0),
            fps_limit=laptop_cam_cfg.get("fps_limit", 10)
        ) if laptop_cam_cfg.get("enabled", False) else None
        # ESP32-CAM uploads JPEGs to /api/vision/hand_frame (it finds the laptop by itself)
        self.hand_cam = CameraGrabber(
            name="HandESP32Cam",
            source="HTTP_POST",
            fps_limit=10
        )

        # 5. Tracker & Verifier
        self.tracker = Tracker(self.config, sim=tracker_sim, sim_car=self.car.sim_car, room_data=self.room_data)
        # Learning: Kalman filter on the camera pose, drive regression, grasp memory (data/learned/)
        self.learning = Learning(os.path.join(HERE, "..", "data", "learned"), self.config, sim=car_sim)
        self.learning.car = self.car
        self.tracker.learning = self.learning
        self.tracker.set_start_pose(*start_pose)
        self.verifier = BoxVerifier(self.config, self.med_list, sim=verifier_sim, camera_grabber=self.hand_cam)

        # 6. Audio Pipeline & Intent Parser
        rcfg = self.config.get("robot", {})
        self.intent_parser = IntentParser(
            display_client=self.display,
            step_mm=float(rcfg.get("manual_step_dist_mm", 200.0)),
            turn_deg=float(rcfg.get("manual_step_turn_deg", 90.0)),
            continuous_mm=float(rcfg.get("voice_continuous_dist_mm", 2000.0))
        )
        self.audio_in = AudioInput(
            parser=self.intent_parser,
            wake_word=self.config.get("audio", {}).get("wake_word", "rover"),
            wake_word_enabled=self.config.get("audio", {}).get("wake_word_enabled", False),
            sim=audio_sim,
            stt_backend=self.config.get("audio", {}).get("stt_backend", "faster-whisper"),
            whisper_model=self.config.get("audio", {}).get("whisper_model", "tiny.en")
        )
        self.audio_out = AudioOutput(
            audio_in=self.audio_in,
            tts_backend=self.config.get("audio", {}).get("tts_backend", "offline")
        )

        # 7. Mission Orchestrator
        self.orchestrator = Orchestrator(
            car_client=self.car,
            arm_client=self.arm,
            tracker=self.tracker,
            verifier=self.verifier,
            display_client=self.display,
            audio_in=self.audio_in,
            audio_out=self.audio_out,
            room_config=self.room_data,
            config=self.config,
            medicines=self.med_list,
            learning=self.learning
        )
        self.orchestrator.medicines_data = self.medicines_data   # teach mode writes medicines.json

        # Mission log history for web UI
        self.log_history = []
        self._max_logs = 100
        bus.subscribe_all(self._on_event)
        self._running = True
        self._tracking_thread = None
        if not tracker_sim:
            self._tracking_thread = threading.Thread(target=self._track_overhead_loop, daemon=True)
            self._tracking_thread.start()

        print("[RobotServerContext] All subsystems initialized successfully!")

    def _track_overhead_loop(self):
        last_timestamp = 0.0
        while self._running:
            frame, timestamp = self.overhead_cam.get_latest_frame()
            if frame is not None and timestamp > last_timestamp:
                self.tracker.process_frame(frame, timestamp)
                last_timestamp = timestamp
            time.sleep(0.02)

    def close(self):
        self._running = False
        self.learning.save(force=True)
        self.overhead_cam.close()
        self.hand_cam.close()
        if self.laptop_cam:
            self.laptop_cam.close()
        self.car.close()
        self.arm.close()

    def _on_event(self, event: Event):
        entry = {
            "timestamp": round(event.timestamp, 2),
            "time_str": time.strftime("%H:%M:%S", time.localtime(event.timestamp)),
            "type": event.type.value if hasattr(event.type, 'value') else str(event.type),
            "source": event.source,
            "data": event.data
        }
        self.log_history.append(entry)
        if len(self.log_history) > self._max_logs:
            self.log_history.pop(0)

# Global context singleton
CTX = RobotServerContext()
CTX.selftest = SelfTest(CTX)

class PharmaRoverHTTPHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        # Silence routine GET polling messages from cluttering console
        pass

    # ---- camera views: a live MJPEG stream, or ONE picture with ?once=1 ----
    # The dashboard uses ?once=1 (refreshing ~6x per second). Browsers allow only ~6
    # connections to one site; 5 endless streams would block the buttons.
    def _stream_begin(self) -> bool:
        if getattr(self, "_once", False):
            return True
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, private")
        self.end_headers()
        return False

    def _stream_write(self, jpeg, once: bool) -> bool:
        data = jpeg.tobytes()
        if once:
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
            return True
        self.wfile.write(b"--frame\r\n")
        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
        self.wfile.write(data)
        self.wfile.write(b"\r\n")
        return False

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        self._once = "once=1" in (parsed.query or "")

        if path == "/" or path == "/index.html":
            self._serve_file(os.path.join(HERE, "static", "index.html"), "text/html")
        elif path.startswith("/static/"):
            rel_path = path[len("/static/"):]
            file_path = os.path.join(HERE, "static", rel_path)
            mime = "text/css" if file_path.endswith(".css") else ("application/javascript" if file_path.endswith(".js") else "text/plain")
            self._serve_file(file_path, mime)
        elif path == "/api/selftest":
            self._serve_json(CTX.selftest.report())
        elif path == "/api/status":
            self._handle_api_status()
        elif path == "/api/medicines":
            self._serve_json(CTX.medicines_data)
        elif path == "/api/doctor":
            diag = doctor.run_diagnostics(CTX.orchestrator, CTX.config)
            self._serve_json(diag)
        elif path == "/api/torque_report":
            rep = torque_report.generate_report(50.0)
            self._serve_json({"report": rep})
        elif path == "/stream/overhead":
            self._handle_mjpeg_overhead()
        elif path == "/stream/phone-camera":
            self._handle_mjpeg_camera(CTX.overhead_cam, "PHONE CAMERA WAITING FOR STREAM")
        elif path == "/stream/laptop-camera":
            self._handle_mjpeg_camera(CTX.laptop_cam, "LAPTOP WEBCAM DISABLED")
        elif path == "/stream/hand":
            self._handle_mjpeg_hand()
        elif path == "/stream/display":
            self._handle_mjpeg_display()
        elif path == "/api/vision/hand_snapshot":
            # Raw (un-annotated) latest ESP32-CAM picture - used by tools/capture_dataset.py
            frame, _t = CTX.hand_cam.get_latest_frame()
            if frame is None:
                self._serve_json({"ok": False, "error": "No hand camera frame yet"}, status=503)
                return
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg.tobytes())
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # Check content length
        content_len = int(self.headers.get("Content-Length", 0))

        if path == "/api/audio/transcribe":
            if content_len <= 0 or content_len > AudioInput.MAX_AUDIO_BYTES:
                self._serve_json({"ok": False, "error": "Audio upload must be between 1 byte and 15 MB."}, status=400)
                return
            audio_bytes = self.rfile.read(content_len)
            try:
                transcript = CTX.audio_in.transcribe_audio(audio_bytes, self.headers.get("Content-Type", "audio/webm"))
            except ValueError as exc:
                self._serve_json({"ok": False, "error": str(exc)}, status=400)
                return
            except RuntimeError as exc:
                self._serve_json({"ok": False, "error": str(exc)}, status=503)
                return

            self._serve_json({"ok": bool(transcript), "transcript": transcript,
                              "error": None if transcript else "No speech was recognized."})
            return

        if path == "/api/vision/hand_frame":
            # Direct binary image upload from ESP32-CAM
            body = self.rfile.read(content_len)
            ok = CTX.hand_cam.push_frame(body)
            # The ESP32-CAM reads "led" and sets its flash LED (used while scanning the shelf)
            self._serve_json({"ok": ok, "received_bytes": content_len, "led": int(getattr(CTX.verifier, "led_duty", 0))})
            return

        # JSON body parsing
        body_bytes = self.rfile.read(content_len) if content_len > 0 else b'{}'
        try:
            req_data = json.loads(body_bytes.decode('utf-8'))
        except Exception:
            req_data = {}

        if path == "/api/start_pose":
            try:
                x_mm = float(req_data["x_mm"])
                y_mm = float(req_data["y_mm"])
                heading_deg = float(req_data.get("heading_deg", 0.0))
            except (KeyError, TypeError, ValueError):
                self._serve_json({"ok": False, "error": "Start pose must include numeric x_mm and y_mm values."}, status=400)
                return

            room_width, room_height = CTX.room_data.get("room_mm", [3000, 2400])
            if not all(math.isfinite(value) for value in (x_mm, y_mm, heading_deg)):
                self._serve_json({"ok": False, "error": "Start pose values must be finite numbers."}, status=400)
                return
            if not (0 <= x_mm <= room_width and 0 <= y_mm <= room_height and -180 <= heading_deg <= 180):
                self._serve_json({"ok": False, "error": "Start pose must be inside the room; heading must be between -180 and 180 degrees."}, status=400)
                return

            start_pose = [x_mm, y_mm, heading_deg]
            updated_room_data = dict(CTX.room_data)
            updated_room_data["start_pose_mm"] = start_pose
            try:
                with open(ROOM_PATH, "w", encoding="utf-8") as room_file:
                    json.dump(updated_room_data, room_file, indent=2)
                    room_file.write("\n")
            except OSError as exc:
                self._serve_json({"ok": False, "error": f"Could not save start location: {exc}"}, status=500)
                return

            CTX.room_data["start_pose_mm"] = start_pose
            CTX.tracker.set_start_pose(x_mm, y_mm, heading_deg)
            self._serve_json({"ok": True, "start_pose": CTX.tracker.start_pose()})

        elif path == "/api/command":
            text = req_data.get("text", "")
            intent = CTX.audio_in.process_text_command(text)
            res = CTX.orchestrator.dispatch_intent(intent)
            self._serve_json({"ok": bool(res.get("ok", False)), "intent": intent.name, "result": res})

        elif path == "/api/mission/fetch":
            med_id = int(req_data.get("medicine_id", 1))
            med = next((m for m in CTX.med_list if m["id"] == med_id), None)
            if med:
                res = CTX.orchestrator.start_fetch_mission(med)
                self._serve_json(res)
            else:
                self._serve_json({"ok": False, "error": f"Medicine {med_id} not found"}, status=400)

        elif path == "/api/car/step":
            dir_val = req_data.get("dir", "fwd")
            dist = float(req_data.get("dist_mm", 200.0))
            deg = float(req_data.get("deg", 45.0))
            if dir_val not in ("fwd", "back", "left", "right"):
                self._serve_json({"ok": False, "error": f"Unknown drive direction: {dir_val}"}, status=400)
            elif dir_val in ("fwd", "back"):
                res = CTX.orchestrator.execute_manual_drive_step(dir_val, dist)
            else:
                sign = 1.0 if dir_val == "left" else -1.0
                res = CTX.orchestrator.execute_manual_turn(deg * sign)
            if dir_val in ("fwd", "back", "left", "right"):
                self._serve_json(res)

        elif path == "/api/car/stop":
            CTX.orchestrator.emergency_stop()
            self._serve_json({"ok": True, "message": "EMERGENCY STOP EXECUTED"})

        elif path == "/api/arm/preset":
            preset = req_data.get("preset", "")
            presets = {
                "home": [0.0, 110.0, -145.0, 60.0, 0.0],
                "search": [0.0, 55.6, -118.0, 62.3, 0.0],
                "pregrasp": [0.0, 39.3, -118.7, 79.4, 0.0],
                "carry": [0.0, 90.0, -120.0, 30.0, 0.0]
            }
            if preset not in presets:
                self._serve_json({"ok": False, "error": f"Unknown arm preset: {preset}"}, status=400)
                return
            ok, msg = CTX.orchestrator.can_move_arm()
            if not ok:
                self._serve_json({"ok": False, "error": msg}, status=403)
                return
            res = CTX.arm.move_joints(presets[preset])
            self._serve_json(res)

        elif path == "/api/arm/jog":
            axis = req_data.get("axis", "z")
            val = float(req_data.get("val", 10.0))
            ok, msg = CTX.orchestrator.can_move_arm()
            if not ok:
                self._serve_json({"ok": False, "error": msg}, status=403)
                return
            # Jog joint
            q = list(CTX.arm.q_cmd)
            idx_map = {"yaw": 0, "shoulder": 1, "elbow": 2, "wrist_pitch": 3, "roll": 4}
            if axis in idx_map:
                q[idx_map[axis]] = val
                res = CTX.arm.move_joints(q)
                self._serve_json(res)
            else:
                self._serve_json({"ok": False, "error": f"Unknown axis {axis}"})

        elif path == "/api/arm/home":
            ok, msg = CTX.orchestrator.can_move_arm()
            if not ok:
                self._serve_json({"ok": False, "error": msg}, status=403)
                return
            res = CTX.arm.home()
            self._serve_json({"ok": res})

        elif path == "/api/arm/gripper":
            target = req_data.get("state", "TOGGLE")
            if target == "OPEN" or (target == "TOGGLE" and CTX.arm.gripper_state == "CLOSED"):
                ok = CTX.arm.open_gripper()
                self._serve_json({"ok": ok, "state": "OPEN" if ok else CTX.arm.gripper_state}, status=200 if ok else 503)
            else:
                # A person pressing "close" on the dashboard IS the authorisation:
                # open the grab gate for this one manual close (missions still use their own token).
                if not CTX.arm.grab_authorized:
                    if not CTX.arm.authorise_grab("OPERATOR", int(req_data.get("medicine_id", 1))):
                        self._serve_json({"ok": False, "error": CTX.arm.last_error or "Arm controller not connected"}, status=503)
                        return
                res = CTX.arm.close_gripper(mission_id=CTX.arm.auth_mission_id, medicine_id=CTX.arm.auth_medicine_id)
                self._serve_json(res)

        elif path == "/api/arm/authorise":
            # Human operator fallback authorization button
            mid = req_data.get("mission_id", "OPERATOR")
            med_id = int(req_data.get("medicine_id", 1))
            ok = CTX.arm.authorise_grab(mid, med_id)
            message = f"Grab manually authorised for medicine {med_id}." if ok else "Arm controller is disconnected; authorization was not sent."
            self._serve_json({"ok": ok, "message": message}, status=200 if ok else 503)

        elif path == "/api/arm/nudge":
            # Teach mode: move the gripper tip a few mm / degrees
            res = CTX.orchestrator.nudge_arm(float(req_data.get("dx", 0)), float(req_data.get("dy", 0)),
                                             float(req_data.get("dz", 0)), float(req_data.get("dyaw", 0)))
            res["tip"] = CTX.orchestrator.arm_tip()
            self._serve_json(res)

        elif path == "/api/arm/teach":
            # Teach mode: save the current gripper position as a medicine's grasp point
            res = CTX.orchestrator.teach_grasp(int(req_data.get("medicine_id", 1)), bool(req_data.get("update_slot", True)))
            self._serve_json(res, status=200 if res.get("ok") else 400)

        elif path == "/api/arm/test_grasp":
            res = CTX.orchestrator.test_grasp(int(req_data.get("medicine_id", 1)), str(req_data.get("stage", "pre_grasp")))
            self._serve_json(res, status=200 if res.get("ok") else 400)

        elif path == "/api/selftest/run":
            res = CTX.selftest.start(bool(req_data.get("allow_motion", False)))
            self._serve_json(res, status=200 if res.get("ok") else 409)

        elif path == "/api/learning/reset":
            CTX.learning.reset()
            self._serve_json({"ok": True, "message": "Learning reset"})

        elif path == "/api/obstacles/clear":
            CTX.orchestrator.clear_obstacles()
            self._serve_json({"ok": True})

        elif path == "/api/display/state":
            st = req_data.get("state", "idle")
            CTX.display.set_state(st)
            self._serve_json({"ok": True, "state": st})

        elif path == "/api/display/card":
            med_id = int(req_data.get("medicine_id", 1))
            med = next((m for m in CTX.med_list if m["id"] == med_id), None)
            if med:
                CTX.display.show_medicine(med, status="MANUAL_CARD", progress=50)
                self._serve_json({"ok": True})
            else:
                self._serve_json({"ok": False, "error": "Medicine not found"}, status=400)

        else:
            self.send_error(404, "Not Found")

    def _handle_api_status(self):
        pose = CTX.tracker.get_pose()
        car_stat = CTX.car.status()
        arm_stat = CTX.arm.get_status()

        can_arm, arm_msg = CTX.orchestrator.can_move_arm()
        can_car, car_msg = CTX.orchestrator.can_drive_car()

        payload = {
            "start_pose": {
                "x": CTX.tracker.start_pose()[0],
                "y": CTX.tracker.start_pose()[1],
                "heading": CTX.tracker.start_pose()[2]
            },
            "mission": {
                "state": CTX.orchestrator.state.value if hasattr(CTX.orchestrator.state, 'value') else str(CTX.orchestrator.state),
                "mission_id": CTX.orchestrator.active_mission.mission_id if CTX.orchestrator.active_mission else None,
                "medicine_id": CTX.orchestrator.active_mission.medicine_id if CTX.orchestrator.active_mission else None,
                "carried_box": CTX.orchestrator.carried_box.get("name") if CTX.orchestrator.carried_box else None,
            },
            "car": {
                "pose": {"x": pose.x_mm, "y": pose.y_mm, "heading": pose.heading_deg} if pose else None,
                "is_moving": car_stat.get("is_moving", False),
                "connected": car_stat.get("connected", False),
                "v": car_stat.get("v", 0.0),
                "w": car_stat.get("w", 0.0),
                "battery_v": car_stat.get("battery_v", 7.4),
                "obstacle_cm": car_stat.get("front_obstacle_cm", 999.0),
                "distances_cm": car_stat.get("distances_cm", {}),
                "obstacle_side": car_stat.get("obstacle_side"),
                "esp32_ip": car_stat.get("esp32_ip"),
                "link_error": car_stat.get("link_error", ""),
                "dynamic_obstacles": len(CTX.orchestrator.dynamic_obstacles)
            },
            "arm": dict(arm_stat, tip=CTX.orchestrator.arm_tip()),
            "grip": CTX.orchestrator.last_grip,
            "learning": dict(CTX.learning.summary(), visual_servo=CTX.orchestrator.last_servo),
            "display": {
                "state": CTX.display.state,
                "status": CTX.display.status,
                "progress": CTX.display.progress,
                "showing_card": CTX.display.sim_renderer.show_card
            },
            "audio": {
                "last_spoken": CTX.audio_out.get_latest_speech(),
                "speech_id": getattr(CTX.audio_out, "speech_id", 0),
                "tts_backend": CTX.audio_out.tts_backend,
                "muted": CTX.audio_in.muted,
                "wake_word_enabled": CTX.audio_in.wake_word_enabled
            },
            "vision": {
                "tracker_status": CTX.tracker.status(),
                "overhead_status": CTX.overhead_cam.status,
                "overhead_fps": CTX.overhead_cam.get_fps(),
                "laptop_fps": CTX.laptop_cam.get_fps() if CTX.laptop_cam else 0.0,
                "laptop_enabled": CTX.laptop_cam is not None,
                "laptop_status": CTX.laptop_cam.status if CTX.laptop_cam else "DISABLED",
                "hand_fps": CTX.hand_cam.get_fps(),
                "hand_status": CTX.hand_cam.status,
                "detector": "yolo" if getattr(CTX.verifier, "yolo", None) is not None else "aruco+barcode",
                "yolo_status": getattr(CTX.verifier, "yolo_status", ""),
                "last_scan": CTX.orchestrator.last_scan
            },
            "devices": {
                "car_connected": car_stat.get("connected", False),
                "arm_connected": arm_stat.get("connected", False),
                "car_sim": CTX.car.sim,
                "arm_sim": CTX.arm.sim,
                "discovered": discovery.snapshot()
            },
            "interlocks": {
                "can_move_arm": can_arm,
                "arm_interlock_msg": arm_msg,
                "can_drive_car": can_car,
                "car_interlock_msg": car_msg,
                "grab_authorized": arm_stat.get("grab_authorized", False)
            },
            "logs": CTX.log_history[-30:] # Last 30 event logs
        }
        self._serve_json(payload)

    def _handle_mjpeg_overhead(self):
        once = self._stream_begin()

        try:
            while True:
                pose = CTX.tracker.get_pose()
                pose_tuple = (pose.x_mm, pose.y_mm, pose.heading_deg) if pose else (400.0, 500.0, 0.0)
                shelf_pose = CTX.tracker.shelf_pose("S1")
                start_pose = CTX.tracker.start_pose()

                frame = streamer.render_room_map(CTX.orchestrator.grid, pose_tuple, shelf_pose, start_pose)
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

                if self._stream_write(jpeg, once):
                    return
                time.sleep(0.1) # ~10 FPS
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _handle_mjpeg_camera(self, camera, waiting_text: str):
        once = self._stream_begin()

        try:
            while True:
                frame = camera.get_latest_frame()[0] if camera else None
                if frame is None:
                    frame = np.full((360, 640, 3), (25, 28, 35), dtype=np.uint8)
                    cv2.putText(frame, waiting_text, (25, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 1)
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if self._stream_write(jpeg, once):
                    return
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _handle_mjpeg_hand(self):
        once = self._stream_begin()

        try:
            while True:
                raw_frame, _ = CTX.hand_cam.get_latest_frame()
                if raw_frame is None:
                    # Synthetic render fallback
                    raw_frame = np.full((360, 480, 3), (25, 25, 30), dtype=np.uint8)
                    cv2.putText(raw_frame, "WAITING FOR ESP32-CAM PICTURES", (55, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 1)

                verdict_data = {
                    "result": "MATCH" if CTX.arm.grab_authorized else "UNCERTAIN",
                    "reason": "ArUco + Barcode verified" if CTX.arm.grab_authorized else "Awaiting search & verify"
                }
                annotated = streamer.render_hand_camera_frame(raw_frame, verdict_data)
                _, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])

                if self._stream_write(jpeg, once):
                    return
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _handle_mjpeg_display(self):
        once = self._stream_begin()

        try:
            while True:
                frame = CTX.display.get_preview_frame()
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

                if self._stream_write(jpeg, once):
                    return
                time.sleep(0.08) # ~12 FPS
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _serve_file(self, file_path: str, content_type: str):
        if not os.path.exists(file_path):
            self.send_error(404, "File Not Found")
            return
        with open(file_path, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_json(self, data: Any, status: int = 200):
        body = json.dumps(data, indent=2).encode('utf-8')
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

def run_server(port=8000):
    server_address = ("0.0.0.0", port)
    httpd = ThreadingHTTPServer(server_address, PharmaRoverHTTPHandler)
    print("=" * 70)
    print(f"  PHARMA ROVER UNIFIED WEBSERVER ACTIVE")
    print(f"  Open in Browser: http://localhost:{port}")
    print(f"  Network Address: http://0.0.0.0:{port}")
    sub = CTX.config.get("system", {}).get("subsystems", {})
    sim = [k.replace("_sim", "") for k, v in sub.items() if v]
    print(f"  Simulated (pretend): {', '.join(sim) if sim else 'nothing - all real hardware'}")
    print(f"  Real connections: see the 'Live Robot & Teach' tab, or run the self-test there.")
    print("=" * 70)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[Server] Shutting down safely...")
        CTX.orchestrator.emergency_stop()
        httpd.server_close()
        CTX.close()

if __name__ == "__main__":
    p = 8000
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        p = int(sys.argv[1])
    run_server(port=p)