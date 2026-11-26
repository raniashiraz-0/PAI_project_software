"""
selftest.py - One-button robot check. Tests every part and explains problems in plain words.

Run from the dashboard (Live Robot & Teach -> Run self-test) or:
    python tools/self_test.py            (checks only, nothing moves)
    python tools/self_test.py --move     (also moves the arm and nudges the car)

Each check returns PASS / WARN / FAIL / SKIP / LOOK (= look at the robot to confirm)
plus what to do about it.
"""
import math
import threading
import time
from typing import Any, Dict, List, Optional

import cv2

HOME_Q = [0.0, 110.0, -145.0, 60.0, 0.0]
SEARCH_Q = [0.0, 55.6, -118.0, 62.3, 0.0]


class SelfTest:
    def __init__(self, ctx):
        self.ctx = ctx
        self.running = False
        self.results: List[Dict[str, Any]] = []
        self.started = 0.0
        self.finished = 0.0
        self.allow_motion = False
        self._lock = threading.Lock()
        self._det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

    # ------------------------------------------------------------------
    def start(self, allow_motion: bool = False) -> Dict[str, Any]:
        with self._lock:
            if self.running:
                return {"ok": False, "error": "Self-test is already running"}
            if self.ctx.orchestrator.state.value not in ("IDLE", "STOPPED", "SUCCESS", "ERROR"):
                return {"ok": False, "error": "Robot is busy with a mission - say stop first"}
            self.running = True
            self.results = []
            self.allow_motion = bool(allow_motion)
            self.started = time.time()
        threading.Thread(target=self._run, daemon=True, name="SelfTest").start()
        return {"ok": True, "message": "Self-test started"}

    def report(self) -> Dict[str, Any]:
        with self._lock:
            res = list(self.results)
        counts = {k: sum(1 for r in res if r["status"] == k) for k in ("PASS", "WARN", "FAIL", "SKIP", "LOOK")}
        return {"running": self.running, "allow_motion": self.allow_motion, "results": res, "counts": counts,
                "duration_s": round((self.finished or time.time()) - self.started, 1) if self.started else 0.0}

    def _add(self, name: str, status: str, detail: str, fix: str = ""):
        with self._lock:
            self.results.append({"name": name, "status": status, "detail": detail, "fix": fix})
        print(f"[SelfTest] {status:4} {name}: {detail}" + (f"  -> {fix}" if fix else ""))

    def _run(self):
        steps = [self._t_modes, self._t_esp32, self._t_pca, self._t_sonar, self._t_battery, self._t_display,
                 self._t_phone_camera, self._t_tracking, self._t_hand_camera, self._t_arm_motion, self._t_drive_motion]
        try:
            for step in steps:
                try:
                    step()
                except Exception as exc:                      # a broken check must not stop the others
                    self._add(step.__name__[3:].replace("_", " "), "FAIL", f"check crashed: {exc}")
        finally:
            self.finished = time.time()
            self.running = False
            c = self.report()["counts"]
            msg = (f"Self test finished. {c['PASS']} passed, {c['WARN']} warnings, {c['FAIL']} failed."
                   if c["FAIL"] or c["WARN"] else f"Self test finished. All {c['PASS']} checks passed.")
            try:
                self.ctx.audio_out.speak(msg)
            except Exception:
                pass

    # ------------------------------------------------------------------ checks
    def _t_modes(self):
        sub = self.ctx.config.get("system", {}).get("subsystems", {})
        sim = [k.replace("_sim", "") for k, v in sub.items() if v]
        real = [k.replace("_sim", "") for k, v in sub.items() if not v]
        self._add("Modes", "PASS" if not sim else "WARN",
                  f"real: {', '.join(real) or 'none'} | simulated: {', '.join(sim) or 'none'}",
                  "" if not sim else "Simulated parts are pretend. Set them to false in server/config.yaml when the hardware is ready.")

    def _t_esp32(self):
        car, arm = self.ctx.car, self.ctx.arm
        if car.sim and arm.sim:
            self._add("Main ESP32", "SKIP", "car and arm are simulated")
            return
        st = car.status()
        if st.get("connected") and arm.get_status().get("connected"):
            age = st.get("telemetry_age_s", 0)
            if age > 1.0:
                self._add("Main ESP32", "WARN", f"connected to {st.get('esp32_ip')} but data is {age:.1f} s old",
                          "Weak Wi-Fi? Move the robot closer to the hotspot.")
            else:
                self._add("Main ESP32", "PASS", f"connected at {st.get('esp32_ip')}, data arriving")
            return
        disc = __import__("discovery").discovery.snapshot()
        self._add("Main ESP32", "FAIL", f"not connected ({st.get('link_error') or 'no reply'}); found on network: {disc or 'nothing'}",
                  "Is the ESP32 powered and on the SAME hotspot? Check its Serial Monitor shows [WIFI] connected. "
                  "If it is connected but not found, put its IP in config.yaml -> controller_esp32 -> fallback_ip.")

    def _t_pca(self):
        arm = self.ctx.arm
        if arm.sim:
            self._add("Arm servo drivers (PCA9685)", "SKIP", "arm is simulated")
            return
        if not arm.get_status().get("connected"):
            self._add("Arm servo drivers (PCA9685)", "SKIP", "ESP32 not connected")
            return
        ok = arm.get_status().get("pca_ok", [False, False])
        if all(ok):
            self._add("Arm servo drivers (PCA9685)", "PASS", "0x40 and 0x41 both answering")
        else:
            bad = [a for a, o in zip(("0x40 (MG995 board)", "0x41 (MG90/SG90 board)"), ok) if not o]
            self._add("Arm servo drivers (PCA9685)", "FAIL", f"not answering: {', '.join(bad)}",
                      "Check SDA->GPIO21, SCL->GPIO22, VCC->3.3V, GND, and that board #2 has its A0 pad bridged.")

    def _t_sonar(self):
        car = self.ctx.car
        if car.sim:
            self._add("Ultrasonic sensors", "SKIP", "car is simulated")
            return
        if not car.status().get("connected"):
            self._add("Ultrasonic sensors", "SKIP", "ESP32 not connected")
            return
        samples = []
        for _ in range(10):
            samples.append(dict(car.status().get("distances_cm", {})))
            time.sleep(0.15)
        for side in ("front", "back", "left", "right"):
            vals = [s.get(side, 999.0) for s in samples]
            seen = [v for v in vals if v < 999.0]
            if not seen:
                self._add(f"Ultrasonic {side}", "WARN", "no echo at all (nothing within 3 m, or not working)",
                          f"Hold your hand 20 cm in front of the {side} sensor and run again. Still nothing? "
                          f"Check its 5 V power, TRIG wire and ECHO wire (through the 1k/2k divider).")
            elif min(seen) < 3.0:
                self._add(f"Ultrasonic {side}", "FAIL", f"reads {min(seen):.0f} cm (impossibly close)",
                          "Usually a wiring problem on the ECHO line, or something touching the sensor.")
            else:
                spread = max(seen) - min(seen)
                st = "PASS" if spread < 15 or len(seen) < 3 else "WARN"
                self._add(f"Ultrasonic {side}", st, f"{sum(seen) / len(seen):.0f} cm" + (f" (jumping by {spread:.0f} cm)" if st == "WARN" else ""),
                          "" if st == "PASS" else "Readings jump around - check the sensor is fixed firmly and wires are tight.")

    def _t_battery(self):
        car = self.ctx.car
        if car.sim or not car.status().get("connected"):
            self._add("Base battery", "SKIP", "car simulated or not connected")
            return
        v = car.status().get("battery_v")
        if not v or v < 1.0:
            self._add("Base battery", "SKIP", "no battery reading (sense wire on GPIO32 not fitted)")
        elif v < 6.8:
            self._add("Base battery", "FAIL", f"{v:.2f} V - too low", "Charge the 2S battery (full = 8.4 V).")
        elif v < 7.2:
            self._add("Base battery", "WARN", f"{v:.2f} V - getting low", "Charge soon.")
        else:
            self._add("Base battery", "PASS", f"{v:.2f} V")

    def _t_display(self):
        d = self.ctx.display
        if d.mode == "sim":
            self._add("TFT display", "SKIP", "display is simulated")
            return
        old = d.state
        d.set_state("happy")
        time.sleep(1.5)
        link_ok = (d._sock is not None) if d.mode == "relay" else (d._serial is not None)
        d.set_state(old if old else "idle")
        if not link_ok:
            how = "ESP32 TFT relay (port 8080)" if d.mode == "relay" else d.serial_port
            self._add("TFT display", "FAIL", f"could not send to the {how}",
                      "relay: the main ESP32 must be connected. serial: close the Arduino IDE Serial Monitor (COM4).")
            return
        self._add("TFT display", "LOOK", "sent a HAPPY face for 1.5 s",
                  "Did the TFT face change? If not: check ESP32 GPIO17 -> Uno D0 and GND -> GND, and that the Uno has the TFT code.")

    def _latest(self, cam):
        if cam is None:
            return None, 0.0
        frame, t = cam.get_latest_frame()
        return frame, t

    def _t_phone_camera(self):
        src = self.ctx.config.get("devices", {}).get("cameras", {}).get("overhead_phone", {}).get("source", "sim")
        if str(src).lower() == "sim":
            self._add("Phone camera", "SKIP", "no camera address set (source: sim)",
                      "Run: python tools/phone_camera_setup.py")
            return
        cam = self.ctx.overhead_cam
        time.sleep(0.5)
        frame, t = self._latest(cam)
        if frame is None or time.time() - t > 2.0:
            self._add("Phone camera", "FAIL", f"no pictures from {src} (status {cam.status})",
                      "Open IP Webcam and tap Start server. Check the phone's address at the bottom of the app - "
                      "if it changed, run: python tools/phone_camera_setup.py")
            return
        fps = cam.get_fps()
        st = "PASS" if fps >= 5 else "WARN"
        self._add("Phone camera", st, f"{frame.shape[1]}x{frame.shape[0]} at {fps:.1f} fps",
                  "" if st == "PASS" else "Slow video: lower resolution/quality in IP Webcam, keep the phone near the router.")
        _, ids, _ = self._det.detectMarkers(frame)
        found = sorted(set(int(i) for i in ids.flatten())) if ids is not None else []
        floor = [i for i in (0, 1, 2, 3) if i in found]
        if len(floor) == 4:
            self._add("Floor tags 0-3", "PASS", "all 4 visible - the 2D map works")
        else:
            self._add("Floor tags 0-3", "FAIL", f"visible: {floor or 'none'}",
                      "Tape all 4 floor tags (printed at 100%) and aim the phone straight down so it sees every one.")
        for tag, name in ((5, "Car tag 5"), (10, "Shelf tag 10"), (6, "Start tag 6")):
            self._add(name, "PASS" if tag in found else "WARN", "visible" if tag in found else "not visible",
                      "" if tag in found else f"Put tag {tag} where the guide says and keep it in the camera view.")

    def _t_tracking(self):
        tr = self.ctx.tracker
        if tr.sim:
            self._add("Car tracking", "SKIP", "tracker is simulated")
            return
        pose = None
        t0 = time.time()
        while time.time() - t0 < 2.0 and pose is None:
            pose = tr.get_pose()
            time.sleep(0.1)
        if pose is None:
            self._add("Car tracking", "FAIL", "the server cannot work out where the car is",
                      "Needs all 4 floor tags AND car tag 5 in the phone's view, with good light.")
        else:
            self._add("Car tracking", "PASS", f"car at x={pose.x_mm:.0f} mm, y={pose.y_mm:.0f} mm, facing {pose.heading_deg:.0f} deg")

    def _t_hand_camera(self):
        if self.ctx.verifier.sim:
            self._add("ESP32-CAM (gripper)", "SKIP", "box checking is simulated (verifier_sim: true)")
            return
        frame, t = self._latest(self.ctx.hand_cam)
        if frame is None or time.time() - t > 3.0:
            self._add("ESP32-CAM (gripper)", "FAIL", "no pictures arriving",
                      "Power the ESP32-CAM (5 V boost), same hotspot; its Serial Monitor should show frames ok=... counting up.")
        else:
            self._add("ESP32-CAM (gripper)", "PASS", f"pictures arriving ({self.ctx.hand_cam.get_fps():.1f} fps)")

    # ------------------------------------------------------------------ motion (only if allowed)
    def _t_arm_motion(self):
        arm, orch = self.ctx.arm, self.ctx.orchestrator
        if not self.allow_motion:
            self._add("Arm movement", "SKIP", "movement not allowed (tick 'Allow the robot to move')")
            return
        ok, msg = orch.can_move_arm()
        if not ok:
            self._add("Arm movement", "FAIL", msg)
            return
        for name, q in (("HOME", HOME_Q), ("SEARCH", SEARCH_Q), ("HOME", HOME_Q)):
            r = arm.move_joints(list(q))
            if not r.get("ok"):
                self._add("Arm movement", "FAIL", f"move to {name} failed: {r.get('error')}",
                          "Check servo power (bucks on), the PCA9685 boards, and JOINT_TRIM if a joint is off.")
                return
        self._add("Arm movement", "PASS", "HOME -> SEARCH -> HOME, every move confirmed")
        if not arm.open_gripper():
            self._add("Gripper", "FAIL", f"open failed: {arm.last_error}")
            return
        if not arm.authorise_grab("selftest", 1):
            self._add("Gripper", "FAIL", f"could not authorise the test grip: {arm.last_error}")
            return
        r = arm.close_gripper(mission_id="selftest", medicine_id=1)
        arm.deauthorise_grab()
        reopened = arm.open_gripper()
        if not r.get("ok") or not reopened:
            self._add("Gripper", "FAIL", r.get("error") or arm.last_error, "Check the gripper servo and GRIP_OPEN_DEG / GRIP_CLOSED_DEG.")
        else:
            self._add("Gripper", "LOOK", "closed and opened", "Did the claws really close and open fully?")

    def _t_drive_motion(self):
        car, orch, tr = self.ctx.car, self.ctx.orchestrator, self.ctx.tracker
        if not self.allow_motion:
            self._add("Driving", "SKIP", "movement not allowed (tick 'Allow the robot to move')")
            return
        ok, msg = orch.can_drive_car()
        if not ok:
            self._add("Driving", "FAIL", msg)
            return
        d = car.status().get("distances_cm", {})
        if not car.sim and (d.get("front", 999) < 30 or d.get("back", 999) < 30):
            self._add("Driving", "SKIP", "less than 30 cm free in front or behind - not safe to test",
                      "Give the car some room and run again.")
            return
        p0 = None if tr.sim else tr.get_pose()
        if not car.drive_forward(100, 50):
            self._add("Driving", "FAIL", f"forward refused: {car.last_error or 'not sent'}")
            return
        res = car.wait_step_done(5.0)
        time.sleep(0.6)
        p1 = None if tr.sim else tr.get_pose()
        car.drive_backward(100, 50)
        car.wait_step_done(5.0)
        if not res.startswith("done"):
            self._add("Driving", "FAIL", f"forward step: {res}", "Check the L298N wiring and motor battery.")
            return
        if p0 is not None and p1 is not None:
            moved = math.hypot(p1.x_mm - p0.x_mm, p1.y_mm - p0.y_mm)
            dot = (p1.x_mm - p0.x_mm) * math.cos(math.radians(p0.heading_deg)) + (p1.y_mm - p0.y_mm) * math.sin(math.radians(p0.heading_deg))
            if moved < 30:
                self._add("Driving", "FAIL", f"camera saw the car move only {moved:.0f} mm (asked 100 mm)",
                          "Motors not turning? Check the motor battery, ENA/ENB wires and MOTOR_MIN_PWM.")
            elif dot < 0:
                self._add("Driving", "FAIL", "the car drove BACKWARDS when told forward",
                          "Set INVERT_LEFT_MOTOR and INVERT_RIGHT_MOTOR to 1 in config.h (or swap the motor wires).")
            else:
                st = "PASS" if 60 <= moved <= 160 else "WARN"
                self._add("Driving", st, f"forward 100 mm -> camera measured {moved:.0f} mm, then came back",
                          "" if st == "PASS" else "Distance is off - adjust CAL_MM_PER_S_AT_100 in config.h (the learning will also correct it).")
        else:
            self._add("Driving", "LOOK", "drove forward 10 cm and back",
                      "Did it go FORWARD first? If backwards: set INVERT_LEFT_MOTOR / INVERT_RIGHT_MOTOR to 1.")
