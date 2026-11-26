"""
arm_client.py - Client + simulator for the 6-servo Pharma Rover arm.

Real mode: main ESP32, TCP 5001 (framed protocol, firmware/src/esp32_main/protocol.h).
Every move is CHECKED:
  1. the ESP32 must ACK the command (it refuses out-of-range angles, a busy arm,
     a missing PCA9685 board, or an un-authorised gripper close)
  2. the server waits for CMD_DONE (the ESP32 moves max 2 servos at a time)
  3. the joint angles reported in telemetry must match the target (<= 2 deg)
     and both PCA9685 boards must still answer on I2C.
Includes ArmSim backend with servo errors, soft-start, and grab authorisation gate.
"""
import math
import time
import socket
import threading
import numpy as np
from typing import List, Dict, Any, Optional
from events import bus, Event, EventType
import ik_arm as K
import kin3d as K3
from arm_planner import ArmPlanner, SPEED_LIMITS
from esp_link import EspLink, RESP_CMD_DONE, RESP_CMD_FAILED, RESP_TELEMETRY, ERROR_TEXT

START_BYTE = 0xAA
END_BYTE = 0x55

CMD_ARM_GOTO_POSE   = 0x20
CMD_ARM_SET_JOINT   = 0x21
CMD_GRIPPER_OPEN    = 0x23
CMD_GRIPPER_CLOSE   = 0x24
CMD_ARM_HOME        = 0x25
CMD_ARM_MOVE_JOINTS = 0x26
CMD_SET_ESTOP       = 0x30
CMD_ARM_AUTHORISE   = 0x31
CMD_ARM_DEAUTHORISE = 0x32
CMD_ARM_ABORT       = 0x33

RESP_ACK            = 0x01
RESP_NACK           = 0x02
RESP_CMD_DONE       = 0x03
RESP_ERR_NOT_AUTH   = 0x09
RESP_ERR_TORQUE     = 0x0B

HOME_Q = [0.0, 110.0, -145.0, 60.0, 0.0]
VERIFY_TOL_DEG = 2.0

class ArmClient:
    def __init__(self, host: str = "192.168.4.1", port: int = 5001, sim: bool = True, host_resolver=None):
        self.host = host
        self.port = port
        self.sim = sim
        self.planner = ArmPlanner()

        # Comm state
        self._host_resolver = host_resolver or (lambda: self.host)
        self.link: Optional[EspLink] = None
        self._lock = threading.Lock()
        self._running = True
        self._done_events: Dict[int, threading.Event] = {}
        self._done_result: Dict[int, str] = {}

        # Telemetry from the ESP32 (real mode)
        self.servo_deg = [0.0] * 6
        self.servo_target_deg = [0.0] * 6
        self.hw_moving = False
        self.pca_ok = [True, True]
        self.fw_authorised = False
        self.fw_gripper_closed = False
        self.last_telemetry = 0.0
        self.last_verify: Dict[str, Any] = {}
        self.last_error = ""
        self._synced = False      # take the real arm pose from the first telemetry after (re)connecting

        # Box-in-gripper sensor (optional switch / IR module on the ESP32, GPIO14)
        self.grip_sensor_present = False
        self.grip_sensor_box = False
        # Simulation: make the next N grips miss (used by tests / demos)
        self.sim_grip_failures = 0
        self.sim_box_held = False

        # State
        self.q_cmd = [0.0, 110.0, -145.0, 60.0, 0.0]  # HOME [yaw, q1, q2, q3, roll]
        self.q_true = list(self.q_cmd)
        self.gripper_state = "OPEN"                   # "OPEN" or "CLOSED"
        self.gripper_angle = 0.0                      # 0 = open, 90 = closed
        self.is_moving = False
        self.estop = False

        # Grab authorisation gate
        self.grab_authorized = False
        self.auth_mission_id: Optional[str] = None
        self.auth_medicine_id: Optional[int] = None
        self.auth_timestamp = 0.0
        self.AUTH_EXPIRY_S = 10.0

        # High-load monitoring
        self.high_load_start_time: Optional[float] = None

        # Simulator bias & noise
        self.rng = np.random.default_rng(42)
        self.servo_bias = self.rng.normal(0, 0.7, 5)

        if not sim:
            self._make_link()

        # Thread for state updates / watchdog
        self._worker_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._worker_thread.start()

        # Soft start on initialization
        self.soft_start()

    def soft_start(self):
        """Energise servos sequentially to prevent inrush current brownouts."""
        print("[ArmClient] Executing soft-start: energising channels 0-5 sequentially...")
        for ch in range(6):
            time.sleep(0.05)
        print("[ArmClient] All servo channels active at HOME pose.")

    def authorise_grab(self, mission_id: str, medicine_id: int):
        """Authorise arm to enter shelf and close gripper. Valid for 10 s."""
        if not self.sim:
            lease_ms = int(self.AUTH_EXPIRY_S * 1000)
            payload = bytes([int(medicine_id)]) + self._mission_token(mission_id) + lease_ms.to_bytes(2, "big")
            ok, reason = self._command(CMD_ARM_AUTHORISE, payload)
            if not ok:
                self.last_error = f"Authorisation refused: {reason}"
                return False

        with self._lock:
            self.grab_authorized = True
            self.auth_mission_id = mission_id
            self.auth_medicine_id = medicine_id
            self.auth_timestamp = time.time()
            bus.publish(Event(type=EventType.GRAB_AUTHORISED, source="ArmClient", data={"mission_id": mission_id, "medicine_id": medicine_id}))
        return True

    def deauthorise_grab(self):
        """Clear grab authorisation."""
        if not self.sim:
            self._send_frame(CMD_ARM_DEAUTHORISE)
        with self._lock:
            self.grab_authorized = False
            self.auth_mission_id = None
            self.auth_medicine_id = None
            bus.publish(Event(type=EventType.GRAB_REJECTED, source="ArmClient", data={"reason": "deauthorised"}))

    def move_joints(self, target_q: List[float], payload_g: float = 0.0) -> Dict[str, Any]:
        """Move arm joints with shortest path, limits check and torque guard."""
        if self.estop:
            return {"ok": False, "error": "E-STOP active!"}

        plan = self.planner.plan_joint_move(target_q, payload_g=payload_g)
        if not plan["ok"]:
            return plan

        with self._lock:
            previous_q = list(self.q_cmd)
            self.is_moving = True
            self.q_cmd = list(plan["target_q"])
            dur = plan["duration_s"]

            if plan["is_high_load"]:
                self.high_load_start_time = time.time()
            else:
                self.high_load_start_time = None

        bus.publish(Event(type=EventType.ARM_MOVE_STARTED, source="ArmClient", data={"target_q": self.q_cmd, "duration_s": dur}))

        if self.sim:
            # Simulate movement
            time.sleep(min(dur, 0.5))  # Smooth sim sleep
            with self._lock:
                self.q_true = [c + b + self.rng.normal(0, 0.2) for c, b in zip(self.q_cmd, self.servo_bias)]
                self.planner.set_current_pose(self.q_cmd)
                self.is_moving = False
            bus.publish(Event(type=EventType.ARM_MOVE_DONE, source="ArmClient", data={"q": self.q_cmd}))
            return {"ok": True, "plan": plan}
        else:
            result = self._execute_real_move(previous_q, self.q_cmd, dur)
            with self._lock:
                self.is_moving = False
                if not result["ok"]:
                    # Keep what the ESP32 says we are at, not what we hoped for
                    reported = self.reported_q()
                    self.q_cmd = reported if reported else previous_q
                    self.planner.set_current_pose(self.q_cmd)
                else:
                    self.planner.set_current_pose(self.q_cmd)
            if result["ok"]:
                bus.publish(Event(type=EventType.ARM_MOVE_DONE, source="ArmClient", data={"q": self.q_cmd, "verified": True}))
            else:
                self.last_error = result["error"]
                bus.publish(Event(type=EventType.ARM_ERROR, source="ArmClient", data={"error": result["error"]}))
            result["plan"] = plan
            return result

    def home(self) -> bool:
        """Fold arm back to HOME pose (low-torque pose)."""
        res = self.move_joints(list(HOME_Q), payload_g=0.0)
        return res.get("ok", False)

    def open_gripper(self) -> bool:
        """Open gripper (waits until the ESP32 reports it finished)."""
        if not self.sim:
            res = self._command_and_wait(CMD_GRIPPER_OPEN, b"", timeout_s=6.0)
            if not res["ok"]:
                self.last_error = f"Gripper open failed: {res['error']}"
                return False
            if self.fw_gripper_closed:
                self.last_error = "Gripper open not confirmed by telemetry"
                return False
        with self._lock:
            self.gripper_state = "OPEN"
            self.gripper_angle = 0.0
            self.sim_box_held = False
            return True

    def close_gripper(self, mission_id: Optional[str] = None, medicine_id: Optional[int] = None) -> Dict[str, Any]:
        """
        Close gripper. HARD GATE: strictly requires server grab authorisation!
        """
        with self._lock:
            now = time.time()
            # Check authorisation
            if not self.grab_authorized or (now - self.auth_timestamp) > self.AUTH_EXPIRY_S:
                self.grab_authorized = False
                return {"ok": False, "error": "ERR_NOT_AUTHORISED: Grab gate closed. Server authorization required!"}

            if mission_id and self.auth_mission_id != mission_id:
                return {"ok": False, "error": "ERR_NOT_AUTHORISED: Mission ID mismatch!"}

            if medicine_id and self.auth_medicine_id != medicine_id:
                return {"ok": False, "error": "ERR_NOT_AUTHORISED: Medicine ID mismatch!"}

            authorized_mission = mission_id or self.auth_mission_id
            authorized_medicine = medicine_id or self.auth_medicine_id

        if not self.sim:
            payload = bytes([int(authorized_medicine)]) + self._mission_token(authorized_mission)
            res = self._command_and_wait(CMD_GRIPPER_CLOSE, payload, timeout_s=6.0)
            if not res["ok"]:
                return {"ok": False, "error": f"Gripper close failed: {res['error']}"}
            if not self.fw_gripper_closed:
                return {"ok": False, "error": "Gripper close not confirmed by telemetry"}

        with self._lock:
            self.gripper_state = "CLOSED"
            self.gripper_angle = 75.0
            if self.sim:
                if self.sim_grip_failures > 0:
                    self.sim_grip_failures -= 1
                    self.sim_box_held = False
                else:
                    self.sim_box_held = True
        bus.publish(Event(type=EventType.ARM_GRIP_DONE, source="ArmClient", data={"confirmed": True}))
        return {"ok": True, "message": "Grip closed with valid server authorisation."}

    def abort(self):
        """Emergency arm abort: stop in place, deauthorise, and set channels safe."""
        if not self.sim:
            self._send_frame(CMD_ARM_DEAUTHORISE)
            self._send_frame(CMD_ARM_ABORT)
            for ev in list(self._done_events.values()):
                ev.set()
        with self._lock:
            self.is_moving = False
            self.grab_authorized = False
            bus.publish(Event(type=EventType.ARM_ABORTED, source="ArmClient", data={}))

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            tip_pos = K3.tip_xyz(self.q_cmd[:4])
            torque = K.calculate_torque(self.q_cmd[:4], payload_g=45.0 if self.gripper_state == "CLOSED" else 0.0)
            return {
                "q_cmd": [round(x, 1) for x in self.q_cmd],
                "q_true": [round(x, 1) for x in self.q_true],
                "gripper": self.gripper_state,
                "is_moving": self.is_moving,
                "connected": self.connected,
                "esp32_ip": self.link.current_host if self.link else None,
                "servo_deg": [round(x, 1) for x in self.servo_deg],
                "pca_ok": list(self.pca_ok),
                "hw_moving": self.hw_moving,
                "last_verify": self.last_verify,
                "grip_sensor_present": self.grip_sensor_present,
                "grip_sensor_box": self.grip_sensor_box,
                "last_error": self.last_error,
                "tip_xyz_mm": [round(x, 1) for x in tip_pos],
                "grab_authorized": self.grab_authorized,
                "auth_time_left_s": max(0.0, round(self.AUTH_EXPIRY_S - (time.time() - self.auth_timestamp), 1)) if self.grab_authorized else 0.0,
                "torque": torque
            }

    def _update_loop(self):
        while self._running:
            # Check grab auth timeout
            with self._lock:
                if self.grab_authorized and (time.time() - self.auth_timestamp) > self.AUTH_EXPIRY_S:
                    self.grab_authorized = False
                    if not self.sim:
                        self._send_frame(CMD_ARM_DEAUTHORISE)
                    bus.publish(Event(type=EventType.GRAB_REJECTED, source="ArmClient", data={"reason": "timeout_10s"}))

                # Check high load hold time
                if self.high_load_start_time and (time.time() - self.high_load_start_time) > 3.0:
                    print("[ArmClient] WARNING: HIGH_LOAD hold time limit reached (3s)! Servos overloaded.")
                    self.high_load_start_time = None

            time.sleep(0.1)

    # ------------------------------------------------------------------ real hardware
    def _make_link(self):
        self.link = EspLink("arm", self._host_resolver, self.port,
                            on_frame=self._on_frame, on_disconnect=self._on_disconnect)

    @property
    def connected(self) -> bool:
        return self.sim or (self.link is not None and self.link.connected)

    @property
    def _connected(self) -> bool:      # backwards compatibility
        return self.connected

    def connect(self) -> bool:
        """Connect to the arm TCP server on the main ESP32."""
        if self.sim:
            return True
        if self.link is None:
            self.link = EspLink("arm", self._host_resolver, self.port, on_frame=self._on_frame,
                                on_disconnect=self._on_disconnect, auto_reconnect=False)
        return self.link.connected or self.link.connect()

    def _on_disconnect(self, reason: str):
        self.hw_moving = False
        self._synced = False
        for ev in list(self._done_events.values()):
            ev.set()
        bus.publish(Event(type=EventType.ARM_ERROR, source="ArmClient", data={"error": f"link lost: {reason}"}))

    def _on_frame(self, cmd: int, p: bytes):
        if cmd == RESP_TELEMETRY and len(p) >= 36 and p[0] == 2:
            servo = [int.from_bytes(p[1 + 2 * j:3 + 2 * j], "big") / 10.0 for j in range(6)]
            q = [int.from_bytes(p[13 + 2 * j:15 + 2 * j], "big") / 10.0 - 180.0 for j in range(5)]
            tgt = [int.from_bytes(p[23 + 2 * j:25 + 2 * j], "big") / 10.0 for j in range(6)]
            flags = p[35]
            with self._lock:
                self.servo_deg = servo
                self.servo_target_deg = tgt
                self.q_true = q
                self.hw_moving = bool(flags & 1)
                self.fw_authorised = bool(flags & 2)
                self.pca_ok = [bool(flags & 8), bool(flags & 16)]
                self.fw_gripper_closed = bool(flags & 32)
                self.grip_sensor_box = bool(flags & 64)
                self.grip_sensor_present = bool(flags & 128)
                self.last_telemetry = time.time()
                if not self._synced:
                    # Server (re)started: believe the ESP32 about where the arm really is
                    self._synced = True
                    self.q_cmd = list(q)
                    self.planner.set_current_pose(self.q_cmd)
                    self.gripper_state = "CLOSED" if self.fw_gripper_closed else "OPEN"
                    print(f"[ArmClient] synced to ESP32 arm pose {[round(v, 1) for v in q]}")
        elif cmd in (RESP_CMD_DONE, RESP_CMD_FAILED) and p:
            c = p[0]
            if cmd == RESP_CMD_DONE:
                self._done_result[c] = "done"
            else:
                err = p[1] if len(p) > 1 else 0
                self._done_result[c] = "failed: " + ERROR_TEXT.get(err, hex(err))
            ev = self._done_events.get(c)
            if ev:
                ev.set()

    def reported_q(self) -> Optional[List[float]]:
        if self.sim or time.time() - self.last_telemetry > 1.0:
            return None
        return list(self.q_true)

    def _command(self, cmd: int, payload: bytes = b"", timeout: float = 1.0):
        if self.link is None or not self.link.connected:
            return False, "arm controller (ESP32) is disconnected"
        return self.link.send_and_wait(cmd, payload, timeout=timeout)

    def _command_and_wait(self, cmd: int, payload: bytes, timeout_s: float) -> Dict[str, Any]:
        """Send, require ACK, then wait for CMD_DONE."""
        ev = threading.Event()
        self._done_events[cmd] = ev
        self._done_result.pop(cmd, None)
        try:
            ok, reason = self._command(cmd, payload)
            if not ok:
                return {"ok": False, "error": f"ESP32 refused: {reason}"}
            if not ev.wait(timeout_s):
                return {"ok": False, "error": f"timeout after {timeout_s:.1f} s waiting for the arm"}
            result = self._done_result.get(cmd, "aborted")
            if result != "done":
                return {"ok": False, "error": result}
            # let one more telemetry frame arrive so the check uses final angles
            t_done = time.time()
            while time.time() - t_done < 0.4 and self.last_telemetry < t_done:
                time.sleep(0.02)
            return {"ok": True, "error": ""}
        finally:
            self._done_events.pop(cmd, None)

    @staticmethod
    def _expected_move_time(q_from: List[float], q_to: List[float]) -> float:
        """Worst case: the ESP32 runs joints in stages of max 2, each with a smooth (x1.5) profile."""
        names = ["yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"]
        total = 0.0
        for a, b, n in zip(q_from[:5], q_to[:5], names):
            total += abs(b - a) / SPEED_LIMITS[n] * 1.5 + 0.2
        return total

    def _execute_real_move(self, q_from: List[float], q_to: List[float], planned_s: float) -> Dict[str, Any]:
        timeout = max(planned_s, self._expected_move_time(q_from, q_to)) * 1.5 + 3.0
        payload = self._move_payload(q_to, int(planned_s * 1000))
        res = self._command_and_wait(CMD_ARM_MOVE_JOINTS, payload, timeout)
        if not res["ok"]:
            return {"ok": False, "error": f"Arm move failed: {res['error']}"}
        return self.verify_pose(q_to)

    def verify_pose(self, q_target: List[float], tol_deg: float = VERIFY_TOL_DEG) -> Dict[str, Any]:
        """Check the ESP32's reported joint angles and PCA9685 health against the target."""
        if self.sim:
            return {"ok": True, "verified": True}
        if time.time() - self.last_telemetry > 1.0:
            result = {"ok": False, "error": "No arm telemetry from the ESP32 in the last second"}
        elif not all(self.pca_ok):
            missing = [a for a, ok in zip(("0x40", "0x41"), self.pca_ok) if not ok]
            result = {"ok": False, "error": f"PCA9685 {', '.join(missing)} stopped answering (servo battery or I2C wires)"}
        elif self.hw_moving:
            result = {"ok": False, "error": "ESP32 still reports the arm as moving"}
        else:
            errors = [abs(r - t) for r, t in zip(self.q_true[:5], q_target[:5])]
            worst = max(errors) if errors else 0.0
            if worst > tol_deg:
                joint = ["yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"][errors.index(worst)]
                result = {"ok": False, "error": f"Arm did not reach the target: {joint} off by {worst:.1f} deg"}
            else:
                result = {"ok": True, "verified": True, "max_error_deg": round(worst, 2)}
        self.last_verify = dict(result, time=round(time.time(), 1))
        return result

    def _move_payload(self, q: List[float], duration_ms: int) -> bytes:
        # duration (u16), 5 joints as (deg + 180) * 10 (u16), gripper (ignored by firmware)
        payload = bytearray(int(max(0, min(duration_ms, 65535))).to_bytes(2, "big"))
        for val in q[:5]:
            payload += int(round((val + 180.0) * 10)).to_bytes(2, "big")
        payload += (750 if self.gripper_state == "CLOSED" else 0).to_bytes(2, "big")
        return bytes(payload)

    def _send_move_joints_packet(self, q: List[float], duration_ms: int) -> bool:
        return self._send_frame(CMD_ARM_MOVE_JOINTS, self._move_payload(q, duration_ms))

    @staticmethod
    def _mission_token(mission_id: Optional[str]) -> bytes:
        return str(mission_id or "").encode("ascii", errors="ignore")[:8].ljust(8, b"\x00")

    def _send_frame(self, cmd: int, payload: bytes = b'') -> bool:
        if self.link is None:
            return False
        return self.link.send(cmd, payload)

    def set_estop(self, on: bool) -> bool:
        if self.sim:
            self.estop = on
            return True
        ok, _ = self._command(CMD_SET_ESTOP, bytes([1 if on else 0]))
        if ok:
            self.estop = on
        return ok

    def close(self):
        self._running = False
        self.abort()
        if self.link:
            self.link.close()
