"""
car_client.py - Pharma Rover base (4 DC motors + 4 ultrasonic sensors) client.
Real mode talks to the main ESP32 on TCP 5000 using esp_link (framed protocol,
see firmware/src/esp32_main/protocol.h). Sim mode uses a kinematic model.

Real mode receives telemetry 10x per second: the 4 ultrasonic distances, whether
the wheels are turning, and obstacle-stop events raised by the ESP32 itself.
"""
import math
import time
import socket
import threading
import numpy as np
from collections import deque
from typing import Optional, Dict, Any, Tuple
from events import bus, Event, EventType
from esp_link import EspLink, RESP_CMD_DONE, RESP_CMD_FAILED, RESP_TELEMETRY, RESP_OBSTACLE, ERROR_TEXT

# Protocol constants from Part 1 / Part 1C
START_BYTE = 0xAA
END_BYTE = 0x55

CMD_DRIVE_FORWARD  = 0x10
CMD_DRIVE_BACKWARD = 0x11
CMD_TURN_LEFT      = 0x12
CMD_TURN_RIGHT     = 0x13
CMD_STOP_MOTORS    = 0x14
CMD_DRIVE_VEL      = 0x16
CMD_SET_ESTOP      = 0x30

RESP_ACK           = 0x01
RESP_NACK          = 0x02
RESP_CMD_DONE      = 0x03
RESP_CMD_FAILED    = 0x04
RESP_HEARTBEAT     = 0x05
RESP_TELEMETRY     = 0x06
RESP_ESTOP         = 0x0D
CMD_SET_OBSTACLE   = 0x17

SENSOR_NAMES = ["front", "back", "left", "right"]
STEP_CMDS = (CMD_DRIVE_FORWARD, CMD_DRIVE_BACKWARD, CMD_TURN_LEFT, CMD_TURN_RIGHT)

def make_frame(cmd: int, payload: bytes = b'') -> bytes:
    length = 1 + len(payload)
    checksum = length ^ cmd
    for b in payload:
        checksum ^= b
    return bytes([START_BYTE, length, cmd]) + payload + bytes([checksum, END_BYTE])

class SimCarKinematics:
    """Kinematic differential drive simulator with noise, drift and acceleration limits."""
    def __init__(self, x=400.0, y=500.0, heading_rad=0.0, wheel_base=150.0, seed=42):
        self.x = float(x)
        self.y = float(y)
        self.heading = float(heading_rad)
        self.wheel_base = wheel_base
        self.v = 0.0
        self.w = 0.0
        self.v_target = 0.0
        self.w_target = 0.0
        self.is_moving = False
        self.rng = np.random.default_rng(seed)
        self.last_update = time.time()
        self.left_obstacle_cm = 999.0
        self.front_obstacle_cm = 999.0
        self.right_obstacle_cm = 999.0

    def step(self, dt=0.05):
        # Acceleration limits
        v_accel = 400.0 * dt
        w_accel = 3.0 * dt
        self.v += np.clip(self.v_target - self.v, -v_accel, v_accel)
        self.w += np.clip(self.w_target - self.w, -w_accel, w_accel)

        if abs(self.v) < 1.0 and abs(self.w) < 0.01 and self.v_target == 0 and self.w_target == 0:
            self.v = 0.0
            self.w = 0.0
            self.is_moving = False
        else:
            self.is_moving = True

        # Wheel slip and calibration error: 2% speed error, 2% turn drift
        vs = self.v * 1.02
        ws = self.w * 0.98 + (0.01 * (vs / 250.0))

        self.heading = (self.heading + ws * dt + math.pi) % (2 * math.pi) - math.pi
        self.x += vs * math.cos(self.heading) * dt
        self.y += vs * math.sin(self.heading) * dt


class CarClient:
    def __init__(self, host: str = "192.168.4.1", port: int = 5000, sim: bool = True,
                 initial_pose=(400.0, 500.0, 0.0), host_resolver=None):
        self.host = host
        self.port = port
        self.sim = sim
        self._lock = threading.Lock()
        self._running = True

        self.sim_car = SimCarKinematics(x=initial_pose[0], y=initial_pose[1], heading_rad=initial_pose[2]) if sim else None
        self.is_moving = False
        self.battery_v = 7.4
        self.obstacle_side: Optional[str] = None
        self.front_obstacle_cm = 120.0
        self.distances_cm: Dict[str, float] = {n: 999.0 for n in SENSOR_NAMES}
        self.estop = False
        self.last_error = ""
        self.last_status_time = time.time()
        self._idle_since = time.time()
        self._step_event = threading.Event()
        self._step_result = "idle"
        # (time, v_mm_s, w_rad_s, kind) of every command sent - used by the learning module
        self._cmd_hist = deque(maxlen=400)

        # Real hardware link (created only when not simulating)
        self._host_resolver = host_resolver or (lambda: self.host)
        self.link: Optional[EspLink] = None
        if not sim:
            self._make_link()

        # Background thread: physics in sim mode
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ link
    def _make_link(self):
        self.link = EspLink("car", self._host_resolver, self.port,
                            on_frame=self._on_frame, on_disconnect=self._on_disconnect)

    @property
    def connected(self) -> bool:
        return self.sim or (self.link is not None and self.link.connected)

    def connect(self) -> bool:
        if self.sim:
            return True
        if self.link is None:
            self._make_link()
        return self.link.connected or self.link.connect()

    def _on_disconnect(self, reason: str):
        self.is_moving = False
        bus.publish(Event(type=EventType.CAR_LINK_LOST, source="CarClient", data={"error": reason}))

    def _on_frame(self, cmd: int, p: bytes):
        if cmd == RESP_TELEMETRY and len(p) >= 17 and p[0] == 1:
            d = {}
            for i, name in enumerate(SENSOR_NAMES):
                mm = int.from_bytes(p[1 + 2 * i:3 + 2 * i], "big")
                d[name] = 999.0 if mm == 0xFFFF else mm / 10.0
            flags = p[9]
            moving = bool(flags & 1)
            with self._lock:
                self.distances_cm = d
                self.front_obstacle_cm = d["front"]
                if moving != self.is_moving and not moving:
                    self._idle_since = time.time()
                self.is_moving = moving
                self.estop = bool(flags & 2)
                mv = int.from_bytes(p[14:16], "big")
                if mv > 0:
                    self.battery_v = round(mv / 1000.0, 2)
                self.last_status_time = time.time()
        elif cmd == RESP_OBSTACLE and len(p) >= 3:
            side = SENSOR_NAMES[p[0]].upper() if p[0] < 4 else "FRONT"
            dist_cm = int.from_bytes(p[1:3], "big") / 10.0
            with self._lock:
                self.obstacle_side = side
                self.is_moving = False
            bus.publish(Event(type=EventType.CAR_OBSTACLE, source="CarClient", data={"side": side, "dist_cm": dist_cm}))
        elif cmd == RESP_CMD_DONE and p and p[0] in STEP_CMDS:
            self._step_result = "done"
            self._step_event.set()
            bus.publish(Event(type=EventType.CAR_ARRIVED, source="CarClient", data={}))
        elif cmd == RESP_CMD_FAILED and p and p[0] in STEP_CMDS:
            err = p[1] if len(p) > 1 else 0
            self._step_result = "failed: " + ERROR_TEXT.get(err, hex(err))
            self._step_event.set()

    def _send_cmd(self, cmd: int, payload: bytes = b"", wait_ack: bool = True) -> bool:
        """Send to the ESP32. With wait_ack, a NACK (e.g. obstacle ahead) returns False."""
        if self.link is None or not self.link.connected:
            self.last_error = "Car controller is disconnected"
            return False
        if not wait_ack:
            return self.link.send(cmd, payload)
        ok, reason = self.link.send_and_wait(cmd, payload, timeout=0.8)
        self.last_error = "" if ok else reason
        return ok

    # Kept for older callers / tests
    def _send_packet(self, cmd: int, payload: bytes = b"") -> bool:
        return self._send_cmd(cmd, payload, wait_ack=False)

    # ------------------------------------------------------------------ motion
    def _log_cmd(self, v: float, w: float, kind: str):
        self._cmd_hist.append((time.time(), float(v), float(w), kind))

    def current_command(self):
        """(v mm/s, w rad/s) of the velocity command in force now (0,0 if stopped / stepping)."""
        if not self._cmd_hist:
            return 0.0, 0.0
        t, v, w, kind = self._cmd_hist[-1]
        if kind != "vel" or time.time() - t > 0.5:
            return 0.0, 0.0
        return v, w

    def command_history(self, t0: float, t1: float):
        """Velocity commands between t0 and t1 (empty if a timed step happened in that window)."""
        out = []
        prev = None
        for entry in self._cmd_hist:
            if entry[0] < t0:
                prev = entry
                continue
            if entry[0] > t1:
                break
            out.append(entry)
        if prev is not None:
            out.insert(0, prev)
        if any(e[3] != "vel" for e in out):
            return []
        return out

    def _start_step(self, cmd: int, payload: bytes) -> bool:
        self._log_cmd(0.0, 0.0, "step")
        self._step_event.clear()
        self._step_result = "running"
        sent = self._send_cmd(cmd, payload)
        with self._lock:
            self.is_moving = sent
        if not sent:
            self._step_result = "failed: " + (self.last_error or "not sent")
        return sent

    def drive_forward(self, dist_mm: float, speed_pct: int = 60) -> bool:
        """Drive straight forward by dist_mm (open-loop timed on the ESP32)."""
        speed_pct = int(np.clip(speed_pct, 10, 100))
        dist_mm = int(min(abs(dist_mm), 65535))
        payload = dist_mm.to_bytes(2, "big") + bytes([speed_pct])
        if self.sim:
            self._log_cmd(0.0, 0.0, "step")
            with self._lock:
                self.is_moving = True
                v = 200.0 * (speed_pct / 100.0)
                self.sim_car.v_target = v
                self.sim_car.w_target = 0.0
                threading.Timer(dist_mm / max(1.0, v), self._sim_reach_target).start()
            return True
        return self._start_step(CMD_DRIVE_FORWARD, payload)

    def drive_backward(self, dist_mm: float, speed_pct: int = 60) -> bool:
        speed_pct = int(np.clip(speed_pct, 10, 100))
        dist_mm = int(min(abs(dist_mm), 65535))
        payload = dist_mm.to_bytes(2, "big") + bytes([speed_pct])
        if self.sim:
            self._log_cmd(0.0, 0.0, "step")
            with self._lock:
                self.is_moving = True
                v = -150.0 * (speed_pct / 100.0)
                self.sim_car.v_target = v
                self.sim_car.w_target = 0.0
                threading.Timer(dist_mm / max(1.0, abs(v)), self._sim_reach_target).start()
            return True
        return self._start_step(CMD_DRIVE_BACKWARD, payload)

    def turn(self, deg: float, speed_pct: int = 50) -> bool:
        """Turn in place. Positive deg = left, negative = right."""
        speed_pct = int(np.clip(speed_pct, 10, 100))
        deg_abs = int(min(abs(deg), 3600))
        payload = deg_abs.to_bytes(2, "big") + bytes([speed_pct])
        if self.sim:
            self._log_cmd(0.0, 0.0, "step")
            with self._lock:
                self.is_moving = True
                w = 1.0 * (speed_pct / 100.0) * (1.0 if deg >= 0 else -1.0)
                self.sim_car.v_target = 0.0
                self.sim_car.w_target = w
                threading.Timer(math.radians(deg_abs) / max(0.1, abs(w)), self._sim_reach_target).start()
            return True
        return self._start_step(CMD_TURN_LEFT if deg >= 0 else CMD_TURN_RIGHT, payload)

    def wait_step_done(self, timeout_s: float) -> str:
        """Wait for a drive/turn step to finish. Returns "done", "failed: ..." or "timeout"."""
        if self.sim:
            t0 = time.time()
            while time.time() - t0 < timeout_s:
                if not self.is_moving:
                    return "done"
                time.sleep(0.05)
            return "timeout"
        if self._step_event.wait(timeout_s):
            return self._step_result
        return "timeout"

    def drive_velocity(self, v_mm_s: float, w_mrad_s: float) -> bool:
        """Continuous driving (CMD_DRIVE_VEL). Resend every ~100 ms; the ESP32 stops after 500 ms of silence."""
        v_int = int(np.clip(v_mm_s, -300, 300))
        w_int = int(np.clip(w_mrad_s, -1500, 1500))
        payload = v_int.to_bytes(2, "big", signed=True) + w_int.to_bytes(2, "big", signed=True)
        self._log_cmd(v_int, w_int / 1000.0, "vel")
        if self.sim:
            with self._lock:
                self.sim_car.v_target = float(v_int)
                self.sim_car.w_target = float(w_int) / 1000.0
                self.is_moving = (abs(v_int) > 5 or abs(w_int) > 20)
            return True
        # No ACK on success (10 Hz); a NACK arrives only for obstacles / e-stop
        sent = self._send_cmd(CMD_DRIVE_VEL, payload, wait_ack=False)
        if sent and (abs(v_int) > 5 or abs(w_int) > 20):
            with self._lock:
                self.is_moving = True
        return sent

    def stop(self) -> bool:
        """Immediate motor stop."""
        self._log_cmd(0.0, 0.0, "stop")
        if self.sim:
            with self._lock:
                self.sim_car.v_target = 0.0
                self.sim_car.w_target = 0.0
                self.sim_car.v = 0.0
                self.sim_car.w = 0.0
                self.is_moving = False
            bus.publish(Event(type=EventType.CAR_STOPPED, source="CarClient", data={"speed": 0.0}))
            return True
        res = self._send_cmd(CMD_STOP_MOTORS, wait_ack=False)
        if self.link is not None and self.link.connected:
            self.link.send(CMD_STOP_MOTORS)          # send twice: cheap insurance
        with self._lock:
            self.is_moving = False
        self._step_result = "stopped"
        self._step_event.set()
        bus.publish(Event(type=EventType.CAR_STOPPED, source="CarClient", data={"speed": 0.0}))
        return res

    def set_obstacle_thresholds(self, front_mm: int, back_mm: int) -> bool:
        """Change the distance at which the ESP32 stops by itself (0 disables). Used while docking."""
        if self.sim:
            return True
        payload = int(max(0, front_mm)).to_bytes(2, "big") + int(max(0, back_mm)).to_bytes(2, "big")
        return self._send_cmd(CMD_SET_OBSTACLE, payload)

    def set_estop(self, on: bool) -> bool:
        if self.sim:
            self.estop = on
            if on:
                self.stop()
            return True
        return self._send_cmd(CMD_SET_ESTOP, bytes([1 if on else 0]))

    def clear_obstacle_flag(self):
        with self._lock:
            self.obstacle_side = None
            if self.sim:
                self.front_obstacle_cm = 120.0
                self.distances_cm["front"] = 120.0

    def idle_for(self) -> float:
        """Seconds since the wheels last stopped (0 while moving)."""
        if self.sim:
            return 0.0 if (self.sim_car and self.sim_car.is_moving) else 999.0
        return 0.0 if self.is_moving else time.time() - self._idle_since

    def status(self) -> Dict[str, Any]:
        with self._lock:
            if self.sim:
                return {
                    "is_moving": self.sim_car.is_moving,
                    "connected": True,
                    "v": round(self.sim_car.v, 1),
                    "w": round(self.sim_car.w, 2),
                    "pose": (round(self.sim_car.x, 1), round(self.sim_car.y, 1), round(math.degrees(self.sim_car.heading), 1)),
                    "battery_v": self.battery_v,
                    "front_obstacle_cm": self.front_obstacle_cm,
                    "distances_cm": dict(self.distances_cm, front=self.front_obstacle_cm),
                    "obstacle_side": self.obstacle_side,
                    "estop": self.estop,
                }
            link = self.link
            return {
                "is_moving": self.is_moving,
                "connected": bool(link and link.connected),
                "esp32_ip": link.current_host if link else None,
                "link_error": link.last_error if (link and not link.connected) else "",
                "telemetry_age_s": round(time.time() - self.last_status_time, 2),
                "battery_v": self.battery_v,
                "front_obstacle_cm": self.front_obstacle_cm,
                "distances_cm": dict(self.distances_cm),
                "obstacle_side": self.obstacle_side,
                "estop": self.estop,
            }

    # ------------------------------------------------------------------ sim
    def _sim_reach_target(self):
        with self._lock:
            if self.sim_car:
                self.sim_car.v_target = 0.0
                self.sim_car.w_target = 0.0
                self.sim_car.v = 0.0
                self.sim_car.w = 0.0
            self.is_moving = False
        bus.publish(Event(type=EventType.CAR_ARRIVED, source="CarClient", data={}))
        bus.publish(Event(type=EventType.CAR_STOPPED, source="CarClient", data={"speed": 0.0}))

    def _run_loop(self):
        while self._running:
            if self.sim and self.sim_car:
                self.sim_car.step(dt=0.05)
                # Simulated obstacle in the room (tests the avoid + re-plan logic)
                if self.sim_car.x > 2500 and 1000 < self.sim_car.y < 1100 and self.obstacle_side is None:
                    self.front_obstacle_cm = 12.0
                    self.obstacle_side = "FRONT"
                    bus.publish(Event(type=EventType.CAR_OBSTACLE, source="CarClient", data={"side": "FRONT", "dist_cm": 12.0}))
            time.sleep(0.05)

    def close(self):
        self._running = False
        try:
            self.stop()
        finally:
            if self.link:
                self.link.close()
