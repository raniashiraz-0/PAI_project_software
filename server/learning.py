"""
learning.py - The parts of Pharma Rover that LEARN from experience.

1. PoseFilter  - Extended Kalman Filter (unicycle model). Fuses the phone-camera pose
                 (noisy, ~0.25 s late) with the motor commands: smoother pose, latency
                 compensation, and short dead-reckoning when the camera misses a frame.
2. DriveModel  - Online regression (recursive least squares with forgetting) that learns
                 how the REAL car responds to commands:
                     v_real = a_v * v_cmd + b_v
                     w_real = a_w * w_cmd + c_w * v_cmd + b_w      (c_w = drift when driving straight)
                 and inverts it, so the car does what the planner asks (and manual steps
                 like "move forward 50 cm" become accurate).
3. GraspMemory - Learns a per-slot grasp correction from visual servoing (exponential
                 moving average), remembers where each medicine was last found on the
                 shelf, and logs every mission for statistics.

Everything is saved in data/learned/ and survives restarts.
"""
import json
import math
import os
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# =============================================================================
#  1. Extended Kalman Filter for the car pose
# =============================================================================
class PoseFilter:
    """State x = [x_mm, y_mm, theta_rad]. Predict with (v, w), update with camera poses."""

    def __init__(self, pos_noise_mm: float = 6.0, heading_noise_deg: float = 1.0,
                 process_v: float = 40.0, process_w: float = 0.25, latency_s: float = 0.25,
                 max_predict_s: float = 0.8):
        self.x: Optional[np.ndarray] = None
        self.P = np.diag([400.0, 400.0, 0.05])
        self.t = 0.0                                  # time the state refers to
        self.R = np.diag([pos_noise_mm ** 2, pos_noise_mm ** 2, math.radians(heading_noise_deg) ** 2])
        self.q_v, self.q_w = process_v, process_w     # process noise (mm/s, rad/s)
        self.latency_s = latency_s
        self.max_predict_s = max_predict_s
        self.last_update = 0.0
        self.innovations: Deque[float] = deque(maxlen=50)
        self._lock = threading.Lock()

    @staticmethod
    def _f(x: np.ndarray, v: float, w: float, dt: float) -> np.ndarray:
        th = x[2]
        if abs(w) < 1e-4:
            return np.array([x[0] + v * dt * math.cos(th), x[1] + v * dt * math.sin(th), th])
        return np.array([x[0] + v / w * (math.sin(th + w * dt) - math.sin(th)),
                         x[1] - v / w * (math.cos(th + w * dt) - math.cos(th)),
                         wrap(th + w * dt)])

    def _predict(self, x, P, v, w, dt):
        if dt <= 0:
            return x, P
        th = x[2]
        F = np.array([[1, 0, -v * dt * math.sin(th)],
                      [0, 1, v * dt * math.cos(th)],
                      [0, 0, 1]])
        G = np.array([[dt * math.cos(th), 0], [dt * math.sin(th), 0], [0, dt]])
        Q = G @ np.diag([self.q_v ** 2, self.q_w ** 2]) @ G.T + np.diag([1.0, 1.0, 1e-5]) * dt
        return self._f(x, v, w, dt), F @ P @ F.T + Q

    def update(self, meas: Tuple[float, float, float], t_meas: float, v: float = 0.0, w: float = 0.0):
        """meas = (x_mm, y_mm, theta_rad) taken at t_meas (= receive time - latency)."""
        z = np.array(meas, dtype=float)
        with self._lock:
            if self.x is None or t_meas - self.t > 2.0:
                self.x, self.P, self.t = z.copy(), np.diag([100.0, 100.0, 0.01]), t_meas
                self.last_update = time.time()
                return
            self.x, self.P = self._predict(self.x, self.P, v, w, max(0.0, t_meas - self.t))
            self.t = max(self.t, t_meas)
            y = z - self.x
            y[2] = wrap(y[2])
            S = self.P + self.R
            K = self.P @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.x[2] = wrap(self.x[2])
            self.P = (np.eye(3) - K) @ self.P
            self.last_update = time.time()
            self.innovations.append(float(math.hypot(y[0], y[1])))

    def predict_now(self, now: float, v: float = 0.0, w: float = 0.0) -> Optional[Tuple[float, float, float]]:
        """Pose at 'now' (does not change the stored state). None if the camera is too old."""
        with self._lock:
            if self.x is None or now - self.t > self.max_predict_s + self.latency_s:
                return None
            x, _ = self._predict(self.x.copy(), self.P.copy(), v, w, now - self.t)
            return float(x[0]), float(x[1]), float(x[2])

    def reset(self):
        with self._lock:
            self.x = None

    def summary(self) -> Dict[str, Any]:
        inn = list(self.innovations)
        return {"active": self.x is not None,
                "camera_age_s": round(time.time() - self.last_update, 2) if self.last_update else None,
                "avg_correction_mm": round(float(np.mean(inn)), 1) if inn else None,
                "std_pos_mm": round(float(math.sqrt(max(self.P[0, 0], 0))), 1) if self.x is not None else None}


# =============================================================================
#  2. Online regression (recursive least squares)
# =============================================================================
class RLS:
    """y = theta . phi, updated one sample at a time; lam < 1 slowly forgets old data."""

    def __init__(self, n: int, lam: float = 0.995, theta0=None, p0: float = 100.0):
        self.n = n
        self.lam = lam
        self.theta = np.array(theta0 if theta0 is not None else np.zeros(n), dtype=float)
        self.P = np.eye(n) * p0
        self.samples = 0
        self.err_sq: Deque[float] = deque(maxlen=200)

    def update(self, phi, y: float):
        phi = np.asarray(phi, dtype=float)
        e = y - float(self.theta @ phi)
        Pphi = self.P @ phi
        k = Pphi / (self.lam + float(phi @ Pphi))
        self.theta = self.theta + k * e
        self.P = (self.P - np.outer(k, Pphi)) / self.lam
        self.samples += 1
        self.err_sq.append(e * e)

    def rmse(self) -> Optional[float]:
        return float(math.sqrt(np.mean(self.err_sq))) if self.err_sq else None

    def to_dict(self):
        return {"theta": self.theta.tolist(), "P": self.P.tolist(), "samples": self.samples}

    def load(self, d):
        self.theta = np.array(d["theta"], dtype=float)
        self.P = np.array(d["P"], dtype=float)
        self.samples = int(d.get("samples", 0))


class DriveModel:
    MIN_SAMPLES = 40

    def __init__(self):
        # v model: [a_v, b_v] on [v_cmd, 1]; w model: [a_w, c_w, b_w] on [w_cmd, v_cmd/1000, 1]
        self.v_rls = RLS(2, theta0=[1.0, 0.0], p0=10.0)
        self.w_rls = RLS(3, theta0=[1.0, 0.0, 0.0], p0=10.0)
        self.enabled = True

    # ---- learning ----
    def add_sample(self, v_cmd: float, w_cmd: float, v_meas: float, w_meas: float):
        if abs(v_cmd) > 30.0:
            self.v_rls.update([v_cmd, 1.0], v_meas)
        if abs(w_cmd) > 0.12 or abs(v_cmd) > 60.0:
            self.w_rls.update([w_cmd, v_cmd / 1000.0, 1.0], w_meas)

    # ---- parameters (clamped so a bad sample can never make the car crazy) ----
    def params(self) -> Dict[str, float]:
        a_v, b_v = self.v_rls.theta
        a_w, c_w, b_w = self.w_rls.theta
        return {"a_v": float(np.clip(a_v, 0.6, 1.6)), "b_v": float(np.clip(b_v, -30, 30)),
                "a_w": float(np.clip(a_w, 0.6, 1.6)), "c_w": float(np.clip(c_w, -0.3, 0.3)),
                "b_w": float(np.clip(b_w, -0.15, 0.15))}

    # A model is only USED when it has enough samples AND predicts well (small error).
    MAX_V_RMSE = 35.0      # mm/s
    MAX_W_RMSE = 0.12      # rad/s

    @property
    def v_ready(self) -> bool:
        r = self.v_rls.rmse()
        return self.enabled and self.v_rls.samples >= self.MIN_SAMPLES and r is not None and r < self.MAX_V_RMSE

    @property
    def w_ready(self) -> bool:
        r = self.w_rls.rmse()
        return self.enabled and self.w_rls.samples >= self.MIN_SAMPLES and r is not None and r < self.MAX_W_RMSE

    # ---- use ----
    def predict(self, v_cmd: float, w_cmd: float) -> Tuple[float, float]:
        """What the car will REALLY do for a command (used by the Kalman filter)."""
        p = self.params()
        v = p["a_v"] * v_cmd + (p["b_v"] if abs(v_cmd) > 1 else 0.0) if self.v_ready else v_cmd
        w = (p["a_w"] * w_cmd + p["c_w"] * v_cmd / 1000.0 + (p["b_w"] if abs(v_cmd) > 1 or abs(w_cmd) > 0.01 else 0.0)) \
            if self.w_ready else w_cmd
        return v, w

    def correct(self, v_des: float, w_des: float) -> Tuple[float, float]:
        """Command to send so the car does (v_des, w_des)."""
        if abs(v_des) < 1.0 and abs(w_des) < 0.01:
            return 0.0, 0.0
        p = self.params()
        v_cmd = (v_des - (p["b_v"] if abs(v_des) > 1 else 0.0)) / p["a_v"] if self.v_ready else v_des
        if self.w_ready:
            w_cmd = (w_des - p["c_w"] * v_cmd / 1000.0 - p["b_w"]) / p["a_w"]
        else:
            w_cmd = w_des
        return float(v_cmd), float(w_cmd)

    def step_distance(self, dist_mm: float) -> float:
        """Distance to send for an open-loop step so the car REALLY travels dist_mm."""
        return dist_mm / self.effective_speed_gain(150.0) if self.v_ready else dist_mm

    def step_angle(self, deg: float) -> float:
        return deg / self.params()["a_w"] if self.w_ready else deg

    def effective_speed_gain(self, v: float = 180.0) -> float:
        """Learned real/commanded speed at a typical speed (shown even before it is used)."""
        p = self.params()
        return (p["a_v"] * v + p["b_v"]) / v

    def summary(self) -> Dict[str, Any]:
        p = self.params()
        return {"speed_gain": round(self.effective_speed_gain(), 3), "turn_gain": round(p["a_w"], 3),
                "drift_rad_per_m": round(p["c_w"], 3), "turn_bias_rad_s": round(p["b_w"], 3),
                "speed_samples": self.v_rls.samples, "turn_samples": self.w_rls.samples,
                "speed_ready": self.v_ready, "turn_ready": self.w_ready,
                "speed_rmse_mm_s": round(self.v_rls.rmse(), 1) if self.v_rls.rmse() is not None else None,
                "turn_rmse_rad_s": round(self.w_rls.rmse(), 3) if self.w_rls.rmse() is not None else None}

    def to_dict(self):
        return {"v": self.v_rls.to_dict(), "w": self.w_rls.to_dict()}

    def load(self, d):
        self.v_rls.load(d["v"])
        self.w_rls.load(d["w"])


# =============================================================================
#  3. Grasp corrections, shelf memory, mission log
# =============================================================================
class GraspMemory:
    def __init__(self, alpha: float = 0.35):
        self.alpha = alpha
        self.offsets: Dict[str, Dict[str, float]] = {}   # slot -> {"dy": mm, "dz": mm, "n": grips}
        self.last_seen: Dict[str, Dict[str, Any]] = {}   # medicine id -> {"slot_u": mm, "time": ...}
        self.missions: List[Dict[str, Any]] = []

    @staticmethod
    def _key(slot_u: float) -> str:
        return str(int(round(slot_u / 10.0) * 10))       # group slots to 10 mm

    def offset_for(self, slot_u: float) -> Tuple[float, float]:
        o = self.offsets.get(self._key(slot_u))
        return (o["dy"], o["dz"]) if o else (0.0, 0.0)

    def learn_offset(self, slot_u: float, dy: float, dz: float):
        """Called after a CONFIRMED grip with the total correction that worked."""
        k = self._key(slot_u)
        o = self.offsets.get(k)
        if o is None:
            self.offsets[k] = {"dy": round(dy, 1), "dz": round(dz, 1), "n": 1}
        else:
            o["dy"] = round((1 - self.alpha) * o["dy"] + self.alpha * dy, 1)
            o["dz"] = round((1 - self.alpha) * o["dz"] + self.alpha * dz, 1)
            o["n"] += 1

    def remember_location(self, med_id: int, slot_u: float):
        self.last_seen[str(med_id)] = {"slot_u": float(slot_u), "time": time.strftime("%Y-%m-%d %H:%M:%S")}

    def scan_order(self, med_id: int, expected_slot: float, slots: List[float]) -> List[float]:
        """Last place this medicine was seen first, then its home slot, then the closest others."""
        seen = self.last_seen.get(str(med_id))
        first = seen["slot_u"] if seen else expected_slot
        order = sorted(slots, key=lambda u: (abs(u - first), abs(u - expected_slot)))
        return order

    def log_mission(self, entry: Dict[str, Any]):
        self.missions.append(entry)
        self.missions = self.missions[-200:]

    def stats(self) -> Dict[str, Any]:
        m = self.missions
        if not m:
            return {"missions": 0}
        ok = [x for x in m if x.get("success")]
        by_med: Dict[str, Dict[str, int]] = {}
        for x in m:
            d = by_med.setdefault(x.get("medicine", "?"), {"runs": 0, "ok": 0})
            d["runs"] += 1
            d["ok"] += 1 if x.get("success") else 0
        recent = m[-10:]
        return {"missions": len(m), "success_rate": round(len(ok) / len(m), 3),
                "recent_success_rate": round(sum(1 for x in recent if x.get("success")) / len(recent), 3),
                "avg_duration_s": round(float(np.mean([x["duration_s"] for x in ok])), 1) if ok else None,
                "avg_grip_attempts": round(float(np.mean([x.get("grip_attempts", 1) for x in ok])), 2) if ok else None,
                "by_medicine": by_med,
                "last_failure": next((x.get("error") for x in reversed(m) if not x.get("success")), None)}

    def to_dict(self):
        return {"offsets": self.offsets, "last_seen": self.last_seen, "missions": self.missions}

    def load(self, d):
        self.offsets = d.get("offsets", {})
        self.last_seen = d.get("last_seen", {})
        self.missions = d.get("missions", [])


# =============================================================================
#  The learner: collects samples, applies models, saves to disk
# =============================================================================
class Learning:
    def __init__(self, data_dir: str, config: Optional[Dict[str, Any]] = None, sim: bool = True):
        cfg = (config or {}).get("learning", {})
        self.enabled = bool(cfg.get("enabled", True))
        self.drive_learning = bool(cfg.get("drive_model", True))
        self.sim = sim
        os.makedirs(data_dir, exist_ok=True)
        suffix = "_sim" if sim else ""
        self.drive_path = os.path.join(data_dir, f"drive_model{suffix}.json")
        self.memory_path = os.path.join(data_dir, f"grasp_memory{suffix}.json")

        self.filter = PoseFilter(latency_s=float(cfg.get("camera_latency_s", 0.25)),
                                 pos_noise_mm=float(cfg.get("camera_noise_mm", 6.0)))
        self.drive = DriveModel()
        self.drive.enabled = self.enabled and self.drive_learning
        self.memory = GraspMemory(alpha=float(cfg.get("grasp_learning_rate", 0.35)))
        self._load()

        self.car = None                                   # set by server (command history)
        self._poses: Deque[Tuple[float, float, float, float]] = deque(maxlen=60)   # t, x, y, th
        self._dirty = False
        self._last_save = time.time()
        self._lock = threading.Lock()

    # ---------------- persistence ----------------
    def _load(self):
        for path, obj in ((self.drive_path, self.drive), (self.memory_path, self.memory)):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    obj.load(json.load(f))
                print(f"[Learning] loaded {os.path.basename(path)}")
            except FileNotFoundError:
                pass
            except Exception as exc:
                print(f"[Learning] could not read {path}: {exc} (starting fresh)")

    def save(self, force: bool = False):
        if not (self._dirty or force):
            return
        for path, obj in ((self.drive_path, self.drive), (self.memory_path, self.memory)):
            try:
                tmp = path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(obj.to_dict(), f, indent=1)
                os.replace(tmp, path)
            except OSError as exc:
                print(f"[Learning] could not save {path}: {exc}")
        self._dirty = False
        self._last_save = time.time()

    def maybe_save(self):
        if self._dirty and time.time() - self._last_save > 20.0:
            self.save()

    def reset(self):
        self.drive = DriveModel()
        self.drive.enabled = self.enabled and self.drive_learning
        self.memory = GraspMemory(alpha=self.memory.alpha)
        self.filter.reset()
        self._dirty = True
        self.save(force=True)

    # ---------------- commands ----------------
    def command_now(self) -> Tuple[float, float]:
        """Real (v mm/s, w rad/s) the car is doing right now, from the last command."""
        if self.car is None:
            return 0.0, 0.0
        v, w = self.car.current_command()
        return self.drive.predict(v, w)

    def correct_command(self, v_des: float, w_des: float) -> Tuple[float, float]:
        return self.drive.correct(v_des, w_des) if self.enabled else (v_des, w_des)

    # ---------------- camera poses -> drive samples ----------------
    def on_camera_pose(self, x: float, y: float, th: float, t: float):
        """Called for every RAW camera measurement (t = capture time)."""
        if not (self.enabled and self.drive.enabled and self.car is not None):
            return
        with self._lock:
            self._poses.append((t, x, y, th))
            # need ~0.5 s of motion with an almost constant command
            # newest pose that is at least 0.7 s older than this one (longer window = less camera noise)
            old = next((p for p in reversed(self._poses) if t - p[0] >= 0.7), None)
            if old is None or t - old[0] > 1.5:
                return
            # the command must have been steady since 0.3 s BEFORE the window, so the car
            # had time to reach that speed (skip the speeding-up / slowing-down part)
            cmds = self.car.command_history(old[0] - 0.3, t)
            if len(cmds) < 2:
                return
            vs = [c[1] for c in cmds]
            ws = [c[2] for c in cmds]
            if np.std(vs) > 8.0 or np.std(ws) > 0.03:
                return                                      # command changed: not a clean sample
            dt = t - old[0]
            dx, dy = x - old[1], y - old[2]
            th_mid = wrap(old[3] + wrap(th - old[3]) / 2)
            v_meas = (dx * math.cos(th_mid) + dy * math.sin(th_mid)) / dt
            w_meas = wrap(th - old[3]) / dt
            if abs(v_meas) > 800 or abs(w_meas) > 4.0:
                return                                      # tracking glitch
            self.drive.add_sample(float(np.mean(vs)), float(np.mean(ws)), v_meas, w_meas)
            self._dirty = True
            # drop the used window so samples are independent
            while self._poses and self._poses[0][0] < t - 0.1:
                self._poses.popleft()
        self.maybe_save()

    # ---------------- arm ----------------
    def learn_grasp(self, slot_u: float, dy: float, dz: float):
        if self.enabled:
            self.memory.learn_offset(slot_u, dy, dz)
            self._dirty = True
            self.save()

    def remember_location(self, med_id: int, slot_u: float):
        self.memory.remember_location(med_id, slot_u)
        self._dirty = True

    def log_mission(self, entry: Dict[str, Any]):
        self.memory.log_mission(entry)
        self._dirty = True
        self.save()

    def summary(self) -> Dict[str, Any]:
        return {"enabled": self.enabled, "kalman": self.filter.summary(), "drive": self.drive.summary(),
                "grasp_offsets": self.memory.offsets, "shelf_memory": self.memory.last_seen,
                "stats": self.memory.stats()}
