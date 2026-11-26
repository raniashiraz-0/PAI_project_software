"""
arm_planner.py - Shortest-movement trajectory planning, predicted-torque guard,
and high-load hold-time limiter for the 6-servo Pharma Rover arm.
Hardware configuration: 3x MG995 (yaw, shoulder, elbow) + 3x SG90 (wrist pitch, roll, gripper).
"""
import math
import numpy as np
from typing import List, Tuple, Dict, Any, Optional
import ik_arm as K
import kin3d as K3

# Joint weights for shortest joint-space travel calculation (shoulder counts more than wrist)
JOINT_WEIGHTS = [1.2, 2.5, 1.8, 1.0, 0.5]  # yaw, shoulder, elbow, wrist_pitch, roll

# Speed limits per joint (deg/s) from hardware change specs
SPEED_LIMITS = {
    "yaw": 50.0,
    "shoulder": 30.0,       # Slowest: high torque & overloaded servo
    "elbow": 40.0,
    "wrist_pitch": 45.0,
    "wrist_roll": 60.0,
    "gripper": 60.0
}

# Max payload cap (50g)
MAX_PAYLOAD_G = 50.0

# Max hold time in HIGH_LOAD pose (>50% stall torque) before returning to low-torque pose
MAX_HIGH_LOAD_HOLD_S = 3.0

class ArmPlanner:
    def __init__(self, d_off: float = 0.0):
        self.d_off = d_off
        self.current_q = list(K.LIMIT_YAW) # [yaw, q1, q2, q3, roll]
        self.current_q = [0.0, 110.0, -145.0, 60.0, 0.0] # HOME pose default

    def set_current_pose(self, q: List[float]):
        self.current_q = list(q)
        if len(self.current_q) == 4:
            self.current_q.append(0.0)

    def plan_joint_move(self, target_q: List[float], payload_g: float = 0.0) -> Dict[str, Any]:
        """
        Plan a shortest joint-space move using a smooth quintic polynomial.
        Evaluates torque guard and calculates weighted joint distance.
        """
        if payload_g > MAX_PAYLOAD_G:
            return {"ok": False, "error": f"Payload {payload_g}g exceeds 50g hard cap"}

        target_q_full = list(target_q)
        if len(target_q_full) == 4:
            target_q_full.append(self.current_q[4] if len(self.current_q) >= 5 else 0.0)

        # 1. Joint limit check
        if not K.in_limits(target_q_full[:4]):
            return {"ok": False, "error": "Target joints outside physical limits"}

        # 2. Predicted torque guard
        torque_info = K.calculate_torque(target_q_full[:4], payload_g=payload_g)
        if torque_info["is_overloaded"]:
            return {
                "ok": False,
                "error": f"Torque guard: predicted torque exceeds stall limit! {torque_info}"
            }

        # 3. Weighted joint travel
        deltas = [abs(t - c) for t, c in zip(target_q_full[:5], self.current_q[:5])]
        weighted_travel = sum(w * d for w, d in zip(JOINT_WEIGHTS, deltas))

        # 4. Move duration T from slowest joint (quintic curve: T = max(0.8, 1.875 * delta / v_max))
        names = ["yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"]
        durations = [1.875 * d / SPEED_LIMITS[n] for d, n in zip(deltas, names)]
        T = max(0.8, max(durations))

        # 5. Euclidean tip-to-target distance
        tip_start = K3.tip_xyz(self.current_q[:4])
        tip_end = K3.tip_xyz(target_q_full[:4])
        euclidean_tip_dist = float(np.linalg.norm(tip_end - tip_start))

        return {
            "ok": True,
            "target_q": target_q_full,
            "duration_s": round(T, 2),
            "weighted_travel_deg": round(weighted_travel, 1),
            "tip_dist_mm": round(euclidean_tip_dist, 1),
            "torque_info": torque_info,
            "is_high_load": torque_info["is_high_load"],
            "max_hold_time_s": MAX_HIGH_LOAD_HOLD_S if torque_info["is_high_load"] else 999.0
        }

    def plan_cartesian_approach(self, start_xyz: np.ndarray, end_xyz: np.ndarray, step_mm: float = 5.0, phi_deg: float = 0.0) -> Dict[str, Any]:
        """
        Straight-line Cartesian approach in small steps (IK at every 5mm).
        Used strictly for final approach into the shelf and linear retreat.
        """
        dist = float(np.linalg.norm(end_xyz - start_xyz))
        steps = max(2, int(math.ceil(dist / step_mm)))
        waypoint_q = []

        for i in range(steps + 1):
            t = i / float(steps)
            p = start_xyz * (1.0 - t) + end_xyz * t
            q = K3.ik_xyz(p, phi_deg=phi_deg)
            if q is None:
                return {"ok": False, "error": f"Cartesian approach unreachable at step {i}/{steps}: point {p}"}
            waypoint_q.append(q)

        return {
            "ok": True,
            "waypoints": waypoint_q,
            "num_steps": steps,
            "distance_mm": round(dist, 1)
        }

    def select_best_search_pose(self, candidate_poses: List[Tuple[float, float, float]]) -> Optional[Tuple[float, float, float]]:
        """
        Choose pose with smallest weighted joint travel from current position.
        """
        best_pose = None
        min_travel = 1e9

        for p in candidate_poses:
            q = K3.ik_xyz(p, 0.0)
            if q is None:
                continue
            deltas = [abs(t - c) for t, c in zip(q[:4], self.current_q[:4])]
            travel = sum(w * d for w, d in zip(JOINT_WEIGHTS[:4], deltas))
            if travel < min_travel:
                min_travel = travel
                best_pose = p

        return best_pose
