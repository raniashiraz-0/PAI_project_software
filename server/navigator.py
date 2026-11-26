"""
navigator.py - Closed-loop Pure Pursuit waypoint follower and precision dock controller.
Handles dynamic obstacle collision checks and fine shelf docking alignment.
"""
import math
import numpy as np
from typing import List, Tuple
from planner import GridMap, line_free

def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, +pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi

class PurePursuit:
    def __init__(self, path: List[Tuple[float, float]], lookahead: float = 200.0, v_max: float = 250.0, w_max: float = 1.2):
        self.path = [tuple(p[:2]) for p in path]
        self.L = lookahead
        self.v_max = v_max
        self.w_max = w_max

    def command(self, pose: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        pose = (x_mm, y_mm, heading_rad)
        Returns (v_mm_s, w_rad_s, dist_to_goal_mm).
        """
        x, y, th = pose
        P = self.path
        if len(P) == 0:
            return 0.0, 0.0, 0.0

        # Find closest path segment
        best_d = 1e18
        best_i = 0
        pos = np.array([x, y], dtype=float)

        for i in range(len(P) - 1):
            a, b = np.array(P[i], dtype=float), np.array(P[i + 1], dtype=float)
            ab = b - a
            denom = np.dot(ab, ab) + 1e-9
            t = np.clip(np.dot(pos - a, ab) / denom, 0.0, 1.0)
            proj = a + t * ab
            d = np.linalg.norm(proj - pos)
            if d < best_d:
                best_d = d
                best_i = i

        # Lookahead point search
        remaining = self.L
        tgt = np.array(P[-1], dtype=float)
        p_walk = pos.copy()

        for i in range(best_i, len(P) - 1):
            b = np.array(P[i + 1], dtype=float)
            seg_len = np.linalg.norm(b - p_walk)
            if seg_len >= remaining:
                tgt = p_walk + (b - p_walk) * (remaining / max(1e-6, seg_len))
                break
            remaining -= seg_len
            p_walk = b

        goal_dist = float(math.hypot(P[-1][0] - x, P[-1][1] - y))
        alpha = wrap_angle(math.atan2(tgt[1] - y, tgt[0] - x) - th)

        # Large heading error: turn in place first
        if abs(alpha) > math.radians(60):
            w = float(np.clip(2.0 * alpha, -self.w_max, self.w_max))
            return 0.0, w, goal_dist

        dist_to_tgt = max(math.hypot(tgt[0] - x, tgt[1] - y), 1.0)
        curvature = 2.0 * math.sin(alpha) / dist_to_tgt

        # Speed profiling: slow down smoothly near goal
        v = min(self.v_max, 0.9 * goal_dist + 30.0) * max(0.3, math.cos(alpha))
        w = float(np.clip(v * curvature, -self.w_max, self.w_max))
        return float(v), w, goal_dist

def dock_command(pose: Tuple[float, float, float], dock: Tuple[float, float, float], v_cruise: float = 100.0) -> Tuple[float, float, float, float, float, bool]:
    """
    Final straight-line alignment to the shelf slot.
    pose = (x, y, th_rad)
    dock = (xd, yd, td_rad)
    Returns (v, w, along_mm, lat_mm, eth_rad, done).
    """
    x, y, th = pose
    xd, yd, td = dock
    dx, dy = x - xd, y - yd

    # Transform error to dock coordinate frame
    along = dx * math.cos(td) + dy * math.sin(td)     # negative = before dock, positive = overshoot
    lat = -dx * math.sin(td) + dy * math.cos(td)      # lateral error
    eth = wrap_angle(th - td)

    done = (abs(along) < 10.0 and abs(lat) < 10.0 and abs(eth) < math.radians(2.0))
    if done:
        return 0.0, 0.0, along, lat, eth, True

    # Speed control: cruise forward if before dock
    v = float(np.clip(-0.8 * along, 0.0, v_cruise)) if along < -10.0 else 0.0
    # Steering control to correct heading and lateral drift
    w = float(np.clip(-1.5 * eth - 0.006 * lat * (1.0 if v > 0 else 0.0), -0.8, 0.8))
    if v == 0.0 and not done and abs(eth) > math.radians(2.0):
        w = float(np.clip(-1.5 * eth, -0.5, 0.5))

    return v, w, along, lat, eth, done

def path_blocked_ahead(grid: GridMap, path: List[Tuple[float, float]], pose: Tuple[float, float, float], look_mm: float = 700.0) -> bool:
    """True if dynamic obstacle appears on path within look_mm ahead of robot."""
    pos = np.array(pose[:2], dtype=float)
    walked = 0.0
    for i in range(len(path) - 1):
        a = np.array(path[i], dtype=float)
        b = np.array(path[i + 1], dtype=float)
        if np.dot(pos - a, b - a) > np.dot(b - a, b - a):
            continue # Already behind us
        a0 = pos if walked == 0 else a
        if not line_free(grid, tuple(a0), tuple(b), relax_mm=40.0):
            return True
        walked += float(np.linalg.norm(b - a0))
        if walked > look_mm:
            break
    return False
