"""
planner.py - Obstacle-aware 2D grid path planning and shortest-path A* for Pharma Rover.
Uses OpenCV distanceTransform for high-performance obstacle inflation.
"""
import math
import heapq
import cv2
import numpy as np
from typing import List, Tuple, Optional

class GridMap:
    def __init__(self, width_mm: float = 3000.0, height_mm: float = 2400.0, cell_mm: float = 20.0):
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.cell = cell_mm
        self.nx = int(width_mm // cell_mm)
        self.ny = int(height_mm // cell_mm)
        self.occ = np.zeros((self.ny, self.nx), dtype=bool)          # True = raw obstacle
        self.inflated = np.zeros((self.ny, self.nx), dtype=bool)
        self.dist = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.keep_free_mask = np.zeros((self.ny, self.nx), dtype=bool)
        self.hard_mm = 165.0
        self.soft_mm = 300.0

    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        ix = int(np.clip(x // self.cell, 0, self.nx - 1))
        iy = int(np.clip(y // self.cell, 0, self.ny - 1))
        return ix, iy

    def to_mm(self, ix: int, iy: int) -> Tuple[float, float]:
        return (ix + 0.5) * self.cell, (iy + 0.5) * self.cell

    def add_box(self, x0: float, y0: float, x1: float, y1: float):
        """Add rectangular obstacle box in mm coordinates."""
        a, b = self.to_cell(x0, y0)
        c, d = self.to_cell(x1, y1)
        x_min, x_max = min(a, c), max(a, c)
        y_min, y_max = min(b, d), max(b, d)
        self.occ[y_min:y_max + 1, x_min:x_max + 1] = True

    def inflate(self, hard_mm: float = 165.0, soft_mm: float = 300.0, keep_free: List[Tuple[float, float, float]] = None):
        """
        Inflate obstacles using Euclidean distance transform.
        hard_mm: robot hard radius + tracking tolerance.
        soft_mm: buffer zone with progressive cost penalty.
        keep_free: list of (x, y, radius) spheres to keep unblocked (e.g. dock area).
        """
        self.hard_mm = hard_mm
        self.soft_mm = soft_mm if soft_mm else hard_mm

        # Pad boundaries so walls count as obstacles
        free_mask = (~self.occ).astype(np.uint8)
        # Set border to 0 (obstacle)
        free_mask[0, :] = 0
        free_mask[-1, :] = 0
        free_mask[:, 0] = 0
        free_mask[:, -1] = 0

        # Fast Euclidean distance transform via OpenCV
        self.dist = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5) * self.cell
        self.inflated = self.dist < hard_mm

        self.keep_free_mask = np.zeros_like(self.inflated)
        if keep_free:
            for (kx, ky, kr) in keep_free:
                k_ix, k_iy = self.to_cell(kx, ky)
                rad_cells = int(kr // self.cell)
                y0, y1 = max(0, k_iy - rad_cells), min(self.ny, k_iy + rad_cells + 1)
                x0, x1 = max(0, k_ix - rad_cells), min(self.nx, k_ix + rad_cells + 1)
                self.inflated[y0:y1, x0:x1] = False
                self.keep_free_mask[y0:y1, x0:x1] = True

    def is_blocked(self, x: float, y: float, relax_mm: float = 0.0) -> bool:
        ix, iy = self.to_cell(x, y)
        if not (0 <= ix < self.nx and 0 <= iy < self.ny):
            return True
        if self.keep_free_mask[iy, ix]:
            return False
        if relax_mm == 0.0:
            return bool(self.inflated[iy, ix])
        return bool(self.dist[iy, ix] < (self.hard_mm - relax_mm))

def astar(grid: GridMap, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
    """8-connected A* search with soft obstacle avoidance."""
    s = grid.to_cell(*start[:2])
    g = grid.to_cell(*goal[:2])

    if grid.inflated[g[1], g[0]] and not grid.keep_free_mask[g[1], g[0]]:
        return None

    hard_eff = min(grid.hard_mm, grid.dist[s[1], s[0]] - 2.0)
    def cell_blocked(c):
        if grid.keep_free_mask[c[1], c[0]]:
            return False
        return bool(grid.dist[c[1], c[0]] < hard_eff)

    moves = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
    ]
    h = lambda a: max(abs(a[0] - g[0]), abs(a[1] - g[1])) + 0.414 * min(abs(a[0] - g[0]), abs(a[1] - g[1]))

    open_queue = [(h(s), 0.0, s)]
    came_from = {s: None}
    cost_so_far = {s: 0.0}

    while open_queue:
        _, current_cost, current = heapq.heappop(open_queue)
        if current == g:
            break
        if current_cost > cost_so_far[current]:
            continue

        for dx, dy, step_w in moves:
            nxt = (current[0] + dx, current[1] + dy)
            if not (0 <= nxt[0] < grid.nx and 0 <= nxt[1] < grid.ny):
                continue
            if cell_blocked(nxt):
                continue
            # Corner cutting check
            if dx != 0 and dy != 0:
                if cell_blocked((nxt[0], current[1])) or cell_blocked((current[0], nxt[1])):
                    continue

            # Soft obstacle penalty
            d_val = grid.dist[nxt[1], nxt[0]]
            near = min(1.0, max(0.0, (grid.soft_mm - d_val) / max(grid.soft_mm - grid.hard_mm, 1.0)))
            new_cost = current_cost + step_w * (1.0 + 2.0 * near)

            if new_cost < cost_so_far.get(nxt, 1e18):
                cost_so_far[nxt] = new_cost
                came_from[nxt] = current
                heapq.heappush(open_queue, (new_cost + h(nxt), new_cost, nxt))

    if g not in came_from:
        return None

    # Reconstruct path
    path = []
    curr = g
    while curr is not None:
        path.append(grid.to_mm(*curr))
        curr = came_from[curr]
    path.reverse()
    path[0] = tuple(start[:2])
    path[-1] = tuple(goal[:2])
    return path

def line_free(grid: GridMap, a: Tuple[float, float], b: Tuple[float, float], relax_mm: float = 0.0) -> bool:
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    n = int(dist / (grid.cell * 0.5)) + 1
    for i in range(n + 1):
        t = i / float(n)
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        if grid.is_blocked(x, y, relax_mm=relax_mm):
            return False
    return True

def smooth_path(grid: GridMap, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Greedy line-of-sight shortcutting: reduce path to minimal corner waypoints."""
    if not path or len(path) <= 2:
        return path
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_free(grid, path[i], path[j]):
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed

def path_length(path: List[Tuple[float, float]]) -> float:
    return sum(math.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]) for i in range(len(path) - 1))
