"""
navigation.py - obstacle-aware path planning and driving for the Pharma Rover (runs on the laptop).
  occupancy grid -> inflate by the robot size -> A* -> straight-line smoothing -> pure-pursuit driving -> dock controller
The phone camera (vision.py) supplies the robot pose and the obstacle map; this file turns them into (v, w) commands.
Units: mm, radians, mm/s, rad/s.  Grid index is [row=iy, col=ix]; cell (ix,iy) covers x in [ix*cell, (ix+1)*cell).
"""
import heapq, math
import numpy as np
from scipy import ndimage

class Grid:
    def __init__(self, width_mm=3000, height_mm=2400, cell_mm=20):
        self.cell = cell_mm
        self.nx, self.ny = int(width_mm // cell_mm), int(height_mm // cell_mm)
        self.occ = np.zeros((self.ny, self.nx), bool)          # True = obstacle (raw)
        self.inflated = self.occ.copy()
    def to_cell(self, x, y): return int(x // self.cell), int(y // self.cell)
    def to_mm(self, ix, iy): return (ix + 0.5) * self.cell, (iy + 0.5) * self.cell
    def add_box(self, x0, y0, x1, y1): 
        a, b = self.to_cell(x0, y0); c, d = self.to_cell(x1, y1)
        self.occ[max(b, 0):d + 1, max(a, 0):c + 1] = True
    def inflate(self, hard_mm, soft_mm=None, keep_free=()):
        """hard_mm: the robot CENTRE may never come closer than this to an obstacle or wall (robot half-size + tracking margin).
        soft_mm: between hard and soft the planner adds extra cost, so paths keep a comfortable distance when there is room."""
        padded = np.pad(~self.occ, 1, constant_values=False)    # the walls count as obstacles
        dist = ndimage.distance_transform_edt(padded)[1:-1, 1:-1] * self.cell
        self.dist, self.hard, self.soft = dist, hard_mm, (soft_mm or hard_mm)
        self.inflated = dist < hard_mm
        self.keep = np.zeros_like(self.inflated)
        for (x, y, r) in keep_free:                              # e.g. the dock area in front of the shelf
            ix, iy = self.to_cell(x, y); rr = int(r // self.cell)
            self.inflated[max(iy - rr, 0):iy + rr + 1, max(ix - rr, 0):ix + rr + 1] = False
            self.keep[max(iy - rr, 0):iy + rr + 1, max(ix - rr, 0):ix + rr + 1] = True
    def blocked(self, x, y, relax_mm=0.0):
        ix, iy = self.to_cell(x, y)
        if not (0 <= ix < self.nx and 0 <= iy < self.ny): return True
        return self.inflated[iy, ix] if relax_mm == 0 else bool(self.dist[iy, ix] < self.hard - relax_mm)

def astar(grid, start, goal):
    """8-connected A*. start/goal in mm. Returns [(x,y) mm,...] or None. Prefers cells away from obstacles."""
    start, goal = tuple(start[:2]), tuple(goal[:2])
    s, g = grid.to_cell(*start), grid.to_cell(*goal)
    if grid.inflated[g[1], g[0]]: return None
    hard_eff = min(grid.hard, grid.dist[s[1], s[0]] - 2.0)       # if we are already too close to something, allow moving AWAY from it
    is_blocked = lambda c: bool(grid.dist[c[1], c[0]] < hard_eff and not grid.keep[c[1], c[0]])
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    h = lambda a: max(abs(a[0] - g[0]), abs(a[1] - g[1])) + 0.414 * min(abs(a[0] - g[0]), abs(a[1] - g[1]))
    openq = [(h(s), 0.0, s)]; came = {s: None}; cost = {s: 0.0}
    while openq:
        _, c, cur = heapq.heappop(openq)
        if cur == g: break
        if c > cost[cur]: continue
        for dx, dy, w in moves:
            n = (cur[0] + dx, cur[1] + dy)
            if not (0 <= n[0] < grid.nx and 0 <= n[1] < grid.ny): continue
            if is_blocked(n): continue
            if dx and dy and (is_blocked((n[0], cur[1])) or is_blocked((cur[0], n[1]))): continue   # no corner cutting
            near = min(1.0, max(0.0, (grid.soft - grid.dist[n[1], n[0]]) / max(grid.soft - grid.hard, 1.0)))   # 1 at the hard limit, 0 at soft
            nc = c + w * (1.0 + 2.0 * near)
            if nc < cost.get(n, 1e18):
                cost[n] = nc; came[n] = cur; heapq.heappush(openq, (nc + h(n), nc, n))
    if g not in came: return None
    path = []; cur = g
    while cur is not None: path.append(grid.to_mm(*cur)); cur = came[cur]
    path.reverse(); path[0] = tuple(start); path[-1] = tuple(goal)
    return path

def line_free(grid, a, b, relax_mm=0.0):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) / (grid.cell / 2)) + 1
    for i in range(n + 1):
        t = i / n; x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        if grid.blocked(x, y, relax_mm): return False
    return True

def smooth(grid, path):
    """Greedy line-of-sight shortcutting: keep only the corner points that are really needed."""
    out = [path[0]]; i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_free(grid, path[i], path[j]): j -= 1
        out.append(path[j]); i = j
    return out

def path_blocked_ahead(grid, path, pose, look_mm=700.0):
    """True if the planned path within look_mm of the robot now crosses a forbidden cell (a new obstacle appeared)."""
    pos = np.array(pose[:2], float); walked = 0.0
    for i in range(len(path) - 1):
        a, b = np.array(path[i], float), np.array(path[i + 1], float)
        if np.dot(pos - a, b - a) > np.dot(b - a, b - a): continue            # segment already behind us
        a0 = pos if walked == 0 else a
        if not line_free(grid, tuple(a0), tuple(b), relax_mm=40.0): return True
        walked += np.linalg.norm(b - a0)
        if walked > look_mm: break
    return False

def path_length(p): return sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

class PurePursuit:
    def __init__(self, path, lookahead=250.0, v_max=250.0, w_max=1.2):
        self.path, self.L, self.vmax, self.wmax = path, lookahead, v_max, w_max
    def command(self, pose):
        x, y, th = pose; P = self.path
        # closest segment, then a lookahead point further along the path
        best, bi = 1e18, 0
        for i in range(len(P) - 1):
            a, b = np.array(P[i]), np.array(P[i + 1]); ab = b - a
            t = np.clip(np.dot((x, y) - a, ab) / (np.dot(ab, ab) + 1e-9), 0, 1); d = np.linalg.norm(a + t * ab - (x, y))
            if d < best: best, bi = d, i
        remaining, tgt = self.L, np.array(P[-1], float); pos = np.array([x, y])
        for i in range(bi, len(P) - 1):
            b = np.array(P[i + 1], float); seg = np.linalg.norm(b - pos)
            if seg >= remaining: tgt = pos + (b - pos) * remaining / seg; break
            remaining -= seg; pos = b
        goal_dist = math.dist((x, y), P[-1])
        alpha = wrap(math.atan2(tgt[1] - y, tgt[0] - x) - th)
        if abs(alpha) > math.radians(60):                       # large heading error: turn on the spot first
            return 0.0, float(np.clip(2.0 * alpha, -self.wmax, self.wmax)), goal_dist
        kappa = 2 * math.sin(alpha) / max(math.dist((x, y), tgt), 1.0)
        v = min(self.vmax, 0.9 * goal_dist + 30.0) * max(0.3, math.cos(alpha))
        w = float(np.clip(v * kappa, -self.wmax, self.wmax))
        return float(v), w, goal_dist

def dock_command(pose, dock, v_cruise=100.0):
    """Final approach along the dock line. dock = (x, y, heading). Returns (v, w, along_mm, lateral_mm, heading_err_rad, done)."""
    x, y, th = pose; xd, yd, td = dock
    dx, dy = x - xd, y - yd
    along = dx * math.cos(td) + dy * math.sin(td)               # >0 = past the dock point, <0 = still before it
    lat = -dx * math.sin(td) + dy * math.cos(td)                # >0 = left of the dock line
    eth = wrap(th - td)
    done = abs(along) < 10 and abs(lat) < 10 and abs(eth) < math.radians(2)
    v = float(np.clip(-0.8 * along, 0, v_cruise)) if along < -10 else 0.0
    w = float(np.clip(-1.5 * eth - 0.006 * lat * (1 if v > 0 else 0), -0.8, 0.8))
    if v == 0 and not done and abs(eth) > math.radians(2): w = float(np.clip(-1.5 * eth, -0.5, 0.5))
    return v, w, along, lat, eth, done

# ---------------- simulation helpers (used by tests and by mission.py --sim) ----------------
class SimRobot:
    def __init__(self, pose, wheel_base=150.0, seed=0):
        self.x, self.y, self.th = pose; self.b = wheel_base; self.rng = np.random.default_rng(seed)
        self.v = self.w = 0.0
    def step(self, v, w, dt=0.1):
        v = np.clip(v, -300, 300); w = np.clip(w, -1.5, 1.5)
        self.v += np.clip(v - self.v, -400 * dt, 400 * dt); self.w += np.clip(w - self.w, -3 * dt, 3 * dt)   # acceleration limits
        vs, ws = self.v * 1.02, self.w * 0.98                                                                   # 2% wheel-calibration error
        self.th = wrap(self.th + ws * dt); self.x += vs * math.cos(self.th) * dt; self.y += vs * math.sin(self.th) * dt
    def measure(self, sigma_mm=6.0, sigma_deg=0.6):                                                             # what the phone camera reports
        return (self.x + self.rng.normal(0, sigma_mm), self.y + self.rng.normal(0, sigma_mm), wrap(self.th + math.radians(self.rng.normal(0, sigma_deg))))
