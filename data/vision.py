"""
vision.py - the two cameras of the Pharma Rover (runs on the laptop).
  RoomCamera : the PHONE on the wall. Finds the floor plane from 4 floor markers, the robot and the shelf from their markers,
               and the obstacles by comparing the live view with a picture of the empty room.
  HandCamera : the ESP32-CAM on the gripper. Finds a package marker and returns its 3D pose (4x4 matrix) in the camera frame.
All geometry is linear algebra: a pinhole camera K, a pose (R, t), and ray-plane intersection.
"""
import math
import cv2
import numpy as np

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_DET = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())

def detect_markers(img):
    """Return {marker_id: 4x2 corners (TL, TR, BR, BL)} for all ArUco markers in the image."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    corners, ids, _ = _DET.detectMarkers(gray)
    return {} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}

def square_object_points(size_mm):
    s = size_mm / 2.0                       # marker frame: x right, y up, z out of the marker toward the viewer
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

class RoomCamera:
    def __init__(self, K, dist=None):
        self.K = np.asarray(K, np.float64); self.dist = np.zeros(5) if dist is None else np.asarray(dist, np.float64)
        self.R = self.t = self.C = None

    # ---- step 1: find the camera pose from the 4 floor markers (their room positions are known) ----
    def calibrate(self, img, floor_centres_mm, size_mm):
        found = detect_markers(img); obj, im = [], []
        for mid, (cx, cy) in floor_centres_mm.items():
            mid = int(mid)
            if mid not in found: continue
            s = size_mm / 2.0                                   # seen from above: TL, TR, BR, BL (y up)
            obj += [[cx - s, cy + s, 0], [cx + s, cy + s, 0], [cx + s, cy - s, 0], [cx - s, cy - s, 0]]; im += found[mid].tolist()
        if len(obj) < 8: raise RuntimeError("need at least 2 floor markers, found %d" % (len(obj) // 4))
        obj, im = np.array(obj, np.float64), np.array(im, np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, im, self.K, self.dist, flags=cv2.SOLVEPNP_ITERATIVE)
        self.R, _ = cv2.Rodrigues(rvec); self.t = tvec.reshape(3); self.C = -self.R.T @ self.t
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.dist)
        self.reproj_px = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - im) ** 2, axis=1))))
        return self.reproj_px

    # ---- pixel -> room, for a point at a known height z (ray / plane intersection) ----
    def pixel_to_room(self, uv, z=0.0):
        uvn = cv2.undistortPoints(np.array([[uv]], np.float64), self.K, self.dist).reshape(2)
        ray = self.R.T @ np.array([uvn[0], uvn[1], 1.0])         # direction of the pixel's ray in room coordinates
        s = (z - self.C[2]) / ray[2]                              # distance along the ray until it reaches height z
        return self.C + s * ray

    def marker_pose(self, corners, height_mm):
        """Pose (x, y, heading) of a FLAT marker at height_mm whose 'up' edge points in the robot/shelf direction."""
        pts = np.array([self.pixel_to_room(c, height_mm)[:2] for c in corners])      # 4 corners in the room plane
        centre = pts.mean(axis=0)
        up = (pts[0] + pts[1]) / 2 - (pts[3] + pts[2]) / 2                           # bottom edge -> top edge
        return float(centre[0]), float(centre[1]), float(math.atan2(up[1], up[0]))

    # ---- obstacles: top-down view of the floor, compare with the empty room ----
    def floor_homography(self):
        """3x3 matrix mapping floor points (x_mm, y_mm, 1) to image pixels (for z = 0)."""
        return self.K @ np.column_stack([self.R[:, 0], self.R[:, 1], self.t])

    def top_down(self, img, room_mm=(3000, 2400), px_per_mm=0.25):
        W, H = int(room_mm[0] * px_per_mm), int(room_mm[1] * px_per_mm)
        A = np.array([[1 / px_per_mm, 0, 0], [0, -1 / px_per_mm, room_mm[1]], [0, 0, 1.0]])   # top-down pixel -> floor mm
        M = self.floor_homography() @ A                                                      # top-down pixel -> camera pixel
        return cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)

    def obstacle_grid(self, img, ref, cell_mm=20, room_mm=(3000, 2400), px_per_mm=0.25, thresh=45, exclude=()):
        """Boolean occupancy grid [iy, ix]. exclude = [(x_mm, y_mm, radius_mm), ...] areas to ignore (robot, shelf, dock)."""
        a = cv2.GaussianBlur(self.top_down(img, room_mm, px_per_mm), (5, 5), 0)
        b = cv2.GaussianBlur(self.top_down(ref, room_mm, px_per_mm), (5, 5), 0)
        diff = cv2.cvtColor(cv2.absdiff(a, b), cv2.COLOR_BGR2GRAY)
        mask = (diff > thresh).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        for (x, y, r) in exclude:
            cv2.circle(mask, (int(x * px_per_mm), int((room_mm[1] - y) * px_per_mm)), int(r * px_per_mm), 0, -1)
        nx, ny = room_mm[0] // cell_mm, room_mm[1] // cell_mm
        frac = cv2.resize(mask[::-1].astype(np.float32), (nx, ny), interpolation=cv2.INTER_AREA)   # flip so row 0 = y near 0
        return frac > 0.35

class HandCamera:
    """ESP32-CAM on the gripper. K is the 3x3 pinhole matrix from a chessboard calibration of the OV2640."""
    def __init__(self, K, dist=None, marker_mm=30.0):
        self.K = np.asarray(K, np.float64); self.dist = np.zeros(5) if dist is None else np.asarray(dist, np.float64); self.size = marker_mm
    def marker_poses(self, frame):
        """{id: 4x4 T_cam_marker} (marker pose in the camera frame, mm)."""
        out = {}
        for mid, c in detect_markers(frame).items():
            ok, rvec, tvec = cv2.solvePnP(square_object_points(self.size), c.astype(np.float64), self.K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok: continue
            T = np.eye(4); T[:3, :3], _ = cv2.Rodrigues(rvec); T[:3, 3] = tvec.reshape(3); out[mid] = T
        return out
