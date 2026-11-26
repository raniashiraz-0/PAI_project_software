"""
kin3d.py - 3D kinematics with 4x4 homogeneous transforms (linear algebra) for the Pharma Rover arm.

Frames (all mm, right-handed, x forward, y left, z up):
  R  = room frame (floor), set by the phone camera (Part 1C)
  A  = arm frame: origin on the yaw axis at SHOULDER height, x = robot forward
  W  = wrist frame (after the wrist-pitch joint), x = gripper pointing direction
  C  = hand camera frame (OpenCV: x right, y down, z forward)
  M  = marker frame (OpenCV aruco: z points OUT of the marker toward the viewer)

Joint vector q = [yaw, q1, q2, q3] in degrees (roll does not move the tip).
"""
import math
import numpy as np
import ik_arm as K

L1, L2, L3, ZS = K.L1, K.L2, K.L3, K.Z_SHOULDER

# ---------- basic transforms ----------
def Rz(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
def Ry(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]])
def Rx(a):
    c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])
def Tr(x, y, z):
    T = np.eye(4); T[:3, 3] = (x, y, z); return T
def inv(T):
    R, t = T[:3, :3], T[:3, 3]; Ti = np.eye(4); Ti[:3, :3] = R.T; Ti[:3, 3] = -R.T @ t; return Ti
def rad(d): return math.radians(d)
def deg(r): return math.degrees(r)

# ---------- hand-eye (PLACEHOLDERS: measure on the real gripper, see Part 2C) ----------
CAM_OFFSET_W = (40.0, 0.0, 45.0)      # camera origin in the wrist frame: 40 mm forward, 45 mm above the wrist axis
R_W_CAM = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])   # camera looks along +x of the wrist frame
def T_W_CAM():
    T = np.eye(4); T[:3, :3] = R_W_CAM; T[:3, 3] = CAM_OFFSET_W; return T

# ---------- forward kinematics ----------
def fk_chain(yaw, q1, q2, q3, roll=0.0, d_off=0.0):
    """Return dict of 4x4 poses in the arm frame A (angles in degrees)."""
    T0 = Rz(rad(yaw)) @ Tr(d_off, 0, 0)                 # shoulder axis
    T_elbow = T0 @ Ry(-rad(q1)) @ Tr(L1, 0, 0)          # elbow axis
    T_wrist = T_elbow @ Ry(-rad(q2)) @ Tr(L2, 0, 0)     # wrist-pitch axis
    T_W = T_wrist @ Ry(-rad(q3))                        # wrist frame (x along the gripper)
    T_tip = T_W @ Tr(L3, 0, 0) @ Rx(rad(roll))          # grasp point
    return dict(shoulder=T0, elbow=T_elbow, wrist=T_wrist, W=T_W, tip=T_tip)

def tip_xyz(q, d_off=0.0):
    return fk_chain(*q, d_off=d_off)["tip"][:3, 3]

# ---------- inverse kinematics (closed form, 3D) ----------
def ik_xyz(p, phi_deg=0.0, elbow_up=True, d_off=0.0):
    """p = (x,y,z) in frame A. Returns [yaw,q1,q2,q3] (deg) or None if unreachable / outside limits."""
    x, y, z = p
    yaw = deg(math.atan2(y, x))
    r = math.hypot(x, y) - d_off
    q = K.ik_planar(r, z, phi_deg, elbow_up)
    if q is None: return None
    q = [yaw] + list(q)
    return q if (-90 <= yaw <= 90 and K.in_limits(q[1:])) else None

# ---------- Jacobian + damped least squares (used by visual servoing and as an IK cross-check) ----------
def jacobian(q, h=1e-4):
    J = np.zeros((3, 4)); q = np.array(q, float); p0 = tip_xyz(q)
    for i in range(4):
        dq = q.copy(); dq[i] += math.degrees(h)
        J[:, i] = (tip_xyz(dq) - p0) / h            # mm per radian
    return J

def dls_step(q, target, lam=20.0, max_step_deg=3.0):
    """One damped-least-squares step: dq = J^T (J J^T + lam^2 I)^-1 e   (e = target - tip)."""
    e = np.array(target, float) - tip_xyz(q)
    J = jacobian(q)
    dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(3), e)
    dq = np.degrees(dq)
    m = np.max(np.abs(dq))
    if m > max_step_deg: dq *= max_step_deg / m
    return np.array(q, float) + dq, float(np.linalg.norm(e))

def ik_dls(target, q0, iters=200, tol=0.2):
    q = np.array(q0, float)
    for _ in range(iters):
        q, err = dls_step(q, target)
        if err < tol: break
    return q, float(np.linalg.norm(np.array(target) - tip_xyz(q)))

# ---------- distances ----------
def distance_tip_to_target(q, target):
    """Shortest (straight-line, Euclidean) distance from the gripper tip to the target point, mm."""
    return float(np.linalg.norm(np.array(target, float) - tip_xyz(q)))

def reach_info(p, phi_deg=0.0):
    """Reachability numbers for a target point in frame A."""
    r = math.hypot(p[0], p[1])
    wx = r - L3 * math.cos(rad(phi_deg)); wz = p[2] - L3 * math.sin(rad(phi_deg))
    d = math.hypot(wx, wz)
    return dict(r=r, wrist_dist=d, min=abs(L1 - L2), max=L1 + L2, ok=abs(L1 - L2) <= d <= L1 + L2)

# ---------- camera -> arm frame ----------
def marker_to_arm(q, T_cam_marker, roll=0.0, d_off=0.0):
    """T_A_marker = T_A_W(q) * T_W_C * T_C_M"""
    return fk_chain(*q, roll=roll, d_off=d_off)["W"] @ T_W_CAM() @ T_cam_marker

def grasp_point_from_marker(T_A_marker, grasp_depth_mm=35.0):
    """Point inside the package, grasp_depth_mm behind the marked front face (marker z points out of the face)."""
    n_out = T_A_marker[:3, 2]
    return T_A_marker[:3, 3] - n_out * grasp_depth_mm, n_out

# ---------- servo mapping (placeholders until the Phase-3 calibration) ----------
def to_servo(q, roll=0.0):
    yaw, q1, q2, q3 = q
    return dict(yaw=90 + yaw, shoulder=q1, elbow=180 + q2, wrist_pitch=90 + q3, wrist_roll=90 + roll)

def _selftest():
    import random
    random.seed(1); worst = 0.0; n = 0
    for _ in range(2000):
        q = [random.uniform(-60, 60), random.uniform(20, 160), random.uniform(-145, -15), random.uniform(-80, 80)]
        p = tip_xyz(q); q2 = ik_xyz(p, phi_deg=q[1] + q[2] + q[3])
        if q2 is None: continue
        worst = max(worst, float(np.linalg.norm(tip_xyz(q2) - p))); n += 1
    print(f"closed-form IK round trip: {n} poses, worst position error {worst:.2e} mm")
    # cross-check with DLS
    p = (262.5, -80.0 * 0 + 80.0, -12.0)
    qa = ik_xyz(p, 0.0); qd, e = ik_dls(p, [0, 60, -100, 40])
    print("target", p, "closed-form q =", [round(v, 2) for v in qa], "| DLS q =", [round(float(v), 2) for v in qd], "err", round(e, 3), "mm")
    # frame chain sanity: tip direction when q1+q2+q3=0 is along +x
    T = fk_chain(*qa)["tip"]; print("tip x-axis", np.round(T[:3, 0], 3), "(should be horizontal, pointing at yaw angle)")
    # hand-eye chain: marker 150 mm straight ahead of the camera
    Tcm = Tr(0, 0, 150); TAm = marker_to_arm(qa, Tcm); print("marker 150 mm ahead of camera ->", np.round(TAm[:3, 3], 1))
    print("distance tip->target for the pre-grasp pose:", round(distance_tip_to_target(qa, p), 3), "mm")
if __name__ == "__main__":
    _selftest()
