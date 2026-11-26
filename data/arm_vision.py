"""
arm_vision.py - the hand camera decides the arm movement (eye-in-hand visual servoing).
  1. move to a SEARCH pose that looks at the preset position of the medicine
  2. find its ArUco marker, turn the marker pose into a 3D grasp point in the arm frame (4x4 matrix chain)
  3. report the shortest (straight-line) distance tip -> medicine
  4. solve inverse kinematics for a PRE-GRASP point, move, look again from closer, refine
  5. slide straight in to the grasp point
Also contains ArmSim (a 'real' arm with servo errors) so the whole loop can be tested without hardware.
"""
import math
import numpy as np
import kin3d as K3
import ik_arm as K
import synth

PRE_GRASP_MM = 40.0

class ArmSim:
    """Stands in for the real arm. The controller only knows q_cmd; the camera sees the arm at q_true = q_cmd + servo errors."""
    def __init__(self, seed=0, bias_deg=0.7, noise_deg=0.3):
        self.rng = np.random.default_rng(seed); self.bias = self.rng.normal(0, bias_deg, 4); self.noise = noise_deg
        self.q_cmd = np.array([0.0, 110.0, -145.0, 60.0]); self.q_true = self.q_cmd + self.bias
    def move(self, q):
        self.q_cmd = np.array(q, float); self.q_true = self.q_cmd + self.bias + self.rng.normal(0, self.noise, 4)

def T_marker_front_face(x, y, z, face_yaw_deg=0.0):
    """Pose (arm frame) of a marker on a package front face; marker z points out of the face toward the robot."""
    R0 = np.array([[0, 0, -1.0], [-1, 0, 0], [0, 1, 0]])           # marker x = robot right, y = up, z = toward the robot
    T = K3.Rz(math.radians(face_yaw_deg)); T[:3, :3] = T[:3, :3] @ R0; T[:3, 3] = (x, y, z); return T

def search_pose(marker_xyz, standoff_mm=150.0, image_size=(640, 480), f_px=500.0):
    """Arm pose that puts the hand camera about standoff_mm in front of the expected marker, looking at it."""
    for phi in (0, -10, -20, -30, -40, -50):
        for so in (standoff_mm, standoff_mm - 20, standoff_mm + 20, standoff_mm - 40):
            yaw = math.atan2(marker_xyz[1], marker_xyz[0])
            d = np.array([math.cos(yaw) * math.cos(math.radians(phi)), math.sin(yaw) * math.cos(math.radians(phi)), math.sin(math.radians(phi))])
            R = (K3.Rz(yaw) @ K3.Ry(-math.radians(phi)))[:3, :3]
            cam = np.array(marker_xyz) - so * d
            wrist = cam - R @ np.array(K3.CAM_OFFSET_W); tip = wrist + K.L3 * d
            q = K3.ik_xyz(tip, phi)
            if q is None: continue
            T_cam = K3.fk_chain(*q)["W"] @ K3.T_W_CAM()
            pc = np.linalg.inv(T_cam) @ np.append(marker_xyz, 1.0)
            if pc[2] > 40 and abs(pc[0] / pc[2] * f_px) < image_size[0] / 2 - 60 and abs(pc[1] / pc[2] * f_px) < image_size[1] / 2 - 60:
                return q, phi, so
    return None, None, None

def estimate_grasp(hand_cam, frame, aruco_id, q_cmd, grasp_depth=35.0):
    poses = hand_cam.marker_poses(frame)
    if aruco_id not in poses: return None
    T_A_m = K3.marker_to_arm(q_cmd, poses[aruco_id])
    p_g, n_out = K3.grasp_point_from_marker(T_A_m, grasp_depth)
    return p_g, n_out, T_A_m

def fetch(arm, hand_cam, Kh, true_marker_T, aruco_id, nominal_marker_xyz, grasp_depth=35.0, verbose=True):
    """Full eye-in-hand pick approach. true_marker_T is the REAL marker pose (only the simulator knows it)."""
    log = (lambda *a: print(*a)) if verbose else (lambda *a: None)
    def look():
        T_cam = K3.fk_chain(*arm.q_true)["W"] @ K3.T_W_CAM()                     # where the camera REALLY is
        Tcm = np.linalg.inv(T_cam) @ true_marker_T
        return synth.render_hand_marker(Kh, Tcm, aruco_id, hand_cam.size, seed=int(abs(Tcm[0, 3]) * 10))
    q, phi, so = search_pose(nominal_marker_xyz)
    if q is None: return dict(ok=False, why="no search pose")
    arm.move(q); est = None
    for sweep in (0, 6, -6, 12, -12, 20, -20):                                     # if the marker is not seen, sweep the base yaw
        qq = np.array(q, float); qq[0] += sweep; arm.move(qq)
        est = estimate_grasp(hand_cam, look(), aruco_id, arm.q_cmd, grasp_depth)
        if est: break
    if not est: return dict(ok=False, why="marker not found")
    p_g, n_out, _ = est
    tip0 = K3.tip_xyz(arm.q_cmd)
    log("  look 1 (%.0f mm away): grasp point %s | shortest distance tip -> medicine = %.1f mm | distance from arm base = %.1f mm"
        % (so, np.round(p_g, 1), np.linalg.norm(p_g - tip0), np.linalg.norm(p_g)))
    for it in (2, 3):                                                              # approach to the pre-grasp point, look again, refine
        n_h = np.array([n_out[0], n_out[1], 0.0]); n_h /= np.linalg.norm(n_h)
        pre = p_g + n_h * PRE_GRASP_MM
        qp = K3.ik_xyz(pre, 0.0)
        if qp is None: return dict(ok=False, why="pre-grasp unreachable")
        arm.move(qp)
        e2 = estimate_grasp(hand_cam, look(), aruco_id, arm.q_cmd, grasp_depth)
        if e2:
            p_new, n_out, _ = e2
            log("  look %d: grasp point %s (moved %.1f mm from the last estimate) | distance tip -> medicine %.1f mm" % (it, np.round(p_new, 1), np.linalg.norm(p_new - p_g), np.linalg.norm(p_new - K3.tip_xyz(arm.q_cmd))))
            p_g = p_new
        else:
            log("  look %d: marker too close or out of view, keeping the last estimate" % it); break
    steps = max(2, int(PRE_GRASP_MM / 5)); qg = None
    for i in range(1, steps + 1):                                                  # straight-line final approach, IK at every 5 mm
        pt = (p_g + n_h * PRE_GRASP_MM) * (1 - i / steps) + p_g * (i / steps)
        qg = K3.ik_xyz(pt, 0.0)
        if qg is None: return dict(ok=False, why="approach unreachable")
        arm.move(qg)
    tip_true = K3.tip_xyz(arm.q_true)
    true_grasp = true_marker_T[:3, 3] - true_marker_T[:3, 2] * grasp_depth
    err = tip_true - true_grasp
    face_yaw_err = math.degrees(math.atan2(-true_marker_T[1, 2], -true_marker_T[0, 2]) - math.atan2(tip_true[1], tip_true[0]))
    return dict(ok=True, err=err, q=qg, p_est=p_g, face_misalign_deg=face_yaw_err)

def run_trials(n=20, seed=0):
    import json, os
    Kh = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1.0]]); hc = __import__("vision").HandCamera(Kh, marker_mm=30)
    data = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "medicines.json")))
    rng = np.random.default_rng(seed); out = {}
    for m in data["medicines"]:
        nominal = np.array(m["grasp_point_arm_frame_mm"]) - np.array([35.0, 0, 0]) * 0                  # grasp point; marker is on the face 35 mm in front
        marker_nom = nominal - np.array([GRASP, 0, 0]) if False else nominal + np.array([-35.0, 0, 0])
        rows = []
        for k in range(n):
            jit = np.array([rng.uniform(-12, 12), rng.uniform(-15, 15), rng.uniform(-4, 4)]); eps = rng.uniform(-3, 3)    # real position differs from the preset
            true_T = T_marker_front_face(*(marker_nom + jit), face_yaw_deg=eps)
            arm = ArmSim(seed=seed * 100 + k)
            r = fetch(arm, hc, Kh, true_T, m["aruco_id"], marker_nom, verbose=False)
            # baseline: ignore the camera and drive to the preset joint angles
            base = ArmSim(seed=seed * 100 + k); base.move(m["joint_angles_deg"]["grasp"])
            true_grasp = true_T[:3, 3] - true_T[:3, 2] * 35.0
            rows.append((r, np.linalg.norm((K3.tip_xyz(base.q_true) - true_grasp)[1:]), jit))
        out[m["name"]] = rows
    return out

if __name__ == "__main__":
    res = run_trials(20)
    print("20 random placements per medicine (position jitter up to +/-15 mm sideways, +/-12 mm deep, +/-4 mm height, package turned +/-3 deg)")
    print("success = sideways error < 5 mm AND height error < 5 mm (jaws open 60 mm for a 50 mm package)\n")
    for name, rows in res.items():
        ok = [r for r, b, j in rows if r["ok"]]
        err = np.array([r["err"] for r in ok])
        succ = sum(1 for e in err if abs(e[1]) < 5 and abs(e[2]) < 5)
        bsucc = sum(1 for r, b, j in rows if abs(j[1]) < 5 and abs(j[2]) < 5)
        print(f"{name:20s} camera-guided: found {len(ok)}/20, success {succ}/20, mean error x {np.abs(err[:,0]).mean():.1f} y {np.abs(err[:,1]).mean():.1f} z {np.abs(err[:,2]).mean():.1f} mm"
              f"  | preset-only baseline success {bsucc}/20")
