"""
ik_arm.py - Planar 3-link closed-form inverse kinematics and torque calculator for the Pharma Rover arm.
Treats the shoulder, elbow, and wrist pitch as a planar 3-link arm in frame A (vertical arm plane).
Reference: Part 2B Kinematics and Physics.
"""
import math
import numpy as np

# Dimensions (mm)
L1 = 120.8              # Shoulder to elbow (Arm 01, pivot to pivot)
L2 = 90.0               # Elbow to wrist pitch axis (Arm 02 v3)
L3 = 100.0              # Wrist pitch axis to grasp centre (Arm 03 + gripper)
Z_SHOULDER = 162.0      # Shoulder axis height above ground (mm)

# Joint limits (degrees)
LIMIT_YAW = (-90.0, 90.0)
LIMIT_Q1 = (15.0, 165.0)        # Shoulder angle above horizontal
LIMIT_Q2 = (-150.0, -10.0)      # Forearm angle relative to upper arm (negative = elbow-up)
LIMIT_Q3 = (-90.0, 90.0)        # Wrist pitch relative to forearm
LIMIT_ROLL = (-90.0, 90.0)

# Mass model (grams) from Part 2B (with ESP32-CAM hand camera)
MASSES = [
    {"name": "Arm 01", "m_g": 35.9, "link": 1, "pos_frac": 0.50},
    {"name": "Elbow servo MG995", "m_g": 55.0, "link": 1, "pos_frac": 1.00},
    {"name": "Arm 02", "m_g": 28.2, "link": 2, "pos_frac": 0.50},
    {"name": "Wrist pitch servo SG90", "m_g": 13.4, "link": 2, "pos_frac": 1.00},
    {"name": "Arm 03", "m_g": 9.2, "link": 3, "pos_frac": 0.30},
    {"name": "Wrist roll servo SG90 + bracket", "m_g": 18.4, "link": 3, "pos_frac": 0.45},
    {"name": "ESP32-CAM + mount", "m_g": 12.0, "link": 3, "pos_frac": 0.35},
    {"name": "Gripper + SG90", "m_g": 45.0, "link": 3, "pos_frac": 0.75},
]

# Servo Ratings @ 6.0 V (stall and safe continuous = 50% stall)
SERVO_RATINGS = {
    "yaw": {"type": "MG995", "stall_kg_cm": 11.0, "safe_kg_cm": 5.5},
    "shoulder": {"type": "MG995", "stall_kg_cm": 11.0, "safe_kg_cm": 5.5},       # Overloaded during lift/extended pose
    "elbow": {"type": "MG995", "stall_kg_cm": 11.0, "safe_kg_cm": 5.5},
    "wrist_pitch": {"type": "SG90", "stall_kg_cm": 2.2, "safe_kg_cm": 1.1},      # Overloaded (~1.48 kg.cm)
    "wrist_roll": {"type": "SG90", "stall_kg_cm": 2.2, "safe_kg_cm": 1.1},
    "gripper": {"type": "SG90", "stall_kg_cm": 2.2, "safe_kg_cm": 1.1},
}

def in_limits(q):
    """
    Check if joint angles (degrees) are within physical limits.
    Accepts [q1, q2, q3] or [yaw, q1, q2, q3] or [yaw, q1, q2, q3, roll].
    """
    if len(q) == 3:
        q1, q2, q3 = q
        return (LIMIT_Q1[0] <= q1 <= LIMIT_Q1[1] and
                LIMIT_Q2[0] <= q2 <= LIMIT_Q2[1] and
                LIMIT_Q3[0] <= q3 <= LIMIT_Q3[1])
    elif len(q) >= 4:
        yaw, q1, q2, q3 = q[:4]
        ok = (LIMIT_YAW[0] <= yaw <= LIMIT_YAW[1] and
              LIMIT_Q1[0] <= q1 <= LIMIT_Q1[1] and
              LIMIT_Q2[0] <= q2 <= LIMIT_Q2[1] and
              LIMIT_Q3[0] <= q3 <= LIMIT_Q3[1])
        if len(q) >= 5:
            ok = ok and (LIMIT_ROLL[0] <= q[4] <= LIMIT_ROLL[1])
        return ok
    return False

def fk_planar(q1_deg, q2_deg, q3_deg):
    """
    Forward kinematics in planar coordinates:
    Returns (elbow_pos, wrist_pos, tip_pos) as (r, z) pairs in mm.
    """
    q1 = math.radians(q1_deg)
    q12 = math.radians(q1_deg + q2_deg)
    q123 = math.radians(q1_deg + q2_deg + q3_deg)

    elbow = (L1 * math.cos(q1), L1 * math.sin(q1))
    wrist = (elbow[0] + L2 * math.cos(q12), elbow[1] + L2 * math.sin(q12))
    tip = (wrist[0] + L3 * math.cos(q123), wrist[1] + L3 * math.sin(q123))
    return elbow, wrist, tip

def ik_planar(r, z, phi_deg=0.0, elbow_up=True):
    """
    Closed-form 2D inverse kinematics for planar 3-link arm.
    r = horizontal distance from shoulder axis to target tip (mm)
    z = vertical height from shoulder axis to target tip (mm)
    phi_deg = gripper pitch angle above horizontal (0 = horizontal)
    elbow_up = True picks negative q2 (elbow-up elbow geometry)

    Returns (q1, q2, q3) in degrees, or None if unreachable.
    """
    phi_rad = math.radians(phi_deg)
    wx = r - L3 * math.cos(phi_rad)
    wz = z - L3 * math.sin(phi_rad)
    D = wx**2 + wz**2

    # Law of cosines
    c2 = (D - L1**2 - L2**2) / (2.0 * L1 * L2)
    if c2 < -1.0 or c2 > 1.0:
        return None  # Unreachable

    q2_rad = -math.acos(c2) if elbow_up else math.acos(c2)
    q1_rad = math.atan2(wz, wx) - math.atan2(L2 * math.sin(q2_rad), L1 + L2 * math.cos(q2_rad))
    q3_rad = phi_rad - q1_rad - q2_rad

    q1 = math.degrees(q1_rad)
    q2 = math.degrees(q2_rad)
    q3 = math.degrees(q3_rad)

    return (q1, q2, q3)

def calculate_torque(q, payload_g=50.0):
    """
    Calculate predicted static torques (kg.cm) at shoulder, elbow, and wrist pitch
    for given angles q = [yaw, q1, q2, q3] or [q1, q2, q3] and payload mass in grams.
    Returns:
      {
        "shoulder_kg_cm": float,
        "elbow_kg_cm": float,
        "wrist_pitch_kg_cm": float,
        "shoulder_util_pct": float,
        "elbow_util_pct": float,
        "wrist_pitch_util_pct": float,
        "is_overloaded": bool,       # Exceeds 100% of stall
        "is_high_load": bool         # Exceeds 50% safe continuous rating
      }
    """
    if len(q) == 4:
        _, q1_deg, q2_deg, q3_deg = q
    else:
        q1_deg, q2_deg, q3_deg = q[:3]

    q1 = math.radians(q1_deg)
    q12 = math.radians(q1_deg + q2_deg)
    q123 = math.radians(q1_deg + q2_deg + q3_deg)

    # Calculate horizontal coordinates relative to shoulder (x=0)
    # Link 1 items
    items = []
    for m in MASSES:
        link = m["link"]
        frac = m["pos_frac"]
        mass = m["m_g"]
        if link == 1:
            x = frac * L1 * math.cos(q1)
        elif link == 2:
            x = L1 * math.cos(q1) + frac * L2 * math.cos(q12)
        elif link == 3:
            x = L1 * math.cos(q1) + L2 * math.cos(q12) + frac * L3 * math.cos(q123)
        items.append((m["name"], mass, x, link))

    # Add payload at tip
    tip_x = L1 * math.cos(q1) + L2 * math.cos(q12) + L3 * math.cos(q123)
    items.append(("Payload", payload_g, tip_x, 3))

    wrist_x = L1 * math.cos(q1) + L2 * math.cos(q12)
    elbow_x = L1 * math.cos(q1)

    # Torques in g.mm / 10000 = kg.cm
    tau_shoulder = sum(m * max(0.0, x) for (_, m, x, _) in items) / 10000.0
    tau_elbow = sum(m * max(0.0, x - elbow_x) for (_, m, x, link) in items if link >= 2) / 10000.0
    tau_wrist = sum(m * max(0.0, x - wrist_x) for (_, m, x, link) in items if link == 3) / 10000.0

    # Utilisation against safe continuous (50% stall)
    sh_safe = SERVO_RATINGS["shoulder"]["safe_kg_cm"]
    el_safe = SERVO_RATINGS["elbow"]["safe_kg_cm"]
    wr_safe = SERVO_RATINGS["wrist_pitch"]["safe_kg_cm"]

    sh_stall = SERVO_RATINGS["shoulder"]["stall_kg_cm"]
    el_stall = SERVO_RATINGS["elbow"]["stall_kg_cm"]
    wr_stall = SERVO_RATINGS["wrist_pitch"]["stall_kg_cm"]

    sh_pct = (tau_shoulder / sh_stall) * 100.0
    el_pct = (tau_elbow / el_stall) * 100.0
    wr_pct = (tau_wrist / wr_stall) * 100.0

    is_overloaded = (tau_shoulder > sh_stall) or (tau_elbow > el_stall) or (tau_wrist > wr_stall)
    is_high_load = (tau_shoulder > sh_safe) or (tau_elbow > el_safe) or (tau_wrist > wr_safe)

    return {
        "shoulder_kg_cm": round(tau_shoulder, 2),
        "elbow_kg_cm": round(tau_elbow, 2),
        "wrist_pitch_kg_cm": round(tau_wrist, 2),
        "shoulder_util_pct": round(sh_pct, 1),
        "elbow_util_pct": round(el_pct, 1),
        "wrist_pitch_util_pct": round(wr_pct, 1),
        "is_overloaded": is_overloaded,
        "is_high_load": is_high_load
    }
