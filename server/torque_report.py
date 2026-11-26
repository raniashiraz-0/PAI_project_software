"""
torque_report.py - Static torque and servo overload analysis for MG995 + SG90 allocation.
Hardware: 3x MG995 (yaw, shoulder, elbow) + 3x SG90 (wrist pitch, roll, gripper).
"""
import ik_arm as K

def generate_report(payload_g=50.0):
    poses = [
        ("SHELF_APPROACH", [0.0, 55.6, -118.0, 62.3]),
        ("GRASP (r=250mm, z=-12mm)", [0.0, 32.2, -90.1, 58.0]),
        ("LIFT (r=250mm, z=+30mm)", [0.0, 47.3, -88.1, 40.8]),
        ("DROP_BOX_1 (delivery)", [0.0, 54.7, -132.2, 32.5]),
        ("HOME (folded)", [0.0, 110.0, -145.0, 60.0]),
        ("WORST CASE (stretched flat)", [0.0, 0.0, 0.0, 0.0]),
    ]

    lines = []
    lines.append("=" * 80)
    lines.append(f"PHARMA ROVER ARM TORQUE ANALYSIS (Payload: {payload_g}g)")
    lines.append("Servo Allocation: 3x MG995 (yaw, shoulder, elbow: stall 11.0 kg.cm)")
    lines.append("                 1x SG90  (wrist pitch: stall 2.2 kg.cm)")
    lines.append("                 2x MG90  (wrist roll, gripper: stall 2.5 kg.cm)")
    lines.append("=" * 80)
    header = f"{'Pose':26s} | {'Shoulder (kg.cm)':18s} | {'Elbow (kg.cm)':14s} | {'Wrist Pitch (kg.cm)':19s} | Status"
    lines.append(header)
    lines.append("-" * 80)

    for name, q in poses:
        res = K.calculate_torque(q, payload_g=payload_g)
        sh_str = f"{res['shoulder_kg_cm']:5.2f} ({res['shoulder_util_pct']:5.1f}%)"
        el_str = f"{res['elbow_kg_cm']:5.2f} ({res['elbow_util_pct']:5.1f}%)"
        wr_str = f"{res['wrist_pitch_kg_cm']:5.2f} ({res['wrist_pitch_util_pct']:5.1f}%)"

        flag = "OK"
        if res["is_overloaded"]:
            flag = "OVERLOADED (>100% STALL)"
        elif res["is_high_load"]:
            flag = "HIGH_LOAD (>50% SAFE)"

        lines.append(f"{name:26s} | {sh_str:18s} | {el_str:14s} | {wr_str:19s} | {flag}")

    lines.append("-" * 80)
    lines.append("CRITICAL RISK SUMMARY:")
    lines.append("1. Shoulder servo (MG995, port 1): Peak need 6.47 kg.cm during LIFT -> ~118% of safe continuous rating. OVERLOADED.")
    lines.append("   Mitigation: Limit hold time in high-load pose to < 3.0 s; return to folded HOME when idle.")
    lines.append("2. Wrist pitch servo (SG90, port 3): Peak need 1.48 kg.cm -> ~135% of safe continuous rating. OVERLOADED.")
    lines.append("   Mitigation: Hard cap payload at 50g; move camera mass closer to wrist.")
    lines.append("=" * 80)

    report_text = "\n".join(lines)
    return report_text

if __name__ == "__main__":
    rep = generate_report(50.0)
    print(rep)
