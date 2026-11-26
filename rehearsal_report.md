# Pharma Rover Mission Rehearsal Report

**Date:** 2026-11-10 21:55:35  
**Execution Environment:** High-Fidelity Closed-Loop Simulation (Digital Twin)  
**Tolerances Enforced:** Docking: 10 mm & 2.0&deg; &bull; Tracker Freshness: 0.5 s &bull; Payload: 45 g

---

## 1. Summary Statistics per Medicine

| Medicine | Trials | Completed Stages | Verification Verdict | Grasp Success Rate | Avg Stage Time |
|---|:---:|:---:|:---:|:---:|:---:|
| **Paracetamol 500mg** (Shelf S1 / Slot +100mm) | 3 | Plan &rarr; Nav &rarr; Dock &rarr; Scan &rarr; Grip &rarr; Return &rarr; Deliver | `MATCH` (ArUco #21 + EAN-13 `8961000000112`) | **100.0%** | ~28.4 s |
| **Cetirizine 10mg** (Shelf S1 / Slot 0mm) | 3 | Plan &rarr; Nav &rarr; Dock &rarr; Scan &rarr; Grip &rarr; Return &rarr; Deliver | `MATCH` (ArUco #22 + EAN-13 `8961000000228`) | **100.0%** | ~27.8 s |
| **Omeprazole 20mg** (Shelf S1 / Slot -100mm) | 3 | Plan &rarr; Nav &rarr; Dock &rarr; Scan &rarr; Grip &rarr; Return &rarr; Deliver | `MATCH` (ArUco #23 + EAN-13 `8961000000334`) | **100.0%** | ~29.1 s |

---

## 2. Hard Safety Interlocks Verification

1. **Interlock 1 (Arm moves ONLY when car is stationary):** Passed on 100% of runs. Arm movement commands sent while car is driving were strictly rejected by the motion arbiter (`can_move_arm()`).
2. **Interlock 2 (Car drives ONLY when arm is folded at HOME/CARRY):** Passed on 100% of runs. Drive commands while arm was extended into the shelf were rejected (`can_drive_car()`).
3. **Interlock 3 (No simultaneous actuator commands):** Handoff via explicit state transition acks strictly verified.
4. **Interlock 4 (STOP beats everything within 200 ms):** Car stops immediately, arm aborts, grab gate is deauthorised, mission token cancelled.
5. **Interlock 5 (Grab Authorisation Gate):** Gripper cannot close without server `GRAB_OK` matching both `mission_id` and `medicine_id`.
6. **Interlock 6 (Stale tracking protection):** Pose older than 0.5 s counts as invalid and halts vehicle.
7. **Interlock 7 (One mission at a time):** Second command during active mission returns busy warning.
8. **Rule 8 (Servo Overload / Carry Hold Time):** Shoulder (118% stress) & Wrist pitch (135% stress) safe hold limits (<3.0s) monitored and logged.

---

## 3. Demo Recommendations

- **Primary Demo Showcase:** Paracetamol 500mg demonstrates optimal straight-line docking corridor and rapid barcode recognition.
- **Operator Fallback Console:** In case of demo hall acoustic interference, the browser web dashboard push-to-talk button or typed input box allows instant trigger of all missions.
