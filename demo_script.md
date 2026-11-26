# Pharma Rover: Demo Day Presentation Script & Fallback Ladder

**Demo Duration:** 5 - 7 minutes  
**Team Allocation (5 Members):**
1. **Operator:** Operates voice commands and web dashboard on laptop.
2. **Narrator:** Explains system architecture, vision server, and safety interlocks to evaluators.
3. **Safety / E-Stop Person:** Stands beside the robot with hand ready on the physical E-Stop button.
4. **Camera / Dashboard Watcher:** Monitors live overhead tracker view and ESP32-CAM stream.
5. **Shelf / Box Handler:** Resets medicine boxes on the shelf and clears delivery tray.

---

## Scene-by-Scene Demo Execution Plan

### Scene 1: System Boot & Expressive Sad Idle Face (0:00 - 1:00)
- **Action:** Narrator introduces project context ("Pharma Rover, autonomous hospital fetch prototype").
- **Visual:** Arduino Uno 3.5" TFT display shows **Sad Idle Face** (droopy lids, downcast pupils, slow blinking, tear drop).
- **Web Dashboard:** Laptop browser displays live room map, car stationary at Start (400, 500 mm), and all subsystems reporting PASS in Doctor diagnostics.

### Scene 2: Interactive Voice Command (1:00 - 2:00)
- **Operator speaks:** *"Rover, go forward."*
- **Robot reaction:**
  - TFT Face instantly transitions to **Listening Face** (wide attentive cyan eyes).
  - Voice pipeline spots intent: `DRIVE_STEP(fwd, 200mm)`.
  - Base drives 200 mm forward in closed-loop; overhead tracker confirms final position.
  - Operator: *"A bit more."* &rarr; Rover repeats 100 mm step.

### Scene 3: The Full Autonomous Fetch Mission (2:00 - 4:30)
- **Operator speaks:** *"Rover, bring me paracetamol."*
- **Sequence:**
  1. **Thinking:** Display switches to Thinking face ("..." animation), laptop speaks: *"Okay, fetching paracetamol."*
  2. **Navigating:** Display shows Alert eyes. Pure Pursuit follows A* path across room, smoothly steering clear of obstacle boxes.
  3. **Fine Docking:** Rover aligns within 10 mm and 2&deg; of shelf S1, confirms zero velocity.
  4. **Scanning & Verification:** Arm rises to Search pose. Hand camera inspects box. Quality gate checks brightness & blur.
  5. **Grab Authorisation Gate:** Server detects ArUco #21 and validates EAN-13 barcode `8961000000112`. Server issues `GRAB_OK(mission_id, 1)`.
  6. **Grip & Lift:** Gripper approaches in 5 mm steps, closes securely, lifts 20 mm, folds to safe CARRY pose.
  7. **Return Drive:** Rover returns to START pose at reduced carry speed.
  8. **Delivery & Celebration:** Arm extends over delivery tray, opens gripper, deposits box. Display triggers **Happy Celebration Face** with checkmark!
  9. Rover folds arm to HOME and returns to sad idle face.

### Scene 4: Emergency Stop Demonstration (4:30 - 5:30)
- **Operator commands:** *"Rover, fetch omeprazole."*
- While robot begins driving, operator says loudly: *"STOP!"* (or presses the big red web button).
- **Reaction:** Within 200 ms, car motors halt, arm aborts, grab gate deauthorises, display shows error face, laptop speaks: *"Stopping immediately."*

---

## 4-Tier Fallback Ladder (If Hardware or Environment Degrades)

1. **Tier 1 (Acoustic Noise in Demo Hall):**
   - Fallback: Click the Push-To-Talk button or type command into the Web Dashboard box.
2. **Tier 2 (Overhead Lighting Glare on Phone Camera):**
   - Fallback: Switch to open-loop waypoint path from Start to Dock.
3. **Tier 3 (Hand Camera Image Motion Blur):**
   - Fallback: Click the green **"Authorise Paracetamol (ID 1)"** button in the web operator panel. The server grab gate verifies the operator verdict.
4. **Tier 4 (Hardware Battery / Motor Issue):**
   - Fallback: Toggle `--sim-car` or `--sim-arm` on the server and present the live high-fidelity digital twin simulation on the projector screen.
