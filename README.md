# Pharma Rover: Autonomous Medicine-Fetching Robot System

> **Setting up the real robot? Start with [SETUP_GUIDE.md](SETUP_GUIDE.md)** – wiring, power, uploading the
> ESP32 + ESP32-CAM firmware (VS Code + PlatformIO), the Uno TFT sketch (Arduino IDE), calibration, YOLO training,
> and the AI & learning features (Kalman filter, drive regression, visual servoing, grip check).

**Pharma Rover** is an autonomous, voice-controlled, computer-vision-guided medicine-fetching robot prototype. The robot base and arm are **dumb hardware actuators**; all high-level thinking—voice activity detection, speech-to-text, intent parsing, overhead camera tracking, A* path planning, ESP32-CAM box verification, and inverse kinematics—runs on a centralized **Python Server** with a rich interactive **Web Dashboard**.

---

## 1. Quick Start: Launching the Web Server

### 1.1 Install Dependencies
From the project directory:

```bash
python -m pip install -r requirements.txt
```

The first voice transcription downloads the configured Faster-Whisper model.

### 1.2 Run the Master Server
From the project directory:

```bash
# Start the unified web server (default port 8000, high-fidelity sim mode)
python start_all.py
```

Or run directly from the server folder:
```bash
python server/server.py 8000
```

### 1.3 Open the Web Dashboard
Open your web browser and navigate to:
**[http://localhost:8000](http://localhost:8000)**

### 1.3 Interactive Web Dashboard Features
The modern single-page dashboard provides full real-time telemetry and control:
- **Mission Center:** One-click fetch cards for *Paracetamol 500mg*, *Cetirizine 10mg*, and *Omeprazole 20mg* with live 8-stage mission timeline tracking.
- **Voice & Audio Console:** Push-To-Talk microphone button with waveform visualizer, typed command box with autocomplete chips, and spoken response ($SAY) bubble.
- **Computer Vision Studio:**
  - *Overhead Room Map Stream:* Live MJPEG rendering of the 3000x2400 mm room, obstacles, shelf, start point, and car pose with heading arrow.
    - *Overhead Phone Feed:* Live MJPEG/RTSP-compatible phone stream configured in `server/config.yaml`.
    - *Laptop Webcam:* Optional second feed enabled in `server/config.yaml`.
  - *Hand ESP32-CAM Stream:* Live video showing ArUco bounding boxes, EAN-13 barcode detection, image quality gate metrics, and verification verdict badge (`MATCH` / `NO_MATCH` / `UNCERTAIN`).
  - *Human Fallback Operator Gate:* Manual server grab authorisation override button.
- **Movement & Base Control:** Responsive D-Pad for manual jog (Forward, Reverse, Left, Right), custom step distance (mm), and turn angles.
- **Robotic Arm Station:** Joint angle sliders (Yaw, Shoulder, Elbow, Wrist Pitch), presets (HOME, SEARCH, PRE-GRASP, CARRY), gripper toggle, and **Live Torque Meters** with overload warnings.
- **Arduino Uno TFT Display Screen Preview:** Live 480x320 video clone of the physical Uno LCD shield rendering expressive face emotions (sad idle, listening, thinking, scanning, alert, happy, error) and 3-page medicine cards.
- **System Doctor & Diagnostics:** Live table of all subsystem health checks and scrolling JSONL event log terminal.
- **Emergency STOP:** High-priority red button accessible from every tab.

---

## 2. System Architecture

```
                          +-------------------------------------------------------+
                          |                 LAPTOP SERVER (Python)                |
                          |                                                       |
 [Web Browser UI] <=====> |  - HTTP REST API & Web Dashboard (/api/*)             |
                          |  - Real-Time MJPEG Streamer (/stream/*)               |
 [Overhead Phone] ------> |  - Overhead ArUco Car Tracker (ID 5, floor-calibrated) |
 [Hand ESP32-CAM] ------> |  - Box Verifier (Quality Gate + ArUco + EAN-13 + OCR) |
[Browser Mic/TTS] <====> |  - Local Faster-Whisper STT & browser speech output  |
                          |  - Motion Arbiter & Orchestrator (10 Hard Interlocks) |
                          +--------------------+-------------+--------------------+
                                                  |                       |
                                           TCP framed  |                       | TCP framed
                                           car :5000   |                       | arm :5001
                                                  v                       v
                                              +------------------+    +------------------+
                                              | Car ESP32        |    | Arm ESP32        |
                                              | Motor controller |    | Arm/gripper      |
                                              +------------------+    +------------------+
                                                  |
                                                  | Optional display relay :8080
                                                  v
                                              +------------------+
                                              | Arduino TFT      |
                                              +------------------+

```

One main ESP32 (`firmware/src/esp32_main`) runs the base (TCP 5000), the arm through two PCA9685 boards on SDA 21 / SCL 22 (TCP 5001) and the TFT relay (TCP 8080). The ESP32-CAM (`firmware/src/esp32_cam`) uploads pictures to the server. The server finds both boards automatically with a UDP broadcast (port 4210). Every command is acknowledged by the ESP32; arm moves are confirmed with CMD_DONE and checked against the joint angles reported in telemetry.

Voice PTT records audio in the browser, uploads it to `/api/audio/transcribe`, and runs Faster-Whisper locally on the laptop. The phone camera must provide a live MJPEG/RTSP stream; a saved video recording is not treated as a live tracker feed. The laptop webcam is an optional additional stream.

---

## 3. The 10 Hard Safety Interlocks

| # | Interlock Rule | Enforcement Layer | Failure Behavior |
|---|---|---|---|
| **1** | **Arm moves ONLY when car is stationary & docked** | `orchestrator.can_move_arm()` | Tracker speed must be near zero for &ge; 0.5 s. Refuses command if car moving. |
| **2** | **Car drives ONLY when arm is at HOME or CARRY** | `orchestrator.can_drive_car()` | Refuses drive command if arm extended into shelf or grab authorised. |
| **3** | **No simultaneous actuator commands** | Motion Arbiter | Commands sent strictly with explicit handoff ack events (`CMD_DONE`). |
| **4** | **STOP beats everything within 200 ms** | Global Event Bus (`STOP_TRIGGERED`) | Car motors halt, arm aborts, grab gate deauthorises, display shows error, speech says "Stopping". |
| **5** | **Grab Authorisation Gate** | Dual Gate (Server + Firmware) | Gripper can close **ONLY** if server issued `GRAB_OK(mission_id, med_id)` within last 10 s. |
| **6** | **Stale or lost tracking protection** | `tracker.get_pose()` | Poses older than 0.5 s count as invalid; car halts immediately. |
| **7** | **One mission at a time** | `MissionToken` | New `FETCH` while busy returns "I'm busy, say stop first". Old threads cannot overwrite new tasks. |
| **8** | **Overload & Carry Hold Time Rule** | `ik_arm.calculate_torque()` | Shoulder (118% stress) & Wrist pitch (135% stress) hold time capped at max 3.0 s before safe pose. |
| **9** | **Display follows state** | `DisplayClient` | State machine updates Uno display on every step (listening &rarr; thinking &rarr; alert &rarr; scanning &rarr; happy &rarr; success). |
| **10** | **Watchdog timers** | Controller loops | Per-step timeouts trigger safe abort and spoken error explanation. |

---

## 4. Hardware Wiring & Critical Power Notes

### 4.1 Servo Power Connector (PCA9685 6.0V Rail)
- The 6.0 V servo rail enters the PCA9685 through the **DC barrel jack**.
- **Connector Rating:** Must be rated for **at least 5 A**. Standard 1-2A barrel jacks will melt or cause severe brownouts under stall current of 3x MG995 + 3x SG90 (~5.4 A peak).
- **Polarity:** Verify centre-positive before connecting battery.
- **Filter Capacitor:** Add a **1000 &mu;F electrolytic capacitor** directly across V+ and GND at the PCA9685 screw terminal.

### 4.2 ESP32-CAM Hand Camera Power
- The ESP32-CAM is powered via 5V logic lines along the arm.
- **WiFi Reset Prevention:** Solder a **470 &mu;F capacitor** close to the 5V and GND pins of the ESP32-CAM to prevent brownout reboots during WiFi TX burst packets.

### 4.3 Arduino Uno Display Shield (D0/D1 Conflict Warning)
- The 3.5" TFT shield occupies all digital and analog pins except D0, D1, and A5.
- Connect ESP32 TX to Uno D0 (RX) only.
- **CRITICAL WARNING:** **Disconnect the ESP32 wire from Uno D0 while uploading sketches over USB**, as D0 is shared with the USB UART programmer.

---

## 5. Servo Overload Analysis & Mechanical Mitigations

The actual hardware uses **3x MG995 + 3x SG90** instead of the larger design servos:
- **Port 1 (Shoulder - MG995):** Peak requirement during lift is **6.47 kg.cm** vs 5.5 kg.cm safe continuous limit (**118% utilisation, OVERLOADED**).
- **Port 3 (Wrist Pitch - SG90):** Peak requirement is **1.48 kg.cm** vs 1.1 kg.cm safe continuous limit (**135% utilisation, OVERLOADED**).

### Software Safeguards:
1. Hard cap payload at **50 g** (all 3 medicines weigh 40-48 g).
2. Dynamic static torque guard refuses poses exceeding stall torque.
3. Maximum hold time in high-load pose capped at **3.0 s**.
4. Soft-start energises servo channels sequentially on power-up.

### Recommended Mechanical Fixes for Real Hardware:
- Install a **counterbalance extension spring** at the shoulder hinge to offload gravity.
- Replace the plastic-gear SG90 wrist pitch servo with a metal-gear **MG92B (3.5 kg.cm)**.
- Shorten link L3 by 15 mm to reduce wrist torque arm.

---

## 6. Testing & Bring-Up Verification

### 6.1 Run Unit & Interlock Test Suite
```bash
python tests/test_server.py
```
*Tests closed-form IK reproduction, torque limits, hard interlocks, grab authorisation gate, EAN-13 checksums, and A* navigation.*

### 6.2 Run Staged Bring-Up (Stages 0 to 10)
```bash
python bringup/bringup_runner.py
```
*Executes hardware validation checklist and writes `bringup/results.json`.*

### 6.3 Run Mission Rehearsals
```bash
python rehearse.py
```
*Performs repetitive mission cycles per medicine and writes `rehearsal_report.md`.*

---

## 7. Project File Structure

```
robot_controller/
├── start_all.py                   # Master launcher script
├── rehearse.py                    # Mission rehearsal & statistics runner
├── demo_script.md                 # 5-7 minute scripted demo plan & fallback ladder
├── power_and_safety.md            # Power budget table & first power-up protocol
├── troubleshooting.md             # Root-cause diagnostic & fix guide
├── network_setup.md               # Static IP hotspot configuration
├── rehearsal_report.md            # Verified mission statistics report
├── README.md                      # Complete system documentation
│
├── data/
│   ├── medicines.json             # Canonical 3-medicine database (Paracetamol, Cetirizine, Omeprazole)
│   ├── room.json                  # Room dimensions, marker layout, camera offsets
│   ├── kin3d.py                   # 3D homogeneous kinematics
│   ├── vision.py                  # OpenCV overhead vision utilities
│   ├── arm_vision.py              # Visual servoing helpers
│   └── navigation.py              # Pure Pursuit and grid navigation
│
├── server/
│   ├── server.py                  # Multi-threaded web server & MJPEG streaming engine
│   ├── orchestrator.py            # Mission state machine, arbiter & 10 hard interlocks
│   ├── events.py                  # Event types, mission states, and event bus
│   ├── config.yaml                # Unified hardware, speeds, ports & marker config
│   ├── car_client.py              # Differential drive car adapter (TCP 5000 + sim)
│   ├── arm_client.py              # Arm controller client (TCP 5001 + ArmSim)
│   ├── arm_planner.py             # Shortest-path quintic mover & torque guard
│   ├── ik_arm.py                  # Planar 3-link closed-form IK & statics calculator
│   ├── tracker.py                 # Overhead phone camera ArUco car tracker (ID 5)
│   ├── camera.py                  # Threaded frame grabbers & POST endpoint
│   ├── verifier.py                # Multi-frame box verifier (ArUco + Barcode + Quality gate)
│   ├── planner.py                 # GridMap & 8-connected A* obstacle avoidance
│   ├── navigator.py               # Pure Pursuit closed-loop waypoint follower
│   ├── display_client.py          # Arduino Uno TFT display client (TCP 8080 / serial / sim)
│   ├── sim_display.py             # 480x320 ILI9486 software emotion & card renderer
│   ├── audio_in.py                # Local Faster-Whisper transcription & command input
│   ├── audio_out.py               # Half-duplex muting TTS engine
│   ├── intents.py                 # Unified fuzzy intent parser (rapidfuzz / difflib)
│   ├── doctor.py                  # Diagnostic system health checker
│   ├── torque_report.py           # MG995 + SG90 torque analysis generator
│   ├── streamer.py                # Live 2D room map & hand camera annotator
│   └── static/
│       ├── index.html             # Responsive web control dashboard
│       ├── dashboard.css          # Dark-theme cyberpunk medical control center UI
│       └── dashboard.js           # Live telemetry poller & interactive controls
│
├── firmware/                      # PlatformIO project (VS Code) for the two ESP32 boards
│   ├── platformio.ini             # envs: esp32_main (COM3), esp32_cam
│   ├── src/esp32_main/            # base + 4 ultrasonic + arm (2x PCA9685, SDA21/SCL22) + TFT relay
│   └── src/esp32_cam/             # gripper camera, uploads frames to the server
│
├── arduino/                       # Arduino IDE
│   ├── uno_tft_display/           # 3.5" TFT faces + medicine cards (Uno, COM4)
│   ├── libraries/                 # MCUFRIEND_kbv, Adafruit GFX, BusIO, TouchScreen (backup copy)
│   └── install_libraries.bat      # copies those into Documents/Arduino/libraries
│
├── tools/                         # capture_dataset.py + train_yolo.py (YOLO box detector)
│
├── bringup/
│   ├── bringup_runner.py          # Staged bring-up checklist runner
│   └── results.json               # Recorded stage results
│
└── tests/
    └── test_server.py             # Complete unit and interlock test suite
```
