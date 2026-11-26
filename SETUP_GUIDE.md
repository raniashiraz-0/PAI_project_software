# Pharma Rover – Setup Guide (real hardware)

This guide takes you from "files on the laptop" to "rover fetches paracetamol on voice command".
Do the steps **in order**. Test each one before going to the next.

---

## 1. What runs where

**The two ESP32 boards are uploaded with VS Code + PlatformIO; the Uno + TFT with the Arduino IDE.**
The connections are the same as before. Each board has its own fixed COM port, so the two programs
never get in each other's way, and **neither program is needed while the robot runs**.

| Board | Upload with | Code | Port |
|---|---|---|---|
| **Main ESP32** (base motors, 4 ultrasonic sensors, both PCA9685 arm drivers, TFT relay) | PlatformIO, env `esp32_main` | `firmware/src/esp32_main/` | **COM3** |
| **ESP32-CAM** (gripper camera) | PlatformIO, env `esp32_cam` | `firmware/src/esp32_cam/` | COM3 (change in `firmware/platformio.ini` if Device Manager shows another number) |
| **Arduino Uno + 3.5" TFT** (faces + medicine cards) | **Arduino IDE** | `arduino/uno_tft_display/uno_tft_display.ino` | **COM4** |
| **Server** (Whisper voice, YOLO, phone-camera tracking, A* path planning, dashboard) | – | `server/` → `python start_all.py` | http://localhost:8000 |

How they talk:

```
 Laptop server ──Wi-Fi──► Main ESP32  TCP 5000 = base (drive, ultrasonic telemetry)
      ▲   ▲                          TCP 5001 = arm  (moves are ACKed, finished, then checked)
      │   │                          TCP 8080 = TFT text ──GPIO17──► Uno D0
      │   └── ESP32-CAM uploads JPEG pictures (POST /api/vision/hand_frame)
      └────── Phone camera stream (IP Webcam app), looking straight down at the floor
```

The server finds the ESP32 and the ESP32-CAM by itself (UDP broadcast on port 4210),
so you don't have to type their IP addresses anywhere.

---

## 2. Wiring

### 2.1 Main ESP32 pins

| ESP32 pin | Goes to | Notes |
|---|---|---|
| **GPIO21 (SDA)** | SDA of **both** PCA9685 boards | as in your diagram |
| **GPIO22 (SCL)** | SCL of **both** PCA9685 boards | |
| 3V3 | VCC (logic) of both PCA9685 | |
| GPIO2 | L298N **ENA** (right-side speed) | same as your old car code |
| GPIO4 / GPIO5 | L298N IN1 / IN2 (right-side direction) | same as your old car code |
| GPIO18 / GPIO19 | L298N IN3 / IN4 (left-side direction) | same as your old car code |
| **GPIO25** | L298N **ENB** (left-side speed) | ⚠️ **moved from GPIO21** – GPIO21 is now SDA for the arm |
| GPIO13 / **GPIO34** | Front HC-SR04 TRIG / ECHO | same as your old car code; ECHO through a divider ↓ |
| GPIO27 / **GPIO35** | Back HC-SR04 TRIG / ECHO | same as your old car code |
| GPIO23 / **GPIO33** | Left HC-SR04 TRIG / ECHO | same as your old car code |
| GPIO26 / **GPIO36 (VP)** | Right HC-SR04 TRIG / ECHO | same as your old car code |
| GPIO17 (TX2) | **Uno D0 (RX)** | unplug this wire while uploading to the Uno |
| GPIO32 | battery sense (optional) | 7.4 V → 10 kΩ → GPIO32 → 3.3 kΩ → GND |
| GPIO14 | **box-in-gripper sensor** (optional, recommended) | limit switch to GND, or IR module (FC-51) OUT; set `GRIP_SENSOR_PIN 14` in config.h |
| GND | **common ground** (see 2.4) | |

> **HC-SR04 ECHO is 5 V; the ESP32 is 3.3 V.** Put a divider on every ECHO line:
> ECHO → **1 kΩ** → ESP32 pin → **2 kΩ** → GND. Power the sensors from 5 V.

The motor and sensor pins are the ones from **your old Bluetooth car code**, so keep that wiring.
The **only wire to move is ENB: GPIO21 → GPIO25**, because GPIO21 is the I2C SDA line for the
arm. As in your old code, IN1/IN2/ENA drive the right-side motors and IN3/IN4/ENB the left side.

> GPIO2 (ENA) is also a boot pin. If an upload fails with "wrong boot mode", unplug the ENA wire
> while uploading, then plug it back in.

If one side spins
the wrong way, either swap its two motor wires, or set `INVERT_LEFT_MOTOR` / `INVERT_RIGHT_MOTOR`
to `1` in `firmware/src/esp32_main/config.h`.
*(I assumed an L298N-type driver. If you use a different one, tell me and I'll change the pins.)*

### 2.2 Arm servos

| PCA9685 | Address | Ch 0 | Ch 1 | Ch 2 |
|---|---|---|---|---|
| #1 (XL4015 buck) | **0x40** (no pads bridged) | Yaw MG995 | Shoulder MG995 | Elbow MG995 |
| #2 (second buck) | **0x41** (bridge the **A0** pad) | Wrist pitch **MG90** | Wrist roll SG90 | Gripper MG90 |

* Servo power (5 V from each buck) goes into the **V+ screw terminal** (green block) of each PCA9685,
  **not** the thin V+ header pin and not VCC. VCC is the 3.3 V logic from the ESP32.
* Put a **1000 µF** capacitor across V+ / GND on each PCA9685.
* Put an **MG90 on the wrist pitch**, not the SG90. The torque calculation shows the wrist pitch
  is overloaded with an SG90 (135 %). The SG90 is fine on the wrist roll.
* The firmware never moves more than **2 servos at the same time**, keeps every MG995 inside **0–180°**,
  and keeps the gripper inside **90–180°**.

### 2.3 Uno + TFT

The TFT shield sits on the Uno. Only D0, D1 and A5 are free.
ESP32 **GPIO17 → Uno D0**, ESP32 **GND → Uno GND**. Power the Uno from the base battery (7.4 V into
the barrel jack or VIN) or from USB.

### 2.4 Power (very important)

| Battery | Feeds |
|---|---|
| Base 2S 7.4 V | L298N + 4 gear motors; a **5 V buck** → ESP32 5V/VIN pin + 4× HC-SR04; Uno VIN |
| Arm 2S 7.4 V #1 | XL4015 → 5 V → PCA9685 #1 V+ (3× MG995) |
| Arm 2S 7.4 V #2 | buck 5 V 3 A → PCA9685 #2 V+ (2× MG90 + SG90) |
| 1S 3.7 V | **5 V boost converter (e.g. MT3608)** → ESP32-CAM **5V** pin |

* **All grounds must be connected together**: base battery −, both arm battery −, ESP32 GND,
  Uno GND, both PCA9685 GND, and the HC-SR04 GND. Without a common ground, I2C, the TFT link and
  the sensors will act randomly.
* **The ESP32-CAM cannot run straight from one 3.7 V cell.** Its 5 V pin feeds a regulator that
  needs about 4.5 V or more, so a single cell makes it brown out and reboot when Wi-Fi transmits.
  Use a boost converter to 5 V, plus a 470 µF capacitor at the CAM's 5V/GND pins.
* Switch on the **servo batteries before** (or at the same time as) the ESP32. If you switch them
  on later, the ESP32 re-detects the PCA9685 boards within 1 s anyway.

---

## 3. Upload the firmware

> **Why the two programs can't disturb each other:**
> * PlatformIO only uses COM3 (fixed in `platformio.ini`), and the Arduino IDE only uses COM4.
> * Uploading is a one-time job. After that every board runs on its own and they talk over
>   Wi-Fi and the GPIO17 → D0 wire. The Arduino IDE and VS Code can both be closed.
> * The only rule: a COM port can be open in one program at a time. Close a Serial Monitor
>   before uploading to that board, and close the Arduino IDE Serial Monitor if the server
>   uses the Uno in `serial` mode.

### 3.0 Open the project once
1. VS Code → **File → Open Folder** → select the **`firmware`** folder (the one containing `platformio.ini`).
   *Not* `PharmaRover`, and not a subfolder. PlatformIO only works when `platformio.ini` is at the
   top of the opened folder.
2. Wait until PlatformIO finishes "Initializing" (bottom bar).
3. Click the **PlatformIO (alien) icon** on the left. Under **PROJECT TASKS** you'll see two boards:
   `esp32_main` and `esp32_cam`. Each one has its own **Build / Upload / Monitor**.
   (The blue → button in the bottom bar uploads `esp32_main` only.)
4. The first build of each board downloads its compiler once, so it needs internet and takes a few minutes.

> Red squiggles in the editor before the first build are VS Code's IntelliSense, not real errors.
> Build once, then Ctrl+Shift+P → **PlatformIO: Rebuild IntelliSense Index**. The truth is the
> **Terminal**: it ends with `[SUCCESS]` or `[FAILED]`.
> Only one program can use a COM port: close any open Monitor before uploading to that board.

### 3.1 Wi-Fi name and password (do this first)
Use a **2.4 GHz** phone hotspot or router (not campus Wi-Fi). Put the same name and password in:
* `firmware/src/esp32_main/config.h` → `WIFI_SSID`, `WIFI_PASSWORD`
* `firmware/src/esp32_cam/config.h` → `WIFI_SSID`, `WIFI_PASSWORD`

Connect the laptop to the same hotspot.

### 3.2 Main ESP32 → `esp32_main` (COM3)
1. Plug in the ESP32. PROJECT TASKS → **esp32_main → Upload**. If it hangs at `Connecting....`,
   hold the **BOOT** button on the ESP32 until it starts writing.
2. **esp32_main → Monitor**. You should see:
   ```
   === Pharma Rover main ESP32 ===
   [ARM] PCA 0x40 OK, PCA 0x41 OK
   [WIFI] connected, IP 192.168.x.x
   ```
   `MISSING` = check SDA 21 / SCL 22, VCC, the A0 pad on board #2, and the servo battery.

### 3.3 ESP32-CAM → `esp32_cam`
1. Put the CAM on the **ESP32-CAM-MB** adapter and plug it in. If Windows gives it a different COM
   number than COM3 (Device Manager → Ports), change the two `COM3` lines under `[env:esp32_cam]`
   in `firmware/platformio.ini`.
2. **esp32_cam → Upload** (with a plain USB-TTL adapter instead: GPIO0 → GND while uploading,
   then remove it and press reset).
3. **esp32_cam → Monitor** shows `[DISCOVERY] server found at ...` once the server is running,
   then `frames ok=...` counting up.

### 3.4 Uno TFT → Arduino IDE (COM4)
1. **Libraries:** you already installed **MCUFRIEND_kbv** and **Adafruit GFX Library** with the
   Library Manager. If not, double-click `arduino/install_libraries.bat` (it copies them from
   `arduino/libraries`), then restart the Arduino IDE.
2. **Unplug the ESP32 GPIO17 → Uno D0 wire.** D0 is shared with the USB upload, so this is only
   needed while uploading.
3. Arduino IDE → **File → Open** → `arduino/uno_tft_display/uno_tft_display.ino`.
4. **Tools → Board → Arduino Uno**, **Tools → Port → COM4**, then **Upload**.
5. Test without the ESP32: **Tools → Serial Monitor**, set **115200 baud** and **Newline**, type
   `STATE|happy` and press Enter, then `MED|paracetamol`. If the picture is upside down, change
   `SCREEN_ROTATION` to `3` in the sketch.
6. **Close the Serial Monitor and plug the D0 wire back in.** When the ESP32 is on, the
   bottom-left of the screen shows `ESP32 192.168.x.x`.

The ESP32 sends the TFT lines at a steady pace and waits about 0.4 s after every command that
redraws the screen, so the Uno never loses a line while it's drawing.

## 4. Server on the laptop

```bash
pip install -r requirements.txt
pip install ultralytics          # only needed for YOLO
python start_all.py              # open http://localhost:8000
```

Allow the server through the Windows firewall once (PowerShell **as Administrator**):
```powershell
New-NetFirewallRule -DisplayName "PharmaRover HTTP" -Direction Inbound -LocalPort 8000 -Protocol TCP -Action Allow
New-NetFirewallRule -DisplayName "PharmaRover Discovery" -Direction Inbound -Protocol UDP -Action Allow -Program (Get-Command python).Source
```

### 4.0 Robot self-test (do this every time something doesn't work)

`server/config.yaml` is already set for the **real robot**: car, arm, TFT, phone camera
(`http://192.168.18.11:8080/video`) and voice are real. Only `verifier_sim` stays `true` until the
ESP32-CAM and the box tags are ready.

Dashboard → **Live Robot & Teach** → **Run self-test**, or in the terminal:
```
python tools/self_test.py           # checks only, nothing moves
python tools/self_test.py --move    # also moves the arm and drives 10 cm forward and back
```
It checks the ESP32 link, both PCA9685 boards, all 4 ultrasonic sensors, the battery, the TFT, the phone
camera, the floor/car/shelf/start tags, car tracking and the ESP32-CAM. With **Allow the robot to move**
ticked, it also tests the arm (HOME → SEARCH → HOME), the gripper, and a 10 cm drive measured by the camera.
Every problem comes with what to do about it, for example *"PCA9685 0x41 not answering: check A0 pad"* or
*"the car drove BACKWARDS: set INVERT_LEFT_MOTOR…"*. **LOOK** means look at the robot to confirm
(e.g. did the TFT face change?).

When a mission fails, the robot now **says why** (e.g. *"I can't see the car. Check the phone camera, the
four floor tags, and tag five on the car."*).

### 4.1 Switch from simulation to the real robot – one part at a time
In `server/config.yaml`, under `system.subsystems`:

1. `display_sim: false`, then restart. The TFT face should change when you use the dashboard.
   *Bench option:* with the Uno plugged into the laptop by USB (COM4) and the ESP32 → D0 wire unplugged,
   set `devices.display_uno.mode: "serial"`. The server then drives the TFT over COM4 directly.
   On the robot, use `"relay"` (over Wi-Fi through the ESP32).
2. `car_sim: false` and `arm_sim: false` (both are on the same ESP32). The dashboard should show
   `car_connected: true`; the ultrasonic distances are in `/api/status → car.distances_cm`.
   Try the D-pad and "move forward". The arm presets should move it step by step.
3. **Phone camera (easiest way):** start IP Webcam (Android) or IP Camera Lite (iPhone) on the phone,
   then in the PharmaRover folder run:
   ```
   python tools/phone_camera_setup.py
   ```
   It asks Android or iPhone and the phone's IP, tests the video (frames per second, which tags it
   can see, a test picture in `markers/phone_camera_test.jpg`), and writes the address into
   `config.yaml` with `tracker_sim: false` for you. (By hand instead: set `tracker_sim: false`, and set
   `devices.cameras.overhead_phone.source` to e.g. `http://192.168.43.20:8080/video` for Android or
   `http://admin:admin@192.168.43.20:8081/video` for iPhone.) Mount the phone **looking straight
   down**, put `phone_camera.height_mm` (lens height above the floor) in `data/room.json`, and lay
   out floor ArUco tags 0–3, car tag 5 on top of the car, shelf tag 10 on top of the shelf.
   Print the tags from **`markers/PharmaRover_markers.pdf`** at **100% / Actual size**
   (or regenerate with `python tools/make_markers.py`). Measure the centre of each floor tag in mm
   from tag 0 and put the numbers in `data/room.json → floor_markers → centres_mm`.
4. `verifier_sim: false`. The ESP32-CAM pictures are used to recognise the boxes.
5. `audio_sim: false`. Whisper runs on the laptop (push-to-talk button on the dashboard).

If discovery doesn't find the ESP32 (some hotspots block broadcasts), put the IP from the ESP32
Serial Monitor (or the TFT) into `devices.controller_esp32.fallback_ip`, and the laptop's IP into
`SERVER_IP` in `firmware/src/esp32_cam/config.h`.

---

## 5. Calibration (10 minutes, makes everything work better)

All in `firmware/src/esp32_main/config.h`, then upload `esp32_main` again:

| Setting | How to measure |
|---|---|
| `MOTOR_MIN_PWM` | the smallest value at which the rover starts moving on your floor |
| `CAL_MM_PER_S_AT_100` | dashboard: drive forward 1000 mm at 100 %; measure the real distance d → new value = old × d / 1000 |
| `CAL_DEG_PER_S_AT_100` | same with a 360° turn |
| `JOINT_TRIM` | send the arm HOME; if a joint is a few degrees off, put the correction here |
| `GRIP_OPEN_DEG` / `GRIP_CLOSED_DEG` | open = claws fully open; closed = just squeezing a medicine box (don't let it stall) |

The phone camera corrects the open-loop driving during missions, so these values only need to
be roughly right. Manual voice commands ("move forward 50 cm") use them directly.

---

## 5b. Teach mode – show the arm where the boxes are

Instead of measuring grasp positions with a ruler, show the arm on the dashboard
(**Live Robot & Teach** tab):

1. Put the boxes on the shelf, and dock the car in front of the shelf **centre**: run a fetch once
   and press STOP when it starts scanning, or drive it there by hand.
2. Click **SEARCH pose**, then **Open gripper**.
3. Choose the medicine. Use **Forward / Back / Up / Down / Turn** to put the open claws around the
   box, exactly where they should close.
4. Click **Save as grasp position**. This is saved in `data/medicines.json` (position, height,
   gripper tilt, and the box's side position if the tick box is on).
5. Click **HOME**, then **Test grasp** to check the arm goes back to the same spot.
6. Repeat for the other two boxes.

## 5c. Does it really hold the box? (grip check)

After lifting, the arm pulls back to the carry pose and the server checks the claws:

* **Grip sensor fitted** (GPIO14, a micro switch or IR module inside the claws): the sensor decides,
  and it is also watched the whole way home. If the box drops, the rover stops and says so.
* **No sensor:** the ESP32-CAM decides. A held box still fills the picture, and its YOLO class or
  ArUco tag is still visible, or the picture still looks like the one taken at the shelf. An empty
  gripper sees the room instead.

If the claws are empty, it opens them, looks at the slot again, and has another go (`grip_check.retries`
in `server/config.yaml`). It checks once more before handing the box over. The result is shown on the
dashboard under **Last grip check**.

> A switch or IR sensor in the claws costs very little and is far more reliable than the camera.
> Recommended.

## 5d. AI & learning – how the server gets better by itself

| What | AI / learning method | Effect on the real robot |
|---|---|---|
| Speech → text | **Whisper** (local), with a vocabulary prompt of your commands and medicine names | hears "paracetamol" or "cetirizine" correctly much more often |
| Text → command | rules + **fuzzy matching** ("moove forwad" → move forward) | small mishearings still work |
| Which box is which | **YOLO** (or ArUco + barcode until it's trained) | finds the right box even if the boxes are swapped |
| Where the car is | **Extended Kalman filter**: phone camera + motor commands | smooth pose, about 0.25 s camera lag compensated, survives short camera dropouts |
| How the motors really behave | **Online regression** (recursive least squares) on camera measurements | learns real speed, turn rate and drift, then corrects every command and every manual step |
| Lining up the grab | **Visual servoing** with the gripper camera, plus a **learned correction per slot** | claws centred on the box; next time it starts closer |
| Where the boxes are | **Shelf memory** | looks first where each medicine was last found |
| Is it improving? | **Mission log** | success rate, time and grab attempts on the dashboard |

**Safety rules for the learning:** a model is only used after 40 clean samples and when its
prediction error is small. Learned corrections are limited to a safe range (speed and turn ×0.6–×1.6).
Everything is saved in `data/learned/`, and **Reset learning** on the dashboard (or deleting that folder)
starts fresh. Learning from simulation goes in separate `*_sim.json` files, so it never mixes with the real robot.

**Tip:** after building the robot, drive a few missions (or a minute of driving around in view of
the phone camera). The **AI & Learning** panel shows "(in use)" once the drive model is active.

## 6. Recognising the medicine boxes (YOLO)

Until you train a model, the server recognises boxes by an **ArUco tag (21/22/23)** plus the
**barcode** printed on each box, so stick those on the boxes for your first tests.

To use YOLO:
1. `python tools/capture_dataset.py paracetamol 80` (repeat for cetirizine, omeprazole, and some
   pictures with all three boxes). Move the boxes around while it captures.
2. Label the pictures (makesense.ai or Roboflow, export **YOLO** format) with the classes
   `paracetamol`, `cetirizine`, `omeprazole`. Put them in `dataset/images/{train,val}` and
   `dataset/labels/{train,val}`.
3. `python tools/train_yolo.py`. This saves `data/yolo/medicine_boxes.pt`. Restart the server;
   it prints `YOLO loaded medicine_boxes.pt`.

---

## 7. What happens on "fetch paracetamol"

1. Whisper turns the speech into text, and the intent parser picks the medicine.
2. The arm goes HOME. The ESP32 must ACK the move and report it finished, and the reported angles
   must match within 2°.
3. A* plans the shortest route to the shelf on the room map from the phone camera, and Pure
   Pursuit drives it.
4. **Obstacles:** front sensor < 25 cm → stop, put the obstacle on the map, plan a new shortest
   route. Side sensors < 12 cm → put on the map, and re-plan if the route goes through it.
   The ESP32 also stops by itself at 15 cm, even if Wi-Fi lags.
5. The rover docks in front of the shelf centre. The front stop distance is reduced to 5 cm while
   docking, so the shelf doesn't count as an obstacle.
6. The arm looks at each of the 3 slots (expected slot first). The ESP32-CAM flash turns on, and
   YOLO (or ArUco + barcode) says which box is there.
7. The matching slot gets a grab authorisation (the gripper firmware refuses to close without it),
   then IK computes pre-grasp → grasp → close → lift → carry pose.
8. The rover drives back to the start with the same obstacle handling, opens the gripper, and the
   arm goes HOME. The TFT celebrates.

Say **"stop"** at any time: motors stop, the arm holds where it is, and the mission is cancelled.

**Voice commands:** "move forward", "move forward 50 cm", "go back one meter", "keep moving
forward" (until "stop"), "turn left", "move right 30 degrees", "stop", "fetch paracetamol",
"bring me the allergy medicine", "go home", "arm home", "open gripper".

---

## 8. Troubleshooting

| Problem | Fix |
|---|---|
| TFT shows **NO LINK** | GPIO17 → D0 wire and common GND; the ESP32 must be powered |
| TFT white | already handled (forces ILI9486). Check the shield is fully seated |
| Upload fails "could not open port COM3/COM4" / "access denied" | close the Monitor (or any other program) using that port |
| Uno upload fails "programmer is not responding" | unplug the GPIO17 → D0 wire while uploading |
| "cannot open source file" / red squiggles | open the `firmware` folder itself, build once, then *Rebuild IntelliSense Index* |
| `PCA 0x41 MISSING` | bridge the A0 pad on PCA #2; check SDA=21, SCL=22 |
| Arm move fails "did not reach target" | a joint's TRIM is off, or the servo is too weak / stalled |
| Car won't move forward, "obstacle in the way" | front sensor sees something < 15 cm, or its ECHO divider is missing (reads 0) |
| ESP32-CAM keeps rebooting | power: use a 5 V boost + 470 µF capacitor |
| Server can't find ESP32 | same hotspot? Use `fallback_ip` (IP is on the TFT / Serial Monitor) |
| Car stops during missions, "tracking lost" | the phone must see tag 5 and all 4 floor tags; aim for ≥ 5 fps; raise `stale_pose_timeout_s` |
