/*
  ====================================================================================
  PHARMA ROVER - EXPRESSIVE TFT DISPLAY FIRMWARE (ARDUINO UNO + 3.5" TFT LCD SHIELD)
  ====================================================================================
  Hardware:
    - Microcontroller: Arduino Uno (ATmega328P, 16 MHz, 2 KB RAM, 32 KB Flash)
    - Display Shield: 3.5" TFT LCD Shield (ILI9486 driver, 8-bit parallel bus)
    - Resolution: 480 x 320 dots, Landscape orientation (setRotation(1))
    - Touch: Resistive touch controller included (SKU: MAR3501)
    - Free Uno Pins: D0 (RX), D1 (TX), and A5 ONLY. (Shield occupies all other pins).
    - Serial Port: Hardware Serial on D0/D1 at 115200 baud.

  CRITICAL HARDWARE WIRING WARNING:
    *** DISCONNECT THE ESP32 WIRE FROM UNO PIN D0 WHILE UPLOADING OVER USB! ***
    Pin D0 is shared directly with the onboard USB-to-UART chip (ATmega16U2 / CH340).
    If the ESP32 TX line remains connected to D0 during upload, the serial lines fight,
    causing programmer timeout errors ("avrdude: stk500_recv(): programmer is not responding").
    Reconnect D0 only after the sketch upload completes.

  COMMAND PROTOCOL REFERENCE (Arrives over Hardware Serial at 115200 baud):
    1. STATE|<face_name>
       Switches expressive robot face.
       Available faces:
         - idle       : Sad humanlike face (droopy lids, downcast pupils, slow blink, falling tear)
         - listening  : Wide attentive cyan eyes, small 'O' mouth, sound wave brackets
         - thinking   : Eyes looking up-right, wavy mouth, animated "..." bubbles
         - scanning   : Narrow focused slit eyes, animated sweeping green laser beam
         - alert      : Alert wide eyes looking straight ahead
         - happy      : Celebratory curved arc eyes (^ ^), big smile, green checkmark
         - success    : Same as happy (delivery celebration)
         - obstacle   : Startled wide red eyes, exclamation mark "!" icon, open mouth
         - error      : Red 'X' eyes, zigzag mouth, status banner

    2. CARDSHOW|<status_text>|<progress_0_100>
       Switches to 3-page auto-carousel medicine info card (cycles every 3 seconds).
       Example: CARDSHOW|NAVIGATING|30

    3. STATUS|<status_text>|<progress_0_100>
       Updates live status line and progress bar without resetting the active screen.
       Example: STATUS|DOCKING|45

    4. CARDF|<field_name>|<field_value>
       Sets medicine field. Fields: name, strength, category, shelf, dose, freq, max, use, warn.
       Example: CARDF|name|Paracetamol
       Example: CARDF|strength|500mg

    5. MED|<0|1|2|paracetamol|cetirizine|omeprazole>
       Shortcut command to load one of the 3 built-in PROGMEM medicine profiles:
         - 0 or paracetamol : Paracetamol 500mg (ArUco 21, Slot +100mm, 45g)
         - 1 or cetirizine  : Cetirizine 10mg  (ArUco 22, Slot 0mm, 40g)
         - 2 or omeprazole  : Omeprazole 20mg  (ArUco 23, Slot -100mm, 48g)

    6. CLEAR
       Clears medicine card and returns to default sad idle face.

    7. PING
       Watchdog heartbeat packet. Must arrive at least once every 10 seconds.
       If no PING or valid line arrives for >10s, display shows "NO LINK" error.

    8. INFO|<text>
       Small grey info line at the bottom-left of the face screen. The main ESP32
       uses it to show its Wi-Fi IP address ("INFO|ESP32 192.168.43.57").

  UPLOAD WITH THE ARDUINO IDE:
    Tools -> Board -> Arduino Uno, Tools -> Port -> COM4, then Upload.
    Libraries: MCUFRIEND_kbv + Adafruit GFX Library (Library Manager), or copy the
    folders in ../libraries into Documents/Arduino/libraries.

  HOW IT TALKS TO THE REST OF THE ROBOT (same connections as before):
    * From the ESP32: ESP32 GPIO17 (TX2) -> Uno D0 (RX), plus ESP32 GND -> Uno GND,
      115200 baud. The ESP32 paces the lines so none are lost while the Uno draws.
    * Or from the laptop over USB (COM4, 115200) - bench test / server mode "serial".
    The Arduino IDE is only needed to UPLOAD. Unplug the GPIO17 -> D0 wire while
    uploading (D0 is shared with USB), then plug it back. While the robot runs,
    the Arduino IDE can stay closed - nothing on the laptop is needed for the TFT.

  QUICK TEST LINES (Arduino IDE -> Tools -> Serial Monitor, 115200 baud, "Newline"):
    - Test Sad Idle:        STATE|idle
    - Test Listening:       STATE|listening
    - Test Thinking:        STATE|thinking
    - Test Scanning:        STATE|scanning
    - Test Obstacle:        STATE|obstacle
    - Test Happy/Success:   STATE|happy
    - Test Paracetamol Card: MED|paracetamol
    - Test Cetirizine Card:  MED|cetirizine
    - Test Omeprazole Card:  MED|omeprazole
    - Test Status Update:    STATUS|GRASPING|75
    - Test Clear to Face:    CLEAR
    - Test Heartbeat Ping:   PING
  ====================================================================================
*/

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <avr/pgmspace.h>

// ----------------------------------------------------------------------------------
// DRIVER CONFIGURATION
// ----------------------------------------------------------------------------------
// Set FORCE_ID to 0x9486 if your screen stays white upon boot.
#define FORCE_ID 0
// 1 = landscape. Change to 3 if the picture is upside down on your robot.
#define SCREEN_ROTATION 1

MCUFRIEND_kbv tft;

// Screen geometry in landscape mode (setRotation(1))
#define SCREEN_W 480
#define SCREEN_H 320

// 16-bit RGB565 Color Constants
#define BLACK      0x0000
#define WHITE      0xFFFF
#define CYAN       0x07FF
#define BLUE       0x001F
#define NAVY       0x0825   // Deep soothing navy background for eyes (RGB: 16, 25, 45)
#define GREEN      0x07E0
#define RED        0xF800
#define YELLOW     0xFFE0
#define DARKGREY   0x2104
#define LIGHTGREY  0xBDF7
#define TEAR_BLUE  0x5D1F   // Soft glowing cyan-blue for tear drop

// ----------------------------------------------------------------------------------
// BUILT-IN MEDICINE DATABASE (Stored in PROGMEM to consume ZERO RAM)
// ----------------------------------------------------------------------------------
struct MedicineProfile {
  char name[16];
  char strength[8];
  char category[24];
  char shelf_slot[18];
  char aruco[4];
  char weight[6];
  char dose[18];
  char freq[22];
  char max_dose[16];
  char use[26];
  char note[32];
  char grasp[16];
  char ean[14];
  char box_size[14];
};

const MedicineProfile BUILTIN_MEDS[3] PROGMEM = {
  // Medicine 0: Paracetamol 500mg
  {
    "Paracetamol", "500mg", "Painkiller / Analgesic", "S1, slot +100mm",
    "21", "45g", "1-2 tablets", "every 4-6 h", "max 4000mg/24h",
    "Fever, pain, headache", "Liver care: do not overdose",
    "[250, 0, -12]", "8961000000112", "100x50x20"
  },
  // Medicine 1: Cetirizine 10mg
  {
    "Cetirizine", "10mg", "Antihistamine / Allergy", "S1, slot 0mm",
    "22", "40g", "1 tablet (10mg)", "once daily (evening)", "max 10mg/24h",
    "Allergic rhinitis, hives", "May cause mild drowsiness",
    "[260, 0, -12]", "8961000000228", "100x50x20"
  },
  // Medicine 2: Omeprazole 20mg
  {
    "Omeprazole", "20mg", "Antacid / Proton-Pump", "S1, slot -100mm",
    "23", "48g", "1 capsule (20mg)", "once daily (morning)", "max 40mg/24h",
    "Acid reflux, heartburn", "Swallow whole before meals",
    "[245, 0, -12]", "8961000000334", "100x50x20"
  }
};

// ----------------------------------------------------------------------------------
// SYSTEM STATE VARIABLES (Kept minimal for ATmega328P 2KB RAM constraint)
// ----------------------------------------------------------------------------------
enum FaceState {
  FACE_IDLE,
  FACE_LISTENING,
  FACE_THINKING,
  FACE_SCANNING,
  FACE_ALERT,
  FACE_HAPPY,
  FACE_OBSTACLE,
  FACE_ERROR
};

FaceState currentFace = FACE_IDLE;
bool showCard = false;
uint8_t selectedMed = 0;       // 0 = Paracetamol, 1 = Cetirizine, 2 = Omeprazole, 255 = custom
uint8_t cardPage = 0;          // 0 = Overview, 1 = Clinical, 2 = Robot Grasp
uint8_t progressPercent = 0;   // 0 to 100%
char currentStatus[17] = "IDLE";
char infoText[27] = "Waiting for ESP32...";

// Line buffer for incoming commands (ESP32 on D0, or the laptop over USB)
struct LineReader { char buf[44]; uint8_t idx; bool discard; };
LineReader lineReader = { {0}, 0, false };

// Dynamic custom override buffers (used if server sends unknown medicine)
char customName[16] = "Custom Med";
char customStrength[8] = "10mg";
char customCat[20] = "General";
char customShelf[16] = "S1/L1";
char customDose[16] = "As prescribed";
char customFreq[20] = "Daily";
char customMax[16] = "See label";
char customUse[22] = "Therapeutic";
char customWarn[30] = "Demo medicine only";

// Non-blocking timers
unsigned long lastPingTime = 0;
unsigned long lastPageSwitch = 0;
unsigned long lastAnimTick = 0;
unsigned long stateStartTime = 0;

#define WATCHDOG_TIMEOUT_MS 10000  // 10 seconds heartbeat limit
#define CARD_CYCLE_MS       3000   // 3 seconds per medicine card page

// Animation sub-states
uint8_t blinkState = 0;       // 0 = open, 1 = closed
int16_t tearY = 0;            // Tear drop Y position (0 = inactive)
uint8_t scanLineY = 60;       // Scanning laser beam Y coordinate
int8_t  scanDir = 4;          // Laser beam direction (+4 down, -4 up)
uint8_t thinkStep = 0;        // Thinking dots counter

// Fixed serial input buffer (44 bytes is plenty for protocol lines, safe for RAM)

// Forward function declarations
void readSerialNonBlocking();
void parseProtocolLine(char* line);
void applyFaceState(const char* stateName);
void renderFaceFull();
void updateFaceAnimations();
void renderCardPage();
void renderProgressBarOnly();
void drawSadIdleEyes(bool closed);
void drawHappyEyes();
void drawListeningEyes();
void drawThinkingEyes();
void drawScanningEyes();
void drawAlertEyes();
void drawObstacleEyes();
void drawErrorEyes();
void selectMedByName(const char* nameStr);
void drawInfoLine();

// ----------------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------------
void setup() {
  // Hardware serial on D0: ESP32 GPIO17 (or the laptop over USB) at 115200 baud
  Serial.begin(115200);

  // Initialize display controller
  uint16_t detectedID = FORCE_ID;
  if (detectedID == 0) {
    detectedID = tft.readID();
  }
  // Bogus IDs seen on MAR3501 / write-only ILI9486 shields -> force ILI9486
  if (detectedID == 0xD3D3 || detectedID == 0x0000 || detectedID == 0xFFFF || detectedID == 0x9481) {
    detectedID = 0x9486;
  }
  tft.begin(detectedID);
  tft.setRotation(SCREEN_ROTATION); // Landscape 480x320

  // Quick boot splash
  tft.fillScreen(NAVY);
  tft.setTextColor(CYAN);
  tft.setTextSize(3);
  tft.setCursor(65, 120);
  tft.print(F("PHARMA ROVER"));

  tft.setTextSize(1);
  tft.setTextColor(LIGHTGREY);
  tft.setCursor(120, 165);
  tft.print(F("Autonomous Medicine Delivery Robot"));
  tft.setCursor(140, 185);
  tft.print(F("TFT 3.5\" ILI9486 (480x320)"));

  delay(1200);

  // Reset watchdog and draw initial sad idle face
  lastPingTime = millis();
  currentFace = FACE_IDLE;
  showCard = false;
  renderFaceFull();

  Serial.println(F("OK|DISPLAY_READY"));
}

// ----------------------------------------------------------------------------------
// MAIN LOOP (Strictly non-blocking: uses millis(), NO delay() calls)
// ----------------------------------------------------------------------------------
void loop() {
  // 1. Process incoming serial bytes continuously
  readSerialNonBlocking();

  unsigned long currentMillis = millis();

  // 2. Watchdog Heartbeat Check (10 seconds timeout -> NO LINK)
  if (currentMillis - lastPingTime > WATCHDOG_TIMEOUT_MS) {
    if (currentFace != FACE_ERROR || strcmp(currentStatus, "NO LINK") != 0) {
      currentFace = FACE_ERROR;
      showCard = false;
      strncpy(currentStatus, "NO LINK", sizeof(currentStatus) - 1);
      renderFaceFull();
    }
  }

  // 3. Auto-cycle medicine card pages every 3 seconds
  if (showCard) {
    if (currentMillis - lastPageSwitch >= CARD_CYCLE_MS) {
      lastPageSwitch = currentMillis;
      cardPage = (cardPage + 1) % 3; // Cycle: 0 -> 1 -> 2 -> 0
      renderCardPage();
    }
  } else {
    // 4. Update expressive face micro-animations (every 80 ms)
    if (currentMillis - lastAnimTick >= 80) {
      lastAnimTick = currentMillis;
      updateFaceAnimations();
    }
  }

  // 5. Celebration auto-timeout (return to sad idle after 2.5s)
  if (currentFace == FACE_HAPPY && !showCard) {
    if (currentMillis - stateStartTime > 2500) {
      currentFace = FACE_IDLE;
      renderFaceFull();
    }
  }
}

// ----------------------------------------------------------------------------------
// NON-BLOCKING SERIAL PARSER
// ----------------------------------------------------------------------------------
// Feed one received character into a line buffer; runs the command at end of line
void feedChar(LineReader &r, char c) {
  lastPingTime = millis(); // Refresh watchdog on any incoming byte

  if (c == '\n' || c == '\r') {
    if (r.idx > 0 && !r.discard) {
      r.buf[r.idx] = '\0';
      // Link came back after a "NO LINK" error -> go back to the idle face
      if (currentFace == FACE_ERROR && strcmp(currentStatus, "NO LINK") == 0) {
        strncpy(currentStatus, "IDLE", sizeof(currentStatus) - 1);
        currentFace = FACE_IDLE;
        if (!showCard) renderFaceFull();
      }
      parseProtocolLine(r.buf);
    }
    r.idx = 0;
    r.discard = false;
  } else if (!r.discard) {
    if (r.idx < sizeof(r.buf) - 1) {
      r.buf[r.idx++] = c;
    } else {
      // Line too long: drop the rest of it (prevents overflow and garbage commands)
      r.idx = 0;
      r.discard = true;
    }
  }
}

void readSerialNonBlocking() {
  while (Serial.available() > 0) feedChar(lineReader, (char)Serial.read());
}

void parseProtocolLine(char* line) {
  char* cmd = strtok(line, "|");
  if (!cmd) return;

  if (strcmp(cmd, "STATE") == 0) {
    char* arg = strtok(NULL, "|");
    if (arg) applyFaceState(arg);
  }
  else if (strcmp(cmd, "CARDSHOW") == 0) {
    char* st = strtok(NULL, "|");
    char* pr = strtok(NULL, "|");
    if (st) strncpy(currentStatus, st, sizeof(currentStatus) - 1);
    if (pr) progressPercent = (uint8_t)constrain(atoi(pr), 0, 100);
    showCard = true;
    cardPage = 0;
    lastPageSwitch = millis();
    renderCardPage();
  }
  else if (strcmp(cmd, "STATUS") == 0) {
    char* st = strtok(NULL, "|");
    char* pr = strtok(NULL, "|");
    if (st) strncpy(currentStatus, st, sizeof(currentStatus) - 1);
    if (pr) progressPercent = (uint8_t)constrain(atoi(pr), 0, 100);

    if (showCard) {
      if (cardPage == 0 || cardPage == 2) {
        renderProgressBarOnly();
      }
    }
  }
  else if (strcmp(cmd, "CARDF") == 0) {
    char* field = strtok(NULL, "|");
    char* val = strtok(NULL, "|");
    if (field && val) {
      if (strcmp(field, "name") == 0) {
        selectMedByName(val);
        strncpy(customName, val, sizeof(customName) - 1);
      } else if (strcmp(field, "strength") == 0) {
        strncpy(customStrength, val, sizeof(customStrength) - 1);
      } else if (strcmp(field, "category") == 0) {
        strncpy(customCat, val, sizeof(customCat) - 1);
      } else if (strcmp(field, "shelf") == 0) {
        strncpy(customShelf, val, sizeof(customShelf) - 1);
      } else if (strcmp(field, "dose") == 0) {
        strncpy(customDose, val, sizeof(customDose) - 1);
      } else if (strcmp(field, "freq") == 0) {
        strncpy(customFreq, val, sizeof(customFreq) - 1);
      } else if (strcmp(field, "max") == 0) {
        strncpy(customMax, val, sizeof(customMax) - 1);
      } else if (strcmp(field, "use") == 0) {
        strncpy(customUse, val, sizeof(customUse) - 1);
      } else if (strcmp(field, "warn") == 0) {
        strncpy(customWarn, val, sizeof(customWarn) - 1);
      }
    }
  }
  else if (strcmp(cmd, "MED") == 0) {
    char* medArg = strtok(NULL, "|");
    if (medArg) {
      selectMedByName(medArg);
      showCard = true;
      cardPage = 0;
      lastPageSwitch = millis();
      renderCardPage();
    }
  }
  else if (strcmp(cmd, "CLEAR") == 0) {
    showCard = false;
    currentFace = FACE_IDLE;
    renderFaceFull();
  }
  else if (strcmp(cmd, "PING") == 0) {
    // Watchdog successfully refreshed
  }
  else if (strcmp(cmd, "INFO") == 0) {
    char* txt = strtok(NULL, "|");
    if (txt) {
      strncpy(infoText, txt, sizeof(infoText) - 1);
      infoText[sizeof(infoText) - 1] = '\0';
      if (!showCard) drawInfoLine();
    }
  }
}

// Small grey text at the bottom-left of the face screens (IP address, Wi-Fi state)
void drawInfoLine() {
  tft.fillRect(0, 304, 166, 16, NAVY);
  tft.setTextSize(1);
  tft.setTextColor(LIGHTGREY);
  tft.setCursor(4, 308);
  tft.print(infoText);
}

void selectMedByName(const char* str) {
  // Names first (a name like "Cetirizine 10mg" contains digits too), then exact numbers
  if (strstr(str, "para") || strstr(str, "Para") || strstr(str, "PARA")) {
    selectedMed = 0;
  } else if (strstr(str, "ceti") || strstr(str, "Ceti") || strstr(str, "CETI")) {
    selectedMed = 1;
  } else if (strstr(str, "omep") || strstr(str, "Omep") || strstr(str, "OMEP")) {
    selectedMed = 2;
  } else if (strcmp(str, "0") == 0 || strcmp(str, "21") == 0) {
    selectedMed = 0;
  } else if (strcmp(str, "1") == 0 || strcmp(str, "22") == 0) {
    selectedMed = 1;
  } else if (strcmp(str, "2") == 0 || strcmp(str, "23") == 0) {
    selectedMed = 2;
  } else {
    selectedMed = 255; // Custom medicine
  }
}

void applyFaceState(const char* name) {
  showCard = false;
  stateStartTime = millis();

  if (strcmp(name, "idle") == 0)          currentFace = FACE_IDLE;
  else if (strcmp(name, "listening") == 0) currentFace = FACE_LISTENING;
  else if (strcmp(name, "thinking") == 0)  currentFace = FACE_THINKING;
  else if (strcmp(name, "scanning") == 0)  currentFace = FACE_SCANNING;
  else if (strcmp(name, "alert") == 0)     currentFace = FACE_ALERT;
  else if (strcmp(name, "happy") == 0 || strcmp(name, "success") == 0) currentFace = FACE_HAPPY;
  else if (strcmp(name, "obstacle") == 0) currentFace = FACE_OBSTACLE;
  else if (strcmp(name, "error") == 0)    currentFace = FACE_ERROR;

  renderFaceFull();
}

// ----------------------------------------------------------------------------------
// FACE RENDERING ROUTINES
// ----------------------------------------------------------------------------------
void renderFaceFull() {
  tft.fillScreen(NAVY);
  blinkState = 0;
  tearY = 0;
  scanLineY = 60;
  thinkStep = 0;

  switch (currentFace) {
    case FACE_IDLE:
      drawSadIdleEyes(false);
      break;
    case FACE_LISTENING:
      drawListeningEyes();
      break;
    case FACE_THINKING:
      drawThinkingEyes();
      break;
    case FACE_SCANNING:
      drawScanningEyes();
      break;
    case FACE_ALERT:
      drawAlertEyes();
      break;
    case FACE_HAPPY:
      drawHappyEyes();
      break;
    case FACE_OBSTACLE:
      drawObstacleEyes();
      break;
    case FACE_ERROR:
      drawErrorEyes();
      break;
  }
  drawInfoLine();
}

// 1. Sad Idle Face: Droopy eyelids, downcast pupils, sad eyebrows, downturned mouth
void drawSadIdleEyes(bool closed) {
  int cx1 = 150, cx2 = 330, cy = 130, r = 50;

  if (closed) {
    // Clear open eye area
    tft.fillCircle(cx1, cy, r + 4, NAVY);
    tft.fillCircle(cx2, cy, r + 4, NAVY);
    // Draw closed drooping eye lines
    tft.drawLine(cx1 - r, cy, cx1 + r, cy + 12, CYAN);
    tft.drawLine(cx1 - r, cy + 1, cx1 + r, cy + 13, CYAN);
    tft.drawLine(cx2 - r, cy + 12, cx2 + r, cy, CYAN);
    tft.drawLine(cx2 - r, cy + 13, cx2 + r, cy + 1, CYAN);
  } else {
    // Eyeball outlines
    tft.drawCircle(cx1, cy, r, CYAN);
    tft.drawCircle(cx1, cy, r - 1, CYAN);
    tft.drawCircle(cx2, cy, r, CYAN);
    tft.drawCircle(cx2, cy, r - 1, CYAN);

    // Downcast pupils
    tft.fillCircle(cx1, cy + 20, 15, CYAN);
    tft.fillCircle(cx2, cy + 20, 15, CYAN);

    // Sad droopy eyebrows slanted downward toward the outside
    tft.drawLine(cx1 - r - 8, cy - r + 15, cx1 + r - 10, cy - r - 10, CYAN);
    tft.drawLine(cx1 - r - 8, cy - r + 16, cx1 + r - 10, cy - r - 9, CYAN);
    tft.drawLine(cx2 - r + 10, cy - r - 10, cx2 + r + 8, cy - r + 15, CYAN);
    tft.drawLine(cx2 - r + 10, cy - r - 9, cx2 + r + 8, cy - r + 16, CYAN);

    // Downturned sad mouth (inverted arc using circles)
    tft.drawCircle(240, 290, 45, CYAN);
    tft.drawCircle(240, 290, 46, CYAN);
    tft.fillRect(170, 275, 140, 50, NAVY);
  }
}

// 2. Listening Face: Wide attentive cyan eyes, small 'O' mouth, sound wave brackets
void drawListeningEyes() {
  int cx1 = 150, cx2 = 330, cy = 130, r = 52;

  tft.fillCircle(cx1, cy, r, CYAN);
  tft.fillCircle(cx2, cy, r, CYAN);
  tft.drawCircle(cx1, cy, r + 2, WHITE);
  tft.drawCircle(cx2, cy, r + 2, WHITE);

  // Dark pupils in center
  tft.fillCircle(cx1, cy, 18, NAVY);
  tft.fillCircle(cx2, cy, 18, NAVY);
  tft.fillCircle(cx1 + 5, cy - 5, 4, WHITE); // Reflection glint
  tft.fillCircle(cx2 + 5, cy - 5, 4, WHITE);

  // Small attentive 'O' mouth
  tft.drawCircle(240, 245, 16, WHITE);
  tft.drawCircle(240, 245, 17, WHITE);

  // Sound wave side brackets
  for (int w = 15; w <= 30; w += 15) {
    tft.drawCircle(cx1 - 50, cy, r + w, CYAN);
    tft.fillRect(cx1 - 50, cy - (r + w) - 2, r + w + 4, (r + w) * 2 + 4, NAVY);
    tft.drawCircle(cx2 + 50, cy, r + w, CYAN);
    tft.fillRect(cx2 + 50 - (r + w + 2), cy - (r + w) - 2, r + w + 4, (r + w) * 2 + 4, NAVY);
  }
}

// 3. Thinking Face: Looking upward/rightward, wavy mouth, animated dots
void drawThinkingEyes() {
  int cx1 = 150, cx2 = 330, cy = 130, r = 48;

  tft.drawCircle(cx1, cy, r, YELLOW);
  tft.drawCircle(cx1, cy, r - 1, YELLOW);
  tft.drawCircle(cx2, cy, r, YELLOW);
  tft.drawCircle(cx2, cy, r - 1, YELLOW);

  // Pupils looking upward and rightward
  tft.fillCircle(cx1 + 15, cy - 18, 16, YELLOW);
  tft.fillCircle(cx2 + 15, cy - 18, 16, YELLOW);

  // Wavy mouth
  tft.drawLine(200, 245, 225, 235, YELLOW);
  tft.drawLine(225, 235, 255, 250, YELLOW);
  tft.drawLine(255, 250, 280, 240, YELLOW);
}

// 4. Scanning Face: Horizontal slit eyes, green laser line
void drawScanningEyes() {
  int cx1 = 150, cx2 = 330, cy = 130, r = 45;

  // Narrow focused slit eyes
  tft.fillRect(cx1 - r, cy - 6, r * 2, 12, GREEN);
  tft.fillRect(cx2 - r, cy - 6, r * 2, 12, GREEN);

  // Concentrated straight mouth
  tft.fillRect(210, 245, 60, 6, GREEN);
}

// 5. Alert Face: Alert wide eyes looking straight ahead
void drawAlertEyes() {
  int cx1 = 150, cx2 = 330, cy = 130, r = 50;

  tft.drawCircle(cx1, cy, r, WHITE);
  tft.drawCircle(cx1, cy, r - 1, WHITE);
  tft.drawCircle(cx2, cy, r, WHITE);
  tft.drawCircle(cx2, cy, r - 1, WHITE);

  tft.fillCircle(cx1, cy, 20, CYAN);
  tft.fillCircle(cx2, cy, 20, CYAN);

  // Firm straight mouth
  tft.fillRect(205, 245, 70, 6, WHITE);
}

// 6. Happy Face: Celebratory upward arc eyes (^ ^), big smile, checkmark
void drawHappyEyes() {
  int cx1 = 150, cx2 = 330, cy = 140, r = 50;

  // Upward smiling arc eyes (^ ^)
  tft.drawCircle(cx1, cy, r, GREEN);
  tft.drawCircle(cx1, cy, r - 1, GREEN);
  tft.drawCircle(cx1, cy, r - 2, GREEN);
  tft.fillRect(cx1 - r - 2, cy, (r * 2) + 4, r + 4, NAVY);

  tft.drawCircle(cx2, cy, r, GREEN);
  tft.drawCircle(cx2, cy, r - 1, GREEN);
  tft.drawCircle(cx2, cy, r - 2, GREEN);
  tft.fillRect(cx2 - r - 2, cy, (r * 2) + 4, r + 4, NAVY);

  // Big wide bright smile
  tft.drawCircle(240, 215, 55, GREEN);
  tft.drawCircle(240, 215, 56, GREEN);
  tft.fillRect(160, 160, 160, 56, NAVY);

  // Prominent Green Checkmark in top-right corner
  tft.drawLine(410, 55, 425, 70, GREEN);
  tft.drawLine(410, 56, 425, 71, GREEN);
  tft.drawLine(410, 57, 425, 72, GREEN);
  tft.drawLine(425, 70, 455, 35, GREEN);
  tft.drawLine(425, 71, 455, 36, GREEN);
  tft.drawLine(425, 72, 455, 37, GREEN);
}

// 7. Obstacle Face: Startled wide red eyes, exclamation mark "!"
void drawObstacleEyes() {
  int cx1 = 150, cx2 = 330, cy = 135, r = 52;

  tft.fillCircle(cx1, cy, r, RED);
  tft.fillCircle(cx2, cy, r, RED);
  tft.fillCircle(cx1, cy, 18, WHITE);
  tft.fillCircle(cx2, cy, 18, WHITE);

  // Exclamation mark in center top
  tft.fillRect(237, 30, 8, 45, RED);
  tft.fillCircle(241, 90, 5, RED);

  // Startled open mouth
  tft.drawCircle(240, 250, 20, RED);
  tft.drawCircle(240, 250, 21, RED);
}

// 8. Error Face: Red 'X' eyes, zigzag mouth, status line
void drawErrorEyes() {
  int cx1 = 150, cx2 = 330, cy = 130;

  for (int d = -2; d <= 2; d++) {
    tft.drawLine(cx1 - 32 + d, cy - 32, cx1 + 32 + d, cy + 32, RED);
    tft.drawLine(cx1 + 32 + d, cy - 32, cx1 - 32 + d, cy + 32, RED);
    tft.drawLine(cx2 - 32 + d, cy - 32, cx2 + 32 + d, cy + 32, RED);
    tft.drawLine(cx2 + 32 + d, cy - 32, cx2 - 32 + d, cy + 32, RED);
  }

  // Zigzag mouth
  tft.drawLine(190, 245, 210, 230, RED);
  tft.drawLine(210, 230, 230, 250, RED);
  tft.drawLine(230, 250, 250, 230, RED);
  tft.drawLine(250, 230, 270, 250, RED);
  tft.drawLine(270, 250, 290, 235, RED);

  // Error status text
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.setCursor(170, 280);
  tft.print(currentStatus);
}

// ----------------------------------------------------------------------------------
// FACE MICRO-ANIMATIONS (Non-blocking, minimal flicker)
// ----------------------------------------------------------------------------------
void updateFaceAnimations() {
  unsigned long now = millis();

  if (currentFace == FACE_IDLE) {
    // A. Natural Blinking Cycle: blink shut for 220ms every ~4.5 seconds
    bool shouldBlink = ((now / 4500) % 2 == 0) && ((now % 4500) < 220);
    if (shouldBlink && blinkState == 0) {
      blinkState = 1;
      drawSadIdleEyes(true);
    } else if (!shouldBlink && blinkState == 1) {
      blinkState = 0;
      drawSadIdleEyes(false);
    }

    // B. Falling Teardrop Animation from left eye (every ~3.5 seconds)
    int tearCycle = now % 3500;
    if (tearCycle < 1400) {
      int newY = 185 + (tearCycle * 70 / 1400); // 185 to 255
      if (tearY != 0 && tearY != newY) {
        // Erase old teardrop with background color
        tft.fillCircle(175, tearY, 5, NAVY);
      }
      tearY = newY;
      tft.fillCircle(175, tearY, 5, TEAR_BLUE);
    } else {
      if (tearY != 0) {
        tft.fillCircle(175, tearY, 5, NAVY);
        tearY = 0;
      }
    }
  }
  else if (currentFace == FACE_SCANNING) {
    // Sweeping laser scan beam moving up and down
    tft.drawFastHLine(50, scanLineY, 380, NAVY);
    scanLineY += scanDir;
    if (scanLineY >= 230) {
      scanLineY = 230;
      scanDir = -4;
    } else if (scanLineY <= 60) {
      scanLineY = 60;
      scanDir = 4;
    }
    tft.drawFastHLine(50, scanLineY, 380, GREEN);
  }
  else if (currentFace == FACE_THINKING) {
    // Thinking dots ("...") in top right
    uint8_t step = (now / 400) % 4;
    if (step != thinkStep) {
      thinkStep = step;
      tft.fillRect(350, 45, 80, 20, NAVY);
      for (uint8_t i = 0; i < thinkStep; i++) {
        tft.fillCircle(360 + (i * 20), 55, 5, YELLOW);
      }
    }
  }
}

// ----------------------------------------------------------------------------------
// MEDICINE CARD RENDERING (3 Auto-Cycling Pages)
// ----------------------------------------------------------------------------------
// Helper function to read string from PROGMEM table into a temporary buffer
void getMedFieldP(uint8_t medIdx, uint8_t fieldOffset, char* dest, uint8_t maxLen) {
  if (medIdx < 3) {
    const char* srcPtr = (const char*)(&BUILTIN_MEDS[medIdx]) + fieldOffset;
    strncpy_P(dest, srcPtr, maxLen - 1);
    dest[maxLen - 1] = '\0';
  } else {
    dest[0] = '\0';
  }
}

void renderCardPage() {
  tft.fillScreen(BLACK);

  // Common Header Banner across all 3 pages
  tft.fillRect(0, 0, SCREEN_W, 46, BLUE);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(16, 14);

  char tempBuf[34];
  if (selectedMed < 3) {
    // Print Name + Strength from PROGMEM
    strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].name, sizeof(tempBuf) - 1);
    tft.print(tempBuf);
    tft.print(F(" "));
    strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].strength, sizeof(tempBuf) - 1);
    tft.print(tempBuf);
  } else {
    tft.print(customName);
    tft.print(F(" "));
    tft.print(customStrength);
  }

  // ---------------- PAGE 1: OVERVIEW & LOGISTICS ----------------
  if (cardPage == 0) {
    tft.setTextSize(2);

    // Type / Category
    tft.setCursor(16, 60);
    tft.setTextColor(CYAN);
    tft.print(F("Type: "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].category, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(customCat);
    }

    // Shelf and Slot
    tft.setCursor(16, 95);
    tft.setTextColor(CYAN);
    tft.print(F("Shelf: "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].shelf_slot, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(customShelf);
    }

    // ArUco Marker ID
    tft.setCursor(16, 130);
    tft.setTextColor(CYAN);
    tft.print(F("ArUco ID: "));
    tft.setTextColor(YELLOW);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].aruco, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(F("21"));
    }

    // Weight
    tft.setCursor(260, 130);
    tft.setTextColor(CYAN);
    tft.print(F("Weight: "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].weight, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(F("45g"));
    }

    // Status Line
    tft.setCursor(16, 175);
    tft.setTextColor(GREEN);
    tft.print(F("Mission: "));
    tft.print(currentStatus);

    // Page Indicator Footer
    tft.setTextSize(1);
    tft.setTextColor(LIGHTGREY);
    tft.setCursor(16, 298);
    tft.print(F("Page 1/3 - Overview | Pharma Rover Mission"));
  }

  // ---------------- PAGE 2: CLINICAL DOSAGE & INDICATIONS ----------------
  else if (cardPage == 1) {
    tft.setTextSize(2);

    // Dose
    tft.setCursor(16, 60);
    tft.setTextColor(CYAN);
    tft.print(F("Dose: "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].dose, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(customDose);
    }

    // Frequency
    tft.setCursor(16, 95);
    tft.setTextColor(CYAN);
    tft.print(F("Freq: "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].freq, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(customFreq);
    }

    // Max Dose
    tft.setCursor(16, 130);
    tft.setTextColor(CYAN);
    tft.print(F("Max:  "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].max_dose, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(customMax);
    }

    // Medical Use
    tft.setCursor(16, 165);
    tft.setTextColor(CYAN);
    tft.print(F("Use:  "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].use, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(customUse);
    }

    // Clinical Precaution / Warning Box
    tft.drawRect(12, 205, SCREEN_W - 24, 45, RED);
    tft.fillRect(13, 206, SCREEN_W - 26, 43, DARKGREY);
    tft.setCursor(20, 220);
    tft.setTextColor(YELLOW);
    tft.setTextSize(1);
    tft.print(F("NOTE: "));
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].note, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(customWarn);
    }

    // Page Indicator Footer
    tft.setTextSize(1);
    tft.setTextColor(LIGHTGREY);
    tft.setCursor(16, 298);
    tft.print(F("Page 2/3 - Clinical Info | Safety Validated"));
  }

  // ---------------- PAGE 3: ROBOT KINEMATICS & GRASP ----------------
  else if (cardPage == 2) {
    tft.setTextSize(2);

    // Arm Grasp Position
    tft.setCursor(16, 60);
    tft.setTextColor(CYAN);
    tft.print(F("Grasp: "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].grasp, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(F("[250, 0, -12]"));
    }
    tft.setTextSize(1);
    tft.print(F(" mm"));
    tft.setTextSize(2);

    // EAN-13 Barcode
    tft.setCursor(16, 95);
    tft.setTextColor(CYAN);
    tft.print(F("EAN:   "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].ean, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(F("8961000000112"));
    }

    // Box Size
    tft.setCursor(16, 130);
    tft.setTextColor(CYAN);
    tft.print(F("Box:   "));
    tft.setTextColor(WHITE);
    if (selectedMed < 3) {
      strncpy_P(tempBuf, BUILTIN_MEDS[selectedMed].box_size, sizeof(tempBuf) - 1);
      tft.print(tempBuf);
    } else {
      tft.print(F("100x50x20"));
    }
    tft.setTextSize(1);
    tft.print(F(" mm"));

    // Render Progress Bar
    renderProgressBarOnly();

    // Page Indicator Footer
    tft.setTextSize(1);
    tft.setTextColor(LIGHTGREY);
    tft.setCursor(16, 298);
    tft.print(F("Page 3/3 - Robot Grasp & Verification Status"));
  }
}

// Partial redraw for progress bar and status line (prevents full-screen flashing)
void renderProgressBarOnly() {
  int barX = 16;
  int barY = 240;
  int barW = SCREEN_W - 32;
  int barH = 18;

  // Status text line
  tft.fillRect(barX, barY - 25, barW, 20, BLACK);
  tft.setCursor(barX, barY - 25);
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.print(currentStatus);
  tft.print(F(" ("));
  tft.print(progressPercent);
  tft.print(F("%)"));

  // Progress Bar Frame
  tft.drawRect(barX, barY, barW, barH, WHITE);

  // Filled bar portion
  int fillW = ((long)(barW - 4) * progressPercent) / 100;
  if (fillW > 0) {
    tft.fillRect(barX + 2, barY + 2, fillW, barH - 4, GREEN);
  }
  // Clear remaining unfilled portion
  int emptyW = (barW - 4) - fillW;
  if (emptyW > 0) {
    tft.fillRect(barX + 2 + fillW, barY + 2, emptyW, barH - 4, BLACK);
  }
}
