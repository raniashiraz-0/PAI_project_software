#pragma once
// ============================================================================
//  Pharma Rover MAIN ESP32 - everything you may need to change is in this file
// ============================================================================

// ---------------- Wi-Fi (use your phone hotspot / router, 2.4 GHz) ----------
#define WIFI_SSID      "66C-2.4G"
#define WIFI_PASSWORD  "ninetyone"
#define DEVICE_HOSTNAME "pharmarover"      // also reachable as pharmarover.local

// ---------------- Network ports (must match server/config.yaml) -------------
#define CAR_TCP_PORT        5000
#define ARM_TCP_PORT        5001
#define DISPLAY_TCP_PORT    8080
#define DISCOVERY_UDP_PORT  4210           // server broadcasts "PRSRV <port>" here

// ---------------- I2C to both PCA9685 boards (see wiring diagram) -----------
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
#define PCA_ARM_BIG   0x40   // PCA9685 #1 : 3x MG995 (yaw, shoulder, elbow)
#define PCA_ARM_SMALL 0x41   // PCA9685 #2 : 2x MG90 + 1x SG90 (A0 pad bridged)

// ONE_PCA_BOARD 1 = both PCA9685 boards may stay at the SAME address 0x40 (A0 not soldered).
//   Both boards receive every command, so they are kept apart by CHANNEL instead:
//     big-servo board   : base ch 0, shoulder ch 1, elbow ch 2   (its ch 3-5 stay empty)
//     small-servo board : wrist pitch ch 3, wrist roll ch 4, gripper ch 5 (its ch 0-2 stay empty)
//   Each board keeps its own servo battery. SDA/SCL/VCC/GND wiring stays as it is.
// ONE_PCA_BOARD 0 = A0 soldered on the small board (0x41): small servos on 0x41 ch 0-2.
#define ONE_PCA_BOARD 1

// ---------------- Base motor driver (L298N) - YOUR existing car wiring ---------
// Same pins as your old Bluetooth car code, EXCEPT ENB: it was GPIO21, which is now
// SDA for the arm's PCA9685 boards -> move the ENB wire to GPIO25.
//
//   L298N pin : ENA  IN1  IN2  IN3  IN4  ENB
//   ESP32 GPIO:  2    4    5   18   19   25 (was 21)
//
// In your old code "Left" = IN1/IN2 forward + IN3/IN4 backward, so IN1/IN2/ENA
// drive the RIGHT-side motors and IN3/IN4/ENB drive the LEFT-side motors.
#define LEFT_EN     25   // L298N ENB (PWM)  <- moved from GPIO21
#define LEFT_IN_A   18   // L298N IN3   (HIGH = forward, as in your Forward())
#define LEFT_IN_B   19   // L298N IN4
#define RIGHT_EN     2   // L298N ENA (PWM)
#define RIGHT_IN_A   4   // L298N IN1   (HIGH = forward)
#define RIGHT_IN_B   5   // L298N IN2

// If one side drives backwards when told to go forward, set its value to 1.
// If "turn left" turns right, swap the LEFT_* and RIGHT_* numbers above.
#define INVERT_LEFT_MOTOR   0
#define INVERT_RIGHT_MOTOR  0

// Smallest PWM (0-255) that actually makes the wheels turn on the floor
#define MOTOR_MIN_PWM  110
#define MOTOR_MAX_PWM  255

// ---- Calibration for open-loop steps (measure once, see SETUP_GUIDE.md) ----
// Drive forward at 100% for 2 s, measure the distance in mm, divide by 2.
#define CAL_MM_PER_S_AT_100    420.0f
// Spin in place at 100% for 2 s, measure the angle in degrees, divide by 2.
#define CAL_DEG_PER_S_AT_100   200.0f
// Distance between left and right wheel centres (mm)
#define TRACK_WIDTH_MM         150.0f

// ---------------- 4x HC-SR04 ultrasonic sensors - YOUR existing wiring ---------
// Same TRIG/ECHO pins as your old Bluetooth car code.
// ECHO is 5 V on the HC-SR04 -> a divider (1k from ECHO, 2k to GND) before the
// ESP32 pin protects it.
#define SONAR_FRONT_TRIG  13
#define SONAR_FRONT_ECHO  34
#define SONAR_BACK_TRIG   27
#define SONAR_BACK_ECHO   35
#define SONAR_LEFT_TRIG   23
#define SONAR_LEFT_ECHO   33
#define SONAR_RIGHT_TRIG  26
#define SONAR_RIGHT_ECHO  36   // "VP" on most boards

#define SONAR_MAX_MM            3000
#define DEFAULT_FRONT_STOP_MM   150    // ESP32 stops by itself below this (server can change it)
#define DEFAULT_BACK_STOP_MM    120

// ---------------- Battery voltage sense (optional) --------------------------
// 7.4 V pack -> 10k / 3.3k divider -> GPIO32. Set to -1 if not wired.
#define BATTERY_ADC_PIN    32
#define BATTERY_DIVIDER    ((10.0f + 3.3f) / 3.3f)

// ---------------- Uno TFT link (same connection as before) -------------------
// ESP32 GPIO17 (TX2) -> Uno D0 (RX).  Also connect ESP32 GND to Uno GND.
// Unplug the D0 wire only while UPLOADING to the Uno with the Arduino IDE.
#define UNO_TX_PIN   17
#define UNO_RX_PIN   16      // not used, the Uno never talks back
#define UNO_BAUD     115200  // must match Serial.begin() in uno_tft_display.ino
#define UNO_LINE_GAP_MS     25    // pause between normal lines
#define UNO_REDRAW_GAP_MS  400    // pause after a line that redraws the screen
                                  // (STATE / CARDSHOW / MED / CLEAR) so the Uno's
                                  // 64-byte buffer never overflows while it draws

// ---------------- Arm servos -------------------------------------------------
// Joint order used everywhere: 0 yaw, 1 shoulder, 2 elbow, 3 wrist pitch,
//                              4 wrist roll, 5 gripper
// {pca address, channel, min pulse us, max pulse us}
struct ServoPort { uint8_t addr; uint8_t ch; uint16_t us_min; uint16_t us_max; };
static const ServoPort SERVO_PORTS[6] = {
  { PCA_ARM_BIG,   0, 500, 2500 },   // yaw         MG995
  { PCA_ARM_BIG,   1, 500, 2500 },   // shoulder    MG995
  { PCA_ARM_BIG,   2, 500, 2500 },   // elbow       MG995
#if ONE_PCA_BOARD
  { PCA_ARM_BIG,   3, 500, 2400 },   // wrist pitch MG90  -> 0x40 channel 3
  { PCA_ARM_BIG,   4, 500, 2400 },   // wrist roll        -> 0x40 channel 4
  { PCA_ARM_BIG,   5, 500, 2400 },   // gripper           -> 0x40 channel 5
#else
  { PCA_ARM_SMALL, 0, 500, 2400 },   // wrist pitch MG90  (use an MG90 here, SG90 is too weak)
  { PCA_ARM_SMALL, 1, 500, 2400 },   // wrist roll  SG90
  { PCA_ARM_SMALL, 2, 500, 2400 },   // gripper     MG90
#endif
};

// Server joint angle -> servo angle:  servo = OFFSET + DIR * joint + TRIM
// (matches kin3d.to_servo in the server). Fix small errors with TRIM.
static const float JOINT_OFFSET[5] = { 90.0f, 0.0f, 180.0f, 90.0f, 90.0f };
static const float JOINT_DIR[5]    = {  1.0f, 1.0f,   1.0f,  1.0f,  1.0f };
static const float JOINT_TRIM[5]   = {  0.0f, 0.0f,   0.0f,  0.0f,  0.0f };

// Hard servo limits (degrees). MG995 = 0..180 only (not continuous).
static const float SERVO_MIN_DEG[6] = {  0,   0,   0,   0,   0,  90 };
static const float SERVO_MAX_DEG[6] = { 180, 180, 180, 180, 180, 180 };

// Gripper (allowed range is 90..180)
#define GRIP_OPEN_DEG     90.0f    // claws open
#define GRIP_CLOSED_DEG  170.0f    // claws closed on a box (tune so it grips without stalling)

// Servo speed limits (deg/s) - slow is safer for the heavy shoulder
static const float SERVO_SPEED_DPS[6] = { 50, 30, 40, 45, 60, 60 };

// HOME pose as SERVO angles (= joints [0, 110, -145, 60, 0])
static const float HOME_SERVO_DEG[6] = { 90, 110, 35, 150, 90, GRIP_OPEN_DEG };

// ---------------- Box-in-gripper sensor (optional, recommended) -------------
// Tells the server for sure that a box is between the claws.
//   * Micro limit switch: one leg -> GPIO14, other leg -> GND (internal pull-up)
//   * or IR obstacle module (FC-51 / TCRT5000): VCC 3.3V, GND, OUT -> GPIO14
// (GPIO4 is now a motor pin, so the sensor uses GPIO14. To enable: set 14 below.)
// Mount it inside the claws so the box presses / covers it when gripped.
// Leave -1 if you have no sensor: the server then checks with the ESP32-CAM.
#define GRIP_SENSOR_PIN         -1
#define GRIP_SENSOR_ACTIVE_LOW   1     // switch to GND / FC-51 output = LOW when a box is there

// Never move more than this many servos at the same time (power budget)
#define MAX_SIMULTANEOUS_SERVOS  2

// ---------------- Safety timers ---------------------------------------------
#define DRIVE_VEL_WATCHDOG_MS   500   // stop if no CMD_DRIVE_VEL for this long
#define TELEMETRY_PERIOD_MS     100
#define HEARTBEAT_PERIOD_MS     1000