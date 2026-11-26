// ============================================================================
//  PHARMA ROVER - MAIN ESP32 FIRMWARE
//
//  One ESP32 does all of this:
//    * 4 DC gear motors through an L298N (TCP 5000, framed protocol)
//    * 4 HC-SR04 ultrasonic sensors (stops by itself before hitting things,
//      sends the distances to the server 10x per second)
//    * Robotic arm: 2x PCA9685 (0x40 = 3x MG995, 0x41 = 2x MG90 + SG90)
//      on SDA 21 / SCL 22 (TCP 5001). Never more than 2 servos move at once.
//    * Relays text lines from the server (TCP 8080) to the Arduino Uno TFT
//      through Serial2 (GPIO17 -> Uno D0)
//    * Answers the server's UDP discovery broadcast so you don't have to type
//      the ESP32's IP address anywhere.
//
//  Edit config.h in this folder (Wi-Fi name/password, pins, calibration).
//  Upload: VS Code -> PlatformIO -> PROJECT TASKS -> esp32_main -> Upload (COM3)
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include "soc/gpio_reg.h"
#include "config.h"
#include "protocol.h"

// ----------------------------------------------------------------------------
//  Small helpers
// ----------------------------------------------------------------------------
static inline uint16_t rd16(const uint8_t* p) { return ((uint16_t)p[0] << 8) | p[1]; }
static inline int16_t  rdi16(const uint8_t* p) { return (int16_t)rd16(p); }
static inline void wr16(uint8_t* p, uint16_t v) { p[0] = v >> 8; p[1] = v & 0xFF; }

// ---- Paced line relay to the Uno ----
// Lines from the server (TCP 8080) and our own INFO lines are queued here and sent
// one at a time, UNO_LINE_GAP_MS apart, so the Uno never misses one while drawing.
static char unoQueue[2048];
static uint16_t unoHead = 0, unoTail = 0, unoLines = 0;
static uint32_t unoLastSend = 0, unoGap = 0;

static void unoPush(char c) {
  uint16_t next = (unoHead + 1) % sizeof(unoQueue);
  if (next == unoTail) return;                 // queue full: drop (never happens in practice)
  unoQueue[unoHead] = c;
  unoHead = next;
  if (c == '\n') unoLines++;
}

static void sendLineToUno(const char* line) {
  while (*line) unoPush(*line++);
  unoPush('\n');
}

static void unoRelayUpdate() {
  if (unoLines == 0 || millis() - unoLastSend < unoGap) return;
  char head[10]; uint8_t n = 0;                 // first letters of the line
  while (unoTail != unoHead) {                  // send exactly one line
    char c = unoQueue[unoTail];
    unoTail = (unoTail + 1) % sizeof(unoQueue);
    Serial2.write((uint8_t)c);
    if (n < sizeof(head) - 1) head[n++] = c;
    if (c == '\n') { unoLines--; break; }
  }
  head[n] = 0;
  bool redraw = !strncmp(head, "STATE|", 6) || !strncmp(head, "CARDSHOW|", 9) ||
                !strncmp(head, "MED|", 4) || !strncmp(head, "CLEAR", 5);
  unoGap = redraw ? UNO_REDRAW_GAP_MS : UNO_LINE_GAP_MS;
  unoLastSend = millis();
}

// ----------------------------------------------------------------------------
//  Framed protocol parser + one TCP port that accepts a single client
// ----------------------------------------------------------------------------
class FramedPort {
public:
  explicit FramedPort(uint16_t port) : _server(port), _port(port) {}

  void begin() { _server.begin(); _server.setNoDelay(true); }

  // Returns true when a new client just connected
  bool acceptClient() {
    WiFiClient incoming = _server.available();
    if (incoming) {
      if (_client && _client.connected()) _client.stop();   // newest client wins
      _client = incoming;
      _client.setNoDelay(true);
      _state = 0;
      Serial.printf("[TCP %u] client connected from %s\n", _port, _client.remoteIP().toString().c_str());
      return true;
    }
    return false;
  }

  bool connected() { return _client && _client.connected(); }

  // Reads bytes; returns true when one complete valid frame is ready
  bool poll(uint8_t& cmd, uint8_t* payload, uint8_t& len) {
    if (!connected()) return false;
    while (_client.available() > 0) {
      uint8_t b = (uint8_t)_client.read();
      switch (_state) {
        case 0: if (b == PROTO_START) _state = 1; break;
        case 1:
          if (b < 1 || b > PROTO_MAX_PAYLOAD + 1) { _state = 0; break; }
          _len = b; _chk = b; _idx = 0; _state = 2; break;
        case 2: _cmd = b; _chk ^= b; _state = (_len > 1) ? 3 : 4; break;
        case 3:
          _buf[_idx++] = b; _chk ^= b;
          if (_idx >= _len - 1) _state = 4;
          break;
        case 4: _state = (b == _chk) ? 5 : 0; break;
        case 5:
          _state = 0;
          if (b == PROTO_END) {
            cmd = _cmd; len = _len - 1;
            memcpy(payload, _buf, len);
            return true;
          }
          break;
      }
    }
    return false;
  }

  void send(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t len = 0) {
    if (!connected()) return;
    uint8_t frame[PROTO_MAX_PAYLOAD + 5];
    uint8_t L = len + 1;
    uint8_t chk = L ^ cmd;
    frame[0] = PROTO_START; frame[1] = L; frame[2] = cmd;
    for (uint8_t i = 0; i < len; i++) { frame[3 + i] = payload[i]; chk ^= payload[i]; }
    frame[3 + len] = chk; frame[4 + len] = PROTO_END;
    _client.write(frame, len + 5);
  }
  void ack(uint8_t cmd)                 { send(RESP_ACK, &cmd, 1); }
  void nack(uint8_t cmd, uint8_t err)   { uint8_t p[2] = {cmd, err}; send(RESP_NACK, p, 2); }
  void done(uint8_t cmd)                { send(RESP_CMD_DONE, &cmd, 1); }
  void failed(uint8_t cmd, uint8_t err) { uint8_t p[2] = {cmd, err}; send(RESP_CMD_FAILED, p, 2); }

private:
  WiFiServer _server;
  WiFiClient _client;
  uint16_t _port;
  uint8_t _state = 0, _len = 0, _cmd = 0, _chk = 0, _idx = 0;
  uint8_t _buf[PROTO_MAX_PAYLOAD];
};

FramedPort carPort(CAR_TCP_PORT);
FramedPort armPort(ARM_TCP_PORT);
WiFiServer displayServer(DISPLAY_TCP_PORT);
WiFiClient displayClient;
WiFiUDP discoveryUdp;

bool estopLatched = false;

// ----------------------------------------------------------------------------
//  Ultrasonic sensors (interrupt based, one sensor fired at a time)
// ----------------------------------------------------------------------------
enum { S_FRONT = 0, S_BACK, S_LEFT, S_RIGHT, S_COUNT };
static const uint8_t TRIG_PINS[S_COUNT] = { SONAR_FRONT_TRIG, SONAR_BACK_TRIG, SONAR_LEFT_TRIG, SONAR_RIGHT_TRIG };
static const uint8_t ECHO_PINS[S_COUNT] = { SONAR_FRONT_ECHO, SONAR_BACK_ECHO, SONAR_LEFT_ECHO, SONAR_RIGHT_ECHO };

volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoWidthUs = 0;
volatile bool echoDone = false;
volatile uint8_t activeSonar = 0;

static inline bool IRAM_ATTR fastRead(uint8_t pin) {
  if (pin >= 32) return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
  return (REG_READ(GPIO_IN_REG) >> pin) & 1;
}

void IRAM_ATTR echoIsr() {
  uint8_t pin = ECHO_PINS[activeSonar];
  uint32_t now = micros();
  if (fastRead(pin)) {
    echoRiseUs = now;
  } else if (echoRiseUs != 0) {
    // GPIO36/39 can see ~80 ns glitches; a real falling edge stays low
    echoWidthUs = now - echoRiseUs;
    echoDone = true;
  }
}

class Sonar {
public:
  uint16_t mm[S_COUNT] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };

  void begin() {
    for (uint8_t i = 0; i < S_COUNT; i++) {
      pinMode(TRIG_PINS[i], OUTPUT);
      digitalWrite(TRIG_PINS[i], LOW);
      pinMode(ECHO_PINS[i], INPUT);
      for (uint8_t k = 0; k < 3; k++) _hist[i][k] = 0xFFFF;
    }
    fire(0);
  }

  void update() {
    uint32_t now = millis();
    if (echoDone) {
      store(activeSonar, (uint16_t)min<uint32_t>((uint32_t)(echoWidthUs * 0.1715f), 0xFFFE));
      next(now);
    } else if (now - _firedAt > 30) {          // no echo within 30 ms = nothing in range
      store(activeSonar, 0xFFFF);
      next(now);
    }
  }

private:
  uint16_t _hist[S_COUNT][3];
  uint8_t _histIdx[S_COUNT] = {0, 0, 0, 0};
  uint32_t _firedAt = 0;

  void next(uint32_t now) {
    (void)now;
    fire((activeSonar + 1) % S_COUNT);
  }

  bool _isrAttached = false;

  void fire(uint8_t idx) {
    if (_isrAttached) detachInterrupt(digitalPinToInterrupt(ECHO_PINS[activeSonar]));
    _isrAttached = true;
    activeSonar = idx;
    echoRiseUs = 0; echoDone = false;
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[idx]), echoIsr, CHANGE);
    digitalWrite(TRIG_PINS[idx], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PINS[idx], LOW);
    _firedAt = millis();
  }

  void store(uint8_t idx, uint16_t v) {
    if (v != 0xFFFF && (v < 20 || v > SONAR_MAX_MM)) v = 0xFFFF;
    _hist[idx][_histIdx[idx]] = v;
    _histIdx[idx] = (_histIdx[idx] + 1) % 3;
    // median of the last 3 readings removes single bad echoes
    uint16_t a = _hist[idx][0], b = _hist[idx][1], c = _hist[idx][2];
    uint16_t med = max(min(a, b), min(max(a, b), c));
    mm[idx] = med;
  }
} sonar;

// ----------------------------------------------------------------------------
//  Base motors
// ----------------------------------------------------------------------------
class Drive {
public:
  enum Mode { IDLE, STEP, VELOCITY };
  Mode mode = IDLE;
  int16_t pwmL = 0, pwmR = 0;
  uint16_t frontStopMm = DEFAULT_FRONT_STOP_MM;
  uint16_t backStopMm  = DEFAULT_BACK_STOP_MM;
  bool obstacleStop = false;
  uint8_t lastObstacleSensor = 0xFF;

  void begin() {
    pinMode(LEFT_IN_A, OUTPUT);  pinMode(LEFT_IN_B, OUTPUT);
    pinMode(RIGHT_IN_A, OUTPUT); pinMode(RIGHT_IN_B, OUTPUT);
    pinMode(LEFT_EN, OUTPUT);    pinMode(RIGHT_EN, OUTPUT);
    stop();
  }

  void stop() {
    setWheels(0, 0);
    mode = IDLE;
    _stepCmd = 0;
  }

  // dir: +1 forward, -1 backward
  uint8_t startStraight(uint8_t cmd, int dir, uint16_t distMm, uint8_t speedPct) {
    if (estopLatched) return ERR_ESTOP_TRIGGERED;
    if (blocked(dir)) return ERR_KEEPOUT;
    speedPct = constrain(speedPct, 10, 100);
    float mmPerS = CAL_MM_PER_S_AT_100 * speedPct / 100.0f;
    int16_t pwm = pctToPwm(speedPct);
    setWheels(dir * pwm, dir * pwm);
    beginStep(cmd, (uint32_t)(1000.0f * distMm / mmPerS));
    return ERR_NONE;
  }

  // dir: +1 = left (counter-clockwise), -1 = right
  uint8_t startTurn(uint8_t cmd, int dir, uint16_t deg, uint8_t speedPct) {
    if (estopLatched) return ERR_ESTOP_TRIGGERED;
    speedPct = constrain(speedPct, 10, 100);
    float degPerS = CAL_DEG_PER_S_AT_100 * speedPct / 100.0f;
    int16_t pwm = pctToPwm(speedPct);
    setWheels(-dir * pwm, dir * pwm);
    beginStep(cmd, (uint32_t)(1000.0f * deg / degPerS));
    return ERR_NONE;
  }

  uint8_t setVelocity(int16_t vMmS, int16_t wMradS) {
    if (estopLatched) return ERR_ESTOP_TRIGGERED;
    float w = wMradS / 1000.0f;
    float vl = vMmS - w * TRACK_WIDTH_MM / 2.0f;
    float vr = vMmS + w * TRACK_WIDTH_MM / 2.0f;
    int16_t pl = speedToPwm(vl), pr = speedToPwm(vr);
    int dir = (pl + pr > 0) ? 1 : ((pl + pr < 0) ? -1 : 0);
    if (dir != 0 && blocked(dir)) { stop(); return ERR_KEEPOUT; }
    setWheels(pl, pr);
    mode = (pl == 0 && pr == 0) ? IDLE : VELOCITY;
    _lastVelMs = millis();
    _stepCmd = 0;
    return ERR_NONE;
  }

  // Called every loop. Returns the cmd of a step that just finished (0 = none).
  uint8_t update(FramedPort& port) {
    uint32_t now = millis();

    // Watchdog for continuous velocity mode
    if (mode == VELOCITY && now - _lastVelMs > DRIVE_VEL_WATCHDOG_MS) {
      stop();
    }

    // Local obstacle protection (works even if Wi-Fi lags)
    int dir = movingDir();
    if (dir != 0 && blocked(dir)) {
      uint8_t sensor = (dir > 0) ? S_FRONT : S_BACK;
      uint8_t cmd = _stepCmd;
      stop();
      obstacleStop = true;
      lastObstacleSensor = sensor;
      uint8_t p[3]; p[0] = sensor; wr16(p + 1, sonar.mm[sensor]);
      port.send(RESP_OBSTACLE, p, 3);
      if (cmd) port.failed(cmd, ERR_KEEPOUT);
      Serial.printf("[DRIVE] obstacle stop, sensor %u at %u mm\n", sensor, sonar.mm[sensor]);
      return 0;
    }
    if (obstacleStop && !blocked(1) && !blocked(-1)) obstacleStop = false;

    if (mode == STEP && now - _stepStart >= _stepDurMs) {
      uint8_t cmd = _stepCmd;
      stop();
      return cmd;
    }
    return 0;
  }

  bool moving() const { return pwmL != 0 || pwmR != 0; }
  bool stepActive() const { return mode == STEP; }

private:
  uint32_t _stepStart = 0, _stepDurMs = 0, _lastVelMs = 0;
  uint8_t _stepCmd = 0;

  void beginStep(uint8_t cmd, uint32_t durMs) {
    mode = STEP; _stepCmd = cmd; _stepStart = millis(); _stepDurMs = durMs;
  }

  // +1 = mostly driving forward, -1 = mostly backward, 0 = stopped / spinning
  int movingDir() const {
    int sum = pwmL + pwmR;
    int diff = abs(pwmL - pwmR);
    if (sum == 0 || abs(sum) * 2 <= diff) return 0;
    return sum > 0 ? 1 : -1;
  }

  bool blocked(int dir) const {
    if (dir > 0 && frontStopMm > 0) return sonar.mm[S_FRONT] < frontStopMm;
    if (dir < 0 && backStopMm > 0)  return sonar.mm[S_BACK]  < backStopMm;
    return false;
  }

  static int16_t pctToPwm(uint8_t pct) {
    return MOTOR_MIN_PWM + (int16_t)((MOTOR_MAX_PWM - MOTOR_MIN_PWM) * pct / 100);
  }

  static int16_t speedToPwm(float mmS) {
    if (fabsf(mmS) < 5.0f) return 0;
    float frac = fabsf(mmS) / CAL_MM_PER_S_AT_100;
    if (frac > 1.0f) frac = 1.0f;
    int16_t pwm = MOTOR_MIN_PWM + (int16_t)((MOTOR_MAX_PWM - MOTOR_MIN_PWM) * frac);
    return mmS > 0 ? pwm : -pwm;
  }

  void setWheels(int16_t l, int16_t r) {
    pwmL = constrain(l, -255, 255);
    pwmR = constrain(r, -255, 255);
    int16_t L = INVERT_LEFT_MOTOR ? -pwmL : pwmL;
    int16_t R = INVERT_RIGHT_MOTOR ? -pwmR : pwmR;
    digitalWrite(LEFT_IN_A, L > 0);  digitalWrite(LEFT_IN_B, L < 0);
    digitalWrite(RIGHT_IN_A, R > 0); digitalWrite(RIGHT_IN_B, R < 0);
    analogWrite(LEFT_EN, abs(L));
    analogWrite(RIGHT_EN, abs(R));
  }
} drive;

// ----------------------------------------------------------------------------
//  PCA9685 (two boards)
// ----------------------------------------------------------------------------
class Pca9685 {
public:
  explicit Pca9685(uint8_t addr) : _addr(addr) {}
  bool ok = false;

  bool probe() {
    Wire.beginTransmission(_addr);
    return Wire.endTransmission() == 0;
  }

  bool begin() {
    ok = probe();
    if (!ok) return false;
    write8(0x00, 0x10);                 // MODE1: sleep
    write8(0xFE, 121);                  // prescale -> 50 Hz
    write8(0x00, 0x20);                 // wake, auto-increment
    delay(2);
    write8(0x00, 0xA0);                 // restart + auto-increment
    write8(0x01, 0x04);                 // MODE2: totem pole outputs
    return true;
  }

  void setPulseUs(uint8_t ch, uint16_t us) {
    uint16_t ticks = (uint32_t)us * 4096UL / 20000UL;
    Wire.beginTransmission(_addr);
    Wire.write(0x06 + 4 * ch);
    Wire.write(0); Wire.write(0);
    Wire.write(ticks & 0xFF); Wire.write(ticks >> 8);
    if (Wire.endTransmission() != 0) ok = false;
  }

private:
  uint8_t _addr;
  void write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
  }
};

Pca9685 pcaBig(PCA_ARM_BIG);
Pca9685 pcaSmall(PCA_ARM_SMALL);

static Pca9685& pcaFor(uint8_t addr) { return addr == PCA_ARM_BIG ? pcaBig : pcaSmall; }

// ----------------------------------------------------------------------------
//  Arm controller - staged moves, max 2 servos at a time
// ----------------------------------------------------------------------------
class Arm {
public:
  float cur[6];          // servo degrees right now
  float target[6];       // final servo degrees of the current move
  bool authorised = false;
  uint8_t authMed = 0;
  uint8_t authToken[8];
  uint32_t authUntil = 0;

  void begin() {
    for (int j = 0; j < 6; j++) { cur[j] = HOME_SERVO_DEG[j]; target[j] = cur[j]; }
    pcaBig.begin();
#if ONE_PCA_BOARD
    Serial.printf("[ARM] shared-address mode: PCA 0x40 %s (big servos ch 0-2, small servos ch 3-5)\n", pcaBig.ok ? "OK" : "MISSING");
#else
    pcaSmall.begin();
    Serial.printf("[ARM] PCA 0x40 %s, PCA 0x41 %s\n", pcaBig.ok ? "OK" : "MISSING", pcaSmall.ok ? "OK" : "MISSING");
#endif
    // Soft start: wake one servo at a time so the batteries don't brown out
    for (int j = 0; j < 6; j++) { writeServo(j, cur[j]); delay(250); }
  }

  bool moving() const { return _stageCount > 0; }
  bool pcaOk() const { return pcaBig.ok && (ONE_PCA_BOARD || pcaSmall.ok); }
  bool gripperClosed() const { return cur[5] > (GRIP_OPEN_DEG + GRIP_CLOSED_DEG) / 2.0f; }

  static float jointToServo(int j, float q) { return JOINT_OFFSET[j] + JOINT_DIR[j] * q + JOINT_TRIM[j]; }
  static float servoToJoint(int j, float s) { return (s - JOINT_OFFSET[j] - JOINT_TRIM[j]) / JOINT_DIR[j]; }

  // Start a move to new servo targets (only joints 0..4, gripper handled separately)
  uint8_t moveTo(uint8_t cmd, const float newTarget[6]) {
    if (estopLatched) return ERR_ESTOP_TRIGGERED;
    if (!pcaOk()) return ERR_PCA_OFFLINE;
    if (moving()) return ERR_BUSY;
    for (int j = 0; j < 6; j++) {
      if (newTarget[j] < SERVO_MIN_DEG[j] - 0.01f || newTarget[j] > SERVO_MAX_DEG[j] + 0.01f) return ERR_JOINT_OUT_OF_RANGE;
    }
    for (int j = 0; j < 6; j++) target[j] = newTarget[j];

    // Order the stages so the arm never swings out while extended:
    //   going OUT (shoulder angle decreasing): yaw -> shoulder+elbow -> wrist pitch+roll
    //   coming IN:                             wrist pitch+roll -> shoulder+elbow -> yaw
    static const int8_t G_YAW[2] = {0, -1}, G_SE[2] = {1, 2}, G_WR[2] = {3, 4}, G_GR[2] = {5, -1};
    const int8_t* order[4];
    bool goingOut = target[1] < cur[1] - 0.5f;
    if (goingOut) { order[0] = G_YAW; order[1] = G_SE; order[2] = G_WR; }
    else          { order[0] = G_WR;  order[1] = G_SE; order[2] = G_YAW; }
    order[3] = G_GR;

    _stageCount = 0; _stageIdx = 0;
    for (int g = 0; g < 4; g++) {
      Stage st; st.n = 0;
      for (int k = 0; k < 2; k++) {
        int8_t j = order[g][k];
        if (j < 0 || fabsf(target[j] - cur[j]) <= 0.3f) continue;
        if (st.n >= MAX_SIMULTANEOUS_SERVOS) { _stages[_stageCount++] = st; st.n = 0; }
        st.j[st.n++] = j;
      }
      if (st.n > 0) _stages[_stageCount++] = st;
    }
    _cmd = cmd;
    if (_stageCount == 0) return ERR_NONE;   // already there -> reported done in update()
    startStage();
    return ERR_NONE;
  }

  uint8_t moveGripper(uint8_t cmd, float deg) {
    if (moving()) return ERR_BUSY;
    float t[6];
    for (int j = 0; j < 6; j++) t[j] = cur[j];
    t[5] = deg;
    return moveTo(cmd, t);
  }

  // Hold every servo where it is. Returns the cmd that was interrupted (0 = none).
  uint8_t abort() {
    for (int j = 0; j < 6; j++) target[j] = cur[j];
    _stageCount = 0;
    uint8_t c = _cmd; _cmd = 0;
    return c;
  }

  // Returns cmd of a move that just finished (0 = none)
  uint8_t update() {
    uint32_t now = millis();
    if (authorised && (int32_t)(now - authUntil) > 0) authorised = false;

    if (_cmd && _stageCount == 0) { uint8_t c = _cmd; _cmd = 0; return c; }
    if (_stageCount == 0) return 0;
    if (now - _lastTick < 20) return 0;            // 50 Hz servo update
    _lastTick = now;

    Stage& st = _stages[_stageIdx];
    float t = (float)(now - _stageStart) / (float)_stageDur;
    if (t > 1.0f) t = 1.0f;
    float s = t * t * (3.0f - 2.0f * t);           // smooth start / stop
    for (int k = 0; k < st.n; k++) {
      int j = st.j[k];
      cur[j] = _from[k] + (target[j] - _from[k]) * s;
      writeServo(j, cur[j]);
    }
    if (t >= 1.0f) {
      _stageIdx++;
      if (_stageIdx >= _stageCount) { _stageCount = 0; uint8_t c = _cmd; _cmd = 0; return c; }
      startStage();
    }
    return 0;
  }

  // Re-detect PCA boards (e.g. servo battery switched on late)
  void checkPca() {
    bool wasBig = pcaBig.ok, wasSmall = pcaSmall.ok;
    if (!pcaBig.ok && pcaBig.probe())     { pcaBig.begin();   for (int j = 0; j < 6; j++) if (SERVO_PORTS[j].addr == PCA_ARM_BIG)   writeServo(j, cur[j]); }
    if (!ONE_PCA_BOARD && !pcaSmall.ok && pcaSmall.probe()) { pcaSmall.begin(); for (int j = 0; j < 6; j++) if (SERVO_PORTS[j].addr == PCA_ARM_SMALL) writeServo(j, cur[j]); }
    if (pcaBig.ok && !pcaBig.probe()) pcaBig.ok = false;
    if (!ONE_PCA_BOARD && pcaSmall.ok && !pcaSmall.probe()) pcaSmall.ok = false;
    if (wasBig != pcaBig.ok || wasSmall != pcaSmall.ok)
      Serial.printf("[ARM] PCA 0x40 %s, PCA 0x41 %s\n", pcaBig.ok ? "OK" : "MISSING", pcaSmall.ok ? "OK" : "MISSING");
  }

private:
  struct Stage { int8_t j[MAX_SIMULTANEOUS_SERVOS]; uint8_t n; };
  Stage _stages[8];
  uint8_t _stageCount = 0, _stageIdx = 0, _cmd = 0;
  float _from[MAX_SIMULTANEOUS_SERVOS];
  uint32_t _stageStart = 0, _stageDur = 1, _lastTick = 0;

  void startStage() {
    Stage& st = _stages[_stageIdx];
    float dur = 0.2f;
    for (int k = 0; k < st.n; k++) {
      int j = st.j[k];
      _from[k] = cur[j];
      float d = fabsf(target[j] - cur[j]) / SERVO_SPEED_DPS[j] * 1.5f;  // x1.5 for the smooth profile
      if (d > dur) dur = d;
    }
    _stageDur = (uint32_t)(dur * 1000.0f);
    _stageStart = millis();
  }

  void writeServo(int j, float deg) {
    deg = constrain(deg, SERVO_MIN_DEG[j], SERVO_MAX_DEG[j]);
    const ServoPort& p = SERVO_PORTS[j];
    uint16_t us = p.us_min + (uint16_t)((p.us_max - p.us_min) * deg / 180.0f);
    Pca9685& pca = pcaFor(p.addr);
    if (pca.ok) pca.setPulseUs(p.ch, us);
  }
} arm;

// ----------------------------------------------------------------------------
//  Box-in-gripper sensor (debounced)
// ----------------------------------------------------------------------------
bool gripBox = false;
uint8_t gripCount = 0;

void gripSensorBegin() {
#if GRIP_SENSOR_PIN >= 0
  pinMode(GRIP_SENSOR_PIN, INPUT_PULLUP);
#endif
}

void gripSensorUpdate() {
#if GRIP_SENSOR_PIN >= 0
  bool raw = digitalRead(GRIP_SENSOR_PIN) == (GRIP_SENSOR_ACTIVE_LOW ? LOW : HIGH);
  // must read the same for 5 loops in a row before it changes
  if (raw != gripBox) { if (++gripCount >= 5) { gripBox = raw; gripCount = 0; } }
  else gripCount = 0;
#endif
}

// ----------------------------------------------------------------------------
//  Command handlers
// ----------------------------------------------------------------------------
void setEstop(bool on) {
  estopLatched = on;
  if (on) {
    drive.stop();
    uint8_t c = arm.abort();
    if (c) armPort.failed(c, ERR_ESTOP_TRIGGERED);
    arm.authorised = false;
  }
  Serial.printf("[SAFETY] E-STOP %s\n", on ? "LATCHED" : "released");
}

void handleCarFrame(uint8_t cmd, const uint8_t* p, uint8_t len) {
  uint8_t err = ERR_NONE;
  switch (cmd) {
    case CMD_DRIVE_FORWARD:
    case CMD_DRIVE_BACKWARD:
      if (len < 3) { err = ERR_BAD_PAYLOAD; break; }
      err = drive.startStraight(cmd, cmd == CMD_DRIVE_FORWARD ? 1 : -1, rd16(p), p[2]);
      break;
    case CMD_TURN_LEFT:
    case CMD_TURN_RIGHT:
      if (len < 3) { err = ERR_BAD_PAYLOAD; break; }
      err = drive.startTurn(cmd, cmd == CMD_TURN_LEFT ? 1 : -1, rd16(p), p[2]);
      break;
    case CMD_DRIVE_VEL:
      if (len < 4) { err = ERR_BAD_PAYLOAD; break; }
      err = drive.setVelocity(rdi16(p), rdi16(p + 2));
      if (err == ERR_NONE) return;                 // no ACK spam at 10 Hz
      break;
    case CMD_STOP_MOTORS:
      drive.stop();
      break;
    case CMD_SET_OBSTACLE:
      if (len < 4) { err = ERR_BAD_PAYLOAD; break; }
      drive.frontStopMm = rd16(p);
      drive.backStopMm = rd16(p + 2);
      break;
    case CMD_SET_ESTOP:
      setEstop(len > 0 && p[0]);
      break;
    case CMD_PING:
      break;
    default:
      err = ERR_UNKNOWN_CMD;
  }
  if (err == ERR_NONE) carPort.ack(cmd); else carPort.nack(cmd, err);
}

void handleArmFrame(uint8_t cmd, const uint8_t* p, uint8_t len) {
  uint8_t err = ERR_NONE;
  switch (cmd) {
    case CMD_ARM_MOVE_JOINTS: {
      if (len < 12) { err = ERR_BAD_PAYLOAD; break; }
      float t[6];
      for (int j = 0; j < 5; j++) {
        float q = rd16(p + 2 + 2 * j) / 10.0f - 180.0f;
        t[j] = Arm::jointToServo(j, q);
      }
      t[5] = arm.cur[5];                            // gripper only moves with its own commands
      err = arm.moveTo(cmd, t);
      break;
    }
    case CMD_ARM_SET_JOINT: {
      if (len < 3 || p[0] > 4) { err = ERR_BAD_PAYLOAD; break; }
      float t[6];
      for (int j = 0; j < 6; j++) t[j] = arm.cur[j];
      t[p[0]] = Arm::jointToServo(p[0], rd16(p + 1) / 10.0f - 180.0f);
      err = arm.moveTo(cmd, t);
      break;
    }
    case CMD_ARM_HOME: {
      float t[6];
      for (int j = 0; j < 5; j++) t[j] = HOME_SERVO_DEG[j];
      t[5] = arm.cur[5];
      err = arm.moveTo(cmd, t);
      break;
    }
    case CMD_GRIPPER_OPEN:
      err = arm.moveGripper(cmd, GRIP_OPEN_DEG);
      break;
    case CMD_GRIPPER_CLOSE: {
      // Grab gate: only close if the server authorised THIS medicine + mission recently
      if (len < 9) { err = ERR_BAD_PAYLOAD; break; }
      bool ok = arm.authorised && p[0] == arm.authMed && memcmp(p + 1, arm.authToken, 8) == 0;
      if (!ok) { err = ERR_NOT_AUTHORISED; break; }
      err = arm.moveGripper(cmd, GRIP_CLOSED_DEG);
      break;
    }
    case CMD_ARM_AUTHORISE:
      if (len < 11) { err = ERR_BAD_PAYLOAD; break; }
      arm.authMed = p[0];
      memcpy(arm.authToken, p + 1, 8);
      arm.authUntil = millis() + rd16(p + 9);
      arm.authorised = !estopLatched;
      if (estopLatched) err = ERR_ESTOP_TRIGGERED;
      break;
    case CMD_ARM_DEAUTHORISE:
      arm.authorised = false;
      break;
    case CMD_ARM_ABORT: {
      uint8_t c = arm.abort();
      if (c) armPort.failed(c, ERR_NONE);
      break;
    }
    case CMD_SET_ESTOP:
      setEstop(len > 0 && p[0]);
      break;
    case CMD_PING:
      break;
    default:
      err = ERR_UNKNOWN_CMD;
  }
  if (err == ERR_NONE) armPort.ack(cmd); else armPort.nack(cmd, err);
}

// ----------------------------------------------------------------------------
//  Telemetry
// ----------------------------------------------------------------------------
uint16_t batteryMv() {
#if BATTERY_ADC_PIN >= 0
  return (uint16_t)(analogReadMilliVolts(BATTERY_ADC_PIN) * BATTERY_DIVIDER);
#else
  return 0;
#endif
}

void sendCarTelemetry() {
  uint8_t p[17];
  p[0] = 1;
  for (int i = 0; i < 4; i++) wr16(p + 1 + 2 * i, sonar.mm[i]);
  p[9] = (drive.moving() ? 1 : 0) | (estopLatched ? 2 : 0) | (drive.obstacleStop ? 4 : 0) | (drive.stepActive() ? 8 : 0);
  wr16(p + 10, (uint16_t)drive.pwmL);
  wr16(p + 12, (uint16_t)drive.pwmR);
  wr16(p + 14, batteryMv());
  p[16] = drive.lastObstacleSensor;
  carPort.send(RESP_TELEMETRY, p, 17);
}

void sendArmTelemetry() {
  uint8_t p[36];
  p[0] = 2;
  for (int j = 0; j < 6; j++) wr16(p + 1 + 2 * j, (uint16_t)(arm.cur[j] * 10.0f + 0.5f));
  for (int j = 0; j < 5; j++) wr16(p + 13 + 2 * j, (uint16_t)((Arm::servoToJoint(j, arm.cur[j]) + 180.0f) * 10.0f + 0.5f));
  for (int j = 0; j < 6; j++) wr16(p + 23 + 2 * j, (uint16_t)(arm.target[j] * 10.0f + 0.5f));
  p[35] = (arm.moving() ? 1 : 0) | (arm.authorised ? 2 : 0) | (estopLatched ? 4 : 0) |
          (pcaBig.ok ? 8 : 0) | ((ONE_PCA_BOARD ? pcaBig.ok : pcaSmall.ok) ? 16 : 0) | (arm.gripperClosed() ? 32 : 0) |
          (gripBox ? 64 : 0) | (GRIP_SENSOR_PIN >= 0 ? 128 : 0);
  armPort.send(RESP_TELEMETRY, p, 36);
}

// ----------------------------------------------------------------------------
//  Wi-Fi, discovery, display relay
// ----------------------------------------------------------------------------
bool wifiWasUp = false;
uint32_t lastWifiAttempt = 0;

void wifiBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.setSleep(false);                 // lower latency, fewer dropped commands
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttempt = millis();
}

void wifiUpdate() {
  bool up = WiFi.status() == WL_CONNECTED;
  if (up && !wifiWasUp) {
    String ip = WiFi.localIP().toString();
    Serial.printf("[WIFI] connected, IP %s\n", ip.c_str());
    static bool mdnsStarted = false;
    if (mdnsStarted) MDNS.end();
    mdnsStarted = MDNS.begin(DEVICE_HOSTNAME);
    discoveryUdp.stop();
    discoveryUdp.begin(DISCOVERY_UDP_PORT);
    char line[48];
    snprintf(line, sizeof(line), "INFO|ESP32 %s", ip.c_str());
    sendLineToUno(line);
  }
  if (!up && wifiWasUp) {
    Serial.println("[WIFI] lost connection");
    drive.stop();
    sendLineToUno("INFO|WiFi lost");
  }
  if (!up && millis() - lastWifiAttempt > 15000) {
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttempt = millis();
  }
  wifiWasUp = up;
}

void discoveryUpdate() {
  int n = discoveryUdp.parsePacket();
  if (n <= 0) return;
  char buf[64];
  int len = discoveryUdp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = 0;
  if (strncmp(buf, "PRSRV", 5) != 0) return;
  char reply[96];
  snprintf(reply, sizeof(reply), "PRDEV main %s %d %d %d",
           WiFi.localIP().toString().c_str(), CAR_TCP_PORT, ARM_TCP_PORT, DISPLAY_TCP_PORT);
  discoveryUdp.beginPacket(discoveryUdp.remoteIP(), discoveryUdp.remotePort());
  discoveryUdp.write((const uint8_t*)reply, strlen(reply));
  discoveryUdp.endPacket();
}

void displayRelayUpdate() {
  WiFiClient incoming = displayServer.available();
  if (incoming) {
    if (displayClient && displayClient.connected()) displayClient.stop();
    displayClient = incoming;
    Serial.println("[DISPLAY] server connected to TFT relay");
  }
  if (displayClient && displayClient.connected()) {
    while (displayClient.available() > 0) {
      char c = (char)displayClient.read();
      if (c != '\r') unoPush(c);
    }
  }
}

// ----------------------------------------------------------------------------
//  setup / loop
// ----------------------------------------------------------------------------
uint32_t lastTelemetry = 0, lastHeartbeat = 0, lastPcaCheck = 0, lastInfo = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Pharma Rover main ESP32 ===");

  Serial2.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
  sendLineToUno("INFO|Booting...");

  drive.begin();                        // motors OFF first
  sonar.begin();
  gripSensorBegin();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  arm.begin();
#if BATTERY_ADC_PIN >= 0
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
#endif

  wifiBegin();
  carPort.begin();
  armPort.begin();
  displayServer.begin();
  displayServer.setNoDelay(true);
}

void loop() {
  uint32_t now = millis();
  wifiUpdate();
  sonar.update();
  gripSensorUpdate();

  // ---- base ----
  if (carPort.acceptClient()) drive.stop();
  static bool carWasConnected = false;
  bool carConn = carPort.connected();
  if (carWasConnected && !carConn) { drive.stop(); Serial.println("[TCP 5000] server gone -> motors stopped"); }
  carWasConnected = carConn;

  uint8_t cmd, len, payload[PROTO_MAX_PAYLOAD];
  while (carPort.poll(cmd, payload, len)) handleCarFrame(cmd, payload, len);
  uint8_t finished = drive.update(carPort);
  if (finished) carPort.done(finished);

  // ---- arm ----
  armPort.acceptClient();
  static bool armWasConnected = false;
  bool armConn = armPort.connected();
  if (armWasConnected && !armConn) { arm.abort(); arm.authorised = false; Serial.println("[TCP 5001] server gone -> arm holding"); }
  armWasConnected = armConn;

  while (armPort.poll(cmd, payload, len)) handleArmFrame(cmd, payload, len);
  finished = arm.update();
  if (finished) armPort.done(finished);

  // ---- network extras ----
  if (WiFi.status() == WL_CONNECTED) {
    discoveryUpdate();
    displayRelayUpdate();
  }
  unoRelayUpdate();

  // ---- periodic ----
  if (now - lastTelemetry >= TELEMETRY_PERIOD_MS) {
    lastTelemetry = now;
    sendCarTelemetry();
    sendArmTelemetry();
  }
  if (now - lastHeartbeat >= HEARTBEAT_PERIOD_MS) {
    lastHeartbeat = now;
    uint8_t p[4]; uint32_t up = now / 1000;
    p[0] = up >> 24; p[1] = up >> 16; p[2] = up >> 8; p[3] = up;
    carPort.send(RESP_HEARTBEAT, p, 4);
    armPort.send(RESP_HEARTBEAT, p, 4);
  }
  if (now - lastPcaCheck >= 1000 && !arm.moving()) {
    lastPcaCheck = now;
    arm.checkPca();
  }
  // Until the server's display client connects, keep showing our IP on the TFT
  if (!(displayClient && displayClient.connected()) && now - lastInfo >= 5000) {
    lastInfo = now;
    char line[48];
    if (WiFi.status() == WL_CONNECTED) snprintf(line, sizeof(line), "INFO|ESP32 %s", WiFi.localIP().toString().c_str());
    else snprintf(line, sizeof(line), "INFO|WiFi: %s", WIFI_SSID);
    sendLineToUno(line);
  }
}