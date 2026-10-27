#include <Wire.h>
#include <Servo.h>

// ---------------- Pins ----------------
const uint8_t PIN_ENC_FL_A = 2;    // INT0
const uint8_t PIN_ENC_FL_B = 4;
const uint8_t PIN_ENC_FR_A = 3;    // INT1
const uint8_t PIN_ENC_FR_B = 7;
const uint8_t PIN_ENC_RL_A = 8;    // PB0, pin-change interrupt (see ISR)
const uint8_t PIN_ENC_RL_B = A2;   // PC2
const uint8_t PIN_ENC_RR_A = 12;   // PB4, pin-change interrupt (see ISR)
const uint8_t PIN_ENC_RR_B = A3;   // PC3

const uint8_t PIN_EN_LEFT  = 5;    // PWM
const uint8_t PIN_EN_RIGHT = 6;    // PWM
const uint8_t PIN_IN1 = 9;         // left dir
const uint8_t PIN_IN2 = 10;        // left dir
const uint8_t PIN_IN3 = A0;        // right dir
const uint8_t PIN_IN4 = A1;        // right dir

const uint8_t PIN_SERVO = 11;

// ---------------- Settings ----------------
const long    SERIAL_BAUD   = 115200;
const uint8_t MPU_ADDR      = 0x68;     // AD0 low (0x69 if AD0 high)
const int     GATE_OPEN_DEG   = 90;
const int     GATE_CLOSED_DEG = 0;
// Motors stop if no M command arrives for this long. 0 disables.
const unsigned long CMD_TIMEOUT_MS = 500;
const float   ALPHA = 0.98;             // complementary filter weight (gyro)

// 7A/160W dual H-bridge (Handsontec datasheet): inputs are ACTIVE HIGH.
//   IN1=1, IN2=0 + PWM on ENA = forward;  IN1=0, IN2=0 = brake;
//   IN1=1, IN2=1 = floating.  So both flags stay false.
// Only set true if a test with the wheels lifted shows motors running when
// stopped, or speed commands working backwards.
const bool DIR_ACTIVE_LOW = false;
const bool PWM_ACTIVE_LOW = false;

// Flip if a side spins the wrong way when commanded forward.
const bool INVERT_LEFT  = true;
const bool INVERT_RIGHT = false;
// Flip if an encoder counts backwards when its wheel goes forward.
// Left and right motors are mirrored, so one side usually needs flipping.
const bool ENC_FL_REVERSE = false;
const bool ENC_FR_REVERSE = false;
const bool ENC_RL_REVERSE = false;
const bool ENC_RR_REVERSE = false;

// ---------------- State ----------------
Servo gate;

volatile long encFL = 0, encFR = 0, encRL = 0, encRR = 0;
volatile uint8_t lastPinb = 0;

float pitch = 0, roll = 0, yaw = 0;
float gxBias = 0, gyBias = 0, gzBias = 0;
unsigned long lastImuUs = 0;
unsigned long lastTelemetry = 0;
unsigned long telemetryPeriod = 50;     // ms
unsigned long lastCmdMs = 0;

// Closed-loop speed control (command V). Speeds are in encoder ticks per second.
const unsigned long PID_PERIOD_MS = 20;
bool  pidOn = false;
float tgtL = 0, tgtR = 0;          // target speed, left and right pair
float pidKp = 0.03;                // PWM per (tick/s) of error
float pidKi = 0.15;                // PWM per (tick/s) of error per second
float maxTps = 1600.0;             // ticks/s at PWM 255, used as feedforward (command W)

// Motor protection
const int  MAX_PWM   = 200;               // never drive harder than this (0-255)
const int  RAMP_STEP = 12;                // largest PWM change per 20 ms
const int  STALL_PWM = 90;                // above this PWM the wheels must be turning
const unsigned long STALL_MS = 600;       // cut a side after this long without movement
int  wantL = 0, wantR = 0;                // requested PWM, after the limit
int  curL  = 0, curR  = 0;                // PWM actually applied (ramped)
bool stallL = false, stallR = false;      // latched until the next command

float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

char cmdBuf[40];
uint8_t cmdLen = 0;

// ---------------- Encoders ----------------
// All four count on every edge of channel A; channel B gives the direction.
void isrEncFL() {
  bool a = digitalRead(PIN_ENC_FL_A);
  bool b = digitalRead(PIN_ENC_FL_B);
  if ((a == b) ^ ENC_FL_REVERSE) encFL++; else encFL--;
}

void isrEncFR() {
  bool a = digitalRead(PIN_ENC_FR_A);
  bool b = digitalRead(PIN_ENC_FR_B);
  if ((a == b) ^ ENC_FR_REVERSE) encFR++; else encFR--;
}

// D8 (PB0) and D12 (PB4) share one pin-change interrupt vector (PORTB).
// If you move these pins, change the masks here and in setup().
ISR(PCINT0_vect) {
  uint8_t pinb = PINB;
  uint8_t changed = pinb ^ lastPinb;
  lastPinb = pinb;
  uint8_t pinc = PINC;

  if (changed & _BV(PB0)) {                 // RL channel A (D8)
    bool a = pinb & _BV(PB0);
    bool b = pinc & _BV(PC2);               // RL channel B (A2)
    if ((a == b) ^ ENC_RL_REVERSE) encRL++; else encRL--;
  }
  if (changed & _BV(PB4)) {                 // RR channel A (D12)
    bool a = pinb & _BV(PB4);
    bool b = pinc & _BV(PC3);               // RR channel B (A3)
    if ((a == b) ^ ENC_RR_REVERSE) encRR++; else encRR--;
  }
}

long readEnc(volatile long &v) {
  noInterrupts();
  long c = v;
  interrupts();
  return c;
}

// ---------------- Motors ----------------
void setSide(uint8_t en, uint8_t inA, uint8_t inB, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  // "on" XOR active-low gives the pin level: on + active-low -> LOW
  digitalWrite(inA, (speed > 0) ^ DIR_ACTIVE_LOW);
  digitalWrite(inB, (speed < 0) ^ DIR_ACTIVE_LOW);
  int duty = abs(speed);
  if (PWM_ACTIVE_LOW) duty = 255 - duty;   // 0 speed -> pin held HIGH (off)
  analogWrite(en, duty);
}

void applyOutputs() {
  setSide(PIN_EN_LEFT,  PIN_IN1, PIN_IN2, curL, INVERT_LEFT);
  setSide(PIN_EN_RIGHT, PIN_IN3, PIN_IN4, curR, INVERT_RIGHT);
}

// Sets the requested power. The ramp in rampMotors() moves the real output toward it.
// A request of zero on both sides stops at once.
void driveMotors(int left, int right) {
  wantL = constrain(left,  -MAX_PWM, MAX_PWM);
  wantR = constrain(right, -MAX_PWM, MAX_PWM);
  if (wantL == 0 && wantR == 0) {
    curL = curR = 0;
    applyOutputs();
  }
}

// Open-loop command: also turns the speed controller off and clears a stall.
void setMotors(int left, int right) {
  static int lastL = 0, lastR = 0;
  pidOn = false;
  if (left != lastL || right != lastR) {      // a repeat of the same command keeps a stall latched
    stallL = stallR = false;
    lastL = left; lastR = right;
  }
  driveMotors(left, right);
  lastCmdMs = millis();
}

// Ramps the power and cuts a side whose wheels are not turning.
void rampMotors() {
  static unsigned long lastMs = 0, lastChk = 0, sinceL = 0, sinceR = 0;
  static long cFL = 0, cFR = 0, cRL = 0, cRR = 0;
  unsigned long now = millis();

  if (now - lastMs >= 20) {
    lastMs = now;
    int tl = stallL ? 0 : wantL;
    int tr = stallR ? 0 : wantR;
    curL += constrain(tl - curL, -RAMP_STEP, RAMP_STEP);
    curR += constrain(tr - curR, -RAMP_STEP, RAMP_STEP);
    applyOutputs();
  }

  if (now - lastChk >= 100) {
    lastChk = now;
    long fl = readEnc(encFL), fr = readEnc(encFR);
    long rl = readEnc(encRL), rr = readEnc(encRR);
    long mvL = labs(fl - cFL) + labs(rl - cRL);
    long mvR = labs(fr - cFR) + labs(rr - cRR);
    cFL = fl; cFR = fr; cRL = rl; cRR = rr;

    if (abs(curL) >= STALL_PWM && mvL < 3) {
      if (sinceL == 0) sinceL = now;
      else if (now - sinceL > STALL_MS && !stallL) {
        stallL = true;
        Serial.println(F("ERR stall left"));
      }
    } else sinceL = 0;

    if (abs(curR) >= STALL_PWM && mvR < 3) {
      if (sinceR == 0) sinceR = now;
      else if (now - sinceR > STALL_MS && !stallR) {
        stallR = true;
        Serial.println(F("ERR stall right"));
      }
    } else sinceR = 0;
  }
}

// ---------------- MPU6050 ----------------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuReadRaw(int16_t *r) {   // r[0..2]=accel, r[3]=temp, r[4..6]=gyro
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) {
    r[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  }
  return true;
}

bool mpuBegin() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  mpuWrite(0x6B, 0x00);   // wake up
  mpuWrite(0x1A, 0x03);   // DLPF ~44 Hz
  mpuWrite(0x1B, 0x00);   // gyro  +-250 deg/s  (131 LSB/deg/s)
  mpuWrite(0x1C, 0x00);   // accel +-2 g        (16384 LSB/g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);       // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1);
  if (!Wire.available()) return false;
  uint8_t id = Wire.read();
  return (id == 0x68 || id == 0x72 || id == 0x70);  // clones vary
}

void calibrateGyro() {
  const int N = 500;
  long sx = 0, sy = 0, sz = 0;
  int16_t r[7];
  int got = 0;
  for (int i = 0; i < N; i++) {
    if (mpuReadRaw(r)) { sx += r[4]; sy += r[5]; sz += r[6]; got++; }
    delay(2);
  }
  if (got > 0) {
    gxBias = (float)sx / got / 131.0;
    gyBias = (float)sy / got / 131.0;
    gzBias = (float)sz / got / 131.0;
  }
}

void updateImu() {
  int16_t r[7];
  if (!mpuReadRaw(r)) return;

  unsigned long now = micros();
  float dt = (now - lastImuUs) * 1e-6;
  lastImuUs = now;
  if (dt <= 0 || dt > 0.5) return;

  ax = r[0] / 16384.0;
  ay = r[1] / 16384.0;
  az = r[2] / 16384.0;
  gx = r[4] / 131.0 - gxBias;
  gy = r[5] / 131.0 - gyBias;
  gz = r[6] / 131.0 - gzBias;

  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  float accRoll  = atan2(ay, az) * RAD_TO_DEG;

  pitch = ALPHA * (pitch + gy * dt) + (1.0 - ALPHA) * accPitch;
  roll  = ALPHA * (roll  + gx * dt) + (1.0 - ALPHA) * accRoll;
  yaw  += gz * dt;                       // gyro only: will drift slowly
}

// ---------------- Serial ----------------
void sendTelemetry() {
  Serial.print(F("D,"));
  Serial.print(millis());         Serial.print(',');
  Serial.print(ax, 3);            Serial.print(',');
  Serial.print(ay, 3);            Serial.print(',');
  Serial.print(az, 3);            Serial.print(',');
  Serial.print(gx, 2);            Serial.print(',');
  Serial.print(gy, 2);            Serial.print(',');
  Serial.print(gz, 2);            Serial.print(',');
  Serial.print(pitch, 2);         Serial.print(',');
  Serial.print(roll, 2);          Serial.print(',');
  Serial.print(yaw, 2);           Serial.print(',');
  Serial.print(readEnc(encFL));   Serial.print(',');
  Serial.print(readEnc(encFR));   Serial.print(',');
  Serial.print(readEnc(encRL));   Serial.print(',');
  Serial.println(readEnc(encRR));
}

void updatePid() {
  static unsigned long lastMs = 0;
  static long pFL = 0, pFR = 0, pRL = 0, pRR = 0;
  static float iL = 0, iR = 0;

  unsigned long now = millis();
  if (now - lastMs < PID_PERIOD_MS) return;
  float dt = (now - lastMs) * 1e-3;
  lastMs = now;

  long fl = readEnc(encFL), fr = readEnc(encFR);
  long rl = readEnc(encRL), rr = readEnc(encRR);
  float spdL = ((fl - pFL) + (rl - pRL)) * 0.5 / dt;
  float spdR = ((fr - pFR) + (rr - pRR)) * 0.5 / dt;
  pFL = fl; pFR = fr; pRL = rl; pRR = rr;

  if (!pidOn) { iL = iR = 0; return; }

  float outL = 0, outR = 0;
  if (tgtL != 0) {
    float e = tgtL - spdL;
    iL = constrain(iL + e * dt * pidKi, -80, 80);
    outL = tgtL * 255.0 / maxTps + pidKp * e + iL;
  } else iL = 0;
  if (tgtR != 0) {
    float e = tgtR - spdR;
    iR = constrain(iR + e * dt * pidKi, -80, 80);
    outR = tgtR * 255.0 / maxTps + pidKp * e + iR;
  } else iR = 0;

  driveMotors((int)constrain(outL, -255, 255), (int)constrain(outR, -255, 255));
}

void handleCommand(char *line) {
  char c = toupper(line[0]);
  char *args = line + 1;
  switch (c) {
    case 'M': {
      int l = 0, r = 0;
      if (sscanf(args, "%d %d", &l, &r) == 2) {
        setMotors(l, r);
        Serial.println(F("OK M"));
      } else Serial.println(F("ERR usage: M <left> <right>"));
      break;
    }
    case 'X':
      setMotors(0, 0);
      Serial.println(F("OK X"));
      break;
    case 'V': {                       // closed loop: V <left ticks/s> <right ticks/s>
      long l = 0, r = 0;
      if (sscanf(args, "%ld %ld", &l, &r) == 2) {
        if (l != (long)tgtL || r != (long)tgtR) stallL = stallR = false;
        tgtL = l; tgtR = r;
        pidOn = true;
        lastCmdMs = millis();
        Serial.println(F("OK V"));
      } else Serial.println(F("ERR usage: V <left> <right>"));
      break;
    }
    case 'P': {                       // gains in thousandths: P <kp*1000> <ki*1000>
      long a = 0, b = 0;
      if (sscanf(args, "%ld %ld", &a, &b) == 2) {
        pidKp = a / 1000.0; pidKi = b / 1000.0;
        Serial.println(F("OK P"));
      } else Serial.println(F("ERR usage: P <kp*1000> <ki*1000>"));
      break;
    }
    case 'W': {                       // W <max ticks/s at PWM 255>
      long w = atol(args);
      if (w > 0) { maxTps = w; Serial.println(F("OK W")); }
      else Serial.println(F("ERR usage: W <ticks/s>"));
      break;
    }
    case 'O':
      gate.write(GATE_OPEN_DEG);
      Serial.println(F("OK gate open"));
      break;
    case 'C':
      gate.write(GATE_CLOSED_DEG);
      Serial.println(F("OK gate closed"));
      break;
    case 'A': {
      int a = atoi(args);
      gate.write(constrain(a, 0, 180));
      Serial.println(F("OK A"));
      break;
    }
    case 'Z':
      noInterrupts(); encFL = encFR = encRL = encRR = 0; interrupts();
      yaw = 0;
      Serial.println(F("OK Z"));
      break;
    case 'T':
      telemetryPeriod = (unsigned long)max(0L, atol(args));
      Serial.println(F("OK T"));
      break;
    default:
      Serial.println(F("ERR unknown command"));
  }
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = 0;
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Preset the "off" level before switching to OUTPUT so the driver never
  // sees a glitch at boot.
  digitalWrite(PIN_EN_LEFT,  PWM_ACTIVE_LOW); digitalWrite(PIN_EN_RIGHT, PWM_ACTIVE_LOW);
  digitalWrite(PIN_IN1, DIR_ACTIVE_LOW); digitalWrite(PIN_IN2, DIR_ACTIVE_LOW);
  digitalWrite(PIN_IN3, DIR_ACTIVE_LOW); digitalWrite(PIN_IN4, DIR_ACTIVE_LOW);

  pinMode(PIN_EN_LEFT, OUTPUT);  pinMode(PIN_EN_RIGHT, OUTPUT);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  setMotors(0, 0);

  pinMode(PIN_ENC_FL_A, INPUT_PULLUP); pinMode(PIN_ENC_FL_B, INPUT_PULLUP);
  pinMode(PIN_ENC_FR_A, INPUT_PULLUP); pinMode(PIN_ENC_FR_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RL_A, INPUT_PULLUP); pinMode(PIN_ENC_RL_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RR_A, INPUT_PULLUP); pinMode(PIN_ENC_RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_FL_A), isrEncFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_FR_A), isrEncFR, CHANGE);

  // Pin-change interrupts for RL (D8 = PCINT0) and RR (D12 = PCINT4)
  lastPinb = PINB;
  PCMSK0 |= _BV(PCINT0) | _BV(PCINT4);
  PCIFR  |= _BV(PCIF0);
  PCICR  |= _BV(PCIE0);

  gate.attach(PIN_SERVO);
  gate.write(GATE_CLOSED_DEG);

  if (mpuBegin()) {
    Serial.println(F("MPU6050 OK - keep robot still, calibrating gyro..."));
    calibrateGyro();
    Serial.println(F("Calibration done"));
  } else {
    Serial.println(F("ERR MPU6050 not found (check SDA=A4, SCL=A5, power, address)"));
  }
  lastImuUs = micros();
  lastCmdMs = millis();
}

void loop() {
  readSerial();
  updateImu();
  updatePid();
  rampMotors();

  if (CMD_TIMEOUT_MS > 0 && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    setMotors(0, 0);
  }

  if (telemetryPeriod > 0 && millis() - lastTelemetry >= telemetryPeriod) {
    lastTelemetry = millis();
    sendTelemetry();
  }
}
