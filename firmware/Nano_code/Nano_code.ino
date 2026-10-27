/*
 * Malkia Bots - Arduino Nano motor firmware (closed loop)
 * Robotics Dojo 2026
 *
 * Talks to malkia_base/base_node.py on the Raspberry Pi over USB serial at 115200 baud.
 *   Pi -> Nano : M <left> <right>   target speed per side, -255..255 (fraction of maxTps)
 *                X                  stop
 *                O / C              open / close the gate servo
 *                H <gain>           gyro heading hold (0 = off)
 *   Nano -> Pi : D,ms,ax,ay,az,gx,gy,gz,pitch,roll,yaw,enc_fl,enc_fr,enc_rl,enc_rr   every 50 ms
 *
 * Every 20 ms a PI loop holds each side at its target speed, measured by the wheel encoders.
 * Full protocol, wiring and constants: firmware/README.md
 */
#include <Wire.h>
#include <Servo.h>

#include <EEPROM.h>
#include <MPU6050_light.h>        // angles in degrees (roll = X, pitch = Y, yaw = Z)

// ---------------- Pins ----------------
const uint8_t PIN_ENC_FL_A = 2;    // INT0
const uint8_t PIN_ENC_FL_B = 4;
const uint8_t PIN_ENC_FR_A = 3;    // INT1
const uint8_t PIN_ENC_FR_B = 7;
const uint8_t PIN_ENC_RL_A = 8;    // PB0, pin-change interrupt (see ISR)
const uint8_t PIN_ENC_RL_B = A2;   // PC2
const uint8_t PIN_ENC_RR_A = 12;   // PB4, pin-change interrupt (see ISR)
const uint8_t PIN_ENC_RR_B = A3;   // PC3

const uint8_t PIN_EN_LEFT  = 5;    // PWM, BOTH left motors: wire the front driver's left ENA here too
const uint8_t PIN_EN_RIGHT = 6;    // PWM
const uint8_t PIN_IN1 = 9;         // left dir
const uint8_t PIN_IN2 = 10;        // left dir
const uint8_t PIN_IN3 = A0;        // right dir
const uint8_t PIN_IN4 = A1;        // right dir

const uint8_t PIN_SERVO = 11;       // gate servo (D11; the front-left motor now shares D5)

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
const bool ENC_FL_REVERSE = true;
const bool ENC_FR_REVERSE = false;
const bool ENC_RL_REVERSE = true;
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
const float I_LIMIT = 100;            // largest correction the integral term may add (PWM units)
float pidKp = 0.01;                // PWM per (tick/s) of error
float pidKi = 0.05;                // PWM per (tick/s) of error per second
float maxTps = 4500.0;             // ticks/s at PWM 255, used as feedforward (command W)
int   minPwm = 0;                  // smallest power sent while a speed is requested (command N)
float minTps = 1300.0;             // smallest speed target, so the PID works above the power floor (command U)
float accelTps = 4000.0;            // ticks/s per second the target may change (soft start, command K, 0 = jump)
float hdgGain = 15;                // heading hold, ticks/s per deg/s of unwanted turn. 0 = off (command H)

// Motor protection
const int  MAX_PWM   = 200;               // never drive harder than this (0-255)
const int  RAMP_STEP = 12;                // largest PWM change per 20 ms
const int  STALL_PWM = 90;                // above this PWM the wheels must be turning
const unsigned long STALL_MS = 600;       // cut a side after this long without movement
int  wantL = 0, wantR = 0;                // requested PWM, after the limit
int  curL  = 0, curR  = 0;                // PWM actually applied (ramped)
bool stallL = false, stallR = false;      // latched until the next command
int  wantFL = 0, curFL = 0;               // front-left power 0..255, direction follows the left side
const bool splitFL = false;               // FL shares D5 with RL, so the left side is always one loop

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
void driveMotorsFL(int left, int right, int fl) {
  wantL = constrain(left,  -MAX_PWM, MAX_PWM);
  wantR = constrain(right, -MAX_PWM, MAX_PWM);
  wantFL = constrain(fl, 0, MAX_PWM);
  if (wantL == 0 && wantR == 0) {
    curL = curR = curFL = 0;
    applyOutputs();
  }
}

// FL follows the left side's power.
void driveMotors(int left, int right) {
  driveMotorsFL(left, right, abs(left));
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
    int tf = stallL ? 0 : (splitFL ? wantFL : abs(wantL));
    curFL += constrain(tf - curFL, -RAMP_STEP, RAMP_STEP);
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

// ---------------- MPU6050 (MPU6050_light library) ----------------
MPU6050 mpu(Wire);
float yawZero = 0;

// The library does not check that the chip answered, so check it here first.
bool imuPresent() {
  Wire.beginTransmission(MPU_ADDR);
  return Wire.endTransmission() == 0;
}

bool mpuBegin() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  return mpu.begin() == 0;
}

void calibrateGyro() {
  mpu.calcOffsets(true, false);          // gyro only: keeps the accelerometer tilt reference
}

void zeroYaw() { yawZero = mpu.getAngleZ(); }

// The level offsets survive restarts: stored in the Nano's EEPROM.
const int EE_MAGIC_ADDR = 0;
const byte EE_MAGIC = 0xA5;

void saveAccOffsets() {
  float o[3] = { mpu.getAccXoffset(), mpu.getAccYoffset(), mpu.getAccZoffset() };
  EEPROM.update(EE_MAGIC_ADDR, EE_MAGIC);
  EEPROM.put(EE_MAGIC_ADDR + 1, o);
}

void loadAccOffsets() {
  if (EEPROM.read(EE_MAGIC_ADDR) != EE_MAGIC) return;
  float o[3];
  EEPROM.get(EE_MAGIC_ADDR + 1, o);
  mpu.setAccOffsets(o[0], o[1], o[2]);
}

void forgetAccOffsets() {
  EEPROM.update(EE_MAGIC_ADDR, 0);
  mpu.setAccOffsets(0, 0, 0);
}

void updateImu() {
  if (!imuPresent()) {                   // no sensor: report exact zeros so the host can tell
    ax = ay = az = gx = gy = gz = 0;
    return;
  }
  mpu.update();
  ax = mpu.getAccX();   ay = mpu.getAccY();   az = mpu.getAccZ();
  gx = mpu.getGyroX();  gy = mpu.getGyroY();  gz = mpu.getGyroZ();
  roll  = mpu.getAngleX();
  pitch = mpu.getAngleY();
  yaw   = mpu.getAngleZ() - yawZero;
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
  static float iL = 0, iR = 0, fL = 0, fR = 0;     // integrals and smoothed speeds
  static float iFL = 0, fFLs = 0, fRLs = 0;        // separate loop for the front-left wheel
  static float effL = 0, effR = 0;                 // the target the PID follows (rises smoothly)
  static float yawRef = 0;
  static bool wasStraight = false;

  unsigned long now = millis();
  if (now - lastMs < PID_PERIOD_MS) return;
  float dt = (now - lastMs) * 1e-3;
  lastMs = now;

  long fl = readEnc(encFL), fr = readEnc(encFR);
  long rl = readEnc(encRL), rr = readEnc(encRR);
  float sFL = (fl - pFL) / dt, sRL = (rl - pRL) / dt;
  float sFR = (fr - pFR) / dt, sRR = (rr - pRR) / dt;
  pFL = fl; pFR = fr; pRL = rl; pRR = rr;

  if (!pidOn) { iL = iR = 0; fL = fR = 0; iFL = fFLs = fRLs = 0; effL = effR = 0; wasStraight = false; return; }

  // 1. never ask for less than minTps (scale both sides together to keep the curve)
  float tL = tgtL, tR = tgtR;
  float big = max(fabs(tL), fabs(tR));
  if (big > 0 && big < minTps) {
    float k = minTps / big;
    tL *= k; tR *= k;
  }

  // 1b. soft start: the target the PID follows moves toward the requested one at accelTps
  if (accelTps > 0) {
    float st = accelTps * dt;
    effL += constrain(tL - effL, -st, st);
    effR += constrain(tR - effR, -st, st);
    tL = effL; tR = effR;
  } else { effL = tL; effR = tR; }

  // 2. heading hold: when driving straight, use the gyro to cancel any unwanted turn
  bool straight = tL != 0 && tR != 0 && ((tL > 0) == (tR > 0)) &&
                  fabs(tL - tR) < 0.03 * max(fabs(tL), fabs(tR));   // only a truly straight command
  if (straight && hdgGain != 0) {
    if (!wasStraight) yawRef = yaw;
    float err = yaw - yawRef;                      // degrees turned since the straight run began
    float trim = constrain(hdgGain * gz + 0.5 * hdgGain * err, -300, 300);
    tL += trim;                                    // turned left (positive): speed up left, slow right
    tR -= trim;
  }
  wasStraight = straight && hdgGain != 0;

  // 3. speed of each side: half the average, half the slower wheel
  float slowL = tL >= 0 ? min(sFL, sRL) : max(sFL, sRL);
  float slowR = tR >= 0 ? min(sFR, sRR) : max(sFR, sRR);
  float rawL = 0.5 * (sFL + sRL) * 0.5 + 0.5 * slowL;
  float rawR = 0.5 * (sFR + sRR) * 0.5 + 0.5 * slowR;
  fL += 0.3 * (rawL - fL);
  fR += 0.3 * (rawR - fR);
  float dirL = tL >= 0 ? 1.0 : -1.0;               // wheel speeds in the direction of travel
  fFLs += 0.3 * (dirL * sFL - fFLs);
  fRLs += 0.3 * (dirL * sRL - fRLs);

  float outL = 0, outR = 0;
  int outFL = 0;
  if (tL != 0) {
    float ff = tL * 255.0 / maxTps;
    if (splitFL) {
      float tmag = fabs(tL), mag = fabs(ff);
      float eR = tmag - fRLs;                                     // rear-left wheel, power on D5
      float uR = mag + pidKp * eR + iL;
      float oR = constrain(uR, 0, MAX_PWM);
      if (uR == oR) iL = constrain(iL + eR * dt * pidKi, -I_LIMIT, I_LIMIT);
      float eF = tmag - fFLs;                                     // front-left wheel, power on D11
      float uF = mag + pidKp * eF + iFL;
      float oF = constrain(uF, 0, MAX_PWM);
      if (uF == oF) iFL = constrain(iFL + eF * dt * pidKi, -I_LIMIT, I_LIMIT);
      outL = tL > 0 ? oR : -oR;
      outFL = (int)oF;
    } else {
      float e = tL - fL;
      float u = ff + pidKp * e + iL;
      float lo = tL > 0 ? 0 : -MAX_PWM, hi = tL > 0 ? MAX_PWM : 0;  // never reverse to brake
      outL = constrain(u, lo, hi);
      if (u == outL) iL = constrain(iL + e * dt * pidKi, -I_LIMIT, I_LIMIT);
      outFL = (int)fabs(outL);
    }
  } else { iL = 0; iFL = 0; }
  if (tR != 0) {
    float e = tR - fR;
    float ff = tR * 255.0 / maxTps;
    float u = ff + pidKp * e + iR;
    float lo = tR > 0 ? 0 : -MAX_PWM, hi = tR > 0 ? MAX_PWM : 0;
    outR = constrain(u, lo, hi);
    if (u == outR) iR = constrain(iR + e * dt * pidKi, -I_LIMIT, I_LIMIT);
  } else iR = 0;

  if (tL != 0 && fabs(outL) < minPwm) outL = tL > 0 ? minPwm : -minPwm;
  if (tR != 0 && fabs(outR) < minPwm) outR = tR > 0 ? minPwm : -minPwm;

  if (tL != 0 && outFL < minPwm) outFL = minPwm;

  driveMotorsFL((int)outL, (int)outR, splitFL ? outFL : abs((int)outL));
}

void handleCommand(char *line) {
  char c = toupper(line[0]);
  char *args = line + 1;
  switch (c) {
    case 'M': {
      int l = 0, r = 0;
      if (sscanf(args, "%d %d", &l, &r) == 2) {
        l = constrain(l, -255, 255);          // -255..255 = fraction of the top speed (maxTps)
        r = constrain(r, -255, 255);
        float nl = l * maxTps / 255.0, nr = r * maxTps / 255.0;
        if (nl != tgtL || nr != tgtR) stallL = stallR = false;
        tgtL = nl; tgtR = nr;
        pidOn = true;
        lastCmdMs = millis();
        if (l == 0 && r == 0) setMotors(0, 0);  // zero stops at once
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
    case 'L':                         // L: treat the present position as level (robot flat, still)
      setMotors(0, 0);
      if (imuPresent()) {
        mpu.calcOffsets(false, true);   // accelerometer offsets only, about 2 s
        saveAccOffsets();               // remembered after a restart
        zeroYaw();
        Serial.println(F("OK L"));
      } else Serial.println(F("ERR no IMU"));
      break;
    case 'R':                         // R: forget the saved level
      if (imuPresent()) { forgetAccOffsets(); Serial.println(F("OK R")); }
      else Serial.println(F("ERR no IMU"));
      break;
    case 'F':                         // kept so ROS can still send it; FL always shares D5 now
      Serial.println(F("OK F"));
      break;
    case 'K':                         // K <ticks/s per second>: soft start rate, 0 = jump
      accelTps = max(0.0f, (float)atof(args));
      Serial.println(F("OK K"));
      break;
    case 'Q':                         // Q: debug, print the controller state
      Serial.print(F("Q pid=")); Serial.print(pidOn);
      Serial.print(F(" tgt=")); Serial.print((long)tgtL); Serial.print('/'); Serial.print((long)tgtR);
      Serial.print(F(" want=")); Serial.print(wantL); Serial.print('/'); Serial.print(wantR);
      Serial.print(F(" cur=")); Serial.print(curL); Serial.print('/'); Serial.print(curR);
      Serial.print(F(" fl=")); Serial.print(curFL);
      Serial.print(F(" accel=")); Serial.print((long)accelTps);
      Serial.print(F(" stall=")); Serial.print(stallL); Serial.print('/'); Serial.println(stallR);
      break;
    case 'H': {                       // H <gain>: heading hold, 0 = off, negative flips the direction
      hdgGain = atof(args);
      Serial.println(F("OK H"));
      break;
    }
    case 'U': {                       // U <ticks/s>: smallest speed target
      long u = atol(args);
      if (u >= 0 && u <= 4000) { minTps = u; Serial.println(F("OK U")); }
      else Serial.println(F("ERR usage: U <0-4000>"));
      break;
    }
    case 'N': {                       // N <min PWM while a speed is requested, 0-200>
      int n = atoi(args);
      if (n >= 0 && n <= MAX_PWM) { minPwm = n; Serial.println(F("OK N")); }
      else Serial.println(F("ERR usage: N <0-200>"));
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
      zeroYaw();
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
    loadAccOffsets();
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
