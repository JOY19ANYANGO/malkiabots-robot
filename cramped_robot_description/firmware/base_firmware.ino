/*
 * cramped_robot base firmware — Arduino Nano/Uno/Mega or ESP32 (Arduino core)
 * Drives two motor channels (left pair, right pair) with per-side PID on wheel speed,
 * counts two quadrature encoders, and talks to scripts/base_driver.py over USB serial.
 *
 * Serial protocol, 115200 baud, newline terminated:
 *   PC -> MCU   "v <left_rad_s> <right_rad_s>"   wheel angular-velocity targets
 *   PC -> MCU   "r"                              reset encoder counts
 *   MCU -> PC   "e <lt> <rt> <lv> <rv>"          cumulative ticks + measured rad/s, 50 Hz
 *
 * Wiring (edit the pin table): one TB6612FNG / L298N / BTS7960 channel per SIDE.
 * Both motors of a side are wired in parallel to that channel (same direction!).
 * One quadrature encoder per side (on a front or rear wheel); channel A on an interrupt pin.
 */

// ---------------- pins (Arduino Uno/Nano defaults; change for ESP32) ----------------
const int L_PWM = 5,  L_IN1 = 7,  L_IN2 = 8;      // left  channel: PWM + two direction pins
const int R_PWM = 6,  R_IN1 = 9,  R_IN2 = 10;     // right channel
const int L_ENC_A = 2, L_ENC_B = 4;                // left encoder (A must be an interrupt pin)
const int R_ENC_A = 3, R_ENC_B = 11;               // right encoder
const bool L_INVERT = false, R_INVERT = true;      // flip if a side runs backwards

// ---------------- robot constants (match base_driver.py) ----------------
const float TICKS_PER_REV = 1320.0;                // encoder CPR x gearbox ratio, per WHEEL revolution -> calibrate
const float MAX_RAD_S     = 10.5;                  // ~100 rpm no-load
const float KP = 18.0, KI = 60.0, KD = 0.0;        // PID on rad/s error -> PWM; start here and tune
const unsigned long CMD_TIMEOUT_MS = 500;          // stop if the PC goes silent
const unsigned long LOOP_MS = 20;                  // 50 Hz control + telemetry

#if defined(ESP32)
#define IRAM_ATTR_OR_EMPTY IRAM_ATTR
#else
#define IRAM_ATTR_OR_EMPTY
#endif

volatile long lTicks = 0, rTicks = 0;
float lTarget = 0, rTarget = 0, lInt = 0, rInt = 0, lPrevErr = 0, rPrevErr = 0;
long lPrev = 0, rPrev = 0;
unsigned long lastCmd = 0, lastLoop = 0;
String rx;

void IRAM_ATTR_OR_EMPTY lIsr() { lTicks += (digitalRead(L_ENC_B) == digitalRead(L_ENC_A)) ? 1 : -1; }
void IRAM_ATTR_OR_EMPTY rIsr() { rTicks += (digitalRead(R_ENC_B) == digitalRead(R_ENC_A)) ? 1 : -1; }

void drive(int pwmPin, int in1, int in2, bool invert, float u) {
  if (invert) u = -u;
  int pwm = constrain((int)fabs(u), 0, 255);
  digitalWrite(in1, u >= 0 ? HIGH : LOW);
  digitalWrite(in2, u >= 0 ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}

float pid(float target, float meas, float &integ, float &prevErr, float dt) {
  if (fabs(target) < 0.05) { integ = 0; prevErr = 0; return 0; }     // coast to stop, no windup
  float err = target - meas;
  integ = constrain(integ + err * dt, -40, 40);
  float d = (err - prevErr) / dt; prevErr = err;
  float ff = target / MAX_RAD_S * 255.0;                             // feed-forward
  return constrain(ff + KP * err + KI * integ + KD * d, -255, 255);
}

void setup() {
  Serial.begin(115200);
  int outs[] = {L_PWM, L_IN1, L_IN2, R_PWM, R_IN1, R_IN2};
  for (int p : outs) pinMode(p, OUTPUT);
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), lIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rIsr, CHANGE);
  lastCmd = lastLoop = millis();
}

void loop() {
  // ---- receive commands ----
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (rx.startsWith("v ")) {
        int sp = rx.indexOf(' ', 2);
        lTarget = constrain(rx.substring(2, sp).toFloat(), -MAX_RAD_S, MAX_RAD_S);
        rTarget = constrain(rx.substring(sp + 1).toFloat(), -MAX_RAD_S, MAX_RAD_S);
        lastCmd = millis();
      } else if (rx == "r") { noInterrupts(); lTicks = rTicks = 0; lPrev = rPrev = 0; interrupts(); }
      rx = "";
    } else if (rx.length() < 40) rx += c;
  }

  // ---- 50 Hz control ----
  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  float dt = (now - lastLoop) / 1000.0; lastLoop = now;
  if (now - lastCmd > CMD_TIMEOUT_MS) lTarget = rTarget = 0;

  long lt, rt; noInterrupts(); lt = lTicks; rt = rTicks; interrupts();
  float lv = (lt - lPrev) / TICKS_PER_REV * TWO_PI / dt;            // measured rad/s
  float rv = (rt - rPrev) / TICKS_PER_REV * TWO_PI / dt;
  lPrev = lt; rPrev = rt;

  drive(L_PWM, L_IN1, L_IN2, L_INVERT, pid(lTarget, lv, lInt, lPrevErr, dt));
  drive(R_PWM, R_IN1, R_IN2, R_INVERT, pid(rTarget, rv, rInt, rPrevErr, dt));

  Serial.print("e "); Serial.print(lt); Serial.print(' '); Serial.print(rt);
  Serial.print(' '); Serial.print(lv, 3); Serial.print(' '); Serial.println(rv, 3);
}
