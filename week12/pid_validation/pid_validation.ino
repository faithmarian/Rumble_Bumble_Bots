#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

// =====================================================================
// Week 12 - IMU + encoder closed loop validation
//
// Three tests prove the closed loop works, each finishing back at the
// starting pose so the error can be measured with a tape measure:
//
//   1. Spin 10 full turns in place   -> heading control, gyro scale
//   2. Drive a 300 mm radius circle  -> coupled distance + heading
//   3. Drive a 300 mm square         -> straight lines and 90 deg turns
//
// The controller is a trajectory tracker, not a plain PID on the raw
// error. A trapezoidal profile generates a moving setpoint, and the
// loop tracks it with velocity feedforward + PD + I. That is what makes
// a 3600 degree turn stop cleanly instead of saturating and overshooting.
//
// Serial: 115200 baud. Commands: 1 2 3 = one test, A = all, D = deadband,
// G = gyro drift check, Z = zero the pose, ? = help.
// With no command the full sequence starts by itself after 10 s.
// =====================================================================

// --------------------------- Pins ---------------------------
const uint8_t LEFT_ENC_A = 2;
const uint8_t LEFT_ENC_B = 7;
const uint8_t RIGHT_ENC_A = 3;
const uint8_t RIGHT_ENC_B = 8;

const uint8_t LEFT_PWM = 11;
const uint8_t LEFT_DIR = 12;
const uint8_t RIGHT_PWM = 9;
const uint8_t RIGHT_DIR = 10;

// The range sensors are unused here. Their XSHUT lines are held low so
// they stay in reset and never touch the I2C bus during a test.
const uint8_t LIDAR_LEFT_XSHUT = A0;
const uint8_t LIDAR_RIGHT_XSHUT = A1;
const uint8_t LIDAR_FRONT_XSHUT = A2;
const uint8_t OLED_ADDR = 0x3C;

// ------------------------- Geometry -------------------------
const float WHEEL_DIAMETER_MM = 44.0f;
const float ENCODER_COUNTS_PER_REV = 356.72f;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_REV;

// Distance between the two wheel contact points. The spin test measures
// this and prints the real value; put that number here afterwards.
float trackWidthMm = 90.0f;

// ------------------------ Test sizes ------------------------
const float SPIN_TURNS = 10.0f;
const float CIRCLE_RADIUS_MM = 300.0f;
const float SQUARE_SIDE_MM = 300.0f;

// ---------------------- Motion profiles ---------------------
// Keep the spin rate far below the gyro range (+-250 deg/s) or the
// measurement saturates and every result becomes meaningless.
const float TURN_RATE_MAX = 120.0f;      // deg/s
const float TURN_ACCEL = 240.0f;         // deg/s^2
const float TURN_RATE_MIN = 12.0f;       // deg/s, profile creep speed
const float DRIVE_SPEED_MAX = 120.0f;    // mm/s
const float DRIVE_ACCEL = 240.0f;        // mm/s^2
const float DRIVE_SPEED_MIN = 12.0f;     // mm/s

// ------------------------ Controllers -----------------------
// Feedforward maps a desired rate to PWM. It does not have to be exact:
// the integral term absorbs whatever it gets wrong.
const float TURN_KV = 0.42f;             // PWM per deg/s
const float TURN_KP = 1.60f;             // PWM per deg of lag
const float TURN_KI = 0.90f;             // PWM per deg*s
const float TURN_KD = 0.10f;             // PWM per deg/s of rate error
const float TURN_I_LIMIT = 18.0f;

const float DRIVE_KV = 0.40f;            // PWM per mm/s
const float DRIVE_KP = 0.55f;            // PWM per mm of lag
const float DRIVE_KI = 0.35f;            // PWM per mm*s
const float DRIVE_KD = 0.03f;            // PWM per mm/s of rate error
const float DRIVE_I_LIMIT = 25.0f;

// Heading hold while driving. Output is a differential PWM.
const float HEADING_KP = 2.40f;
const float HEADING_KI = 0.80f;
const float HEADING_KD = 0.12f;
const float HEADING_I_LIMIT = 12.0f;
const float HEADING_OUT_LIMIT = 45.0f;

const float PID_OUT_LIMIT = 150.0f;
const int MOTOR_MAX = 210;

// Static friction: below this PWM the wheels do not turn at all. The
// D command measures the real value; this is only the starting guess.
int leftDeadband = 18;
int rightDeadband = 18;

// ------------------------- Tolerances -----------------------
const float TURN_TOLERANCE_DEG = 1.0f;
const float TURN_SETTLE_RATE = 8.0f;     // deg/s considered stopped
const float DRIVE_TOLERANCE_MM = 1.5f;
const float DRIVE_SETTLE_RATE = 8.0f;    // mm/s considered stopped
const unsigned long SETTLE_MS = 350UL;
const unsigned long SETTLE_TIMEOUT_MS = 2500UL;
const unsigned long CONTROL_PERIOD_US = 8000UL;   // 125 Hz

const bool USE_DRIFT_COMPENSATION = true;
const unsigned long DRIFT_SAMPLE_MS = 4000UL;
const unsigned long AUTO_START_MS = 10000UL;

MPU6050 mpu(Wire);

// --------------------------- State --------------------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;
int8_t leftSign = 1;              // +1 when forward makes the count rise
int8_t rightSign = 1;

float yawDeg = 0.0f;              // drift compensated, never wrapped
float yawRateDegPerSec = 0.0f;
float gyroBiasDegPerSec = 0.0f;
// This library version has no setAngleZ, so zeroing is done in software.
// Keeping our own offset is better anyway: the library keeps integrating
// one continuous angle and nothing is lost when a test is restarted.
float yawOffsetDeg = 0.0f;
unsigned long driftEpochMs = 0;
float peakYawRate = 0.0f;

float poseX = 0.0f;               // mm, +X is the starting heading
float poseY = 0.0f;
float travelledMm = 0.0f;         // signed path length
float leftDistanceMm = 0.0f;
float rightDistanceMm = 0.0f;
long lastLeftTicks = 0;
long lastRightTicks = 0;


// --------------------------- OLED ---------------------------
const uint8_t DIGITS_5X7[] PROGMEM = {
  0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00,
  0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31,
  0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39,
  0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
  0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E
};

const uint8_t LETTERS_5X7[] PROGMEM = {
  0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36,
  0x3E,0x41,0x41,0x41,0x22, 0x7F,0x41,0x41,0x22,0x1C,
  0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01,
  0x3E,0x41,0x49,0x49,0x7A, 0x7F,0x08,0x08,0x08,0x7F,
  0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01,
  0x7F,0x08,0x14,0x22,0x41, 0x7F,0x40,0x40,0x40,0x40,
  0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F,
  0x3E,0x41,0x41,0x41,0x3E, 0x7F,0x09,0x09,0x09,0x06,
  0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46,
  0x46,0x49,0x49,0x49,0x31, 0x01,0x01,0x7F,0x01,0x01,
  0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F,
  0x3F,0x40,0x38,0x40,0x3F, 0x63,0x14,0x08,0x14,0x63,
  0x07,0x08,0x70,0x08,0x07, 0x61,0x51,0x49,0x45,0x43
};

class TinyOLED {
public:
  void begin() {
    const uint8_t init[] = {
      0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,0x20,0x02,
      0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF
    };
    for (uint8_t i = 0; i < sizeof(init); ++i) command(init[i]);
    for (uint8_t row = 0; row < 8; ++row) drawLine(row, "");
  }

  void drawLine(uint8_t row, const char *text) {
    uint8_t pixels[128] = {0};
    for (uint8_t col = 0; col < 16 && text[col]; ++col) {
      uint8_t glyph[5];
      getGlyph(text[col], glyph);
      for (uint8_t i = 0; i < 5; ++i) pixels[col * 8 + i] = glyph[i];
    }
    setCursor(row);
    for (uint8_t start = 0; start < 128; start += 16) {
      Wire.beginTransmission(OLED_ADDR);
      Wire.write(0x40);
      for (uint8_t i = 0; i < 16; ++i) Wire.write(pixels[start + i]);
      Wire.endTransmission();
    }
  }

private:
  void command(uint8_t value) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x00);
    Wire.write(value);
    Wire.endTransmission();
  }

  void setCursor(uint8_t row) {
    command(0xB0 | (row & 7));
    command(0x00);
    command(0x10);
  }

  void getGlyph(char c, uint8_t glyph[5]) {
    if (c >= 'a' && c <= 'z') c -= 32;
    const uint8_t *source = NULL;
    if (c >= '0' && c <= '9') source = &DIGITS_5X7[(c - '0') * 5];
    if (c >= 'A' && c <= 'Z') source = &LETTERS_5X7[(c - 'A') * 5];
    if (source) {
      for (uint8_t i = 0; i < 5; ++i) glyph[i] = pgm_read_byte(source + i);
      return;
    }
    for (uint8_t i = 0; i < 5; ++i) glyph[i] = 0;
    if (c == '-') { glyph[0]=0x08; glyph[1]=0x08; glyph[2]=0x08; glyph[3]=0x08; glyph[4]=0x08; }
    if (c == '.') { glyph[1]=0x60; glyph[2]=0x60; }
    if (c == ':') { glyph[1]=0x36; glyph[2]=0x36; }
    if (c == '/') { glyph[0]=0x20; glyph[1]=0x10; glyph[2]=0x08; glyph[3]=0x04; glyph[4]=0x02; }
  }
};

TinyOLED oled;

// ------------------------- Encoders -------------------------
void leftEncoderISR() {
  leftTicks += digitalRead(LEFT_ENC_B) ? 1 : -1;
}

void rightEncoderISR() {
  rightTicks += digitalRead(RIGHT_ENC_B) ? 1 : -1;
}

void readEncoders(long &left, long &right) {
  noInterrupts();
  left = leftTicks;
  right = rightTicks;
  interrupts();
}

// --------------------------- Motors -------------------------
void setMotor(uint8_t pwmPin, uint8_t dirPin, int command) {
  command = constrain(command, -255, 255);
  digitalWrite(dirPin, command >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(command));
}

// Positive means that wheel pushes the robot forward. The two motors are
// mirrored on the chassis, hence the opposite signs here.
void driveWheels(float leftForward, float rightForward) {
  setMotor(LEFT_PWM, LEFT_DIR, (int)round(leftForward));
  setMotor(RIGHT_PWM, RIGHT_DIR, (int)round(-rightForward));
}

void stopMotors() {
  driveWheels(0.0f, 0.0f);
}

// Adds the measured static friction so a small controller output still
// produces motion. Without this the loop sits at a few PWM and buzzes.
// The offset is blended in over the first few counts, otherwise a one
// degree residual error would jump straight to a full deadband kick.
const float DEADBAND_BLEND = 4.0f;

float withDeadband(float command, int deadband) {
  float magnitude = fabs(command);
  if (magnitude < 0.35f) return 0.0f;
  float blend = min(1.0f, magnitude / DEADBAND_BLEND);
  float output = min((float)deadband * blend + magnitude, (float)MOTOR_MAX);
  return command > 0.0f ? output : -output;
}

void driveCommand(float leftCommand, float rightCommand) {
  driveWheels(withDeadband(leftCommand, leftDeadband),
              withDeadband(rightCommand, rightDeadband));
}

// Positive turns the robot counter clockwise, which is the direction the
// gyro counts as positive.
void spinCommand(float command) {
  driveCommand(-command, command);
}

// ---------------------------- IMU ---------------------------
void sampleImu() {
  mpu.update();
  float raw = mpu.getAngleZ() - yawOffsetDeg;
  float rate = mpu.getGyroZ();
  if (USE_DRIFT_COMPENSATION) {
    float elapsed = (millis() - driftEpochMs) / 1000.0f;
    raw -= gyroBiasDegPerSec * elapsed;
    rate -= gyroBiasDegPerSec;
  }
  yawDeg = raw;
  yawRateDegPerSec = rate;
  if (fabs(rate) > peakYawRate) peakYawRate = fabs(rate);
}

// ------------------------- Odometry -------------------------
void resetPose() {
  readEncoders(lastLeftTicks, lastRightTicks);
  poseX = 0.0f;
  poseY = 0.0f;
  travelledMm = 0.0f;
  leftDistanceMm = 0.0f;
  rightDistanceMm = 0.0f;
  peakYawRate = 0.0f;
}

// Distance comes from the encoders, heading from the gyro. Fusing them
// this way is exactly what these tests are meant to prove.
void updateOdometry() {
  long left, right;
  readEncoders(left, right);
  float deltaLeft = (float)(left - lastLeftTicks) * MM_PER_COUNT * leftSign;
  float deltaRight = (float)(right - lastRightTicks) * MM_PER_COUNT * rightSign;
  lastLeftTicks = left;
  lastRightTicks = right;
  leftDistanceMm += deltaLeft;
  rightDistanceMm += deltaRight;
  float ds = 0.5f * (deltaLeft + deltaRight);
  travelledMm += ds;
  float heading = radians(yawDeg);
  poseX += ds * cos(heading);
  poseY += ds * sin(heading);
}

float wheelSpeedMmPerSec = 0.0f;

void updateSpeed(float ds, float dt) {
  // Light smoothing: raw encoder differences over 8 ms are very coarse.
  float instant = ds / dt;
  wheelSpeedMmPerSec += 0.25f * (instant - wheelSpeedMmPerSec);
}

// -------------------------- Tracker -------------------------
// One trajectory tracking controller: velocity feedforward, proportional
// on position lag, integral for steady state, derivative on rate error.
struct Tracker {
  float kv, kp, ki, kd, iLimit, outLimit;
  float integral;

  void reset() { integral = 0.0f; }

  float update(float lag, float rateSetpoint, float rateActual, float dt) {
    float unsaturated = kv * rateSetpoint + kp * lag + ki * integral
                        + kd * (rateSetpoint - rateActual);
    // Conditional integration: stop winding up while the output is
    // already saturated in the same direction as the error.
    if (fabs(unsaturated) < outLimit || unsaturated * lag < 0.0f) {
      integral = constrain(integral + lag * dt, -iLimit, iLimit);
    }
    float output = kv * rateSetpoint + kp * lag + ki * integral
                   + kd * (rateSetpoint - rateActual);
    return constrain(output, -outLimit, outLimit);
  }
};

Tracker turnTracker = {TURN_KV, TURN_KP, TURN_KI, TURN_KD,
                       TURN_I_LIMIT, PID_OUT_LIMIT, 0.0f};
Tracker driveTracker = {DRIVE_KV, DRIVE_KP, DRIVE_KI, DRIVE_KD,
                        DRIVE_I_LIMIT, PID_OUT_LIMIT, 0.0f};
Tracker headingTracker = {0.0f, HEADING_KP, HEADING_KI, HEADING_KD,
                          HEADING_I_LIMIT, HEADING_OUT_LIMIT, 0.0f};

// Trapezoidal profile expressed as speed versus position. Ramping up and
// braking are both bounded by the same acceleration, so the move always
// reaches the target with zero speed.
float profileRate(float done, float total, float cruise, float accel, float creep) {
  float rampUp = sqrt(2.0f * accel * max(0.0f, done) + creep * creep);
  float rampDown = sqrt(2.0f * accel * max(0.0f, total - done) + creep * creep);
  return constrain(min(rampUp, rampDown), creep, cruise);
}

// --------------------------- Display ------------------------
unsigned long lastDisplayMs = 0;

void showLine(uint8_t row, const char *label, long value) {
  char line[17];
  snprintf(line, sizeof(line), "%s%7ld", label, value);
  oled.drawLine(row, line);
}

void showProgress(const char *title, float a, float b) {
  unsigned long now = millis();
  if (now - lastDisplayMs < 120) return;
  lastDisplayMs = now;
  oled.drawLine(0, title);
  showLine(1, "SET:", lround(a));
  showLine(2, "NOW:", lround(b));
  showLine(3, "YAW:", lround(yawDeg));
}

// -------------------------- Control -------------------------
// Waits for the next control period and refreshes every measurement.
float controlTick() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();
  while (now - lastMicros < CONTROL_PERIOD_US) {
    now = micros();
  }
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;
  if (dt > 0.05f) dt = 0.05f;     // first call after a pause

  float before = travelledMm;
  sampleImu();
  updateOdometry();
  updateSpeed(travelledMm - before, dt);
  return dt;
}

void primeControl() {
  controlTick();
  controlTick();
}

// Turn in place by a relative angle. Works for 90 degrees and for 3600.
bool turnBy(float deltaDeg, const char *title) {
  float startYaw = yawDeg;
  float total = fabs(deltaDeg);
  float direction = deltaDeg >= 0.0f ? 1.0f : -1.0f;
  float setpoint = 0.0f;                 // profile position, always positive
  turnTracker.reset();
  primeControl();

  unsigned long settleSince = 0;
  unsigned long settleStart = 0;

  while (true) {
    float dt = controlTick();
    float actual = (yawDeg - startYaw) * direction;
    float rate = profileRate(setpoint, total, TURN_RATE_MAX, TURN_ACCEL, TURN_RATE_MIN);

    bool profileDone = setpoint >= total;
    if (!profileDone) {
      setpoint = min(setpoint + rate * dt, total);
    } else {
      rate = 0.0f;
      if (settleStart == 0) settleStart = millis();
    }

    float lag = setpoint - actual;
    float command = turnTracker.update(lag, rate, yawRateDegPerSec * direction, dt);
    spinCommand(command * direction);
    showProgress(title, setpoint, actual);

    if (profileDone) {
      bool inPlace = fabs(total - actual) <= TURN_TOLERANCE_DEG
                     && fabs(yawRateDegPerSec) <= TURN_SETTLE_RATE;
      if (inPlace) {
        if (settleSince == 0) settleSince = millis();
        if (millis() - settleSince >= SETTLE_MS) break;
      } else {
        settleSince = 0;
      }
      if (millis() - settleStart > SETTLE_TIMEOUT_MS) break;
    }
  }

  stopMotors();
  delay(250);
  controlTick();
  return true;
}

// Drive a straight line while holding the current heading.
bool driveStraight(float distanceMm, const char *title) {
  float holdYaw = yawDeg;
  float startTravel = travelledMm;
  float total = fabs(distanceMm);
  float direction = distanceMm >= 0.0f ? 1.0f : -1.0f;
  float setpoint = 0.0f;
  driveTracker.reset();
  headingTracker.reset();
  primeControl();

  unsigned long settleSince = 0;
  unsigned long settleStart = 0;

  while (true) {
    float dt = controlTick();
    float actual = (travelledMm - startTravel) * direction;
    float rate = profileRate(setpoint, total, DRIVE_SPEED_MAX, DRIVE_ACCEL, DRIVE_SPEED_MIN);

    bool profileDone = setpoint >= total;
    if (!profileDone) {
      setpoint = min(setpoint + rate * dt, total);
    } else {
      rate = 0.0f;
      if (settleStart == 0) settleStart = millis();
    }

    float lag = setpoint - actual;
    float forward = driveTracker.update(lag, rate, wheelSpeedMmPerSec * direction, dt);
    float turn = headingTracker.update(holdYaw - yawDeg, 0.0f, yawRateDegPerSec, dt);
    forward *= direction;
    driveCommand(forward - turn, forward + turn);
    showProgress(title, setpoint, actual);

    if (profileDone) {
      bool inPlace = fabs(total - actual) <= DRIVE_TOLERANCE_MM
                     && fabs(wheelSpeedMmPerSec) <= DRIVE_SETTLE_RATE;
      if (inPlace) {
        if (settleSince == 0) settleSince = millis();
        if (millis() - settleSince >= SETTLE_MS) break;
      } else {
        settleSince = 0;
      }
      if (millis() - settleStart > SETTLE_TIMEOUT_MS) break;
    }
  }

  stopMotors();
  delay(250);
  controlTick();
  return true;
}

// Drive a constant radius arc. The heading setpoint is derived from the
// distance already travelled, so no wheel model is needed: heading = s / R.
bool driveArc(float radiusMm, float sweepDeg, const char *title) {
  float startYaw = yawDeg;
  float startTravel = travelledMm;
  float total = fabs(radians(sweepDeg)) * radiusMm;      // arc length
  float sweepSign = sweepDeg >= 0.0f ? 1.0f : -1.0f;
  float setpoint = 0.0f;
  driveTracker.reset();
  headingTracker.reset();
  primeControl();

  // Steady state differential for this radius, used as feedforward so the
  // heading loop only has to correct what the geometry does not explain.
  float differentialRatio = trackWidthMm / (2.0f * radiusMm);

  unsigned long settleStart = 0;

  while (true) {
    float dt = controlTick();
    float actual = travelledMm - startTravel;
    float rate = profileRate(setpoint, total, DRIVE_SPEED_MAX, DRIVE_ACCEL, DRIVE_SPEED_MIN);

    bool profileDone = setpoint >= total;
    if (!profileDone) {
      setpoint = min(setpoint + rate * dt, total);
    } else {
      rate = 0.0f;
      if (settleStart == 0) settleStart = millis();
    }

    // Heading the robot should have after travelling this far along the arc.
    float headingSetpoint = startYaw + sweepSign * degrees(setpoint / radiusMm);
    float headingRate = sweepSign * degrees(rate / radiusMm);

    float forward = driveTracker.update(setpoint - actual, rate,
                                        wheelSpeedMmPerSec, dt);
    float turn = headingTracker.update(headingSetpoint - yawDeg, headingRate,
                                       yawRateDegPerSec, dt);
    // For a left sweep the left wheel is the inner one and runs slower.
    float leftBase = forward * (1.0f - sweepSign * differentialRatio);
    float rightBase = forward * (1.0f + sweepSign * differentialRatio);
    driveCommand(leftBase - turn, rightBase + turn);
    showProgress(title, setpoint, actual);

    if (profileDone) {
      if (fabs(total - actual) <= DRIVE_TOLERANCE_MM
          && fabs(wheelSpeedMmPerSec) <= DRIVE_SETTLE_RATE) break;
      if (millis() - settleStart > SETTLE_TIMEOUT_MS) break;
    }
  }

  stopMotors();
  delay(250);
  controlTick();
  return true;
}

// -------------------------- Reporting -----------------------
void printValue(const __FlashStringHelper *label, float value,
                const __FlashStringHelper *unit, uint8_t decimals = 2) {
  Serial.print(label);
  Serial.print(value, decimals);
  Serial.print(' ');
  Serial.println(unit);
}

// A closed loop finishes with the heading it swept, not with zero: one lap
// of the circle and four 90 degree corners both end at 360 degrees.
void reportPose(float expectedHeadingDeg) {
  Serial.print(F("  closing error : "));
  Serial.print(sqrt(poseX * poseX + poseY * poseY), 1);
  Serial.print(F(" mm  (x "));
  Serial.print(poseX, 1);
  Serial.print(F("  y "));
  Serial.print(poseY, 1);
  Serial.println(F(")"));
  Serial.print(F("  heading       : "));
  Serial.print(yawDeg, 2);
  Serial.print(F(" deg   expected "));
  Serial.print(expectedHeadingDeg, 1);
  Serial.print(F("   error "));
  Serial.print(yawDeg - expectedHeadingDeg, 2);
  Serial.println(F(" deg"));
  Serial.print(F("  path length   : "));
  Serial.print(travelledMm, 1);
  Serial.println(F(" mm  (odometry estimate)"));
}

void showResult(const char *title, float closing, float heading) {
  oled.drawLine(0, title);
  showLine(1, "ERRMM:", lround(closing));
  showLine(2, "ERRDG:", lround(heading));
  oled.drawLine(3, "");
}

// ---------------------------- Tests -------------------------
void testSpin() {
  Serial.println();
  Serial.println(F("=== TEST 1: SPIN 10 TURNS IN PLACE ==="));
  Serial.println(F("The robot must finish facing exactly where it started."));
  resetPose();
  float target = 360.0f * SPIN_TURNS;
  unsigned long started = millis();
  turnBy(target, "SPIN 10X");
  float seconds = (millis() - started) / 1000.0f;

  float measured = yawDeg;
  float sweepRadians = fabs(radians(measured));
  float wheelArc = fabs(leftDistanceMm) + fabs(rightDistanceMm);

  Serial.println(F("-- result --"));
  printValue(F("  commanded     : "), target, F("deg"), 1);
  printValue(F("  imu measured  : "), measured, F("deg"), 2);
  printValue(F("  heading error : "), measured - target, F("deg"), 2);
  printValue(F("  peak turn rate: "), peakYawRate, F("deg/s  (gyro limit 250)"), 1);
  if (peakYawRate > 220.0f) {
    Serial.println(F("  WARNING gyro near saturation, lower TURN_RATE_MAX"));
  }
  printValue(F("  left wheel    : "), leftDistanceMm, F("mm"), 1);
  printValue(F("  right wheel   : "), rightDistanceMm, F("mm"), 1);
  if (sweepRadians > 1.0f) {
    float measuredTrack = wheelArc / sweepRadians;
    printValue(F("  track width   : "), measuredTrack, F("mm  <-- put this in trackWidthMm"), 2);
    trackWidthMm = measuredTrack;
  }
  printValue(F("  wheel balance : "),
             fabs(leftDistanceMm) - fabs(rightDistanceMm), F("mm  (0 = perfectly centred)"), 1);
  printValue(F("  duration      : "), seconds, F("s"), 1);
  Serial.println(F("  Now measure the real heading error with a protractor."));
  showResult("SPIN DONE", 0.0f, measured - target);
}

void testCircle() {
  Serial.println();
  Serial.println(F("=== TEST 2: 300 MM RADIUS CIRCLE ==="));
  Serial.println(F("One full lap, finishing at the start point and heading."));
  resetPose();
  unsigned long started = millis();
  driveArc(CIRCLE_RADIUS_MM, 360.0f, "CIRCLE");
  float seconds = (millis() - started) / 1000.0f;

  Serial.println(F("-- result --"));
  printValue(F("  expected path : "), 2.0f * PI * CIRCLE_RADIUS_MM, F("mm"), 1);
  reportPose(360.0f);
  printValue(F("  duration      : "), seconds, F("s"), 1);
  Serial.println(F("  Measure the real gap between start and finish marks."));
  showResult("CIRCLE DONE", sqrt(poseX * poseX + poseY * poseY), yawDeg - 360.0f);
}

void testSquare() {
  Serial.println();
  Serial.println(F("=== TEST 3: 300 MM SQUARE ==="));
  Serial.println(F("Four sides and four left turns, back to the start pose."));
  resetPose();
  unsigned long started = millis();

  for (uint8_t side = 0; side < 4; ++side) {
    char title[17];
    snprintf(title, sizeof(title), "SQUARE SIDE %u", (unsigned)(side + 1));
    float before = travelledMm;
    driveStraight(SQUARE_SIDE_MM, title);
    float sideLength = travelledMm - before;
    turnBy(90.0f, "SQUARE TURN");
    Serial.print(F("  side "));
    Serial.print(side + 1);
    Serial.print(F(" : "));
    Serial.print(sideLength, 1);
    Serial.print(F(" mm    heading after corner "));
    Serial.print(yawDeg, 2);
    Serial.print(F(" deg   (want "));
    Serial.print(90.0f * (side + 1), 0);
    Serial.println(F(")"));
  }
  float seconds = (millis() - started) / 1000.0f;

  Serial.println(F("-- result --"));
  printValue(F("  expected path : "), 4.0f * SQUARE_SIDE_MM, F("mm"), 1);
  reportPose(360.0f);
  printValue(F("  duration      : "), seconds, F("s"), 1);
  showResult("SQUARE DONE", sqrt(poseX * poseX + poseY * poseY), yawDeg - 360.0f);
}

// --------------------- Calibration helpers ------------------
// Ramps the PWM until each wheel actually starts moving. Run it in both
// directions so the robot ends up roughly where it started.
void measureDeadband() {
  Serial.println();
  Serial.println(F("=== DEADBAND MEASUREMENT ==="));
  Serial.println(F("Robot pivots slightly. Keep it on the ground."));
  int leftTotal = 0;
  int rightTotal = 0;
  uint8_t passes = 0;

  for (int direction = 1; direction >= -1; direction -= 2) {
    long baseLeft, baseRight;
    readEncoders(baseLeft, baseRight);
    int foundLeft = 0;
    int foundRight = 0;

    for (int pwm = 4; pwm <= 90 && (!foundLeft || !foundRight); pwm += 2) {
      driveWheels(-direction * (foundLeft ? 0 : pwm),
                  direction * (foundRight ? 0 : pwm));
      delay(70);
      long left, right;
      readEncoders(left, right);
      if (!foundLeft && labs(left - baseLeft) >= 3) foundLeft = pwm;
      if (!foundRight && labs(right - baseRight) >= 3) foundRight = pwm;
    }
    stopMotors();
    delay(300);

    Serial.print(F("  pass "));
    Serial.print(direction > 0 ? F("CCW") : F("CW "));
    Serial.print(F("  left "));
    Serial.print(foundLeft);
    Serial.print(F("  right "));
    Serial.println(foundRight);
    if (foundLeft && foundRight) {
      leftTotal += foundLeft;
      rightTotal += foundRight;
      ++passes;
    }
  }

  if (passes == 0) {
    Serial.println(F("  FAILED: no wheel movement detected at all."));
    Serial.println(F("  Check motor wiring, battery voltage and encoder pins."));
    return;
  }
  leftDeadband = leftTotal / passes;
  rightDeadband = rightTotal / passes;
  Serial.print(F("  leftDeadband  = "));
  Serial.println(leftDeadband);
  Serial.print(F("  rightDeadband = "));
  Serial.println(rightDeadband);
  if (abs(leftDeadband - rightDeadband) > 8) {
    Serial.println(F("  NOTE the two motors differ a lot; that alone bends straight lines."));
  }
}

// Works out which way each encoder counts when the robot moves forward,
// then returns to the starting point so no test is biased by it.
void calibrateEncoderSigns() {
  Serial.println(F("Encoder direction check..."));
  long startLeft, startRight, endLeft, endRight;
  readEncoders(startLeft, startRight);
  driveWheels(60, 60);
  delay(320);
  stopMotors();
  delay(250);
  readEncoders(endLeft, endRight);

  long deltaLeft = endLeft - startLeft;
  long deltaRight = endRight - startRight;
  leftSign = deltaLeft >= 0 ? 1 : -1;
  rightSign = deltaRight >= 0 ? 1 : -1;

  Serial.print(F("  forward nudge ticks: left "));
  Serial.print(deltaLeft);
  Serial.print(F("  right "));
  Serial.println(deltaRight);
  if (labs(deltaLeft) < 10 || labs(deltaRight) < 10) {
    Serial.println(F("  WARNING one encoder barely moved. Fix that before trusting any result."));
  }
  Serial.print(F("  leftSign "));
  Serial.print(leftSign);
  Serial.print(F("  rightSign "));
  Serial.println(rightSign);

  // Drive back the same amount so the robot is where it started.
  driveWheels(-60, -60);
  delay(320);
  stopMotors();
  delay(250);
}

// Measures how fast the gyro angle wanders while the robot is still.
// Everything in these tests integrates this bias, so it is worth knowing.
void measureGyroDrift() {
  Serial.println(F("Gyro drift check, keep the robot still..."));
  oled.drawLine(0, "DRIFT CHECK");
  gyroBiasDegPerSec = 0.0f;
  driftEpochMs = millis();
  mpu.update();
  float startAngle = mpu.getAngleZ();
  unsigned long started = millis();
  float peak = 0.0f;
  while (millis() - started < DRIFT_SAMPLE_MS) {
    mpu.update();
    float rate = fabs(mpu.getGyroZ());
    if (rate > peak) peak = rate;
    delay(5);
  }
  float elapsed = (millis() - started) / 1000.0f;
  float drift = (mpu.getAngleZ() - startAngle) / elapsed;

  Serial.print(F("  drift rate    : "));
  Serial.print(drift, 4);
  Serial.println(F(" deg/s"));
  Serial.print(F("  over a 40 s test that is "));
  Serial.print(fabs(drift) * 40.0f, 1);
  Serial.println(F(" deg of error"));
  Serial.print(F("  noise peak    : "));
  Serial.print(peak, 3);
  Serial.println(F(" deg/s"));
  if (fabs(drift) > 0.5f) {
    Serial.println(F("  WARNING large drift. Was the robot moved during calibration?"));
  }

  if (USE_DRIFT_COMPENSATION) {
    gyroBiasDegPerSec = drift;
    Serial.println(F("  compensation  : ON (this bias is subtracted from now on)"));
  } else {
    Serial.println(F("  compensation  : OFF"));
  }
  driftEpochMs = millis();
}

void zeroHeading() {
  mpu.update();
  yawOffsetDeg = mpu.getAngleZ();
  driftEpochMs = millis();
  sampleImu();
  resetPose();
}

// ----------------------------- Menu -------------------------
void printMenu() {
  Serial.println();
  Serial.println(F("--------------------------------------------"));
  Serial.println(F(" 1 = spin 10 turns    2 = circle    3 = square"));
  Serial.println(F(" A = all three        D = deadband  G = gyro drift"));
  Serial.println(F(" Z = zero pose        ? = this menu"));
  Serial.println(F("--------------------------------------------"));
}

void pauseBetweenTests() {
  stopMotors();
  Serial.println(F("  ... 3 s pause, do not touch the robot ..."));
  delay(3000);
  zeroHeading();
}

void runAll() {
  zeroHeading();
  testSpin();
  pauseBetweenTests();
  testCircle();
  pauseBetweenTests();
  testSquare();
  Serial.println();
  Serial.println(F("=== SEQUENCE COMPLETE ==="));
  Serial.println(F("Report back: the three closing errors, the three heading"));
  Serial.println(F("errors, the measured track width and the deadband values."));
  printMenu();
  oled.drawLine(0, "ALL TESTS DONE");
}

void handleCommand(char c) {
  switch (c) {
    case '1': zeroHeading(); testSpin(); break;
    case '2': zeroHeading(); testCircle(); break;
    case '3': zeroHeading(); testSquare(); break;
    case 'a': case 'A': runAll(); break;
    case 'd': case 'D': measureDeadband(); break;
    case 'g': case 'G': measureGyroDrift(); break;
    case 'z': case 'Z': zeroHeading(); Serial.println(F("Pose zeroed.")); break;
    case '?': printMenu(); break;
    default: break;
  }
}

// ----------------------------- Main -------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  stopMotors();

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // Hold the range sensors in reset; they are not used by these tests.
  pinMode(LIDAR_LEFT_XSHUT, OUTPUT);
  pinMode(LIDAR_RIGHT_XSHUT, OUTPUT);
  pinMode(LIDAR_FRONT_XSHUT, OUTPUT);
  digitalWrite(LIDAR_LEFT_XSHUT, LOW);
  digitalWrite(LIDAR_RIGHT_XSHUT, LOW);
  digitalWrite(LIDAR_FRONT_XSHUT, LOW);

  Wire.begin();
  Wire.setClock(400000);
  oled.begin();

  Serial.println();
  Serial.println(F("Week 12 IMU + encoder closed loop validation"));
  Serial.print(F("mm per encoder count: "));
  Serial.println(MM_PER_COUNT, 5);

  oled.drawLine(0, "KEEP STILL");
  mpu.begin();
  delay(1000);
  mpu.calcOffsets(true, true);
  yawOffsetDeg = 0.0f;
  driftEpochMs = millis();

  measureGyroDrift();
  calibrateEncoderSigns();
  measureDeadband();
  zeroHeading();

  Serial.println();
  Serial.println(F("Clear a circle about 1.2 m across around the robot."));
  Serial.println(F("Mark the starting position and heading on the floor."));
  printMenu();
  Serial.println(F("Starting the full sequence in 10 s unless a key is sent."));
  oled.drawLine(0, "READY");
}

void loop() {
  static bool autoStarted = false;
  static unsigned long readyAt = 0;
  if (readyAt == 0) readyAt = millis();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    autoStarted = true;
    handleCommand(c);
  }

  if (!autoStarted && millis() - readyAt > AUTO_START_MS) {
    autoStarted = true;
    runAll();
  }

  stopMotors();
  if (!autoStarted) {
    char line[17];
    snprintf(line, sizeof(line), "START IN %2ld",
             (long)((AUTO_START_MS - (millis() - readyAt)) / 1000));
    oled.drawLine(1, line);
  }
  delay(20);
}
