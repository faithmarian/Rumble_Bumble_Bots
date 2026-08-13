#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <math.h>
#include <string.h>

// Week 12 Task 4.1.2 - execute the f/l/r path from Task 4.1.1.
// Race build. The motion core is the one proven on the board in
// autonomous_mapping_pid_migrated.ino, carrying its field fixes:
//
//   * trajectory tracking (trapezoidal profile + velocity feedforward),
//     not a plain PID chasing a raw error;
//   * D terms read the gyro rate directly instead of differentiating the
//     angle - numerical differencing over an irregular loop was the main
//     source of the constant buzzing;
//   * turns coast inside a 2 degree zone instead of chasing the last
//     degree through the static-friction kick;
//   * one-layer micromouse correction, and the correction is scaled by the
//     forward drive so it can never pivot the robot in place. That coupling
//     was the "suddenly twitches then hits the wall" failure;
//   * every range reading needs two consecutive samples that agree, so a
//     single spike cannot collapse the speed;
//   * wall-edge odometry sync pins the longitudinal position to the real
//     maze, so error cannot accumulate into an off-by-one-cell turn;
//   * stall watchdogs in both the straight and the turn.
//
// Race-specific on top of that: consecutive 'f' commands run as ONE
// continuous straight, and turns aim at an absolute heading taken from the
// startup reference so a turn finishing 2 degrees short does not push its
// error into the next one.

// --------------------------- Route --------------------------
// Marking day: replace only this line with the line printed by the notebook.
const char COMMANDS[] = "frfflfffrffflfffll";

// --------------------------- Pins ---------------------------
const uint8_t LEFT_ENC_A = 2;
const uint8_t LEFT_ENC_B = 7;
const uint8_t RIGHT_ENC_A = 3;
const uint8_t RIGHT_ENC_B = 8;
const uint8_t LEFT_PWM = 11;
const uint8_t LEFT_DIR = 12;
const uint8_t RIGHT_PWM = 9;
const uint8_t RIGHT_DIR = 10;
const uint8_t LIDAR_LEFT_XSHUT = A0;
const uint8_t LIDAR_FRONT_XSHUT = A2;
const uint8_t LIDAR_RIGHT_XSHUT = A1;
const uint8_t ADDR_LEFT = 0x54;
const uint8_t ADDR_FRONT = 0x56;
const uint8_t ADDR_RIGHT = 0x55;
const uint8_t OLED_ADDR = 0x3C;

const int16_t HEADING_YAW[4] = {0, -90, 180, 90};   // N, E, S, W

// ------------------------- Geometry -------------------------
const float CELL_MM = 180.0f;
const float ROBOT_SIZE_MM = 76.0f;
const float WALL_GAP_MM = (CELL_MM - ROBOT_SIZE_MM) / 2.0f;
const float WHEEL_DIAMETER_MM = 44.0f;
const float ENCODER_COUNTS_PER_REV = 356.72f;
// Field measured on this chassis: cells came up short before this scale.
const float ENCODER_DISTANCE_SCALE = 1.02f;
const float MM_PER_COUNT = PI * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV
                           * ENCODER_DISTANCE_SCALE;

// ========================= RACE SPEED =======================
// 100 / 100 are the values validated on the board by the 4.3 build. The
// straights here are longer because whole runs are chained, so the cruise
// is lifted a little. Raise DRIVE_SPEED_MAX first and only that; if the
// robot weaves or clips a post, put it back to 100 before touching a gain.
const float DRIVE_SPEED_MAX = 120.0f;    // mm/s   (proven safe: 100)
const float TURN_RATE_MAX = 110.0f;      // deg/s  (proven safe: 100)
// 120 not 160: hard launches slip the wheels, and slipped counts are
// exactly the odometry error that puts a turn one cell early.
const float DRIVE_ACCEL = 120.0f;
const float TURN_ACCEL = 160.0f;
// Creep speeds sit above the stick-slip threshold on purpose. Creeping at
// 5 deg/s or 6 mm/s makes the wheels stick and jerk.
const float DRIVE_SPEED_MIN = 12.0f;
const float TURN_RATE_MIN = 12.0f;
// ============================================================

// ------------------------ Controllers -----------------------
const float TURN_KV = 0.35f;
const float TURN_KP = 1.10f;
const float TURN_KI = 0.18f;
const float TURN_KD = 0.16f;
const float TURN_I_LIMIT = 4.0f;
// Must exceed KV * TURN_RATE_MAX or the feedforward alone saturates and
// the feedback hunts against the clamp.
const float TURN_OUT_LIMIT = 26.0f;

const float DRIVE_KV = 0.36f;
const float DRIVE_KP = 0.50f;
const float DRIVE_KI = 0.10f;
const float DRIVE_KD = 0.06f;
const float DRIVE_I_LIMIT = 7.0f;
const float DRIVE_OUT_LIMIT = 48.0f;

// One-layer classic micromouse correction while driving. Walls visible:
// P on the corridor offset, gain rising past the knee. No walls: P on the
// grid heading. Both damped by the raw gyro rate. No steer state, no mode
// memory, nothing to wind up.
const float WALL_CENTRE_KP = 0.22f;      // PWM per mm of offset
const float HEADING_HOLD_KP = 1.1f;      // PWM per deg of heading error
const float RATE_DAMPING = 0.10f;        // PWM per deg/s
const int CORRECTION_MAX = 10;

// ------------------------- Motors ---------------------------
const int MOTOR_DEADBAND = 18;
// Once the wheels turn, friction is kinetic. Injecting the full static
// kick while cruising makes the small-signal gain huge and the speed loop
// surge, which reads as a residual shake.
const int MOTOR_DEADBAND_MOVING = 10;
const int MOTOR_MAX = 90;
const float DEADBAND_BLEND = 4.0f;

// ------------------------- Sensing --------------------------
const uint16_t SIDE_WALL_MAX_MM = 95;
const uint16_t FRONT_WALL_MAX_MM = 120;
// Two consecutive readings must agree before they are believed. A single
// front spike used to collapse the cruise, zero the forward drive, and let
// the deadband turn the heading correction into an in-place pivot.
const uint16_t SIDE_AGREE_MM = 15;
const uint16_t SIDE_SLOW_MM = 34;
const float SIDE_SLOW_SPEED = 45.0f;
const float FRONT_APPROACH_GAIN = 1.4f;
const unsigned long LIDAR_DRIVE_INTERVAL_MS = 12;
const uint8_t RANGE_SAMPLES = 3;

// ------------------------ Tolerances ------------------------
const float TURN_TOLERANCE_DEG = 1.0f;
// Inside this zone the motors coast instead of chasing the last degree
// through the static-friction kick: kick, overshoot, reverse kick, repeat.
const float TURN_COAST_ZONE_DEG = 2.0f;
const float TURN_SETTLE_RATE = 7.0f;
const float CELL_TOLERANCE_MM = 3.0f;
const unsigned long TURN_STABLE_MS = 150;
const unsigned long SETTLE_TIMEOUT_MS = 1200;
const unsigned long DRIVE_TIMEOUT_MS = 6000;
const unsigned long ACTION_PAUSE_MS = 50;

// Wall-edge odometry sync: when a side wall appears or ends the robot is
// physically at a cell boundary (start centre + 90 + k*180). Snapping the
// measured distance towards it pins the position to the real maze.
const float EDGE_SYNC_WINDOW_MM = 30.0f;
const float EDGE_SYNC_BLEND = 0.5f;
const float EDGE_SYNC_OFFSET_MAX_MM = 60.0f;

// Stall recovery: commanded but not moving means wedged.
const float STALL_LAG_MM = 35.0f;
const float STALL_SPEED_MM_S = 6.0f;
const unsigned long STALL_DETECT_MS = 400;
const uint8_t STALL_RECOVERY_MAX = 2;
const float STALL_BACKOFF_MM = 18.0f;

MPU6050 mpu(Wire);
VL6180X lidarLeft;
VL6180X lidarFront;
VL6180X lidarRight;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
uint16_t leftMm = 999;
uint16_t frontMm = 999;
uint16_t rightMm = 999;
uint16_t prevLeftMm = 999;
uint16_t prevRightMm = 999;
uint16_t prevFrontMm = 999;
uint8_t lidarPhase = 0;
unsigned long lastLidarMs = 0;
unsigned long lastDisplayMs = 0;

float targetYaw = 0.0f;
float northYaw = 0.0f;
uint8_t headingIndex = 0;
float gyroBias = 0.0f;
bool routeFailed = false;
uint8_t turnRetries = 0;

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
void leftEncoderISR() { leftTicks += digitalRead(LEFT_ENC_B) ? 1 : -1; }
void rightEncoderISR() { rightTicks += digitalRead(RIGHT_ENC_B) ? 1 : -1; }

void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

float travelledMm() {
  noInterrupts();
  long left = leftTicks;
  long right = rightTicks;
  interrupts();
  return 0.5f * (labs(left) + labs(right)) * MM_PER_COUNT;
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// ---------------------------- IMU ---------------------------
float yaw() {
  mpu.update();
  return mpu.getAngleZ();
}

// The D terms use this. Reading the rate straight off the gyro instead of
// differentiating the angle over an irregular loop period is what removed
// the constant buzz.
float gyroRateDegPerSec() { return mpu.getGyroZ() - gyroBias; }

// Averages raw rate samples while the robot is still. Never difference the
// angle over a short window for this: angle noise becomes fake bias.
void measureGyroBias() {
  float total = 0.0f;
  for (uint8_t i = 0; i < 32; ++i) {
    mpu.update();
    total += mpu.getGyroZ();
    delay(5);
  }
  float average = total / 32.0f;
  if (fabs(average) <= 2.5f) gyroBias = average;
}

// --------------------------- Motors -------------------------
void setMotor(uint8_t pwmPin, uint8_t dirPin, int command) {
  command = constrain(command, -255, 255);
  digitalWrite(dirPin, command >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(command));
}

void setWheels(float left, float right) {
  setMotor(LEFT_PWM, LEFT_DIR, (int)round(-left));
  setMotor(RIGHT_PWM, RIGHT_DIR, (int)round(right));
}

void stopMotors() { setWheels(0, 0); }

int activeDeadband = MOTOR_DEADBAND;

// Continuous blend between static and kinetic deadband. Switching in one
// step at a threshold injected an 8 PWM square wave and made motion rougher.
void updateActiveDeadband(float magnitude) {
  float blend = constrain(magnitude / 40.0f, 0.0f, 1.0f);
  activeDeadband = MOTOR_DEADBAND
                   - (int)((MOTOR_DEADBAND - MOTOR_DEADBAND_MOVING) * blend);
}

float withDeadband(float command) {
  float magnitude = fabs(command);
  if (magnitude < 0.35f) return 0.0f;
  float blend = min(1.0f, magnitude / DEADBAND_BLEND);
  float output = min(activeDeadband * blend + magnitude, (float)MOTOR_MAX);
  return command > 0.0f ? output : -output;
}

void setControlledWheels(float left, float right) {
  setWheels(withDeadband(left), withDeadband(right));
}

// -------------------------- Tracker -------------------------
struct Tracker {
  float kv, kp, ki, kd, iLimit, outLimit, integral;

  void reset() { integral = 0.0f; }

  float update(float lag, float rateSetpoint, float rateActual, float dt) {
    float raw = kv * rateSetpoint + kp * lag + ki * integral
                + kd * (rateSetpoint - rateActual);
    if (fabs(raw) < outLimit || raw * lag < 0.0f) {
      integral = constrain(integral + lag * dt, -iLimit, iLimit);
    }
    return constrain(kv * rateSetpoint + kp * lag + ki * integral
                     + kd * (rateSetpoint - rateActual),
                     -outLimit, outLimit);
  }
};

Tracker turnTracker = {TURN_KV, TURN_KP, TURN_KI, TURN_KD,
                       TURN_I_LIMIT, TURN_OUT_LIMIT, 0.0f};
Tracker driveTracker = {DRIVE_KV, DRIVE_KP, DRIVE_KI, DRIVE_KD,
                        DRIVE_I_LIMIT, DRIVE_OUT_LIMIT, 0.0f};

float profileRate(float done, float total, float cruise,
                  float acceleration, float creep) {
  float rampUp = sqrt(2.0f * acceleration * max(0.0f, done) + creep * creep);
  float rampDown = sqrt(2.0f * acceleration * max(0.0f, total - done)
                        + creep * creep);
  return constrain(min(rampUp, rampDown), creep, cruise);
}

// ---------------------------- Lidar -------------------------
void startLidar(VL6180X &sensor, uint8_t xshut, uint8_t address) {
  digitalWrite(xshut, HIGH);
  delay(50);
  sensor.setTimeout(60);
  sensor.init();
  sensor.configureDefault();
  sensor.setAddress(address);
}

uint16_t readMm(VL6180X &sensor) {
  uint16_t value = sensor.readRangeSingleMillimeters();
  return sensor.timeoutOccurred() ? 999 : value;
}

// A reading is believed only when the previous one agreed with it.
uint16_t agreedSide(uint16_t value, uint16_t &previous) {
  uint16_t result = 999;
  if (value != 999 && previous != 999
      && abs((int)value - (int)previous) <= (int)SIDE_AGREE_MM) {
    result = value;
  }
  previous = value;
  return result;
}

// One sensor per tick while driving: three blocking reads per loop stretched
// the control period far too much at speed.
void updateLidarsRoundRobin(unsigned long intervalMs) {
  unsigned long now = millis();
  if (now - lastLidarMs < intervalMs) return;
  lastLidarMs = now;
  if (lidarPhase == 0) leftMm = agreedSide(readMm(lidarLeft), prevLeftMm);
  else if (lidarPhase == 1) frontMm = agreedSide(readMm(lidarFront), prevFrontMm);
  else rightMm = agreedSide(readMm(lidarRight), prevRightMm);
  lidarPhase = (lidarPhase + 1) % 3;
}

uint16_t stableRange(VL6180X &sensor) {
  uint16_t values[RANGE_SAMPLES];
  uint8_t count = 0;
  for (uint8_t attempt = 0; attempt < RANGE_SAMPLES * 3 && count < RANGE_SAMPLES; ++attempt) {
    uint16_t value = readMm(sensor);
    if (value != 999) values[count++] = value;
    delay(3);
  }
  if (count == 0) return 999;
  for (uint8_t i = 1; i < count; ++i) {
    uint16_t value = values[i];
    int8_t j = i - 1;
    while (j >= 0 && values[j] > value) { values[j + 1] = values[j]; --j; }
    values[j + 1] = value;
  }
  return values[count / 2];
}

void sampleLidars() {
  leftMm = stableRange(lidarLeft);
  frontMm = stableRange(lidarFront);
  rightMm = stableRange(lidarRight);
  prevLeftMm = leftMm;
  prevRightMm = rightMm;
  prevFrontMm = frontMm;
  lastLidarMs = millis();
}

bool sideWall(uint16_t distance) { return distance >= 20 && distance <= SIDE_WALL_MAX_MM; }
bool frontWall(uint16_t distance) { return distance >= 20 && distance <= FRONT_WALL_MAX_MM; }

// -------------------------- Telemetry -----------------------
void drawTelemetry(const char *status) {
  if (millis() - lastDisplayMs < 150) return;
  lastDisplayMs = millis();
  char line[17];
  oled.drawLine(0, status);
  snprintf(line, sizeof(line), "Z:%4ld T:%4ld", lround(mpu.getAngleZ()), lround(targetYaw));
  oled.drawLine(1, line);
  snprintf(line, sizeof(line), "L:%3u F:%3u", leftMm, frontMm);
  oled.drawLine(2, line);
  snprintf(line, sizeof(line), "R:%3u", rightMm);
  oled.drawLine(3, line);
}

void waitStopped(unsigned long duration, const char *status) {
  stopMotors();
  unsigned long start = millis();
  while (millis() - start < duration) {
    drawTelemetry(status);
    delay(10);
  }
}

// ----------------------- Robot actions ----------------------
// No lidar reads in here: a turn does not need them and each blocking read
// stretched the loop period mid-turn.
bool turnTo(float target, const char *status) {
  float currentYaw = yaw();
  float delta = normalizeAngle(target - currentYaw);
  float direction = delta >= 0.0f ? 1.0f : -1.0f;
  float total = fabs(delta);
  if (total <= TURN_TOLERANCE_DEG) { stopMotors(); return true; }
  float setpoint = 0.0f;
  float actual = 0.0f;
  float lastYaw = currentYaw;
  uint8_t recoveries = 0;
  turnTracker.reset();
  unsigned long lastMs = millis(), stableSince = 0, settleStart = 0, stallSince = 0;

  while (true) {
    unsigned long now = millis();
    float dt = constrain((now - lastMs) / 1000.0f, 0.001f, 0.05f);
    lastMs = now;
    currentYaw = yaw();
    float yawRate = gyroRateDegPerSec() * direction;
    actual += normalizeAngle(currentYaw - lastYaw) * direction;
    lastYaw = currentYaw;

    float rate = profileRate(setpoint, total, TURN_RATE_MAX,
                             TURN_ACCEL, TURN_RATE_MIN);
    bool profileDone = setpoint >= total;
    if (!profileDone) setpoint = min(setpoint + rate * dt, total);
    else { rate = 0.0f; if (settleStart == 0) settleStart = now; }

    float lag = setpoint - actual;
    if (profileDone && fabs(lag) <= TURN_COAST_ZONE_DEG) {
      stopMotors();
      if (fabs(yawRate) <= TURN_SETTLE_RATE) {
        if (stableSince == 0) stableSince = now;
        if (now - stableSince >= TURN_STABLE_MS) break;
      } else stableSince = 0;
    } else {
      stableSince = 0;
      float command = turnTracker.update(lag, rate, yawRate, dt) * direction;
      updateActiveDeadband(fabs(yawRate));
      setControlledWheels(command, -command);
      // Wedged against a wall while turning: output high, no rotation.
      if (fabs(yawRate) < 4.0f && fabs(command) > 12.0f) {
        if (stallSince == 0) stallSince = now;
        if (now - stallSince >= 500 && recoveries < STALL_RECOVERY_MAX) {
          ++recoveries;
          stallSince = 0;
          stopMotors();
          delay(80);
          activeDeadband = MOTOR_DEADBAND;
          unsigned long backStart = millis();
          while (millis() - backStart < 250) {
            setControlledWheels(9.0f, 9.0f);   // positive args = reverse
            delay(10);
          }
          stopMotors();
          delay(80);
          turnTracker.reset();
          lastMs = millis();
          continue;
        }
      } else stallSince = 0;
    }
    if (profileDone && now - settleStart >= SETTLE_TIMEOUT_MS) break;
    drawTelemetry(status);
    delay(10);
  }
  stopMotors();
  return fabs(normalizeAngle(target - yaw())) <= 3.0f;
}

// Aims at an absolute heading built from the startup reference, so an
// imperfect turn never pushes its error into the next one.
void turnToHeading(uint8_t index, const char *status) {
  headingIndex = index & 3;
  targetYaw = normalizeAngle(northYaw + (float)HEADING_YAW[headingIndex]);
  if (fabs(normalizeAngle(targetYaw - yaw())) > TURN_COAST_ZONE_DEG + 0.5f) {
    // A timed-out turn used to be silently accepted and the robot then drove
    // a whole cell badly rotated. Settle and retry once instead.
    if (!turnTo(targetYaw, status)) {
      ++turnRetries;
      waitStopped(120, status);
      turnTo(targetYaw, status);
    }
  }
}

// Drives `cells` cells as ONE continuous profiled move. This is where most
// of the lap time is won: "fff" is a single 540 mm run, not three cells.
bool driveCells(uint8_t cells, const char *status) {
  resetEncoders();
  // The last readings belong to the heading before the turn. A stale front
  // value below the anchor threshold would abort on the first loop.
  leftMm = 999;
  rightMm = 999;
  prevLeftMm = 999;
  prevRightMm = 999;
  frontMm = stableRange(lidarFront);
  prevFrontMm = frontMm;
  lastLidarMs = millis();
  lidarPhase = 0;

  const float total = (float)cells * CELL_MM;
  float setpoint = 0.0f;
  float lastDistance = 0.0f, filteredSpeed = 0.0f;
  float distanceOffset = 0.0f;
  bool leftWallSeen = false, rightWallSeen = false;
  uint8_t recoveries = 0;
  unsigned long lastMs = millis(), settleStart = 0, stuckSince = 0;
  unsigned long startMs = lastMs;
  driveTracker.reset();

  while (true) {
    unsigned long now = millis();
    float dt = constrain((now - lastMs) / 1000.0f, 0.001f, 0.05f);
    lastMs = now;
    float distance = travelledMm() + distanceOffset;
    float speed = (distance - lastDistance) / dt;
    lastDistance = distance;
    filteredSpeed += 0.25f * (speed - filteredSpeed);
    updateLidarsRoundRobin(LIDAR_DRIVE_INTERVAL_MS);

    // Wall-edge odometry sync. Presence needs an agreed pair; absence needs
    // the raw reading beyond wall range too, so a rejected spike cannot
    // fake an edge.
    if (distance > 40.0f) {
      int8_t edgeEvent = 0;
      if (!leftWallSeen && sideWall(leftMm)) { leftWallSeen = true; edgeEvent = 1; }
      else if (leftWallSeen && leftMm == 999
               && prevLeftMm > SIDE_WALL_MAX_MM + 10) { leftWallSeen = false; edgeEvent = 1; }
      if (!rightWallSeen && sideWall(rightMm)) { rightWallSeen = true; edgeEvent = 1; }
      else if (rightWallSeen && rightMm == 999
               && prevRightMm > SIDE_WALL_MAX_MM + 10) { rightWallSeen = false; edgeEvent = 1; }
      if (edgeEvent) {
        float boundary = round((distance - 90.0f) / CELL_MM) * CELL_MM + 90.0f;
        float correction = boundary - distance;
        if (fabs(correction) <= EDGE_SYNC_WINDOW_MM) {
          float step = EDGE_SYNC_BLEND * correction;
          if (fabs(distanceOffset + step) <= EDGE_SYNC_OFFSET_MAX_MM) {
            distanceOffset += step;
            distance += step;
            lastDistance += step;   // keep the speed estimate continuous
          }
        }
      }
    }

    // A wall at the nominal gap is an absolute distance fix. Only accepted
    // near the end of the run: three cells out the wall is still 600 mm away
    // so anything the front sensor reports before then is not the finish.
    bool front = frontWall(frontMm);
    bool nearEnd = total - distance <= (float)FRONT_WALL_MAX_MM;
    if (nearEnd && front && frontMm <= WALL_GAP_MM) break;
    // Stop as soon as the distance is done. A hold-position settle phase
    // let the tracker reverse and re-push against static friction, which
    // shook the robot at every boundary.
    if (distance >= total - CELL_TOLERANCE_MM) break;
    if (now - startMs >= DRIVE_TIMEOUT_MS * cells) break;

    // Stall watchdog: commanded but not moving means wedged.
    if (setpoint - distance > STALL_LAG_MM
        && fabs(filteredSpeed) < STALL_SPEED_MM_S) {
      if (stuckSince == 0) stuckSince = now;
      if (now - stuckSince >= STALL_DETECT_MS) {
        if (recoveries >= STALL_RECOVERY_MAX) break;
        ++recoveries;
        stuckSince = 0;
        stopMotors();
        delay(80);
        float backTo = travelledMm() - STALL_BACKOFF_MM;
        if (backTo < 3.0f) backTo = 3.0f;   // |net| odometry: never chase past zero
        activeDeadband = MOTOR_DEADBAND;    // breaking loose from standstill
        unsigned long backStart = millis();
        while (travelledMm() > backTo && millis() - backStart < 700) {
          setControlledWheels(9.0f, 9.0f);  // positive args = reverse
          drawTelemetry(status);
          delay(10);
        }
        stopMotors();
        delay(80);
        sampleLidars();
        driveTracker.reset();
        filteredSpeed = 0.0f;
        lastDistance = travelledMm() + distanceOffset;
        setpoint = max(0.0f, lastDistance);
        lastMs = millis();
        continue;
      }
    } else stuckSince = 0;

    // Creep up to a visible front wall instead of slamming on the brakes at
    // the 52 mm threshold; crawl while a side wall is very close.
    float cruise = DRIVE_SPEED_MAX;
    if (front && nearEnd) {
      cruise = constrain(FRONT_APPROACH_GAIN * ((float)frontMm - WALL_GAP_MM),
                         DRIVE_SPEED_MIN, DRIVE_SPEED_MAX);
    }
    if ((leftMm != 999 && leftMm < SIDE_SLOW_MM)
        || (rightMm != 999 && rightMm < SIDE_SLOW_MM)) {
      cruise = min(cruise, SIDE_SLOW_SPEED);
    }
    float rate = profileRate(setpoint, total, cruise,
                             DRIVE_ACCEL, DRIVE_SPEED_MIN);
    bool profileDone = setpoint >= total;
    if (!profileDone) setpoint = min(setpoint + rate * dt, total);
    else { rate = 0.0f; if (settleStart == 0) settleStart = now; }
    if (profileDone && now - settleStart >= SETTLE_TIMEOUT_MS) break;

    float currentYaw = yaw();
    float yawRate = gyroRateDegPerSec();
    float forward = driveTracker.update(setpoint - distance, rate,
                                        filteredSpeed, dt);
    if (forward < 0.0f) forward = 0.0f;   // never reverse inside a move

    // Classic micromouse correction, one layer, no mode memory: walls
    // visible -> centre on the physical corridor; no walls -> hold the grid
    // heading by gyro. Both damped by the gyro rate.
    bool left = sideWall(leftMm), right = sideWall(rightMm);
    float correction;
    if (left || right) {
      float wallError = (left && right)
        ? 0.5f * ((float)leftMm - (float)rightMm)
        : left ? (float)leftMm - WALL_GAP_MM
               : WALL_GAP_MM - (float)rightMm;
      float magnitude = fabs(wallError);
      if (magnitude > 12.0f) magnitude = 12.0f + 1.8f * (magnitude - 12.0f);
      correction = (wallError >= 0.0f ? 1.0f : -1.0f)
                     * WALL_CENTRE_KP * magnitude
                   - RATE_DAMPING * yawRate;
    } else {
      correction = HEADING_HOLD_KP * normalizeAngle(targetYaw - currentYaw)
                   - RATE_DAMPING * yawRate;
    }
    correction = constrain(correction,
                           -(float)CORRECTION_MAX, (float)CORRECTION_MAX);
    // With forward at zero the per-wheel deadband turned any differential
    // into a hard in-place twitch. Correction is only allowed in proportion
    // to the drive, so a pivot is impossible by construction.
    correction *= constrain(forward / 8.0f, 0.0f, 1.0f);
    updateActiveDeadband(fabs(filteredSpeed));
    setControlledWheels(-forward + correction, -forward - correction);
    drawTelemetry(status);
    delay(10);
  }
  stopMotors();
  // 45 mm: tolerant of odometry error over a long chain, still far less than
  // one cell, so stopping a whole cell early is always a fault.
  return travelledMm() + distanceOffset >= total - 45.0f;
}

// --------------------------- Route --------------------------
void runRoute() {
  uint8_t count = strlen(COMMANDS);
  uint8_t index = 0;
  char status[17];

  northYaw = yaw();
  headingIndex = 0;
  targetYaw = northYaw;
  sampleLidars();

  while (index < count) {
    char command = COMMANDS[index];

    if (command == 'f') {
      uint8_t cells = 0;
      while (index + cells < count && COMMANDS[index + cells] == 'f') ++cells;
      snprintf(status, sizeof(status), "RUN %uC %u/%u",
               (unsigned)cells, (unsigned)(index + 1), (unsigned)count);
      if (!driveCells(cells, status)) {
        routeFailed = true;
        return;
      }
      index += cells;
    } else if (command == 'l' || command == 'r') {
      uint8_t next = command == 'l' ? (headingIndex + 3) & 3
                                    : (headingIndex + 1) & 3;
      snprintf(status, sizeof(status), "TURN %c %u/%u",
               command, (unsigned)(index + 1), (unsigned)count);
      turnToHeading(next, status);
      ++index;
    } else {
      ++index;
      continue;
    }

    if (index < count) waitStopped(ACTION_PAUSE_MS, "NEXT");
  }
}

// --------------------------- Main ---------------------------
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

  pinMode(LIDAR_LEFT_XSHUT, OUTPUT);
  pinMode(LIDAR_FRONT_XSHUT, OUTPUT);
  pinMode(LIDAR_RIGHT_XSHUT, OUTPUT);
  digitalWrite(LIDAR_LEFT_XSHUT, LOW);
  digitalWrite(LIDAR_FRONT_XSHUT, LOW);
  digitalWrite(LIDAR_RIGHT_XSHUT, LOW);
  delay(20);

  Wire.begin();
  Wire.setClock(400000);
  oled.begin();

  mpu.begin();
  oled.drawLine(0, "KEEP STILL");
  delay(1000);
  mpu.calcOffsets(true, true);
  measureGyroBias();

  startLidar(lidarLeft, LIDAR_LEFT_XSHUT, ADDR_LEFT);
  startLidar(lidarFront, LIDAR_FRONT_XSHUT, ADDR_FRONT);
  startLidar(lidarRight, LIDAR_RIGHT_XSHUT, ADDR_RIGHT);

  Serial.println(F("Week 12 Task 4.1.2 race build"));
  Serial.print(F("Commands: "));
  Serial.println(COMMANDS);
  Serial.print(F("Cruise mm/s: "));
  Serial.println(DRIVE_SPEED_MAX, 0);
  Serial.print(F("Gyro bias deg/s: "));
  Serial.println(gyroBias, 4);
}

void loop() {
  static bool started = false;

  if (!started) {
    started = true;
    unsigned long began = millis();
    runRoute();
    stopMotors();
    Serial.print(F("Route finished in "));
    Serial.print((millis() - began) / 1000.0f, 2);
    Serial.println(F(" s"));
    Serial.print(F("Turn retries: "));
    Serial.println(turnRetries);
    if (routeFailed) Serial.println(F("A run stopped short: check the path."));
    oled.drawLine(0, routeFailed ? "ROUTE CHECK" : "ROUTE DONE");
  }

  stopMotors();
  drawTelemetry(routeFailed ? "ROUTE CHECK" : "ROUTE DONE");
  delay(20);
}
