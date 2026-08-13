#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <string.h>

// PID_MIGRATED: trajectory PID ported from pid_validation.ino.
// The original week12_4_2_waypoint_runner.ino remains unchanged.

// Week 12 Task 4.2 hybrid route: GRID -> CONTINUOUS 5x5 -> GRID.
// T = whole-degree turn, G = normal grid cell with wall following,
// F = straight distance used only inside the cylindrical obstacle course.
// Positive turns are left, negative turns are right.
//
// The controller keeps three separate headings so that a local correction can
// never rotate the planned route:
//   northYaw   fixed grid reference captured at BEGIN, trimmed only by walls,
//   routeYaw   northYaw plus the exact sum of every T token,
//   headingCmd routeYaw plus the temporary lean used to rejoin the track.
// Position error is carried between segments, so the robot always drives back
// onto the planned line instead of continuing parallel to it.

// ================= PASTE GENERATED ROUTE BELOW =================
// Example: #define GENERATED_ROUTE "BEGIN;T-90;F300;END;"

// ================== PASTE GENERATED ROUTE ABOVE =================
#ifndef GENERATED_ROUTE

#define GENERATED_ROUTE "BEGIN;G180;G180;G180;T90;G180;T37;F900;T-37;G180;T90;G180;G180;T90;END;"

#endif
const char ROUTE_DATA[] PROGMEM = GENERATED_ROUTE;

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

// ------------------------- Geometry -------------------------
const float CELL_MM = 180.0f;
const float ROBOT_SIZE_MM = 76.0f;
const float WALL_GAP_MM = (CELL_MM - ROBOT_SIZE_MM) / 2.0f;
const float CORRIDOR_SUM_MM = 2.0f * WALL_GAP_MM;
const float WHEEL_DIAMETER_MM = 44.0f;
const float ENCODER_COUNTS_PER_REV = 356.72f;
// Calibration: the old conversion made a commanded distance run about 10% long.
const float ENCODER_DISTANCE_SCALE = 1.10f;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_REV
                           * ENCODER_DISTANCE_SCALE;
const float DISTANCE_TOLERANCE_MM = 1.0f;

// ---------------------- Trajectory PID ----------------------
// These are deliberately softer than pid_validation.ino: braking starts
// earlier, the integral is tightly bounded, and rate damping is stronger.
const float TURN_RATE_MAX = 48.0f;
const float TURN_ACCEL = 82.0f;
const float TURN_RATE_MIN = 5.0f;
const float TURN_KV = 0.20f;
const float TURN_KP = 0.88f;
const float TURN_KI = 0.18f;
const float TURN_KD = 0.16f;
const float TURN_I_LIMIT = 4.0f;
const float TURN_OUT_LIMIT = 12.0f;

const float GRID_SPEED_MAX = 72.0f;
const float COURSE_SPEED_MAX = 66.0f;
const float RAW_SPEED_MAX = 52.0f;
const float DRIVE_ACCEL = 115.0f;
const float DRIVE_SPEED_MIN = 6.0f;
const float DRIVE_KV = 0.29f;
const float DRIVE_KP = 0.42f;
const float DRIVE_KI = 0.10f;
const float DRIVE_KD = 0.06f;
const float DRIVE_I_LIMIT = 7.0f;
const float DRIVE_OUT_LIMIT = 27.0f;

const float HEADING_KP = 1.35f;
const float HEADING_KI = 0.18f;
const float HEADING_KD = 0.14f;
const float HEADING_I_LIMIT = 5.0f;
const float HEADING_OUT_LIMIT = 12.0f;
const float DRIVE_YAW_DEADBAND = 0.35f;
const float DRIVE_ENCODER_KP = 0.03f;
const float DRIVE_ENCODER_MAX = 2.5f;
const int DRIVE_CORRECTION_MAX = 10;
const int COURSE_CORRECTION_MAX = 12;
const int MOTOR_DEADBAND = 18;
const int MOTOR_MAX = 60;
const float DEADBAND_BLEND = 4.0f;
const float DRIVE_SETTLE_RATE = 7.0f;
const unsigned long SETTLE_TIMEOUT_MS = 1800UL;

// ------------------- Track rejoin (cross) -------------------
// The lean that pulls the robot back onto the planned line.
const float CROSS_LOOKAHEAD_MM = 150.0f;
const float CROSS_GAIN = 0.9f;
const float CROSS_MAX_GRID_DEG = 10.0f;
const float CROSS_MAX_COURSE_DEG = 16.0f;
const float CROSS_LIMIT_MM = 70.0f;
const float CARRY_LIMIT_MM = 45.0f;
const float CROSS_WALL_TRUST = 0.12f;
const float CROSS_HOLD_FADE_MM = 260.0f;
const float HEADING_LEAN_MAX_DEG = 28.0f;

// --------------------- Wall observations --------------------
const uint16_t SIDE_WALL_MIN_MM = 20;
const uint16_t SIDE_WALL_MAX_MM = 90;
const float WALL_HEADING_WINDOW_MM = 70.0f;
const float WALL_HEADING_GAIN = 0.45f;
const float WALL_HEADING_MAX_DEG = 1.5f;
const float WALL_MEASURE_MAX_MM = 45.0f;
const float WALL_SINGLE_MAX_MM = 30.0f;
const float CORRIDOR_SUM_TOLERANCE_MM = 26.0f;
// A wall angle that persists is gyro drift, so a share of it is folded back
// into the grid reference. Both the per-cell step and the total are bounded.
const float NORTH_TRIM_SHARE = 0.35f;
const float NORTH_TRIM_STEP_DEG = 1.2f;
const float NORTH_TRIM_TOTAL_DEG = 12.0f;

// ------------------------- Course safety --------------------
// The planner inflates a cylinder by its own radius plus 35 mm but does not add
// the robot radius, so a planned line can pass a cylinder with almost no gap.
// The side push therefore has to be real avoidance, not just a warning.
const uint16_t COURSE_SIDE_CAUTION_MM = 60;
const uint16_t COURSE_SIDE_STOP_MM = 18;
const uint16_t COURSE_FRONT_CAUTION_MM = 80;
const uint16_t COURSE_FRONT_STOP_MM = 35;
// Close to the end of a segment the object ahead is usually the wall the
// segment aims at, so the robot only slows down instead of steering aside.
const float COURSE_FRONT_STEER_MIN_MM = 120.0f;
const float COURSE_AVOID_DEG_PER_MM = 0.35f;
const float COURSE_AVOID_MAX_DEG = 18.0f;
const float COURSE_SLOW_YAW_DEG = 6.0f;
const float COURSE_REALIGN_DEG = 2.0f;
const uint8_t MAX_RECOVERIES = 4;
const float RECOVERY_BACK_MM = 55.0f;
const float RECOVERY_SIDE_DEG = 35.0f;
const float RECOVERY_SIDE_MM = 90.0f;
const float JAM_BACK_MM = 40.0f;
const uint16_t GRID_FRONT_STOP_MM = 30;
const float GRID_FRONT_DOCK_MM = 80.0f;
const float GRID_ANCHOR_TOLERANCE_MM = 20.0f;

// ---------------------- Stall protection --------------------
// Low PWM cannot always break static friction. When the encoders stop moving
// while an output is commanded, a temporary boost is added until they turn.
const long STALL_TICKS = 4L;
const unsigned long STALL_WINDOW_MS = 220UL;
const float STALL_RISE_PER_SEC = 70.0f;
// The boost is released quickly once the wheels turn, so breaking free gives a
// short kick instead of a fast lurch that overshoots the target angle.
const float STALL_DECAY_PER_SEC = 250.0f;
const float STALL_BOOST_MAX_TURN = 8.0f;
const float STALL_BOOST_MAX_DRIVE = 10.0f;
const unsigned long STALL_ABORT_MS = 1800UL;

// ----------------------------- Turns ------------------------
const float TURN_TOLERANCE_DEG = 1.0f;
const float TURN_ACCEPT_DEG = 4.0f;
const float TURN_GIVEUP_DEG = 10.0f;
const float TURN_SKIP_DEG = 0.8f;
const unsigned long TURN_STABLE_MS = 200UL;
const unsigned long TURN_TIMEOUT_BASE_MS = 2500UL;
const unsigned long TURN_TIMEOUT_PER_DEG_MS = 30UL;
const float TURN_UNJAM_MM = 35.0f;
const unsigned long ACTION_PAUSE_MS = 80UL;

// ---------------------------- Sensing -----------------------
const unsigned long LIDAR_PERIOD_MS = 30UL;
const uint8_t LIDAR_CONVERGENCE_MS = 25;
const unsigned long TELEMETRY_INTERVAL_MS = 45UL;
const uint16_t RANGE_VALID_MIN_MM = 5;
const uint16_t RANGE_VALID_MAX_MM = 200;

MPU6050 mpu(Wire);
VL6180X lidarLeft;
VL6180X lidarFront;
VL6180X lidarRight;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
uint16_t leftMm = 999;
uint16_t frontMm = 999;
uint16_t rightMm = 999;
float yawNow = 0.0f;

// Route frame.
float northYaw = 0.0f;
float routeTurnSum = 0.0f;
float routeYaw = 0.0f;
float northTrimTotal = 0.0f;

// Position error relative to the planned track, in the active segment frame.
float segmentYaw = 0.0f;
float alongMm = 0.0f;
float crossMm = 0.0f;
float carryAlongMm = 0.0f;
float carryCrossMm = 0.0f;
float lastTravelMm = 0.0f;
float crossHoldMm = 0.0f;
float crossHoldFadeMm = 0.0f;

// Stall watch.
long stallTicks = 0;
unsigned long stallMovedMs = 0;
float stallBoost = 0.0f;

bool routeActive = false;
bool routeFinished = false;

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

// ------------------------- Hardware -------------------------
void leftEncoderISR() {
  leftTicks += digitalRead(LEFT_ENC_B) ? 1 : -1;
}

void rightEncoderISR() {
  rightTicks += digitalRead(RIGHT_ENC_B) ? 1 : -1;
}

void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

long encoderTotal() {
  noInterrupts();
  long left = leftTicks;
  long right = rightTicks;
  interrupts();
  return labs(left) + labs(right);
}

float travelledMm() {
  noInterrupts();
  long left = leftTicks;
  long right = rightTicks;
  interrupts();
  return 0.5f * (labs(left) + labs(right)) * MM_PER_COUNT;
}

float encoderCorrection() {
  noInterrupts();
  long left = leftTicks;
  long right = rightTicks;
  interrupts();
  float tickError = (float)labs(left) - (float)labs(right);
  return constrain(DRIVE_ENCODER_KP * tickError,
                   -DRIVE_ENCODER_MAX, DRIVE_ENCODER_MAX);
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float yaw() {
  mpu.update();
  yawNow = mpu.getAngleZ();
  return yawNow;
}

void setMotor(uint8_t pwmPin, uint8_t dirPin, int command) {
  command = constrain(command, -255, 255);
  digitalWrite(dirPin, command >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(command));
}

void setWheels(float left, float right) {
  setMotor(LEFT_PWM, LEFT_DIR, (int)round(-left));
  setMotor(RIGHT_PWM, RIGHT_DIR, (int)round(right));
}

void stopMotors() {
  setWheels(0, 0);
}

float withDeadband(float command) {
  float magnitude = fabs(command);
  if (magnitude < 0.35f) return 0.0f;
  float blend = min(1.0f, magnitude / DEADBAND_BLEND);
  float output = min(MOTOR_DEADBAND * blend + magnitude, (float)MOTOR_MAX);
  return command > 0.0f ? output : -output;
}

void setControlledWheels(float left, float right) {
  setWheels(withDeadband(left), withDeadband(right));
}

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
Tracker headingTracker = {0.0f, HEADING_KP, HEADING_KI, HEADING_KD,
                          HEADING_I_LIMIT, HEADING_OUT_LIMIT, 0.0f};

float profileRate(float done, float total, float cruise,
                  float acceleration, float creep) {
  float rampUp = sqrt(2.0f * acceleration * max(0.0f, done) + creep * creep);
  float rampDown = sqrt(2.0f * acceleration * max(0.0f, total - done)
                        + creep * creep);
  return constrain(min(rampUp, rampDown), creep, cruise);
}

// ---------------------------- Lidar -------------------------
// Continuous ranging with one sensor read per control cycle. Single-shot
// ranging blocked the loop for about 30 ms per cycle, which was long enough
// for the robot to reach a cylinder before the reading that showed it arrived.
const uint8_t LIDAR_ORDER[4] = {1, 0, 1, 2};
uint8_t lidarStep = 0;
unsigned long lidarDueMs[3] = {0UL, 0UL, 0UL};

void startLidar(VL6180X &sensor, uint8_t xshut, uint8_t address) {
  digitalWrite(xshut, HIGH);
  delay(50);
  sensor.setTimeout(50);
  sensor.init();
  sensor.configureDefault();
  sensor.setAddress(address);
  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, LIDAR_CONVERGENCE_MS);
  sensor.startRangeContinuous((uint16_t)LIDAR_PERIOD_MS);
}

bool rangeReady(VL6180X &sensor) {
  return (sensor.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO) & 0x07) == 0x04;
}

uint16_t readReady(VL6180X &sensor, uint16_t previous) {
  if (!rangeReady(sensor)) return previous;
  uint16_t value = sensor.readRangeContinuousMillimeters();
  return sensor.timeoutOccurred() ? 999 : value;
}

void updateLidars() {
  uint8_t which = LIDAR_ORDER[lidarStep];
  lidarStep = (lidarStep + 1) & 3;
  unsigned long now = millis();
  if ((long)(now - lidarDueMs[which]) < 0) return;
  lidarDueMs[which] = now + LIDAR_PERIOD_MS;
  if (which == 0) leftMm = readReady(lidarLeft, leftMm);
  else if (which == 1) frontMm = readReady(lidarFront, frontMm);
  else rightMm = readReady(lidarRight, rightMm);
}

bool validRange(uint16_t value) {
  return value >= RANGE_VALID_MIN_MM && value <= RANGE_VALID_MAX_MM;
}

bool isSideWall(uint16_t value) {
  return value >= SIDE_WALL_MIN_MM && value <= SIDE_WALL_MAX_MM;
}

// ----------------------- Stall watchdog ---------------------
void stallReset() {
  stallTicks = encoderTotal();
  stallMovedMs = millis();
  stallBoost = 0.0f;
}

float stallUpdate(float dt, float maximumBoost) {
  unsigned long now = millis();
  long total = encoderTotal();
  if (total - stallTicks >= STALL_TICKS) {
    stallTicks = total;
    stallMovedMs = now;
    stallBoost = max(0.0f, stallBoost - STALL_DECAY_PER_SEC * dt);
  } else if (now - stallMovedMs >= STALL_WINDOW_MS) {
    stallBoost = min(maximumBoost, stallBoost + STALL_RISE_PER_SEC * dt);
  }
  return stallBoost;
}

bool stallJammed() {
  return millis() - stallMovedMs >= STALL_ABORT_MS;
}

// ------------------------- Track frame ----------------------
void odometryRebase() {
  resetEncoders();
  lastTravelMm = 0.0f;
}

void odometrySetSegment(float yawTarget, float startAlong, float startCross) {
  segmentYaw = yawTarget;
  alongMm = constrain(startAlong, -CARRY_LIMIT_MM, CARRY_LIMIT_MM);
  crossMm = constrain(startCross, -CROSS_LIMIT_MM, CROSS_LIMIT_MM);
  crossHoldMm = 0.0f;
  crossHoldFadeMm = 0.0f;
  odometryRebase();
}

// Integrates one encoder increment into the segment frame. Positive cross is
// left of the planned line, so every avoidance move stays measurable.
void odometryStep(int direction, float currentYaw) {
  float travel = travelledMm();
  float ds = (travel - lastTravelMm) * (float)direction;
  lastTravelMm = travel;
  if (ds == 0.0f) return;
  float psi = radians(normalizeAngle(currentYaw - segmentYaw));
  alongMm += ds * cos(psi);
  crossMm += ds * sin(psi);
  crossMm = constrain(crossMm, -CROSS_LIMIT_MM, CROSS_LIMIT_MM);
  if (crossHoldFadeMm > 0.0f) {
    crossHoldFadeMm = max(0.0f, crossHoldFadeMm - fabs(ds));
  }
}

// A turn keeps the position error but rotates the frame it is expressed in.
void rotateCarry(float turnDeg) {
  float radiansTurn = radians(turnDeg);
  float along = carryAlongMm;
  float cross = carryCrossMm;
  carryAlongMm = constrain(along * cos(radiansTurn) + cross * sin(radiansTurn),
                           -CARRY_LIMIT_MM, CARRY_LIMIT_MM);
  carryCrossMm = constrain(-along * sin(radiansTurn) + cross * cos(radiansTurn),
                           -CARRY_LIMIT_MM, CARRY_LIMIT_MM);
}

float crossHoldTarget() {
  if (crossHoldFadeMm <= 0.0f) return 0.0f;
  return crossHoldMm * (crossHoldFadeMm / CROSS_HOLD_FADE_MM);
}

// -------------------------- Telemetry -----------------------
char statusText[17] = "";
uint8_t telemetryRow = 1;
unsigned long lastDisplayMs = 0;

void drawTelemetry(const char *status, float error, float remaining) {
  unsigned long now = millis();
  if (strncmp(statusText, status, 16) != 0) {
    strncpy(statusText, status, 16);
    statusText[16] = '\0';
    oled.drawLine(0, statusText);
    lastDisplayMs = now;
    telemetryRow = 1;
    return;
  }
  if (now - lastDisplayMs < TELEMETRY_INTERVAL_MS) return;
  lastDisplayMs = now;

  // One row per call keeps the display cost near 3 ms instead of 25 ms.
  char line[17];
  switch (telemetryRow) {
    case 1:
      snprintf(line, sizeof(line), "Z:%5ld R:%4ld", lround(yawNow), lround(routeYaw));
      break;
    case 2:
      snprintf(line, sizeof(line), "E:%5ld D:%4ld", lround(error), lround(remaining));
      break;
    case 3:
      snprintf(line, sizeof(line), "A:%5ld C:%4ld", lround(alongMm), lround(crossMm));
      break;
    case 4:
      snprintf(line, sizeof(line), "L:%4u F:%4u", leftMm, frontMm);
      break;
    default:
      snprintf(line, sizeof(line), "R:%4u N:%4ld", rightMm, lround(northTrimTotal));
      break;
  }
  oled.drawLine(telemetryRow, line);
  telemetryRow = telemetryRow >= 5 ? 1 : telemetryRow + 1;
}

void waitStopped(unsigned long duration, const char *status) {
  stopMotors();
  unsigned long start = millis();
  while (millis() - start < duration) {
    updateLidars();
    yaw();
    drawTelemetry(status, 0.0f, 0.0f);
    delay(10);
  }
}

void beginSensors() {
  pinMode(LIDAR_LEFT_XSHUT, OUTPUT);
  pinMode(LIDAR_FRONT_XSHUT, OUTPUT);
  pinMode(LIDAR_RIGHT_XSHUT, OUTPUT);
  digitalWrite(LIDAR_LEFT_XSHUT, LOW);
  digitalWrite(LIDAR_FRONT_XSHUT, LOW);
  digitalWrite(LIDAR_RIGHT_XSHUT, LOW);
  delay(20);

  mpu.begin();
  oled.drawLine(0, "KEEP STILL");
  delay(1000);
  mpu.calcOffsets(true, true);

  startLidar(lidarLeft, LIDAR_LEFT_XSHUT, ADDR_LEFT);
  startLidar(lidarFront, LIDAR_FRONT_XSHUT, ADDR_FRONT);
  startLidar(lidarRight, LIDAR_RIGHT_XSHUT, ADDR_RIGHT);
  for (uint8_t i = 0; i < 12; ++i) {
    updateLidars();
    delay(12);
  }
}

// ----------------------- Robot actions ----------------------
bool driveRaw(float distanceMm, int direction, float yawTarget, const char *status);

// Moving-setpoint turn from pid_validation. The profile slows before the target
// and the rate term damps the last few degrees instead of driving through them.
bool turnTo(float targetYawDeg, const char *status) {
  float currentYaw = yaw();
  float error = normalizeAngle(targetYawDeg - currentYaw);
  if (fabs(error) <= TURN_SKIP_DEG) {
    odometryRebase();
    return true;
  }

  float direction = error >= 0.0f ? 1.0f : -1.0f;
  float total = fabs(error);
  float setpoint = 0.0f;
  float actual = 0.0f;
  float lastYawValue = currentYaw;
  unsigned long limitMs = TURN_TIMEOUT_BASE_MS
                          + (unsigned long)(total * TURN_TIMEOUT_PER_DEG_MS);
  unsigned long lastMs = millis();
  unsigned long startMs = lastMs;
  unsigned long stableSince = 0;
  bool unjammed = false;
  turnTracker.reset();
  stallReset();

  while (true) {
    unsigned long now = millis();
    float dt = constrain((now - lastMs) / 1000.0f, 0.001f, 0.05f);
    lastMs = now;

    currentYaw = yaw();
    float yawStep = normalizeAngle(currentYaw - lastYawValue);
    lastYawValue = currentYaw;
    actual += yawStep * direction;
    float yawRate = yawStep * direction / dt;
    error = normalizeAngle(targetYawDeg - currentYaw);

    float rate = profileRate(setpoint, total, TURN_RATE_MAX,
                             TURN_ACCEL, TURN_RATE_MIN);
    bool profileDone = setpoint >= total;
    if (!profileDone) setpoint = min(setpoint + rate * dt, total);
    else rate = 0.0f;

    float command = turnTracker.update(setpoint - actual, rate, yawRate, dt)
                    * direction;
    float boost = stallUpdate(dt, STALL_BOOST_MAX_TURN);
    if (fabs(command) >= 0.35f && boost > 0.0f) {
      command += command > 0.0f ? boost : -boost;
    }
    setControlledWheels(command, -command);

    updateLidars();
    drawTelemetry(status, error, 0.0f);

    if (profileDone) {
      bool settled = fabs(total - actual) <= TURN_TOLERANCE_DEG
                     && fabs(yawRate) <= 7.0f;
      if (settled) {
        if (stableSince == 0) stableSince = now;
        if (now - stableSince >= TURN_STABLE_MS) {
          stopMotors();
          odometryRebase();
          return true;
        }
      } else stableSince = 0;
    }

    if (now - startMs > limitMs) {
      stopMotors();
      if (fabs(error) <= TURN_ACCEPT_DEG) {
        odometryRebase();
        return true;
      }
      if (!unjammed) {
        // Probably wedged against a wall or a cylinder: back off and retry once.
        unjammed = true;
        driveRaw(TURN_UNJAM_MM, -1, currentYaw, "UNJAM");
        startMs = millis();
        lastMs = startMs;
        currentYaw = yaw();
        error = normalizeAngle(targetYawDeg - currentYaw);
        direction = error >= 0.0f ? 1.0f : -1.0f;
        total = fabs(error);
        setpoint = 0.0f;
        actual = 0.0f;
        lastYawValue = currentYaw;
        limitMs = TURN_TIMEOUT_BASE_MS
                  + (unsigned long)(total * TURN_TIMEOUT_PER_DEG_MS);
        stableSince = 0;
        turnTracker.reset();
        stallReset();
        continue;
      }
      odometryRebase();
      return fabs(error) <= TURN_GIVEUP_DEG;
    }
    delay(8);
  }
}

// Short open-track move used by recoveries. It still feeds the segment frame,
// so the main controller knows exactly how far off the line the detour went.
bool driveRaw(float distanceMm, int direction, float yawTarget, const char *status) {
  odometryRebase();
  stallReset();
  driveTracker.reset();
  headingTracker.reset();
  float setpoint = 0.0f;
  float lastDistance = 0.0f;
  float filteredSpeed = 0.0f;
  float lastYawValue = yaw();
  unsigned long lastMs = millis();
  unsigned long startMs = lastMs;
  unsigned long stableSince = 0;
  unsigned long settleStart = 0;

  while (true) {
    unsigned long now = millis();
    float dt = constrain((now - lastMs) / 1000.0f, 0.001f, 0.10f);
    lastMs = now;

    float currentYaw = yaw();
    odometryStep(direction, currentYaw);
    float distance = travelledMm();
    float speed = (distance - lastDistance) / dt;
    lastDistance = distance;
    filteredSpeed += 0.25f * (speed - filteredSpeed);
    float yawRate = normalizeAngle(currentYaw - lastYawValue) / dt;
    lastYawValue = currentYaw;
    float error = normalizeAngle(yawTarget - currentYaw);

    float rate = profileRate(setpoint, distanceMm, RAW_SPEED_MAX,
                             DRIVE_ACCEL, DRIVE_SPEED_MIN);
    bool profileDone = setpoint >= distanceMm;
    if (!profileDone) setpoint = min(setpoint + rate * dt, distanceMm);
    else { rate = 0.0f; if (settleStart == 0) settleStart = now; }

    float forwardCommand = driveTracker.update(setpoint - distance, rate,
                                                filteredSpeed, dt);
    float boost = stallUpdate(dt, STALL_BOOST_MAX_DRIVE);
    if (forwardCommand > 0.35f) forwardCommand += boost;
    float correction = headingTracker.update(error, 0.0f, yawRate, dt)
                       + encoderCorrection();
    correction = constrain(correction, -8.0f, 8.0f);
    float forward = direction > 0 ? -forwardCommand : forwardCommand;
    setControlledWheels(forward + direction * correction,
                        forward - direction * correction);

    updateLidars();
    drawTelemetry(status, error, distanceMm - distance);

    // A sidestep must not drive blindly into the next obstacle.
    if (direction > 0 && validRange(frontMm) && frontMm <= COURSE_FRONT_STOP_MM) {
      stopMotors();
      odometryRebase();
      return false;
    }
    if (profileDone) {
      bool settled = fabs(distanceMm - distance) <= DISTANCE_TOLERANCE_MM
                     && fabs(filteredSpeed) <= DRIVE_SETTLE_RATE;
      if (settled) {
        if (stableSince == 0) stableSince = now;
        if (now - stableSince >= TURN_STABLE_MS) break;
      } else stableSince = 0;
      if (now - settleStart >= SETTLE_TIMEOUT_MS) break;
    }
    if (stallJammed() || now - startMs > 5000UL) {
      stopMotors();
      odometryRebase();
      return false;
    }
    delay(8);
  }

  stopMotors();
  delay(40);
  odometryRebase();
  return true;
}

// Backs away from an obstacle, steps aside, and hands the lateral offset to the
// track controller so the robot merges back onto the planned line by itself.
bool escapeObstacle(float lineYaw) {
  updateLidars();
  int side = 1;
  bool leftKnown = validRange(leftMm);
  bool rightKnown = validRange(rightMm);
  if (leftKnown && rightKnown) side = leftMm >= rightMm ? 1 : -1;
  else if (leftKnown) side = -1;
  else if (rightKnown) side = 1;

  driveRaw(RECOVERY_BACK_MM, -1, lineYaw, "BACK");
  float escapeYaw = normalizeAngle(lineYaw + side * RECOVERY_SIDE_DEG);
  turnTo(escapeYaw, "AVOID");
  bool moved = driveRaw(RECOVERY_SIDE_MM, 1, escapeYaw, "AVOID");
  turnTo(lineYaw, "REJOIN");
  odometryRebase();

  // Hold the new offset briefly so the return lean does not steer straight back
  // into the obstacle that was just cleared.
  crossHoldMm = crossMm;
  crossHoldFadeMm = CROSS_HOLD_FADE_MM;
  return moved;
}

// Drives one planned segment. followWall selects grid behaviour (wall centring
// and front-wall docking); otherwise the continuous course behaviour is used.
bool driveSegment(float targetMm, bool followWall) {
  odometrySetSegment(routeYaw, carryAlongMm, carryCrossMm);
  stallReset();

  float setpoint = constrain(max(0.0f, alongMm), 0.0f, targetMm);
  float filteredSpeed = 0.0f;
  float lastYawValue = yaw();
  float lastAlongMm = alongMm;
  float segmentTrim = 0.0f;
  float wallReferenceMm = 0.0f;
  float wallReferenceAlong = 0.0f;
  int8_t trackedWall = 0;
  uint8_t recoveries = 0;
  bool frontAnchored = false;
  unsigned long lastControlMs = millis();
  unsigned long stableSince = 0;
  unsigned long settleStart = 0;
  driveTracker.reset();
  headingTracker.reset();

  while (true) {
    unsigned long now = millis();
    float dt = constrain((now - lastControlMs) / 1000.0f, 0.001f, 0.10f);
    lastControlMs = now;

    float currentYaw = yaw();
    odometryStep(1, currentYaw);
    updateLidars();

    float remaining = targetMm - alongMm;
    float speed = (alongMm - lastAlongMm) / dt;
    lastAlongMm = alongMm;
    filteredSpeed += 0.25f * (speed - filteredSpeed);

    if (followWall) {
      bool leftWall = isSideWall(leftMm);
      bool rightWall = isSideWall(rightMm);

      // Absolute lateral fix from the corridor. Both walls together are only
      // trusted when their sum matches the corridor width, which rejects the
      // false readings taken while passing a doorway or a junction.
      float measured = 0.0f;
      bool measuredValid = false;
      if (leftWall && rightWall) {
        float sum = (float)leftMm + (float)rightMm;
        if (fabs(sum - CORRIDOR_SUM_MM) <= CORRIDOR_SUM_TOLERANCE_MM) {
          measured = 0.5f * ((float)rightMm - (float)leftMm);
          measuredValid = fabs(measured) <= WALL_MEASURE_MAX_MM;
        }
      } else if (leftWall) {
        measured = WALL_GAP_MM - (float)leftMm;
        measuredValid = fabs(measured) <= WALL_SINGLE_MAX_MM;
      } else if (rightWall) {
        measured = (float)rightMm - WALL_GAP_MM;
        measuredValid = fabs(measured) <= WALL_SINGLE_MAX_MM;
      }
      if (measuredValid) {
        crossMm += CROSS_WALL_TRUST * (measured - crossMm);
      }

      // Wall angle over a fixed baseline gives the heading error directly.
      int8_t currentWall = 0;
      float currentWallMm = 0.0f;
      if (trackedWall > 0 && leftWall) { currentWall = 1; currentWallMm = leftMm; }
      else if (trackedWall < 0 && rightWall) { currentWall = -1; currentWallMm = rightMm; }
      else if (leftWall && (!rightWall || leftMm <= rightMm)) { currentWall = 1; currentWallMm = leftMm; }
      else if (rightWall) { currentWall = -1; currentWallMm = rightMm; }

      if (currentWall == 0 || fabs(currentWallMm - WALL_GAP_MM) > 20.0f) {
        trackedWall = 0;
      } else if (currentWall != trackedWall) {
        trackedWall = currentWall;
        wallReferenceMm = currentWallMm;
        wallReferenceAlong = alongMm;
      } else if (alongMm - wallReferenceAlong >= WALL_HEADING_WINDOW_MM) {
        float travel = alongMm - wallReferenceAlong;
        float change = trackedWall > 0 ? currentWallMm - wallReferenceMm
                                       : wallReferenceMm - currentWallMm;
        if (fabs(change) <= 12.0f) {
          float step = constrain(WALL_HEADING_GAIN * degrees(atan2(change, travel)),
                                 -WALL_HEADING_MAX_DEG, WALL_HEADING_MAX_DEG);
          // Only the segment heading moves here; routeYaw stays untouched so a
          // crooked wall can never rotate the rest of the planned route.
          segmentYaw = normalizeAngle(segmentYaw + step);
          segmentTrim += step;
        }
        wallReferenceMm = currentWallMm;
        wallReferenceAlong = alongMm;
      }
    }

    // ---------------------- termination ----------------------
    bool frontClose = validRange(frontMm) && frontMm <= (uint16_t)WALL_GAP_MM;
    if (followWall && frontClose && remaining <= GRID_FRONT_DOCK_MM) {
      frontAnchored = true;
      break;
    }
    // ------------------------ safety -------------------------
    bool frontStop = validRange(frontMm)
                     && frontMm <= (followWall ? GRID_FRONT_STOP_MM
                                               : COURSE_FRONT_STOP_MM);
    bool sideStop = !followWall
                    && ((validRange(leftMm) && leftMm <= COURSE_SIDE_STOP_MM)
                        || (validRange(rightMm) && rightMm <= COURSE_SIDE_STOP_MM));

    if (followWall && frontStop) {
      stopMotors();
      // A wall this close in a grid cell means the cell is finished early.
      if (remaining <= 0.5f * CELL_MM) {
        frontAnchored = true;
        break;
      }
      return false;
    }

    bool courseBlocked = !followWall && (frontStop || sideStop);
    if (courseBlocked || stallJammed()) {
      stopMotors();
      if (recoveries >= MAX_RECOVERIES) return false;
      ++recoveries;
      if (courseBlocked) {
        escapeObstacle(segmentYaw);
      } else if (!driveRaw(JAM_BACK_MM, -1, segmentYaw, "UNJAM")) {
        // The wheels stopped while an output was commanded and the robot could
        // not even back away. Stepping aside inside a 180 mm corridor would
        // only wedge it harder, so the route reports the blockage instead.
        return false;
      }
      setpoint = constrain(max(0.0f, alongMm), 0.0f, targetMm);
      filteredSpeed = 0.0f;
      trackedWall = 0;
      lastYawValue = yaw();
      lastAlongMm = alongMm;
      lastControlMs = millis();
      stableSince = 0;
      settleStart = 0;
      driveTracker.reset();
      headingTracker.reset();
      stallReset();
      continue;
    }

    // ------------------------- speed -------------------------
    float speedLimit = followWall ? GRID_SPEED_MAX : COURSE_SPEED_MAX;

    float lineError = normalizeAngle(segmentYaw - currentYaw);
    float avoidBias = 0.0f;
    if (!followWall) {
      if (validRange(rightMm) && rightMm < COURSE_SIDE_CAUTION_MM) {
        avoidBias += COURSE_AVOID_DEG_PER_MM * (COURSE_SIDE_CAUTION_MM - rightMm);
      }
      if (validRange(leftMm) && leftMm < COURSE_SIDE_CAUTION_MM) {
        avoidBias -= COURSE_AVOID_DEG_PER_MM * (COURSE_SIDE_CAUTION_MM - leftMm);
      }
      if (validRange(frontMm) && frontMm < COURSE_FRONT_CAUTION_MM) {
        speedLimit = min(speedLimit, RAW_SPEED_MAX);
        if (remaining > COURSE_FRONT_STEER_MIN_MM) {
          float push = 0.6f * COURSE_AVOID_DEG_PER_MM
                       * (COURSE_FRONT_CAUTION_MM - frontMm);
          avoidBias += (leftMm >= rightMm) ? push : -push;
        }
      }
      avoidBias = constrain(avoidBias, -COURSE_AVOID_MAX_DEG, COURSE_AVOID_MAX_DEG);
      if (avoidBias != 0.0f) speedLimit = min(speedLimit, RAW_SPEED_MAX);
      if (fabs(lineError) >= COURSE_SLOW_YAW_DEG) {
        speedLimit = min(speedLimit, RAW_SPEED_MAX);
      }
    }

    float rate = profileRate(setpoint, targetMm, speedLimit,
                             DRIVE_ACCEL, DRIVE_SPEED_MIN);
    bool profileDone = setpoint >= targetMm;
    if (!profileDone) setpoint = min(setpoint + rate * dt, targetMm);
    else { rate = 0.0f; if (settleStart == 0) settleStart = now; }

    float forwardCommand = driveTracker.update(setpoint - alongMm, rate,
                                                filteredSpeed, dt);
    float boost = stallUpdate(dt, STALL_BOOST_MAX_DRIVE);
    if (forwardCommand > 0.35f) forwardCommand += boost;

    // ------------------- rejoin the track --------------------
    float crossError = crossMm - crossHoldTarget();
    float crossMaximum = followWall ? CROSS_MAX_GRID_DEG : CROSS_MAX_COURSE_DEG;
    float crossBias = constrain(
      -degrees(atan2(CROSS_GAIN * crossError, CROSS_LOOKAHEAD_MM)),
      -crossMaximum, crossMaximum);
    // An obstacle outranks the line: the lean back to the track fades out while
    // the avoidance push is active and returns as soon as the way is clear.
    float blend = 1.0f - constrain(fabs(avoidBias) / COURSE_AVOID_MAX_DEG, 0.0f, 1.0f);
    float lean = constrain(avoidBias + blend * crossBias,
                           -HEADING_LEAN_MAX_DEG, HEADING_LEAN_MAX_DEG);
    float headingCommand = normalizeAngle(segmentYaw + lean);

    float error = normalizeAngle(headingCommand - currentYaw);
    float yawRate = normalizeAngle(currentYaw - lastYawValue) / dt;
    lastYawValue = currentYaw;
    float yawPid = fabs(error) < DRIVE_YAW_DEADBAND ? 0.0f
      : headingTracker.update(error, 0.0f, yawRate, dt);

    int correctionLimit = followWall ? DRIVE_CORRECTION_MAX : COURSE_CORRECTION_MAX;
    float correction = constrain(yawPid + encoderCorrection(),
                                 -(float)correctionLimit,
                                 (float)correctionLimit);

    float forward = -forwardCommand;
    setControlledWheels(forward + correction, forward - correction);
    drawTelemetry(followWall ? "GRID" : "COURSE", error, remaining);

    if (profileDone) {
      bool settled = fabs(targetMm - alongMm) <= DISTANCE_TOLERANCE_MM
                     && fabs(filteredSpeed) <= DRIVE_SETTLE_RATE;
      if (settled) {
        if (stableSince == 0) stableSince = now;
        if (now - stableSince >= TURN_STABLE_MS) break;
      } else stableSince = 0;
      if (now - settleStart >= SETTLE_TIMEOUT_MS) break;
    }
    delay(8);
  }

  stopMotors();
  delay(30);

  // A wall angle that survived the whole cell is gyro drift rather than a
  // crooked wall, so a bounded share of it corrects the grid reference itself.
  if (followWall && fabs(segmentTrim) > 0.2f) {
    float share = constrain(NORTH_TRIM_SHARE * segmentTrim,
                            -NORTH_TRIM_STEP_DEG, NORTH_TRIM_STEP_DEG);
    float applied = constrain(northTrimTotal + share,
                              -NORTH_TRIM_TOTAL_DEG, NORTH_TRIM_TOTAL_DEG)
                    - northTrimTotal;
    northTrimTotal += applied;
    northYaw = normalizeAngle(northYaw + applied);
    routeYaw = normalizeAngle(northYaw + routeTurnSum);
  }

  // Hand the remaining error to the next segment instead of losing it. A wall
  // ahead is an absolute distance fix, so encoder slip is cleared every cell.
  // Only a reading close to the expected cell gap is accepted, because the move
  // that enters the course finishes with open space or a cylinder ahead.
  float frontAnchorError = WALL_GAP_MM - (float)frontMm;
  if (followWall && validRange(frontMm)
      && fabs(frontAnchorError) <= GRID_ANCHOR_TOLERANCE_MM) {
    carryAlongMm = frontAnchorError;
  } else if (frontAnchored) {
    carryAlongMm = 0.0f;
  } else {
    carryAlongMm = constrain(alongMm - targetMm, -CARRY_LIMIT_MM, CARRY_LIMIT_MM);
  }
  carryCrossMm = constrain(crossMm, -CARRY_LIMIT_MM, CARRY_LIMIT_MM);

  // Re-square on the planned heading before the next token. On the grid this is
  // what keeps the course entry and exit doorways lined up; an imperfect
  // realignment is reported on the display but never aborts the route.
  float residual = normalizeAngle(routeYaw - yaw());
  if (fabs(residual) >= COURSE_REALIGN_DEG) {
    turnTo(routeYaw, followWall ? "SQUARE" : "REALIGN");
  }
  odometryRebase();
  return true;
}

// ------------------------ Route tokens ----------------------
bool executeToken(char *token) {
  if (strcmp(token, "BEGIN") == 0) {
    routeActive = true;
    routeFinished = false;
    northYaw = round(yaw());
    routeTurnSum = 0.0f;
    northTrimTotal = 0.0f;
    routeYaw = northYaw;
    carryAlongMm = 0.0f;
    carryCrossMm = 0.0f;
    crossHoldMm = 0.0f;
    crossHoldFadeMm = 0.0f;
    odometrySetSegment(routeYaw, 0.0f, 0.0f);
    Serial.println(F("DONE"));
    return true;
  }
  if (strcmp(token, "END") == 0) {
    routeActive = false;
    routeFinished = true;
    stopMotors();
    oled.drawLine(0, "ROUTE FINISHED");
    Serial.println(F("FINISHED"));
    return true;
  }
  if (!routeActive || (token[0] != 'T' && token[0] != 'F' && token[0] != 'G')) {
    Serial.println(F("ERROR"));
    return false;
  }

  long value = atol(token + 1);
  bool success = false;
  if (token[0] == 'T' && value >= -180 && value <= 180) {
    float previous = routeYaw;
    routeTurnSum = normalizeAngle(routeTurnSum + (float)value);
    routeYaw = normalizeAngle(northYaw + routeTurnSum);
    rotateCarry(normalizeAngle(routeYaw - previous));
    success = turnTo(routeYaw, "TURN");
  } else if ((token[0] == 'F' || token[0] == 'G')
             && value >= 15 && value <= 1500) {
    success = driveSegment((float)value, token[0] == 'G');
  } else {
    Serial.println(F("ERROR"));
    return false;
  }

  if (!success) {
    routeActive = false;
    stopMotors();
    oled.drawLine(0, "BLOCKED");
    Serial.println(F("BLOCKED"));
    return false;
  }
  waitStopped(ACTION_PAUSE_MS, "NEXT");
  Serial.println(F("DONE"));
  return true;
}

void runEmbeddedRoute() {
  char token[18];
  uint8_t length = 0;
  for (uint16_t index = 0;; ++index) {
    char c = (char)pgm_read_byte(ROUTE_DATA + index);
    if (c == ';' || c == '\0') {
      if (length > 0) {
        token[length] = '\0';
        if (!executeToken(token)) return;
        length = 0;
      }
      if (c == '\0') return;
    } else if (length < sizeof(token) - 1) {
      token[length++] = c;
    } else {
      Serial.println(F("ERROR"));
      return;
    }
  }
}

// --------------------------- Main ---------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  Wire.begin();
  Wire.setClock(400000);
  oled.begin();
  beginSensors();

  northYaw = round(yaw());
  routeTurnSum = 0.0f;
  routeYaw = northYaw;
  odometrySetSegment(routeYaw, 0.0f, 0.0f);
  oled.drawLine(0, "WAITING ROUTE");
  Serial.println(F("READY"));
  if ((char)pgm_read_byte(ROUTE_DATA) != '\0') {
    runEmbeddedRoute();
  }
}

void loop() {
  static char token[18];
  static uint8_t length = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == ';' || c == '\n' || c == '\r') {
      if (length > 0) {
        token[length] = '\0';
        executeToken(token);
        length = 0;
      }
    } else if (length < sizeof(token) - 1) {
      token[length++] = c;
    } else {
      length = 0;
      Serial.println(F("ERROR"));
    }
  }

  stopMotors();
  updateLidars();
  yaw();
  drawTelemetry(
    routeFinished ? "ROUTE FINISHED" : routeActive ? "ROUTE READY" : "WAITING ROUTE",
    0.0f,
    0.0f
  );
  delay(10);
}
