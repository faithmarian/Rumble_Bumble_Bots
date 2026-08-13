#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <string.h>

// PID_MIGRATED: trajectory PID ported from pid_validation.ino.
// The original autonomous_mapping.ino remains unchanged.

const uint8_t ROWS = 9;
const uint8_t COLUMNS = 9;
const uint8_t CELL_COUNT = ROWS * COLUMNS;

// This is the only maze configuration that needs editing:
//   .  explorable cell       #  blocked cell
//   N/E/S/W  start + heading (upper case)
//   n/e/s/w  goal + heading  (lower case)
// Exactly one start and one goal are required. The twelve physical corner
// cells remain blocked even if they are accidentally changed to '.'.
const char MAZE_LAYOUT[ROWS][COLUMNS + 1] PROGMEM = {
  "#########",
  "#.#####n#",
  "..#####..",
  "..#####..",
  "..#####..",
  ".........",
  "N........",
  "#.......#",
  "##.....##"
};

int8_t START_ROW = -1;
int8_t START_COLUMN = -1;
uint8_t START_HEADING = 0;
int8_t GOAL_ROW = -1;
int8_t GOAL_COLUMN = -1;
uint8_t GOAL_HEADING = 0;
uint8_t NAVIGABLE_CELL_COUNT = 0;

const uint8_t HARD_CORNER_BLOCKS[12][2] PROGMEM = {
  {0,0},{0,1},{1,0}, {0,7},{0,8},{1,8},
  {7,0},{8,0},{8,1}, {7,8},{8,7},{8,8}
};
const int8_t DR[4] = {-1, 0, 1, 0};
const int8_t DC[4] = {0, 1, 0, -1};
const int16_t HEADING_YAW[4] = {0, -90, 180, 90};

// Course PCB pins.
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

// Geometry and trajectory control.
const float CELL_MM = 180.0f;
const float ROBOT_SIZE_MM = 76.0f;
const float WALL_GAP_MM = (CELL_MM - ROBOT_SIZE_MM) / 2.0f;
const float WHEEL_DIAMETER_MM = 44.0f;
const float ENCODER_COUNTS_PER_REV = 356.72f;
// Field test 2026-08-13: cells came up about 8% short, so scale down the
// mm-per-count correction (smaller scale = robot drives further per cell).
const float ENCODER_DISTANCE_SCALE = 1.02f;
const float MM_PER_COUNT = PI * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV
                           * ENCODER_DISTANCE_SCALE;
// Rates near the values pid_validation.ino proved on this chassis. The
// creep speeds sit above the stick-slip threshold: creeping at 5 deg/s or
// 6 mm/s made the wheels stick and jerk, which read as small oscillation.
const float TURN_RATE_MAX = 100.0f;
const float TURN_ACCEL = 160.0f;
const float TURN_RATE_MIN = 12.0f;
const float TURN_KV = 0.35f;
const float TURN_KP = 1.10f;
const float TURN_KI = 0.18f;
const float TURN_KD = 0.16f;
const float TURN_I_LIMIT = 4.0f;
// Must exceed KV * TURN_RATE_MAX or the feedforward alone saturates and
// the feedback hunts against the clamp.
const float TURN_OUT_LIMIT = 26.0f;

const float DRIVE_SPEED_MAX = 100.0f;
// 120 not 160: hard launches slipped the wheels, and slipped counts are
// exactly the odometry error that let a "legal" turn happen one cell
// early - physically inside the virtual fence.
const float DRIVE_ACCEL = 120.0f;
const float DRIVE_SPEED_MIN = 12.0f;
const float DRIVE_KV = 0.36f;
const float DRIVE_KP = 0.50f;
const float DRIVE_KI = 0.10f;
const float DRIVE_KD = 0.06f;
const float DRIVE_I_LIMIT = 7.0f;
const float DRIVE_OUT_LIMIT = 40.0f;

const float HEADING_KP = 1.35f;
const float HEADING_KI = 0.18f;
const float HEADING_KD = 0.14f;
const float HEADING_I_LIMIT = 5.0f;
const float HEADING_OUT_LIMIT = 14.0f;
const int CORRECTION_MAX = 10;
// One-layer classic micromouse correction while driving. Walls visible:
// P on the corridor offset (gain rises past the knee so being far off
// pulls back harder). No walls: P on the grid heading. Both terms damped
// by the raw gyro rate. No steer states, no mode memory, nothing to wind
// up - and the whole correction scales with the forward drive so it can
// never pivot the robot in place.
const float WALL_CENTRE_KP = 0.22f;          // PWM per mm of offset
const float HEADING_HOLD_KP = 1.1f;          // PWM per deg of heading error
const float RATE_DAMPING = 0.10f;            // PWM per deg/s
// Crawl while passing very close to a side wall.
const uint16_t SIDE_SLOW_MM = 34;
const float SIDE_SLOW_SPEED = 45.0f;
// Wall-edge odometry sync: when a side wall appears or ends, the robot is
// physically at a cell boundary (start centre +90 mm + k*180). Snapping
// the measured distance towards that boundary pins the longitudinal
// position to the real maze, so odometry error can no longer accumulate
// into an off-by-one-cell turn (the virtual-fence breach).
const float EDGE_SYNC_WINDOW_MM = 30.0f;
const float EDGE_SYNC_BLEND = 0.5f;
const float EDGE_SYNC_OFFSET_MAX_MM = 60.0f;
// Stall recovery: commanded but not moving -> back off and retry.
const float STALL_LAG_MM = 35.0f;
const float STALL_SPEED_MM_S = 6.0f;
const unsigned long STALL_DETECT_MS = 400;
const uint8_t STALL_RECOVERY_MAX = 2;
const float STALL_BACKOFF_MM = 18.0f;
const uint16_t SIDE_WALL_MAX_MM = 95;
const uint16_t FRONT_WALL_MAX_MM = 120;
// Two consecutive readings must agree before they are believed. Applies
// to all three sensors: a single front spike used to collapse the cruise
// speed mid-corridor, zero the forward drive, and let the deadband turn
// the heading correction into an in-place pivot.
const uint16_t SIDE_AGREE_MM = 15;
const int MOTOR_DEADBAND = 18;
// Once the wheels are already turning, friction is kinetic, not static.
// Injecting the full 18 PWM static kick while cruising made the small-
// signal gain huge and the speed loop surge - the residual shake.
const int MOTOR_DEADBAND_MOVING = 10;
const int MOTOR_MAX = 90;
const float DEADBAND_BLEND = 4.0f;
const float TURN_TOLERANCE_DEG = 1.0f;
// Inside this zone the motors coast instead of chasing the last degree.
// Chasing it through the 18 PWM static-friction kick is what caused the
// permanent buzzing: kick, overshoot, reverse kick, repeat.
const float TURN_COAST_ZONE_DEG = 2.0f;
const float TURN_SETTLE_RATE = 7.0f;
const float CELL_TOLERANCE_MM = 3.0f;
const float FRONT_APPROACH_GAIN = 1.4f;
const unsigned long TURN_STABLE_MS = 150;
const unsigned long SETTLE_TIMEOUT_MS = 1200;
const unsigned long DRIVE_TIMEOUT_MS = 6000;
const unsigned long LIDAR_DRIVE_INTERVAL_MS = 12;
const unsigned long CELL_SETTLE_MS = 45;
const unsigned long ACTION_PAUSE_MS = 50;
const unsigned long PHASE_PAUSE_MS = 150;
const float CORNER_DISTANCE_TOLERANCE_MM = 14.0f;
const uint8_t RANGE_SAMPLES = 3;

MPU6050 mpu(Wire);
VL6180X lidarLeft;
VL6180X lidarFront;
VL6180X lidarRight;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
uint16_t leftMm = 999;
uint16_t frontMm = 999;
uint16_t rightMm = 999;
unsigned long lastDisplayMs = 0;
unsigned long lastLidarMs = 0;

// Each cell packs N/E/S/W walls in the low nibble and known edges in the high nibble.
uint8_t cellMap[ROWS][COLUMNS];
uint8_t visitedBits[(CELL_COUNT + 7) / 8];
// Blocked cells cached as bits: the OLED map renderer tests every interior
// pixel and cannot afford the PROGMEM layout walk each time.
uint8_t blockedBits[(CELL_COUNT + 7) / 8];
int8_t dfsParent[ROWS][COLUMNS];
uint8_t bfsQueue[CELL_COUNT];
uint8_t shortestPath[CELL_COUNT];
uint8_t shortestLength = 0;
uint8_t visitedCount = 0;
int8_t robotRow = START_ROW;
int8_t robotColumn = START_COLUMN;
uint8_t robotHeading = START_HEADING;
float targetYaw = 0.0f;
float northYaw = 0.0f;
char phaseCode = 'B';
bool motionFault = false;
bool lastMoveFrontAnchored = false;

// Small direct-Wire OLED driver: no U8g2 dependency.
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

// Left half (x 0..63): live 64x64 pixel map of the whole maze.
// Right half (x 64..127): eight rows of 8-character telemetry text.
// There is no RAM for a 1 KB framebuffer, so map pixels are computed on
// the fly page by page from cellMap.
const uint8_t OLED_TEXT_COLUMN = 64;

class TinyOLED {
public:
  void begin() {
    const uint8_t init[] = {
      0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,0x20,0x02,
      0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF
    };
    memset(cache, 0, sizeof(cache));
    for (uint8_t i = 0; i < sizeof(init); ++i) command(init[i]);
    uint8_t zeros[16] = {0};
    for (uint8_t page = 0; page < 8; ++page) {
      setRegion(page, 0);
      for (uint8_t start = 0; start < 128; start += 16) writeData(zeros, 16);
    }
  }

  void drawText(uint8_t row, const char *text) {
    char next[9] = {0};
    strncpy(next, text, 8);
    if (memcmp(cache[row], next, sizeof(next)) == 0) return;
    memcpy(cache[row], next, sizeof(next));
    uint8_t pixels[64] = {0};
    for (uint8_t column = 0; column < 8 && next[column]; ++column) {
      uint8_t glyph[5];
      getGlyph(next[column], glyph);
      for (uint8_t i = 0; i < 5; ++i) pixels[column * 8 + i] = glyph[i];
    }
    setRegion(row, OLED_TEXT_COLUMN);
    for (uint8_t start = 0; start < 64; start += 16) writeData(pixels + start, 16);
  }

  void setRegion(uint8_t page, uint8_t column) {
    command(0xB0 | (page & 7));
    command(0x00 | (column & 0x0F));
    command(0x10 | (column >> 4));
  }

  void writeData(const uint8_t *bytes, uint8_t count) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x40);
    for (uint8_t i = 0; i < count; ++i) Wire.write(bytes[i]);
    Wire.endTransmission();
  }

private:
  char cache[8][9];

  void command(uint8_t value) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x00); Wire.write(value);
    Wire.endTransmission();
  }

  void getGlyph(char c, uint8_t glyph[5]) {
    if (c >= 'a' && c <= 'z') c -= 32;
    const uint8_t *source = NULL;
    if (c >= '0' && c <= '9') source = &DIGITS_5X7[(c - '0') * 5];
    if (c >= 'A' && c <= 'Z') source = &LETTERS_5X7[(c - 'A') * 5];
    for (uint8_t i = 0; i < 5; ++i) glyph[i] = source ? pgm_read_byte(source + i) : 0;
    if (c == '-') glyph[0]=glyph[1]=glyph[2]=glyph[3]=glyph[4]=0x08;
    if (c == ':') { glyph[1]=0x36; glyph[2]=0x36; }
    if (c == '.') { glyph[1]=0x60; glyph[2]=0x60; }
    if (c == ',') { glyph[1]=0x40; glyph[2]=0x20; }
    if (c == '?') { glyph[0]=0x02; glyph[1]=0x01; glyph[2]=0x51; glyph[3]=0x09; glyph[4]=0x06; }
    if (c == '/') { glyph[0]=0x20; glyph[1]=0x10; glyph[2]=0x08; glyph[3]=0x04; glyph[4]=0x02; }
    if (c == '%') { glyph[0]=0x23; glyph[1]=0x13; glyph[2]=0x08; glyph[3]=0x64; glyph[4]=0x62; }
  }
};

TinyOLED oled;

void leftEncoderISR() { leftTicks += digitalRead(LEFT_ENC_B) ? 1 : -1; }
void rightEncoderISR() { rightTicks += digitalRead(RIGHT_ENC_B) ? 1 : -1; }

void resetEncoders() {
  noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts();
}

void readEncoders(long &left, long &right) {
  noInterrupts(); left = leftTicks; right = rightTicks; interrupts();
}

// Encoder drift self-correction: every dead-end reversal is a run whose
// physical length is known exactly (52 mm from the wall behind to 52 mm
// from the wall ahead), so the mm-per-count conversion is re-calibrated
// on the fly from those runs.
float mmPerCount = MM_PER_COUNT;
bool anchoredStart = false;      // robot stands wall-referenced on this axis
uint8_t anchorHeading = 0;
float anchorGapMm = 52.0f;       // actual front gap at the anchor stop

float travelledMm() {
  long left, right; readEncoders(left, right);
  return 0.5f * (labs(left) + labs(right)) * mmPerCount;
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// IMU drift self-correction: while the robot stands still the yaw must
// not move, so any change during a mapping stop measures the current gyro
// bias directly. The learned bias is subtracted continuously - drift can
// no longer accumulate between stops.
float gyroBias = 0.0f;             // deg/s, learned at every observation stop
float gyroBiasOffset = 0.0f;       // correction already integrated, deg
unsigned long gyroBiasEpoch = 0;

float yaw() {
  mpu.update();
  return mpu.getAngleZ() - gyroBiasOffset
         - gyroBias * (float)(millis() - gyroBiasEpoch) * 0.001f;
}

// Rate straight from the gyro. Differentiating the fused angle against a
// jittery loop period (I2C lidar + OLED stalls) produced +-20 deg/s of fake
// rate, and the derivative gain turned that noise into motor buzz.
float gyroRateDegPerSec() { return mpu.getGyroZ() - gyroBias; }

// Averages the raw gyro rate over ~80 ms while the robot is stationary.
// (The first version differenced the angle over a 0.3 s window; angle
// noise turned into up to 0.6 deg/s of fake drift per update and slowly
// rotated the whole yaw reference - the robot veered into corners.)
void learnGyroBias() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 16; ++i) {
    mpu.update();
    sum += mpu.getGyroZ();
    delay(5);
  }
  float average = sum / 16.0f;
  // Reject anything that cannot be quiet-stand drift.
  if (fabs(average) > 2.5f || fabs(average - gyroBias) > 1.2f) return;
  gyroBiasOffset += gyroBias * (float)(millis() - gyroBiasEpoch) * 0.001f;
  gyroBiasEpoch = millis();
  gyroBias = constrain(gyroBias + 0.25f * (average - gyroBias), -1.5f, 1.5f);
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

void stopMotors() { setWheels(0, 0); }

int activeDeadband = MOTOR_DEADBAND;  // static at rest, reduced when moving

// Continuous blend between the static and kinetic deadband. The first
// version switched in one step at a threshold; the toggling around that
// threshold injected an 8 PWM square wave and made motion rougher.
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

void startLidar(VL6180X &sensor, uint8_t xshut, uint8_t address) {
  digitalWrite(xshut, HIGH); delay(50);
  sensor.setTimeout(60); sensor.init(); sensor.configureDefault(); sensor.setAddress(address);
}

uint16_t readMm(VL6180X &sensor) {
  uint16_t value = sensor.readRangeSingleMillimeters();
  return sensor.timeoutOccurred() ? 999 : value;
}

void updateLidars(unsigned long intervalMs) {
  unsigned long now = millis();
  if (now - lastLidarMs < intervalMs) return;
  lastLidarMs = now;
  leftMm = readMm(lidarLeft);
  frontMm = readMm(lidarFront);
  rightMm = readMm(lidarRight);
}

// One blocking range read per control tick instead of three in a row.
// Reading all three sensors stalled the drive loop for 30-50 ms, which is
// where the ragged dt (and the shaky heading control) came from.
// Every reading is only believed when two consecutive samples agree, so a
// single spike can neither steer the robot nor fake a front wall.
uint8_t lidarPhase = 0;
uint16_t prevLeftMm = 999;
uint16_t prevRightMm = 999;
uint16_t prevFrontMm = 999;

uint16_t agreedSide(uint16_t value, uint16_t &previous) {
  uint16_t result = 999;
  if (value != 999 && previous != 999) {
    uint16_t diff = value > previous ? value - previous : previous - value;
    if (diff <= SIDE_AGREE_MM) result = value;
  }
  previous = value;
  return result;
}

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
  lastLidarMs = millis();
}

bool sideWall(uint16_t distance) { return distance >= 20 && distance <= SIDE_WALL_MAX_MM; }
bool frontWall(uint16_t distance) { return distance >= 20 && distance <= FRONT_WALL_MAX_MM; }

uint8_t completionPercent() {
  return (uint16_t)visitedCount * 100 / NAVIGABLE_CELL_COUNT;
}

// Bar interior is 54 px wide inside a 56 px outline on page 6.
void drawProgressBar() {
  static uint8_t lastFill = 255;
  uint8_t fill = (uint8_t)(((uint16_t)completionPercent() * 54U) / 100U);
  if (fill == lastFill) return;
  lastFill = fill;
  uint8_t buffer[56];
  buffer[0] = 0xFF;
  buffer[55] = 0xFF;
  for (uint8_t i = 1; i < 55; ++i) buffer[i] = (i <= fill) ? 0xFF : 0x81;
  oled.setRegion(6, 68);
  for (uint8_t start = 0; start < 56; start += 14) oled.writeData(buffer + start, 14);
}

// The display shows only the maze map and the progress: visited count,
// percentage and a bar. The status parameter is kept so the motion code
// stays untouched, but no other text is drawn.
void drawTelemetry(const char *status) {
  (void)status;
  if (millis() - lastDisplayMs < 150) return;
  lastDisplayMs = millis();
  char line[9];
  // Map and progress only - except faults, which must never be silent:
  // an invisible fault looks like the robot freezing for no reason.
  oled.drawText(0, phaseCode == 'F' ? "FAULT"
                : phaseCode == 'X' ? "NO PATH"
                : phaseCode == 'D' ? "DONE" : "");
  snprintf(line, sizeof(line), "%u/%u", visitedCount, NAVIGABLE_CELL_COUNT);
  oled.drawText(2, line);
  snprintf(line, sizeof(line), "%u%%", completionPercent());
  oled.drawText(4, line);
  drawProgressBar();
}

// ------------------ OLED 64x64 live maze map ------------------
// 9 cells x 7 px + the final border line = exactly 64 px.
// Solid line: measured wall. Dotted line: unmeasured wall. No line:
// measured open. Centre dot: visited. Solid 3x3 block: robot.
// Hollow 3x3 block: goal. Checkerboard: blocked cell.
bool wasVisited(int8_t row, int8_t column);

const uint8_t DIV7[64] PROGMEM = {
  0,0,0,0,0,0,0, 1,1,1,1,1,1,1, 2,2,2,2,2,2,2, 3,3,3,3,3,3,3,
  4,4,4,4,4,4,4, 5,5,5,5,5,5,5, 6,6,6,6,6,6,6, 7,7,7,7,7,7,7,
  8,8,8,8,8,8,8, 9
};
const uint8_t MOD7[64] PROGMEM = {
  0,1,2,3,4,5,6, 0,1,2,3,4,5,6, 0,1,2,3,4,5,6, 0,1,2,3,4,5,6,
  0,1,2,3,4,5,6, 0,1,2,3,4,5,6, 0,1,2,3,4,5,6, 0,1,2,3,4,5,6,
  0,1,2,3,4,5,6, 0
};

int8_t drawnRobotRow = -1;
int8_t drawnRobotColumn = -1;

bool blockedCellFast(uint8_t row, uint8_t column) {
  uint8_t id = row * COLUMNS + column;
  return blockedBits[id >> 3] & (1 << (id & 7));
}

uint8_t mapPixel(uint8_t x, uint8_t y) {
  uint8_t cx = pgm_read_byte(&DIV7[x]);
  uint8_t lx = pgm_read_byte(&MOD7[x]);
  uint8_t cy = pgm_read_byte(&DIV7[y]);
  uint8_t ly = pgm_read_byte(&MOD7[y]);
  if (lx == 0 && ly == 0) return 1;  // lattice corner
  if (lx == 0) {                     // vertical edge at column boundary cx
    uint8_t cell, bit;
    if (cx < COLUMNS) { cell = cellMap[cy][cx]; bit = 8; }        // west
    else { cell = cellMap[cy][COLUMNS - 1]; bit = 2; }            // east
    if (!(cell & (bit << 4))) return y & 1;                       // unknown
    return (cell & bit) ? 1 : 0;
  }
  if (ly == 0) {                     // horizontal edge at row boundary cy
    uint8_t cell, bit;
    if (cy < ROWS) { cell = cellMap[cy][cx]; bit = 1; }           // north
    else { cell = cellMap[ROWS - 1][cx]; bit = 4; }               // south
    if (!(cell & (bit << 4))) return x & 1;                       // unknown
    return (cell & bit) ? 1 : 0;
  }
  if ((int8_t)cy == robotRow && (int8_t)cx == robotColumn)
    return (lx >= 2 && lx <= 4 && ly >= 2 && ly <= 4) ? 1 : 0;
  if ((int8_t)cy == GOAL_ROW && (int8_t)cx == GOAL_COLUMN)
    return (lx >= 2 && lx <= 4 && ly >= 2 && ly <= 4
            && !(lx == 3 && ly == 3)) ? 1 : 0;
  if (blockedCellFast(cy, cx)) return (x ^ y) & 1;
  if (wasVisited(cy, cx)) return (lx == 3 && ly == 3) ? 1 : 0;
  return 0;
}

void drawFullMap() {
  drawnRobotRow = robotRow;
  drawnRobotColumn = robotColumn;
  uint8_t buffer[16];
  for (uint8_t page = 0; page < 8; ++page) {
    oled.setRegion(page, 0);
    for (uint8_t start = 0; start < 64; start += 16) {
      for (uint8_t i = 0; i < 16; ++i) {
        uint8_t x = start + i;
        uint8_t value = 0;
        for (uint8_t bit = 0; bit < 8; ++bit) {
          if (mapPixel(x, page * 8 + bit)) value |= 1 << bit;
        }
        buffer[i] = value;
      }
      oled.writeData(buffer, 16);
    }
  }
}

// Redraws just one cell (8x8 px spanning at most two pages). Cheap enough
// to run mid-drive when the robot crosses a cell boundary.
void drawCellPatch(uint8_t row, uint8_t column) {
  uint8_t x0 = column * 7;
  uint8_t y0 = row * 7;
  uint8_t lastPage = (uint8_t)(y0 + 7) >> 3;
  uint8_t buffer[8];
  for (uint8_t page = y0 >> 3; page <= lastPage && page < 8; ++page) {
    for (uint8_t i = 0; i < 8; ++i) {
      uint8_t value = 0;
      for (uint8_t bit = 0; bit < 8; ++bit) {
        if (mapPixel(x0 + i, page * 8 + bit)) value |= 1 << bit;
      }
      buffer[i] = value;
    }
    oled.setRegion(page, x0);
    oled.writeData(buffer, 8);
  }
}

void refreshRobotMarker() {
  if (drawnRobotRow == robotRow && drawnRobotColumn == robotColumn) return;
  int8_t oldRow = drawnRobotRow, oldColumn = drawnRobotColumn;
  drawnRobotRow = robotRow;
  drawnRobotColumn = robotColumn;
  if (oldRow >= 0) drawCellPatch(oldRow, oldColumn);
  drawCellPatch(robotRow, robotColumn);
}

void waitStopped(unsigned long duration, const char *status) {
  stopMotors();
  unsigned long start = millis();
  while (millis() - start < duration) {
    updateLidars(100); drawTelemetry(status); delay(10);
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
  mpu.begin(); oled.drawText(0, "CALIB"); delay(1000); mpu.calcOffsets(true, true);
  gyroBiasEpoch = millis();
  startLidar(lidarLeft, LIDAR_LEFT_XSHUT, ADDR_LEFT);
  startLidar(lidarFront, LIDAR_FRONT_XSHUT, ADDR_FRONT);
  startLidar(lidarRight, LIDAR_RIGHT_XSHUT, ADDR_RIGHT);
}

bool turnTo(float target, const char *status) {
  float currentYaw = yaw();
  float delta = normalizeAngle(target - currentYaw);
  float direction = delta >= 0.0f ? 1.0f : -1.0f;
  float total = fabs(delta);
  if (total <= TURN_TOLERANCE_DEG) { stopMotors(); return true; }
  float setpoint = 0.0f;
  float actual = 0.0f;
  float lastYaw = currentYaw;
  uint8_t turnRecoveries = 0;
  turnTracker.reset();
  unsigned long lastMs = millis(), stableSince = 0, settleStart = 0, stallSince = 0;

  // No lidar reads in here: a turn does not need them and each blocking
  // read stretched the loop period mid-turn.
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
      // Back straight off a touch and let the turn resume.
      if (fabs(yawRate) < 4.0f && fabs(command) > 12.0f) {
        if (stallSince == 0) stallSince = now;
        if (now - stallSince >= 500 && turnRecoveries < STALL_RECOVERY_MAX) {
          ++turnRecoveries;
          stallSince = 0;
          stopMotors();
          delay(80);
          activeDeadband = MOTOR_DEADBAND;
          unsigned long backStart = millis();
          while (millis() - backStart < 250) {
            setControlledWheels(9.0f, 9.0f);  // positive args = reverse
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
    drawTelemetry(status); delay(10);
  }
  stopMotors();
  return fabs(normalizeAngle(target - yaw())) <= 3.0f;
}

void turnToDirection(uint8_t direction, const char *status) {
  targetYaw = normalizeAngle(northYaw + HEADING_YAW[direction]);
  // Micro-corrections below the coast zone are pointless: the turn loop
  // coasts inside it anyway, so demanding one just twitched the robot
  // before every straight move.
  if (direction != robotHeading
      || fabs(normalizeAngle(targetYaw - yaw())) > TURN_COAST_ZONE_DEG + 0.5f) {
    // A timed-out turn used to be silently accepted, and the robot then
    // drove a whole cell while badly rotated - another way to slip past
    // a virtual fence. Settle and retry once instead.
    if (!turnTo(targetYaw, status)) {
      waitStopped(120, status);
      turnTo(targetYaw, status);
    }
  }
  robotHeading = direction;
}

// Drives `cells` cells in one continuous profiled move. Backtracking and
// the shortest run chain straight segments through here, so the robot no
// longer stops at every cell boundary.
bool driveCells(uint8_t direction, uint8_t cells, const char *status) {
  resetEncoders();
  lastMoveFrontAnchored = false;
  // The last lidar values belong to the heading before the turn. A stale
  // front reading below the anchor threshold would abort the move on the
  // very first loop, so refresh it and let the round robin redo the sides.
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
  uint8_t crossed = 0;
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

    // Wall-edge odometry sync. Presence needs an agreed reading pair;
    // absence needs the raw reading to be beyond wall range too, so a
    // rejected spike can never fake a wall edge.
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
            lastDistance += step;  // keep the speed estimate continuous
          }
        }
      }
    }

    while (crossed + 1 < cells && distance >= (float)(crossed + 1) * CELL_MM) {
      ++crossed;
      robotRow += DR[direction];
      robotColumn += DC[direction];
      refreshRobotMarker();  // 14 bytes: fits the TX buffer, no mid-drive stall
    }

    bool front = frontWall(frontMm);
    if (front && frontMm <= WALL_GAP_MM) {
      lastMoveFrontAnchored = true;
      break;
    }
    // Stop as soon as the distance is done. The old hold-position settle
    // phase let the tracker reverse and re-push against static friction,
    // which shook the robot at every cell boundary.
    if (distance >= total - CELL_TOLERANCE_MM) break;
    if (now - startMs >= DRIVE_TIMEOUT_MS * cells) break;

    // Stall watchdog: commanded but not moving means the robot is wedged
    // against something. Back off a little and resume instead of pushing
    // until the drive times out.
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
        if (backTo < 3.0f) backTo = 3.0f;  // |net| odometry: never chase past zero
        activeDeadband = MOTOR_DEADBAND;   // breaking loose from standstill
        unsigned long backStart = millis();
        while (travelledMm() > backTo && millis() - backStart < 700) {
          setControlledWheels(9.0f, 9.0f);  // positive args = reverse
          drawTelemetry(status);
          delay(10);
        }
        stopMotors();
        delay(80);
        sampleLidars();
        prevLeftMm = leftMm; prevRightMm = rightMm; prevFrontMm = frontMm;
        driveTracker.reset();
        filteredSpeed = 0.0f;
        anchoredStart = false;  // position disturbed, not wall-referenced
        lastDistance = travelledMm() + distanceOffset;
        setpoint = max(0.0f, lastDistance);
        lastMs = millis();
        continue;
      }
    } else stuckSince = 0;

    // Creep up to a visible front wall instead of slamming the brakes on
    // at the 52 mm threshold; crawl while a side wall is very close.
    float cruise = DRIVE_SPEED_MAX;
    if (front) {
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
    if (forward < 0.0f) forward = 0.0f;  // never reverse inside a cell move

    // Classic micromouse correction, one layer, no mode memory:
    // walls visible -> centre on the physical corridor; no walls -> hold
    // the grid heading by gyro. Both damped by the gyro rate.
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
    // into a hard in-place twitch (the "suddenly pivots then hits the
    // wall" failure). Correction is only allowed in proportion to drive.
    correction *= constrain(forward / 8.0f, 0.0f, 1.0f);
    updateActiveDeadband(fabs(filteredSpeed));
    setControlledWheels(-forward + correction, -forward - correction);
    drawTelemetry(status); delay(10);
  }
  stopMotors();
  sampleLidars();
  if (frontWall(frontMm)
      && fabs((float)frontMm - WALL_GAP_MM) <= CORNER_DISTANCE_TOLERANCE_MM) {
    lastMoveFrontAnchored = true;
  }
  // 45 mm: tolerant of odometry error over a long chain, still far less
  // than one cell, so stopping a whole cell early is always a fault.
  // Judged on the edge-synced distance - the honest one.
  if (travelledMm() + distanceOffset < total - 45.0f) return false;
  while (crossed < cells) {
    ++crossed;
    robotRow += DR[direction];
    robotColumn += DC[direction];
  }
  // Encoder self-calibration on dead-end reversals: the wall behind and
  // the wall ahead were both measured, so the physical length of this run
  // is known exactly regardless of where the two stops actually landed.
  if (lastMoveFrontAnchored) {
    if (anchoredStart && anchorHeading == ((robotHeading + 2) & 3)
        && frontMm != 999) {
      float measured = travelledMm();
      float expected = total + (WALL_GAP_MM - anchorGapMm)
                       + (WALL_GAP_MM - (float)frontMm);
      if (measured > 0.6f * total) {
        float ratio = expected / measured;
        if (ratio > 0.92f && ratio < 1.08f) {
          mmPerCount = constrain(mmPerCount * (1.0f + 0.25f * (ratio - 1.0f)),
                                 MM_PER_COUNT * 0.94f, MM_PER_COUNT * 1.06f);
        }
      }
    }
    anchoredStart = true;
    anchorHeading = robotHeading;
    anchorGapMm = (float)frontMm;
  } else {
    anchoredStart = false;
  }
  refreshRobotMarker();
  return true;
}



bool inside(int8_t row, int8_t column) {
  return row >= 0 && row < ROWS && column >= 0 && column < COLUMNS;
}

char layoutCell(uint8_t row, uint8_t column) {
  return pgm_read_byte(&MAZE_LAYOUT[row][column]);
}

bool hardCornerCell(int8_t row, int8_t column) {
  for (uint8_t i = 0; i < 12; ++i) {
    if (row == (int8_t)pgm_read_byte(&HARD_CORNER_BLOCKS[i][0])
        && column == (int8_t)pgm_read_byte(&HARD_CORNER_BLOCKS[i][1])) {
      return true;
    }
  }
  return false;
}

int8_t headingFromSymbol(char symbol) {
  if (symbol == 'N' || symbol == 'n') return 0;
  if (symbol == 'E' || symbol == 'e') return 1;
  if (symbol == 'S' || symbol == 's') return 2;
  if (symbol == 'W' || symbol == 'w') return 3;
  return -1;
}

bool loadMazeLayout() {
  uint8_t starts = 0, goals = 0;
  NAVIGABLE_CELL_COUNT = 0;
  for (uint8_t row = 0; row < ROWS; ++row) {
    for (uint8_t column = 0; column < COLUMNS; ++column) {
      char symbol = layoutCell(row, column);
      bool allowed = symbol == '.' || symbol == '#'
                     || headingFromSymbol(symbol) >= 0;
      if (!allowed) return false;
      if (hardCornerCell(row, column)) continue;
      if (symbol != '#') ++NAVIGABLE_CELL_COUNT;
      if (symbol >= 'A' && symbol <= 'Z') {
        ++starts;
        START_ROW = row; START_COLUMN = column;
        START_HEADING = headingFromSymbol(symbol);
      } else if (symbol >= 'a' && symbol <= 'z') {
        ++goals;
        GOAL_ROW = row; GOAL_COLUMN = column;
        GOAL_HEADING = headingFromSymbol(symbol);
      }
    }
  }
  return starts == 1 && goals == 1 && NAVIGABLE_CELL_COUNT > 1;
}

bool navigableCell(int8_t row, int8_t column) {
  if (!inside(row, column)) return false;
  if (hardCornerCell(row, column)) return false;
  return layoutCell(row, column) != '#';
}

uint8_t opposite(uint8_t direction) { return (direction + 2) & 3; }

uint8_t cellId(int8_t row, int8_t column) { return row * COLUMNS + column; }

bool wasVisited(int8_t row, int8_t column) {
  uint8_t id = cellId(row, column);
  return visitedBits[id >> 3] & (1 << (id & 7));
}

void markVisited(int8_t row, int8_t column) {
  uint8_t id = cellId(row, column);
  visitedBits[id >> 3] |= 1 << (id & 7);
}

void setEdge(int8_t row, int8_t column, uint8_t direction, bool wall) {
  uint8_t bit = 1 << direction;
  uint8_t knownBit = bit << 4;
  bool wasKnown = cellMap[row][column] & knownBit;
  cellMap[row][column] |= knownBit;
  if (wall) cellMap[row][column] |= bit;
  else if (!wasKnown) cellMap[row][column] &= ~bit;

  int8_t nextRow = row + DR[direction], nextColumn = column + DC[direction];
  if (!inside(nextRow, nextColumn)) return;
  uint8_t reverseBit = 1 << opposite(direction);
  knownBit = reverseBit << 4;
  wasKnown = cellMap[nextRow][nextColumn] & knownBit;
  cellMap[nextRow][nextColumn] |= knownBit;
  if (wall) cellMap[nextRow][nextColumn] |= reverseBit;
  else if (!wasKnown) cellMap[nextRow][nextColumn] &= ~reverseBit;
}

bool knownOpen(int8_t row, int8_t column, uint8_t direction) {
  uint8_t bit = 1 << direction;
  int8_t nextRow = row + DR[direction], nextColumn = column + DC[direction];
  return navigableCell(nextRow, nextColumn)
         && (cellMap[row][column] & (bit << 4))
         && !(cellMap[row][column] & bit);
}

bool edgeKnown(int8_t row, int8_t column, uint8_t direction) {
  return cellMap[row][column] & ((1 << direction) << 4);
}

void initialiseMap() {
  memset(cellMap, 0, sizeof(cellMap));
  memset(visitedBits, 0, sizeof(visitedBits));
  memset(blockedBits, 0, sizeof(blockedBits));
  memset(dfsParent, -1, sizeof(dfsParent));
  for (uint8_t row = 0; row < ROWS; ++row) {
    for (uint8_t column = 0; column < COLUMNS; ++column) {
      if (!navigableCell(row, column)) {
        uint8_t id = row * COLUMNS + column;
        blockedBits[id >> 3] |= 1 << (id & 7);
        continue;
      }
      for (uint8_t direction = 0; direction < 4; ++direction) {
        int8_t nextRow = row + DR[direction];
        int8_t nextColumn = column + DC[direction];
        if (!navigableCell(nextRow, nextColumn)) {
          setEdge(row, column, direction, true);
        }
      }
    }
  }
  dfsParent[START_ROW][START_COLUMN] = -2;
}

void recordRange(uint8_t direction, uint16_t distance, uint16_t wallLimit) {
  if (distance == 999) return;
  setEdge(robotRow, robotColumn, direction, distance >= 20 && distance <= wallLimit);
}

void observeCell() {
  waitStopped(CELL_SETTLE_MS, "MAPPING");
  if (!wasVisited(robotRow, robotColumn)) {
    markVisited(robotRow, robotColumn);
    ++visitedCount;
  }
  uint8_t leftDirection = (robotHeading + 3) & 3;
  uint8_t rightDirection = (robotHeading + 1) & 3;
  for (uint8_t retry = 0; retry < 3; ++retry) {
    sampleLidars();
    recordRange(leftDirection, leftMm, SIDE_WALL_MAX_MM);
    recordRange(robotHeading, frontMm, FRONT_WALL_MAX_MM);
    recordRange(rightDirection, rightMm, SIDE_WALL_MAX_MM);
    if (edgeKnown(robotRow, robotColumn, leftDirection)
        && edgeKnown(robotRow, robotColumn, robotHeading)
        && edgeKnown(robotRow, robotColumn, rightDirection)) break;
  }
  learnGyroBias();
  drawTelemetry("MAPPING");
  drawFullMap();
}

void scanStartRear() {
  uint8_t rear = opposite(robotHeading);
  if (cellMap[robotRow][robotColumn] & ((1 << rear) << 4)) return;
  uint8_t original = robotHeading;
  turnToDirection(rear, "SCAN REAR");
  waitStopped(CELL_SETTLE_MS, "SCAN REAR");
  for (uint8_t retry = 0; retry < 3 && !edgeKnown(robotRow, robotColumn, rear); ++retry) {
    frontMm = stableRange(lidarFront);
    recordRange(robotHeading, frontMm, FRONT_WALL_MAX_MM);
  }
  turnToDirection(original, "SCAN DONE");
  drawFullMap();
}

int8_t chooseUnvisitedDirection() {
  const int8_t relative[4] = {-1, 0, 1, 2};  // left, forward, right, back
  for (uint8_t i = 0; i < 4; ++i) {
    uint8_t direction = (robotHeading + relative[i] + 4) & 3;
    int8_t row = robotRow + DR[direction], column = robotColumn + DC[direction];
    if (knownOpen(robotRow, robotColumn, direction) && !wasVisited(row, column)) return direction;
  }
  return -1;
}

bool hasUnvisitedNeighbour(int8_t row, int8_t column) {
  for (uint8_t direction = 0; direction < 4; ++direction) {
    int8_t r = row + DR[direction], c = column + DC[direction];
    if (knownOpen(row, column, direction) && !wasVisited(r, c)) return true;
  }
  return false;
}

// Reverses with heading hold until the odometry is back near the start of
// the failed move, so exploration can mark the edge blocked and carry on
// instead of declaring a fault in the middle of a cell.
bool retreatToMoveStart(const char *status) {
  activeDeadband = MOTOR_DEADBAND;
  headingTracker.reset();
  unsigned long start = millis(), lastMs = start;
  while (travelledMm() > 6.0f && millis() - start < 2500) {
    unsigned long now = millis();
    float dt = constrain((now - lastMs) / 1000.0f, 0.001f, 0.05f);
    lastMs = now;
    float correction = constrain(
      headingTracker.update(normalizeAngle(targetYaw - yaw()),
                            0.0f, gyroRateDegPerSec(), dt),
      -4.0f, 4.0f);
    setControlledWheels(10.0f + correction, 10.0f - correction);
    drawTelemetry(status);
    delay(10);
  }
  stopMotors();
  waitStopped(CELL_SETTLE_MS, status);
  anchoredStart = false;
  return travelledMm() <= 25.0f;
}

// checkFront is used when entering an unobserved cell during exploration.
// Chained moves over already-mapped cells skip the stationary pre-check;
// the continuous front-wall stop inside driveCells stays as the guard.
bool moveCells(uint8_t direction, uint8_t cells, bool checkFront,
               const char *status) {
  int8_t r = robotRow, c = robotColumn;
  for (uint8_t i = 0; i < cells; ++i) {
    r += DR[direction]; c += DC[direction];
    if (!navigableCell(r, c)) {
      if (i == 0) { setEdge(robotRow, robotColumn, direction, true); drawFullMap(); }
      return false;
    }
  }
  turnToDirection(direction, status);
  waitStopped(ACTION_PAUSE_MS, status);
  if (checkFront) {
    frontMm = stableRange(lidarFront);
    if (frontWall(frontMm)) {
      setEdge(robotRow, robotColumn, direction, true);
      drawFullMap();
      return false;
    }
    setEdge(robotRow, robotColumn, direction, false);
  }
  if (!driveCells(direction, cells, status)) {
    // During exploration a blocked single-cell move is survivable: back
    // up to the cell we came from, record the edge as a wall, move on.
    if (checkFront && cells == 1 && retreatToMoveStart(status)) {
      setEdge(robotRow, robotColumn, direction, true);
      drawFullMap();
      return false;
    }
    motionFault = true;
    phaseCode = 'F';
    drawFullMap();
    return false;
  }
  waitStopped(CELL_SETTLE_MS, status);
  return true;
}

bool moveOneCell(uint8_t direction, const char *status) {
  return moveCells(direction, 1, true, status);
}

void exploreMaze() {
  phaseCode = 'M';
  observeCell();
  scanStartRear();
  while (true) {
    int8_t direction = chooseUnvisitedDirection();
    if (direction >= 0) {
      int8_t nextRow = robotRow + DR[direction], nextColumn = robotColumn + DC[direction];
      if (moveOneCell(direction, "EXPLORE")) {
        dfsParent[nextRow][nextColumn] = opposite(direction);
        observeCell();
      }
      if (motionFault) return;
      continue;
    }
    if (robotRow == START_ROW && robotColumn == START_COLUMN) break;
    direction = dfsParent[robotRow][robotColumn];
    // Chain the backtrack while the parent trail keeps the same direction
    // and passes only through fully-explored cells: one smooth run instead
    // of a stop at every cell.
    uint8_t run = 1;
    int8_t r = robotRow + DR[direction], c = robotColumn + DC[direction];
    while (!(r == START_ROW && c == START_COLUMN)
           && !hasUnvisitedNeighbour(r, c)
           && dfsParent[r][c] == (int8_t)direction) {
      ++run;
      r += DR[direction];
      c += DC[direction];
    }
    if (!moveCells(direction, run, false, "RETURN")) {
      motionFault = true;
      phaseCode = 'F';
      drawFullMap();
      return;
    }
    drawFullMap();
  }
  drawFullMap();
}

bool buildShortestPath() {
  memset(dfsParent, -1, sizeof(dfsParent));
  uint8_t start = cellId(START_ROW, START_COLUMN);
  uint8_t goal = cellId(GOAL_ROW, GOAL_COLUMN);
  uint8_t head = 0, tail = 0;
  bfsQueue[tail++] = start;
  dfsParent[START_ROW][START_COLUMN] = 4;

  while (head < tail && dfsParent[GOAL_ROW][GOAL_COLUMN] < 0) {
    uint8_t current = bfsQueue[head++];
    int8_t row = current / COLUMNS, column = current % COLUMNS;
    for (uint8_t direction = 0; direction < 4; ++direction) {
      if (!knownOpen(row, column, direction)) continue;
      int8_t nextRow = row + DR[direction], nextColumn = column + DC[direction];
      uint8_t next = cellId(nextRow, nextColumn);
      if (dfsParent[nextRow][nextColumn] >= 0) continue;
      dfsParent[nextRow][nextColumn] = direction;
      bfsQueue[tail++] = next;
    }
  }
  if (dfsParent[GOAL_ROW][GOAL_COLUMN] < 0) return false;

  shortestLength = 0;
  for (uint8_t current = goal; current != start;) {
    int8_t row = current / COLUMNS, column = current % COLUMNS;
    uint8_t direction = dfsParent[row][column];
    shortestPath[shortestLength++] = direction;
    current = cellId(row - DR[direction], column - DC[direction]);
  }
  for (uint8_t i = 0; i < shortestLength / 2; ++i) {
    uint8_t value = shortestPath[i];
    shortestPath[i] = shortestPath[shortestLength - 1 - i];
    shortestPath[shortestLength - 1 - i] = value;
  }
  return true;
}


void runAutonomousMapping() {
  initialiseMap();
  robotRow = START_ROW; robotColumn = START_COLUMN; robotHeading = START_HEADING;
  motionFault = false;
  lastMoveFrontAnchored = false;
  float startYaw = round(yaw());
  northYaw = normalizeAngle(startYaw - HEADING_YAW[START_HEADING]);
  targetYaw = normalizeAngle(northYaw + HEADING_YAW[START_HEADING]);
  drawFullMap();

  exploreMaze();
  if (motionFault) return;
  phaseCode = 'R';
  turnToDirection(START_HEADING, "AT START");
  waitStopped(PHASE_PAUSE_MS, "MAP COMPLETE");

  if (!buildShortestPath()) {
    phaseCode = 'X'; drawFullMap();
    while (true) { stopMotors(); drawTelemetry("NO PATH"); delay(20); }
  }
  phaseCode = 'S'; drawFullMap();
  waitStopped(PHASE_PAUSE_MS, "SHORTEST RUN");
  uint8_t index = 0;
  while (index < shortestLength) {
    uint8_t direction = shortestPath[index];
    uint8_t run = 1;
    while (index + run < shortestLength
           && shortestPath[index + run] == direction) ++run;
    if (!moveCells(direction, run, false, "SHORTEST")) {
      motionFault = true;
      phaseCode = 'F';
      drawFullMap();
      return;
    }
    index += run;
    drawFullMap();
  }
  turnToDirection(GOAL_HEADING, "GOAL HEADING");
  phaseCode = 'D'; drawFullMap();
}

bool configurationValid() {
  return loadMazeLayout()
         && navigableCell(START_ROW, START_COLUMN)
         && navigableCell(GOAL_ROW, GOAL_COLUMN)
         && START_HEADING < 4
         && GOAL_HEADING < 4;
}

void setup() {
  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_ENC_A, INPUT_PULLUP); pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
  Wire.begin(); Wire.setClock(400000);
  oled.begin();
  if (!configurationValid()) {
    oled.drawText(0, "CFG ERR");
    while (true) { stopMotors(); delay(100); }
  }
  beginSensors();
}

void loop() {
  static bool started = false;
  if (!started) {
    started = true;
    runAutonomousMapping();
  }
  stopMotors();
  drawTelemetry(phaseCode == 'D' ? "TASK 4.3 DONE"
                : phaseCode == 'F' ? "MOTION FAULT" : "TASK 4.3 CHECK");
  delay(20);
}
