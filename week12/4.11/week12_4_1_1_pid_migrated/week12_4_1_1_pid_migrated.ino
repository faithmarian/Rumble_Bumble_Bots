#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <math.h>
#include <string.h>

// PID_MIGRATED: trajectory PID ported from pid_validation.ino.
// This archive-safe variant leaves week8_4_chaining_new.ino unchanged.

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

// ------------------------- Control --------------------------
const char COMMANDS[] = "lffrfrrf";
const float CELL_MM = 180.0f;
const float ROBOT_SIZE_MM = 76.0f;
const float WALL_GAP_MM = (CELL_MM - ROBOT_SIZE_MM) / 2.0f;
const float WHEEL_DIAMETER_MM = 44.0f;
const float ENCODER_COUNTS_PER_REV = 356.72f;
// Calibration: the old conversion made a commanded distance run about 10% long.
const float ENCODER_DISTANCE_SCALE = 1.10f;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_REV
                           * ENCODER_DISTANCE_SCALE;
const float CELL_TOLERANCE_MM = 2.0f;

// The moving setpoint brakes before the target. Lower turn acceleration and
// integral gain, plus stronger rate damping, reduce the validation overshoot.
const float TURN_RATE_MAX = 52.0f;
const float TURN_ACCEL = 95.0f;
const float TURN_RATE_MIN = 5.0f;
const float TURN_KV = 0.20f;
const float TURN_KP = 0.90f;
const float TURN_KI = 0.22f;
const float TURN_KD = 0.14f;
const float TURN_I_LIMIT = 5.0f;
const float TURN_OUT_LIMIT = 13.0f;

const float DRIVE_SPEED_MAX = 75.0f;
const float DRIVE_ACCEL = 130.0f;
const float DRIVE_SPEED_MIN = 6.0f;
const float DRIVE_KV = 0.28f;
const float DRIVE_KP = 0.45f;
const float DRIVE_KI = 0.12f;
const float DRIVE_KD = 0.05f;
const float DRIVE_I_LIMIT = 8.0f;
const float DRIVE_OUT_LIMIT = 28.0f;

const float HEADING_KP = 1.40f;
const float HEADING_KI = 0.25f;
const float HEADING_KD = 0.12f;
const float HEADING_I_LIMIT = 6.0f;
const float HEADING_OUT_LIMIT = 10.0f;
const float DRIVE_YAW_DEADBAND = 0.35f;
const float DRIVE_ENCODER_KP = 0.04f;
const float DRIVE_WALL_KP = 0.15f;
const int DRIVE_CORRECTION_MAX = 10;
const uint16_t SIDE_WALL_MAX_MM = 90;
const float WALL_HEADING_BASELINE_MM = 35.0f;
const float WALL_HEADING_GAIN = 0.50f;
const float WALL_HEADING_MAX_STEP_DEG = 1.25f;

const int MOTOR_DEADBAND = 18;
const int MOTOR_MAX = 60;
const float DEADBAND_BLEND = 4.0f;
const float TURN_TOLERANCE_DEG = 1.0f;
const float TURN_SETTLE_RATE = 7.0f;
const float DRIVE_SETTLE_RATE = 7.0f;
const unsigned long TURN_STABLE_MS = 220UL;
const unsigned long SETTLE_TIMEOUT_MS = 1800UL;
const unsigned long ACTION_PAUSE_MS = 80UL;

MPU6050 mpu(Wire);
VL6180X lidarLeft;
VL6180X lidarFront;
VL6180X lidarRight;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
uint16_t leftMm = 0;
uint16_t frontMm = 0;
uint16_t rightMm = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastLidarMs = 0;

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
  return constrain(DRIVE_ENCODER_KP * tickError, -3.0f, 3.0f);
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float yaw() {
  mpu.update();
  return mpu.getAngleZ();
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

void updateLidars(unsigned long intervalMs) {
  unsigned long now = millis();
  if (now - lastLidarMs < intervalMs) return;
  lastLidarMs = now;
  leftMm = readMm(lidarLeft);
  frontMm = readMm(lidarFront);
  rightMm = readMm(lidarRight);
}

bool sideWallVisible(uint16_t distance) {
  return distance >= 20 && distance <= SIDE_WALL_MAX_MM;
}

bool frontWallVisible(uint16_t distance) {
  return distance >= 20 && distance <= 120;
}

float wallCorrection() {
  bool leftWall = sideWallVisible(leftMm);
  bool rightWall = sideWallVisible(rightMm);

  if (!leftWall && !rightWall) return 0.0f;

  if (leftWall && (!rightWall || leftMm <= rightMm)) {
    return DRIVE_WALL_KP * ((float)leftMm - WALL_GAP_MM);
  }
  if (rightWall) return DRIVE_WALL_KP * (WALL_GAP_MM - (float)rightMm);
  return 0.0f;
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
}

void drawNumber(uint8_t row, const char *label, float value) {
  char number[9];
  char line[17];
  dtostrf(value, 7, 1, number);
  snprintf(line, sizeof(line), "%s:%s", label, number);
  oled.drawLine(row, line);
}

void drawTelemetry(const char *status, float error, float output) {
  unsigned long now = millis();
  if (now - lastDisplayMs < 120) return;
  lastDisplayMs = now;

  mpu.update();
  char line[17];
  oled.drawLine(0, status);
  drawNumber(1, "X", mpu.getAngleX());
  drawNumber(2, "Y", mpu.getAngleY());
  drawNumber(3, "Z", mpu.getAngleZ());
  drawNumber(4, "E", error);
  drawNumber(5, "O", output);
  snprintf(line, sizeof(line), "L:%4u F:%4u", leftMm, frontMm);
  oled.drawLine(6, line);
  snprintf(line, sizeof(line), "R:%4u", rightMm);
  oled.drawLine(7, line);
}

void waitStopped(unsigned long duration, const char *status) {
  stopMotors();
  unsigned long start = millis();
  while (millis() - start < duration) {
    updateLidars(100);
    drawTelemetry(status, 0.0f, 0.0f);
    delay(10);
  }
}

// ----------------------- Robot actions ----------------------
bool turnTo(float targetYaw, const char *status) {
  float currentYaw = yaw();
  float delta = normalizeAngle(targetYaw - currentYaw);
  float direction = delta >= 0.0f ? 1.0f : -1.0f;
  float total = fabs(delta);
  float setpoint = 0.0f;
  float actual = 0.0f;
  float lastYaw = currentYaw;
  turnTracker.reset();
  unsigned long lastMs = millis(), stableSince = 0, settleStart = 0;

  while (true) {
    unsigned long now = millis();
    float dt = constrain((now - lastMs) / 1000.0f, 0.001f, 0.05f);
    lastMs = now;
    currentYaw = yaw();
    float yawStep = normalizeAngle(currentYaw - lastYaw);
    lastYaw = currentYaw;
    actual += yawStep * direction;
    float rateActual = yawStep * direction / dt;
    float rate = profileRate(setpoint, total, TURN_RATE_MAX,
                             TURN_ACCEL, TURN_RATE_MIN);
    bool profileDone = setpoint >= total;
    if (!profileDone) setpoint = min(setpoint + rate * dt, total);
    else { rate = 0.0f; if (settleStart == 0) settleStart = now; }

    float lag = setpoint - actual;
    float command = turnTracker.update(lag, rate, rateActual, dt) * direction;
    setControlledWheels(command, -command);
    drawTelemetry(status, normalizeAngle(targetYaw - currentYaw), command);

    if (profileDone) {
      bool settled = fabs(total - actual) <= TURN_TOLERANCE_DEG
                     && fabs(rateActual) <= TURN_SETTLE_RATE;
      if (settled) {
        if (stableSince == 0) stableSince = now;
        if (now - stableSince >= TURN_STABLE_MS) break;
      } else stableSince = 0;
      if (now - settleStart >= SETTLE_TIMEOUT_MS) break;
    }
    delay(8);
  }
  stopMotors();
  return fabs(normalizeAngle(targetYaw - yaw())) <= 3.0f;
}

void driveCell(float &targetYaw, const char *status) {
  resetEncoders();
  float lastDistance = 0.0f;
  float filteredSpeed = 0.0f;
  float setpoint = 0.0f;
  float lastYaw = yaw();
  float wallReferenceMm = 0.0f;
  float wallReferenceDistance = 0.0f;
  int8_t trackedWall = 0;
  unsigned long lastControlMs = millis(), stableSince = 0, settleStart = 0;
  driveTracker.reset();
  headingTracker.reset();

  while (true) {
    unsigned long now = millis();
    float dt = max((now - lastControlMs) / 1000.0f, 0.001f);
    lastControlMs = now;
    float distance = travelledMm();
    float remaining = CELL_MM - distance;
    float speed = (distance - lastDistance) / dt;
    lastDistance = distance;
    filteredSpeed += 0.25f * (speed - filteredSpeed);
    updateLidars(30);

    bool leftWallReady = sideWallVisible(leftMm);
    bool rightWallReady = sideWallVisible(rightMm);

    int8_t currentWall = 0;
    float currentWallMm = 0.0f;
    if (trackedWall > 0 && leftWallReady) {
      currentWall = 1;
      currentWallMm = (float)leftMm;
    } else if (trackedWall < 0 && rightWallReady) {
      currentWall = -1;
      currentWallMm = (float)rightMm;
    } else if (leftWallReady
               && (!rightWallReady || leftMm <= rightMm)) {
      currentWall = 1;
      currentWallMm = (float)leftMm;
    } else if (rightWallReady) {
      currentWall = -1;
      currentWallMm = (float)rightMm;
    }

    if (currentWall == 0 || fabs(currentWallMm - WALL_GAP_MM) > 8.0f) {
      trackedWall = 0;
    } else if (currentWall != trackedWall) {
      trackedWall = currentWall;
      wallReferenceMm = currentWallMm;
      wallReferenceDistance = distance;
    } else if (distance - wallReferenceDistance >= WALL_HEADING_BASELINE_MM) {
      float travel = distance - wallReferenceDistance;
      float sideChange = trackedWall > 0
                           ? currentWallMm - wallReferenceMm
                           : wallReferenceMm - currentWallMm;
      if (fabs(sideChange) <= 8.0f) {
        float wallAngle = atan2(sideChange, travel) * RAD_TO_DEG;
        float yawStep = constrain(WALL_HEADING_GAIN * wallAngle,
                                  -WALL_HEADING_MAX_STEP_DEG,
                                  WALL_HEADING_MAX_STEP_DEG);
        targetYaw = normalizeAngle(targetYaw + yawStep);
        headingTracker.reset();
      }
      wallReferenceMm = currentWallMm;
      wallReferenceDistance = distance;
    }

    bool frontWall = frontWallVisible(frontMm);
    if (frontWall && frontMm <= WALL_GAP_MM) break;

    float rate = profileRate(setpoint, CELL_MM, DRIVE_SPEED_MAX,
                             DRIVE_ACCEL, DRIVE_SPEED_MIN);
    bool profileDone = setpoint >= CELL_MM;
    if (!profileDone) setpoint = min(setpoint + rate * dt, CELL_MM);
    else { rate = 0.0f; if (settleStart == 0) settleStart = now; }

    float currentYaw = yaw();
    float yawRate = normalizeAngle(currentYaw - lastYaw) / dt;
    lastYaw = currentYaw;
    float forward = driveTracker.update(setpoint - distance, rate,
                                        filteredSpeed, dt);
    float error = normalizeAngle(targetYaw - currentYaw);
    float correction = fabs(error) < DRIVE_YAW_DEADBAND ? 0.0f
      : headingTracker.update(error, 0.0f, yawRate, dt);
    correction = constrain(correction + encoderCorrection() + wallCorrection(),
                           -(float)DRIVE_CORRECTION_MAX,
                           (float)DRIVE_CORRECTION_MAX);
    setControlledWheels(-forward + correction, -forward - correction);
    drawTelemetry(status, error, CELL_MM - distance);

    if (profileDone) {
      bool settled = fabs(CELL_MM - distance) <= CELL_TOLERANCE_MM
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
}

bool runCommand(char command, float &targetYaw, uint8_t index, uint8_t count) {
  char status[17];
  snprintf(status, sizeof(status), "T4 %c %u/%u", command, index + 1, count);

  if (command == 'f') {
    driveCell(targetYaw, status);
    return true;
  }
  if (command == 'l') targetYaw = normalizeAngle(targetYaw + 90.0f);
  if (command == 'r') targetYaw = normalizeAngle(targetYaw - 90.0f);
  return turnTo(targetYaw, status);
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
}

void loop() {
  static bool completed = false;
  static bool success = false;

  if (!completed) {
    completed = true;
    success = true;
    float targetYaw = yaw();
    uint8_t count = strlen(COMMANDS);

    for (uint8_t i = 0; i < count; ++i) {
      if (!runCommand(COMMANDS[i], targetYaw, i, count)) {
        success = false;
        break;
      }
      if (i + 1 < count) waitStopped(ACTION_PAUSE_MS, "T4 NEXT");
    }
  }

  stopMotors();
  drawTelemetry(success ? "T4 DONE" : "T4 CHECK", 0.0f, 0.0f);
  delay(10);
}
