#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <math.h>
#include <string.h>

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
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_REV;
const float CELL_TOLERANCE_MM = 0.5f;

const int DRIVE_PWM = 42;
const int DRIVE_PWM_FINAL = 28;
const int DRIVE_PWM_TRIM = 22;
const float DRIVE_PWM_RAMP_PER_SEC = 12.0f;
const float DRIVE_PWM_BRAKE_PER_SEC = 50.0f;
const float DRIVE_DISTANCE_KP = 0.40f;
const float DRIVE_DISTANCE_KD = 0.08f;
const float DRIVE_YAW_KP = 0.62f;
const float DRIVE_YAW_KD = 0.05f;
const float DRIVE_YAW_DEADBAND = 0.6f;
const float DRIVE_PD_RAMP_PER_SEC = 12.0f;
const float DRIVE_ENCODER_KP = 0.04f;
const float DRIVE_WALL_KP = 0.15f;
const int DRIVE_CORRECTION_MAX = 10;
const uint16_t SIDE_WALL_MAX_MM = 90;
const float WALL_HEADING_BASELINE_MM = 35.0f;
const float WALL_HEADING_GAIN = 0.50f;
const float WALL_HEADING_MAX_STEP_DEG = 1.25f;

const float TURN_KP = 0.65f;
const float TURN_KD = 0.12f;
const int TURN_PWM_MIN = 20;
const int TURN_PWM_MAX = 28;
const float TURN_PWM_RAMP_PER_SEC = 36.0f;
const float TURN_TOLERANCE_DEG = 1.5f;
const unsigned long TURN_STABLE_MS = 250UL;
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
  unsigned long lastMs = millis();
  unsigned long stableSince = 0;
  float lastError = normalizeAngle(targetYaw - yaw());
  float appliedOutput = 0.0f;

  while (true) {
    unsigned long now = millis();
    float dt = constrain((now - lastMs) / 1000.0f, 0.001f, 0.05f);
    lastMs = now;

    float error = normalizeAngle(targetYaw - yaw());
    if (fabs(error) <= TURN_TOLERANCE_DEG) {
      stopMotors();
      appliedOutput = 0.0f;
      if (stableSince == 0) stableSince = now;
      drawTelemetry(status, error, 0.0f);
      if (now - stableSince >= TURN_STABLE_MS) return true;
    } else {
      stableSince = 0;
      float derivative = normalizeAngle(error - lastError) / dt;
      float closingRate = max(0.0f, -(error * derivative) / fabs(error));
      float magnitude = TURN_KP * fabs(error)
                        - TURN_KD * closingRate;
      float targetOutput = 0.0f;

      if (magnitude > 0.0f) {
        magnitude = constrain(magnitude, (float)TURN_PWM_MIN, (float)TURN_PWM_MAX);
        targetOutput = error > 0.0f ? magnitude : -magnitude;
      }

      if (appliedOutput * targetOutput < 0.0f) appliedOutput = 0.0f;
      if (fabs(targetOutput) < fabs(appliedOutput)) {
        appliedOutput = targetOutput;
      } else {
        float maxStep = TURN_PWM_RAMP_PER_SEC * dt;
        appliedOutput += constrain(targetOutput - appliedOutput, -maxStep, maxStep);
      }

      if (appliedOutput == 0.0f) stopMotors();
      else setWheels(appliedOutput, -appliedOutput);
      drawTelemetry(status, error, appliedOutput);
    }

    lastError = error;
    delay(10);
  }

}

void driveCell(float &targetYaw, const char *status) {
  resetEncoders();
  float pwm = 0.0f;
  float appliedCorrection = 0.0f;
  float lastYawError = normalizeAngle(targetYaw - yaw());
  float lastDistance = 0.0f;
  float wallReferenceMm = 0.0f;
  float wallReferenceDistance = 0.0f;
  int8_t trackedWall = 0;
  unsigned long lastControlMs = millis();

  while (true) {
    unsigned long now = millis();
    float dt = max((now - lastControlMs) / 1000.0f, 0.001f);
    lastControlMs = now;
    float distance = travelledMm();
    float remaining = CELL_MM - distance;
    float speed = max(0.0f, (distance - lastDistance) / dt);
    lastDistance = distance;
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

    bool yawReferenceChanged = false;
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
        yawReferenceChanged = true;
      }
      wallReferenceMm = currentWallMm;
      wallReferenceDistance = distance;
    }

    bool frontWall = frontWallVisible(frontMm);
    if (frontWall && frontMm <= WALL_GAP_MM) break;
    if (remaining <= CELL_TOLERANCE_MM && !(frontWall && frontMm < 80)) break;

    float distancePd = DRIVE_DISTANCE_KP * remaining
                       - DRIVE_DISTANCE_KD * speed;
    int targetPwm = (int)round(constrain(distancePd,
                                         (float)DRIVE_PWM_TRIM,
                                         (float)DRIVE_PWM));
    if (remaining <= 15.0f || (frontWall && frontMm < 65)) {
      targetPwm = DRIVE_PWM_TRIM;
    } else if (remaining <= 60.0f || (frontWall && frontMm < 90)) {
      targetPwm = DRIVE_PWM_FINAL;
    }
    float pwmRamp = targetPwm < pwm
                      ? DRIVE_PWM_BRAKE_PER_SEC : DRIVE_PWM_RAMP_PER_SEC;
    float maxPwmStep = pwmRamp * dt;
    pwm += constrain((float)targetPwm - pwm, -maxPwmStep, maxPwmStep);

    float error = normalizeAngle(targetYaw - yaw());
    float derivative = yawReferenceChanged
                         ? 0.0f : normalizeAngle(error - lastYawError) / dt;
    lastYawError = error;
    float yawPd = fabs(error) < DRIVE_YAW_DEADBAND
                    ? 0.0f : DRIVE_YAW_KP * error
                              + DRIVE_YAW_KD * derivative;
    float sensorCorrection = encoderCorrection() + wallCorrection();
    float targetCorrection = constrain(yawPd + sensorCorrection,
                                       -(float)DRIVE_CORRECTION_MAX,
                                       (float)DRIVE_CORRECTION_MAX);

    float maxCorrectionStep = DRIVE_PD_RAMP_PER_SEC * dt;
    appliedCorrection += constrain(targetCorrection - appliedCorrection,
                                   -maxCorrectionStep, maxCorrectionStep);

    float forward = -pwm;
    setWheels(forward + appliedCorrection, forward - appliedCorrection);
    drawTelemetry(status, error, appliedCorrection);
    delay(10);
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
