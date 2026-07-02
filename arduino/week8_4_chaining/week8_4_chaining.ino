/*
  MTRN3100 Week 8 Task 3.4 - Chaining movements.

  Standalone sketch. No custom .hpp library.

  Commands:
    f = move forward one maze cell
    l = turn 90 degrees counter-clockwise
    r = turn 90 degrees clockwise

  Current physical LiDAR mapping on this robot:
    Left  = A0
    Front = A2
    Right = A1
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

// ------------------------- Pins -------------------------
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

// Replace this with the command string supplied by the demonstrator.
const char COMMAND_STRING[] = "frfrfrflff";

// Maze cell travel distance. Tune this on the actual maze.
const float CELL_DISTANCE_MM = 180.0f;
const unsigned long DRIVE_TIMEOUT_MS = 8000UL;
const unsigned long TURN_TIMEOUT_MS = 6500UL;

// -------------------- Robot Constants -------------------
const float WHEEL_RADIUS_MM = 32.0f;
const float ENCODER_COUNTS_PER_REV = 1400.0f;
const float MM_PER_COUNT = (2.0f * PI * WHEEL_RADIUS_MM) / ENCODER_COUNTS_PER_REV;

const float FORWARD_SIGN = -1.0f;
const float LEFT_FORWARD_TRIM = 1.00f;
const float RIGHT_FORWARD_TRIM = 1.00f;
const float YAW_MOTOR_SIGN = 1.0f;

// Forward control copied from the first task's stable version.
const float DIST_KP = 0.16f;
const float DIST_KI = 0.00f;
const float DIST_KD = 0.02f;
const float YAW_KP = 0.62f;
const int DRIVE_PWM_MIN = 38;
const int DRIVE_PWM_CRUISE = 60;
const int DRIVE_PWM_SLOW = 43;
const int DRIVE_PWM_FINAL = 32;
const int YAW_CORR_MAX = 9;
const float DRIVE_RAMP_PWM_PER_SEC = 45.0f;
const float CORR_FILTER_ALPHA = 0.12f;
const float CORR_SLEW_PWM_PER_SEC = 18.0f;
const float YAW_DEADBAND_DEG = 0.8f;
const float YAW_ERROR_LIMIT_DEG = 14.0f;

// Smooth turning control.
const float TURN_MOTOR_SIGN = 1.0f;
const float TURN_KP = 1.55f;
const float TURN_KD = 0.035f;
const float TURN_ERROR_LIMIT_DEG = 90.0f;
const float TURN_DEADBAND_DEG = 0.7f;
const int TURN_PWM_MAX = 92;
const int TURN_PWM_NEAR = 50;
const int TURN_PWM_FINE = 33;
const int TURN_PWM_MIN = 28;
const float TURN_FILTER_ALPHA = 0.22f;
const float TURN_SLEW_PWM_PER_SEC = 120.0f;
const float TURN_DONE_TOL_DEG = 2.2f;
const unsigned long TURN_STABLE_MS = 350UL;

// ------------------------- Devices -----------------------
MPU6050 mpu(Wire);
VL6180X lidarLeft;
VL6180X lidarFront;
VL6180X lidarRight;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

bool imuOk = false;
bool lidarLeftOk = false;
bool lidarFrontOk = false;
bool lidarRightOk = false;

unsigned long lastDisplayMs = 0;

// ------------------------- OLED --------------------------
class TinyOLED {
public:
  void begin() {
    const uint8_t initCommands[] = {
      0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,0x20,0x00,
      0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF
    };
    for (uint8_t i = 0; i < sizeof(initCommands); ++i) command(initCommands[i]);
    clear();
  }

  void clear() {
    for (uint8_t row = 0; row < 8; ++row) clearRow(row);
  }

  void clearRow(uint8_t row) {
    setCursor(0, row);
    for (uint8_t i = 0; i < 128; ++i) data(0);
  }

  void drawString(uint8_t col, uint8_t row, const char *s) {
    setCursor(col * 8, row);
    while (*s && col < 16) {
      drawChar(*s++);
      col++;
    }
  }

  void drawLine(uint8_t row, const char *s) {
    clearRow(row);
    drawString(0, row, s);
  }

private:
  void command(uint8_t v) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x00);
    Wire.write(v);
    Wire.endTransmission();
  }

  void data(uint8_t v) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x40);
    Wire.write(v);
    Wire.endTransmission();
  }

  void setCursor(uint8_t x, uint8_t row) {
    command(0xB0 | (row & 7));
    command(0x00 | (x & 0x0F));
    command(0x10 | (x >> 4));
  }

  void draw5(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e) {
    data(a); data(b); data(c); data(d); data(e); data(0); data(0); data(0);
  }

  void drawChar(char c) {
    if (c >= 'a' && c <= 'z') c -= 32;
    switch (c) {
      case ' ': draw5(0,0,0,0,0); break;
      case '-': draw5(0x08,0x08,0x08,0x08,0x08); break;
      case '.': draw5(0,0x60,0x60,0,0); break;
      case ':': draw5(0,0x36,0x36,0,0); break;
      case '/': draw5(0x20,0x10,0x08,0x04,0x02); break;
      case '0': draw5(0x3E,0x51,0x49,0x45,0x3E); break;
      case '1': draw5(0,0x42,0x7F,0x40,0); break;
      case '2': draw5(0x42,0x61,0x51,0x49,0x46); break;
      case '3': draw5(0x21,0x41,0x45,0x4B,0x31); break;
      case '4': draw5(0x18,0x14,0x12,0x7F,0x10); break;
      case '5': draw5(0x27,0x45,0x45,0x45,0x39); break;
      case '6': draw5(0x3C,0x4A,0x49,0x49,0x30); break;
      case '7': draw5(0x01,0x71,0x09,0x05,0x03); break;
      case '8': draw5(0x36,0x49,0x49,0x49,0x36); break;
      case '9': draw5(0x06,0x49,0x49,0x29,0x1E); break;
      case 'A': draw5(0x7E,0x11,0x11,0x11,0x7E); break;
      case 'B': draw5(0x7F,0x49,0x49,0x49,0x36); break;
      case 'C': draw5(0x3E,0x41,0x41,0x41,0x22); break;
      case 'D': draw5(0x7F,0x41,0x41,0x22,0x1C); break;
      case 'E': draw5(0x7F,0x49,0x49,0x49,0x41); break;
      case 'F': draw5(0x7F,0x09,0x09,0x09,0x01); break;
      case 'G': draw5(0x3E,0x41,0x49,0x49,0x7A); break;
      case 'H': draw5(0x7F,0x08,0x08,0x08,0x7F); break;
      case 'I': draw5(0,0x41,0x7F,0x41,0); break;
      case 'L': draw5(0x7F,0x40,0x40,0x40,0x40); break;
      case 'M': draw5(0x7F,0x02,0x0C,0x02,0x7F); break;
      case 'N': draw5(0x7F,0x04,0x08,0x10,0x7F); break;
      case 'O': draw5(0x3E,0x41,0x41,0x41,0x3E); break;
      case 'P': draw5(0x7F,0x09,0x09,0x09,0x06); break;
      case 'R': draw5(0x7F,0x09,0x19,0x29,0x46); break;
      case 'S': draw5(0x46,0x49,0x49,0x49,0x31); break;
      case 'T': draw5(0x01,0x01,0x7F,0x01,0x01); break;
      case 'U': draw5(0x3F,0x40,0x40,0x40,0x3F); break;
      case 'V': draw5(0x1F,0x20,0x40,0x20,0x1F); break;
      case 'W': draw5(0x3F,0x40,0x38,0x40,0x3F); break;
      case 'X': draw5(0x63,0x14,0x08,0x14,0x63); break;
      case 'Y': draw5(0x07,0x08,0x70,0x08,0x07); break;
      case 'Z': draw5(0x61,0x51,0x49,0x45,0x43); break;
      default: draw5(0,0,0,0,0); break;
    }
  }
};

TinyOLED oled;

// ------------------------- PID ---------------------------
struct PID {
  float kp, ki, kd, integral, lastError, integralLimit;
  bool first;

  PID(float p, float i, float d, float iLimit)
      : kp(p), ki(i), kd(d), integral(0), lastError(0), integralLimit(iLimit), first(true) {}

  void reset() {
    integral = 0;
    lastError = 0;
    first = true;
  }

  float update(float error, float dt) {
    if (dt <= 0) dt = 0.001f;
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit);
    float derivative = first ? 0 : (error - lastError) / dt;
    first = false;
    lastError = error;
    return kp * error + ki * integral + kd * derivative;
  }
};

PID distancePid(DIST_KP, DIST_KI, DIST_KD, 300.0f);

// ----------------------- Helpers -------------------------
void leftEncoderISR() {
  leftTicks += digitalRead(LEFT_ENC_B) ? 1 : -1;
}

void rightEncoderISR() {
  rightTicks += digitalRead(RIGHT_ENC_B) ? 1 : -1;
}

long atomicLeftTicks() {
  noInterrupts();
  long v = leftTicks;
  interrupts();
  return v;
}

long atomicRightTicks() {
  noInterrupts();
  long v = rightTicks;
  interrupts();
  return v;
}

void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

float normalizeDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

float yawDeg() {
  mpu.update();
  return normalizeDeg(mpu.getAngleZ());
}

float angleError(float target, float current) {
  return normalizeDeg(target - current);
}

float absTravelMm() {
  float l = fabs(atomicLeftTicks() * MM_PER_COUNT);
  float r = fabs(atomicRightTicks() * MM_PER_COUNT);
  return 0.5f * (l + r);
}

void setMotorRaw(uint8_t pwmPin, uint8_t dirPin, int command) {
  command = constrain(command, -255, 255);
  digitalWrite(dirPin, command >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(command));
}

void setMotorCommand(uint8_t motorNumber, uint8_t pwmPin, uint8_t dirPin, float command) {
  if (motorNumber == 2) command = -command;
  setMotorRaw(pwmPin, dirPin, (int)round(command));
}

void setWheelPwm(float leftCommand, float rightCommand) {
  setMotorCommand(1, LEFT_PWM, LEFT_DIR, leftCommand);
  setMotorCommand(2, RIGHT_PWM, RIGHT_DIR, rightCommand);
}

void stopMotors() {
  setWheelPwm(0, 0);
}

void allLidarsOff() {
  pinMode(LIDAR_LEFT_XSHUT, OUTPUT);
  pinMode(LIDAR_FRONT_XSHUT, OUTPUT);
  pinMode(LIDAR_RIGHT_XSHUT, OUTPUT);
  digitalWrite(LIDAR_LEFT_XSHUT, LOW);
  digitalWrite(LIDAR_FRONT_XSHUT, LOW);
  digitalWrite(LIDAR_RIGHT_XSHUT, LOW);
  delay(20);
}

void startLidar(VL6180X &sensor, uint8_t pin, uint8_t address) {
  digitalWrite(pin, HIGH);
  delay(50);
  sensor.setTimeout(80);
  sensor.init();
  sensor.configureDefault();
  sensor.setAddress(address);
  delay(10);
}

uint16_t readLidar(VL6180X &sensor, bool ok) {
  if (!ok) return 9999;
  uint16_t mm = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) return 9998;
  return mm;
}

void drawFloat(uint8_t row, const char *label, float value) {
  char num[9], line[17];
  dtostrf(value, 7, 1, num);
  snprintf(line, sizeof(line), "%s:%s", label, num);
  oled.drawLine(row, line);
}

void drawTelemetry(const char *status, float err, float out) {
  unsigned long now = millis();
  if (now - lastDisplayMs < 250) return;
  lastDisplayMs = now;

  mpu.update();
  uint16_t l = readLidar(lidarLeft, lidarLeftOk);
  uint16_t f = readLidar(lidarFront, lidarFrontOk);
  uint16_t r = readLidar(lidarRight, lidarRightOk);

  char line[17], num[8];
  oled.drawLine(0, status);
  drawFloat(1, "X", mpu.getAngleX());
  drawFloat(2, "Y", mpu.getAngleY());
  drawFloat(3, "Z", mpu.getAngleZ());
  dtostrf(err, 5, 1, num);
  snprintf(line, sizeof(line), "E:%s O:%3d", num, (int)out);
  oled.drawLine(4, line);
  snprintf(line, sizeof(line), "L:%4u F:%4u", l, f);
  oled.drawLine(6, line);
  snprintf(line, sizeof(line), "R:%4u MM", r);
  oled.drawLine(7, line);
}

void settle(unsigned long ms, const char *status) {
  stopMotors();
  unsigned long start = millis();
  while (millis() - start < ms) {
    drawTelemetry(status, 0, 0);
    delay(20);
  }
}

int clampDrivePwm(float pidOut, float distError) {
  if (distError <= 0.0f) return 0;
  int maxPwm = DRIVE_PWM_CRUISE;
  if (distError < 65.0f) maxPwm = DRIVE_PWM_FINAL;
  else if (distError < 160.0f) maxPwm = DRIVE_PWM_SLOW;

  int pwm = (int)round(constrain(pidOut, 0.0f, (float)maxPwm));
  if (pwm > 0 && pwm < DRIVE_PWM_MIN && distError > 65.0f) pwm = DRIVE_PWM_MIN;
  return pwm;
}

bool driveCell(const char *status) {
  resetEncoders();
  distancePid.reset();
  float targetYaw = yawDeg();
  float currentDrivePwm = 0.0f;
  float filteredCorr = 0.0f;
  unsigned long startMs = millis();
  unsigned long lastMs = millis();

  while (millis() - startMs < DRIVE_TIMEOUT_MS) {
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastMs = now;

    float travelled = absTravelMm();
    float distError = CELL_DISTANCE_MM - travelled;
    float yawErr = angleError(targetYaw, yawDeg());

    float drivePidOut = distancePid.update(distError, dt);
    float targetDrivePwm = clampDrivePwm(drivePidOut, distError);
    float maxStep = DRIVE_RAMP_PWM_PER_SEC * dt;
    if (targetDrivePwm > currentDrivePwm + maxStep) currentDrivePwm += maxStep;
    else if (targetDrivePwm < currentDrivePwm - maxStep) currentDrivePwm -= maxStep;
    else currentDrivePwm = targetDrivePwm;
    if (currentDrivePwm < 0.0f) currentDrivePwm = 0.0f;

    float yawForControl = yawErr;
    if (fabs(yawForControl) < YAW_DEADBAND_DEG) yawForControl = 0.0f;
    yawForControl = constrain(yawForControl, -YAW_ERROR_LIMIT_DEG, YAW_ERROR_LIMIT_DEG);

    float corrRaw = YAW_MOTOR_SIGN * YAW_KP * yawForControl;
    float dynamicCorrMax = min((float)YAW_CORR_MAX, currentDrivePwm * 0.20f);
    corrRaw = constrain(corrRaw, -dynamicCorrMax, dynamicCorrMax);
    float targetCorr = filteredCorr + CORR_FILTER_ALPHA * (corrRaw - filteredCorr);
    float maxCorrStep = CORR_SLEW_PWM_PER_SEC * dt;
    if (targetCorr > filteredCorr + maxCorrStep) filteredCorr += maxCorrStep;
    else if (targetCorr < filteredCorr - maxCorrStep) filteredCorr -= maxCorrStep;
    else filteredCorr = targetCorr;
    float corr = constrain(filteredCorr, -dynamicCorrMax, dynamicCorrMax);

    float base = FORWARD_SIGN * currentDrivePwm;
    setWheelPwm((base + corr) * LEFT_FORWARD_TRIM, (base - corr) * RIGHT_FORWARD_TRIM);
    drawTelemetry(status, distError, corr);

    if (distError <= 0.0f) {
      stopMotors();
      return true;
    }
    delay(10);
  }

  stopMotors();
  return false;
}

bool turnToYaw(float targetYaw, unsigned long timeoutMs, const char *status) {
  unsigned long startMs = millis();
  unsigned long lastMs = millis();
  unsigned long stableSince = 0;
  float lastError = angleError(targetYaw, yawDeg());
  float filteredTurn = 0.0f;

  while (millis() - startMs < timeoutMs) {
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastMs = now;

    float error = angleError(targetYaw, yawDeg());
    if (fabs(error) < TURN_DONE_TOL_DEG) {
      stopMotors();
      if (stableSince == 0) stableSince = now;
      if (now - stableSince >= TURN_STABLE_MS) return true;
    } else {
      stableSince = 0;
      float controlError = error;
      if (fabs(controlError) < TURN_DEADBAND_DEG) controlError = 0.0f;
      controlError = constrain(controlError, -TURN_ERROR_LIMIT_DEG, TURN_ERROR_LIMIT_DEG);

      float derivative = (error - lastError) / dt;
      lastError = error;
      float rawTurn = TURN_MOTOR_SIGN * (TURN_KP * controlError + TURN_KD * derivative);

      int maxPwm = TURN_PWM_MAX;
      if (fabs(error) < 25.0f) maxPwm = TURN_PWM_NEAR;
      if (fabs(error) < 8.0f) maxPwm = TURN_PWM_FINE;
      rawTurn = constrain(rawTurn, -maxPwm, maxPwm);
      if (rawTurn > 0.0f && rawTurn < TURN_PWM_MIN) rawTurn = TURN_PWM_MIN;
      if (rawTurn < 0.0f && rawTurn > -TURN_PWM_MIN) rawTurn = -TURN_PWM_MIN;

      float targetTurn = filteredTurn + TURN_FILTER_ALPHA * (rawTurn - filteredTurn);
      float maxStep = TURN_SLEW_PWM_PER_SEC * dt;
      if (targetTurn > filteredTurn + maxStep) filteredTurn += maxStep;
      else if (targetTurn < filteredTurn - maxStep) filteredTurn -= maxStep;
      else filteredTurn = targetTurn;

      setWheelPwm(filteredTurn, -filteredTurn);
      drawTelemetry(status, error, filteredTurn);
    }
    delay(10);
  }

  stopMotors();
  return false;
}

bool turnRelative(float deltaDeg, const char *status) {
  float targetYaw = normalizeDeg(yawDeg() + deltaDeg);
  return turnToYaw(targetYaw, TURN_TIMEOUT_MS, status);
}

bool executeCommand(char command, uint8_t index, uint8_t total) {
  char status[17];
  command = tolower(command);
  snprintf(status, sizeof(status), "T4 %c %u/%u", toupper(command), index + 1, total);

  if (command == 'f') return driveCell(status);
  if (command == 'l') return turnRelative(90.0f, status);
  if (command == 'r') return turnRelative(-90.0f, status);
  stopMotors();
  return false;
}

// ------------------------- Setup -------------------------
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

  Wire.begin();
  Wire.setClock(100000);
  oled.begin();
  oled.drawLine(0, "T4 CHAIN");

  allLidarsOff();
  byte imuStatus = mpu.begin();
  imuOk = (imuStatus == 0);
  oled.drawLine(1, imuOk ? "IMU OK" : "IMU FAIL");
  if (imuOk) {
    oled.drawLine(2, "KEEP STILL");
    delay(1000);
    mpu.calcOffsets(true, true);
  }

  startLidar(lidarLeft, LIDAR_LEFT_XSHUT, ADDR_LEFT);
  lidarLeftOk = true;
  startLidar(lidarFront, LIDAR_FRONT_XSHUT, ADDR_FRONT);
  lidarFrontOk = true;
  startLidar(lidarRight, LIDAR_RIGHT_XSHUT, ADDR_RIGHT);
  lidarRightOk = true;

  settle(1200, "T4 READY");
}

// -------------------------- Loop -------------------------
void loop() {
  static bool hasRun = false;
  static bool taskOk = false;

  if (!hasRun) {
    hasRun = true;
    uint8_t total = min((uint8_t)8, (uint8_t)strlen(COMMAND_STRING));
    uint8_t completed = 0;

    for (uint8_t i = 0; i < total; ++i) {
      bool ok = executeCommand(COMMAND_STRING[i], i, total);
      if (!ok) break;
      completed++;
      settle(250, "T4 NEXT");
    }

    char result[17];
    snprintf(result, sizeof(result), "T4 DONE %u/%u", completed, total);
    settle(1000, result);
    taskOk = (completed == total);
  }

  stopMotors();
  drawTelemetry(taskOk ? "T4 DONE" : "T4 CHECK", 0, 0);
  delay(20);
}
