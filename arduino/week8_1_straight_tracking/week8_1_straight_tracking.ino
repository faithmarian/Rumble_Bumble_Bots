/*
  MTRN3100 Week 8 Task 3.1 - Straight-line tracking.

  Standalone sketch. No custom .hpp library.

  Control:
    - Encoder distance PID controls forward PWM.
    - MPU6050 yaw PID keeps the robot on the starting heading.
    - OLED shows IMU X/Y/Z and LiDAR L/F/R in real time.

  If heading correction is backwards:
    - Change YAW_MOTOR_SIGN between -1 and +1.
    - This does NOT make one-way correction; it only maps IMU sign to motor sign.
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
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

// ---------------------- Assessment ----------------------
const float TARGET_DISTANCE_MM = 1050.0f;  // slightly past the 1 m finish line
const unsigned long RUN_TIMEOUT_MS = 30000UL;

// -------------------- Robot Constants -------------------
const float WHEEL_RADIUS_MM = 32.0f;
const float ENCODER_COUNTS_PER_REV = 1400.0f;
const float MM_PER_COUNT = (2.0f * PI * WHEEL_RADIUS_MM) / ENCODER_COUNTS_PER_REV;

// Your test showed positive equal commands drive backward, so forward is negative.
const float FORWARD_SIGN = -1.0f;

// Keep trims neutral; heading is corrected continuously by the yaw PID.
const float LEFT_FORWARD_TRIM = 1.00f;
const float RIGHT_FORWARD_TRIM = 1.00f;

// This only calibrates the sign between IMU yaw and motor steering.
// The PID correction itself is bidirectional: positive and negative yaw errors
// produce opposite wheel corrections.
const float YAW_MOTOR_SIGN = 1.0f;

// ------------------------ PID Tune -----------------------
// Distance PID is deliberately clamped so it cannot suddenly accelerate.
const float DIST_KP = 0.16f;
const float DIST_KI = 0.00f;
const float DIST_KD = 0.02f;

// Start conservative. If heading error decays too slowly, increase YAW_KP.
// If it snakes/oscillates, decrease YAW_KP or YAW_KD.
const float YAW_KP = 0.62f;
const float YAW_KI = 0.00f;
const float YAW_KD = 0.00f;

const int DRIVE_PWM_MIN = 38;
const int DRIVE_PWM_CRUISE = 64;
const int DRIVE_PWM_SLOW = 45;
const int DRIVE_PWM_FINAL = 34;
const int YAW_CORR_MAX = 9;
const float DRIVE_RAMP_PWM_PER_SEC = 45.0f;
const float CORR_FILTER_ALPHA = 0.12f;
const float CORR_SLEW_PWM_PER_SEC = 18.0f;
const float YAW_DEADBAND_DEG = 0.8f;
const float YAW_ERROR_LIMIT_DEG = 14.0f;

// ------------------------- Devices -----------------------
MPU6050 mpu(Wire);
VL6180X lidarLeft;
VL6180X lidarFront;
VL6180X lidarRight;

const uint8_t ADDR_LEFT = 0x54;
const uint8_t ADDR_FRONT = 0x56;
const uint8_t ADDR_RIGHT = 0x55;
const uint8_t OLED_ADDR = 0x3C;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

bool imuOk = false;
bool lidarLeftOk = false;
bool lidarFrontOk = false;
bool lidarRightOk = false;

unsigned long lastDisplayMs = 0;
unsigned long lastSerialMs = 0;

// ------------------------- OLED --------------------------
// Same tiny SSD1306 text code as the diagnostic sketch, kept inside this .ino.
const uint8_t FONT_5X7[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00, 0x14,0x7F,0x14,0x7F,0x14,
  0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62, 0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00,
  0x00,0x1C,0x22,0x41,0x00, 0x00,0x41,0x22,0x1C,0x00, 0x14,0x08,0x3E,0x08,0x14, 0x08,0x08,0x3E,0x08,0x08,
  0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00, 0x20,0x10,0x08,0x04,0x02,
  0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00, 0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31,
  0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
  0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00, 0x00,0x56,0x36,0x00,0x00,
  0x08,0x14,0x22,0x41,0x00, 0x14,0x14,0x14,0x14,0x14, 0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x51,0x09,0x06,
  0x32,0x49,0x79,0x41,0x3E, 0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
  0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01, 0x3E,0x41,0x49,0x49,0x7A,
  0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41,
  0x7F,0x40,0x40,0x40,0x40, 0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
  0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46, 0x46,0x49,0x49,0x49,0x31,
  0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F, 0x3F,0x40,0x38,0x40,0x3F,
  0x63,0x14,0x08,0x14,0x63, 0x07,0x08,0x70,0x08,0x07, 0x61,0x51,0x49,0x45,0x43, 0x00,0x7F,0x41,0x41,0x00,
  0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04, 0x40,0x40,0x40,0x40,0x40,
  0x00,0x01,0x02,0x04,0x00, 0x20,0x54,0x54,0x54,0x78, 0x7F,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20,
  0x38,0x44,0x44,0x48,0x7F, 0x38,0x54,0x54,0x54,0x18, 0x08,0x7E,0x09,0x01,0x02, 0x0C,0x52,0x52,0x52,0x3E,
  0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x44,0x3D,0x00, 0x7F,0x10,0x28,0x44,0x00,
  0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x18,0x04,0x78, 0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38,
  0x7C,0x14,0x14,0x14,0x08, 0x08,0x14,0x14,0x18,0x7C, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x20,
  0x04,0x3F,0x44,0x40,0x20, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C, 0x3C,0x40,0x30,0x40,0x3C,
  0x44,0x28,0x10,0x28,0x44, 0x0C,0x50,0x50,0x50,0x3C, 0x44,0x64,0x54,0x4C,0x44, 0x00,0x08,0x36,0x41,0x00,
  0x00,0x00,0x7F,0x00,0x00, 0x00,0x41,0x36,0x08,0x00, 0x10,0x08,0x08,0x10,0x08
};

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

  void drawChar(char c) {
    if (c < 32 || c > 126) c = ' ';
    uint16_t offset = (uint16_t)(c - 32) * 5;
    for (uint8_t i = 0; i < 5; ++i) data(pgm_read_byte(&FONT_5X7[offset + i]));
    data(0); data(0); data(0);
  }
};

TinyOLED oled;

// ------------------------- PID ---------------------------
struct PID {
  float kp, ki, kd;
  float integral;
  float lastError;
  float integralLimit;
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
PID yawPid(YAW_KP, YAW_KI, YAW_KD, 80.0f);

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
  float l = fabs((atomicLeftTicks()) * MM_PER_COUNT);
  float r = fabs((atomicRightTicks()) * MM_PER_COUNT);
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

void drawTelemetry(const char *status, float yawErr, float corr) {
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
  dtostrf(yawErr, 5, 1, num);
  snprintf(line, sizeof(line), "YE:%s C:%3d", num, (int)corr);
  oled.drawLine(4, line);
  snprintf(line, sizeof(line), "L:%4u F:%4u", l, f);
  oled.drawLine(6, line);
  snprintf(line, sizeof(line), "R:%4u MM", r);
  oled.drawLine(7, line);
}

int clampDrivePwm(float pidOut, float distError) {
  if (distError <= 0.0f) return 0;
  int maxPwm = DRIVE_PWM_CRUISE;
  if (distError < 250.0f) maxPwm = DRIVE_PWM_SLOW;
  if (distError < 80.0f) maxPwm = DRIVE_PWM_FINAL;

  int pwm = (int)round(constrain(pidOut, 0.0f, (float)maxPwm));
  if (pwm > 0 && pwm < DRIVE_PWM_MIN && distError > 80.0f) pwm = DRIVE_PWM_MIN;
  return pwm;
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
  oled.drawLine(0, "T1 STRAIGHT");

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

  delay(1200);
  oled.clear();
}

// -------------------------- Loop -------------------------
void loop() {
  static bool started = false;
  static bool finished = false;
  static float targetYaw = 0.0f;
  static unsigned long startMs = 0;
  static unsigned long lastMs = 0;
  static float currentDrivePwm = 0.0f;
  static float filteredCorr = 0.0f;

  if (finished) {
    stopMotors();
    drawTelemetry("T1 DONE", 0, 0);
    delay(20);
    return;
  }

  if (!started) {
    started = true;
    resetEncoders();
    distancePid.reset();
    yawPid.reset();
    targetYaw = yawDeg();
    startMs = millis();
    lastMs = millis();
    currentDrivePwm = 0.0f;
    filteredCorr = 0.0f;
    oled.drawLine(0, "T1 RUN");
  }

  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  float travelled = absTravelMm();
  float distError = TARGET_DISTANCE_MM - travelled;
  float currentYaw = yawDeg();
  float yawErr = angleError(targetYaw, currentYaw);

  float drivePidOut = distancePid.update(distError, dt);
  float targetDrivePwm = clampDrivePwm(drivePidOut, distError);
  float maxStep = DRIVE_RAMP_PWM_PER_SEC * dt;
  if (targetDrivePwm > currentDrivePwm + maxStep) {
    currentDrivePwm += maxStep;
  } else if (targetDrivePwm < currentDrivePwm - maxStep) {
    currentDrivePwm -= maxStep;
  } else {
    currentDrivePwm = targetDrivePwm;
  }
  if (currentDrivePwm < 0.0f) currentDrivePwm = 0.0f;

  float yawForControl = yawErr;
  if (fabs(yawForControl) < YAW_DEADBAND_DEG) {
    yawForControl = 0.0f;
  }
  yawForControl = constrain(yawForControl, -YAW_ERROR_LIMIT_DEG, YAW_ERROR_LIMIT_DEG);

  float corrRaw = YAW_MOTOR_SIGN * YAW_KP * yawForControl;
  float dynamicCorrMax = min((float)YAW_CORR_MAX, currentDrivePwm * 0.20f);
  corrRaw = constrain(corrRaw, -dynamicCorrMax, dynamicCorrMax);

  float targetCorr = filteredCorr + CORR_FILTER_ALPHA * (corrRaw - filteredCorr);
  float maxCorrStep = CORR_SLEW_PWM_PER_SEC * dt;
  if (targetCorr > filteredCorr + maxCorrStep) {
    filteredCorr += maxCorrStep;
  } else if (targetCorr < filteredCorr - maxCorrStep) {
    filteredCorr -= maxCorrStep;
  } else {
    filteredCorr = targetCorr;
  }
  float corr = constrain(filteredCorr, -dynamicCorrMax, dynamicCorrMax);

  float base = FORWARD_SIGN * currentDrivePwm;
  float leftCmd = (base + corr) * LEFT_FORWARD_TRIM;
  float rightCmd = (base - corr) * RIGHT_FORWARD_TRIM;
  setWheelPwm(leftCmd, rightCmd);

  drawTelemetry("T1 RUN", yawErr, corr);

  if (now - lastSerialMs > 120) {
    lastSerialMs = now;
    Serial.print(F("mm="));
    Serial.print(travelled, 1);
    Serial.print(F(" distErr="));
    Serial.print(distError, 1);
    Serial.print(F(" yawErr="));
    Serial.print(yawErr, 2);
    Serial.print(F(" drive="));
    Serial.print(currentDrivePwm, 1);
    Serial.print(F(" rawC="));
    Serial.print(corrRaw, 1);
    Serial.print(F(" corr="));
    Serial.print(corr, 1);
    Serial.print(F(" Lcmd="));
    Serial.print(leftCmd, 1);
    Serial.print(F(" Rcmd="));
    Serial.println(rightCmd, 1);
  }

  if (distError <= 0.0f || now - startMs > RUN_TIMEOUT_MS) {
    stopMotors();
    finished = true;
    oled.drawLine(0, distError <= 0.0f ? "T1 DONE" : "T1 TIMEOUT");
  }

  delay(10);
}
