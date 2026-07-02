#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <math.h>

namespace wk8 {

// ------------------------- Pin Map -------------------------
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

const uint8_t OLED_ADDR_PRIMARY = 0x3C;
const uint8_t OLED_ADDR_SECONDARY = 0x3D;

// These match the week4 controller convention that was already driving well.
const float WHEEL_RADIUS_MM = 32.0f;
const float ENCODER_COUNTS_PER_REV = 1400.0f;
const float MM_PER_COUNT = (2.0f * PI * WHEEL_RADIUS_MM) / ENCODER_COUNTS_PER_REV;

const float LEFT_PWM_TRIM = 1.00f;
const float RIGHT_PWM_TRIM = 1.00f;

// On this build, equal positive motor commands drive the chassis backward.
// Keep turning untouched, but invert straight-line translation commands.
const float FORWARD_COMMAND_SIGN = -1.0f;
const float DRIVE_YAW_CORRECTION_SIGN = 1.0f;

// ------------------------- Globals -------------------------
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

uint16_t lastLeftMm = 9999;
uint16_t lastFrontMm = 9999;
uint16_t lastRightMm = 9999;
unsigned long lastTelemetryMs = 0;
unsigned long lastLidarAllMs = 0;

// ------------------------- OLED ----------------------------
class TinySSD1306 {
public:
  bool begin() {
    address = detectAddress();
    bool acknowledged = (address != 0);
    if (!acknowledged) {
      address = OLED_ADDR_PRIMARY;
    }

    const uint8_t initCommands[] = {
      0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
      0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
      0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
      0x2E, 0xAF
    };

    for (uint8_t i = 0; i < sizeof(initCommands); ++i) {
      command(initCommands[i]);
    }
    clear();
    return acknowledged;
  }

  void clear() {
    for (uint8_t page = 0; page < 8; ++page) {
      clearRow(page);
    }
  }

  void clearRow(uint8_t row) {
    setCursor(0, row);
    for (uint8_t col = 0; col < 128; ++col) {
      data(0x00);
    }
  }

  void drawString(uint8_t col, uint8_t row, const char *text) {
    setCursor(col * 8, row);
    while (*text && col < 16) {
      drawChar(*text++);
      col++;
    }
  }

  void drawLine(uint8_t row, const char *text) {
    clearRow(row);
    drawString(0, row, text);
  }

  uint8_t i2cAddress() const {
    return address;
  }

private:
  uint8_t address = 0;

  uint8_t detectAddress() {
    Wire.beginTransmission(OLED_ADDR_PRIMARY);
    if (Wire.endTransmission() == 0) {
      return OLED_ADDR_PRIMARY;
    }
    Wire.beginTransmission(OLED_ADDR_SECONDARY);
    if (Wire.endTransmission() == 0) {
      return OLED_ADDR_SECONDARY;
    }
    return 0;
  }

  void command(uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(0x00);
    Wire.write(value);
    Wire.endTransmission();
  }

  void data(uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(0x40);
    Wire.write(value);
    Wire.endTransmission();
  }

  void setCursor(uint8_t x, uint8_t page) {
    command(0xB0 | (page & 0x07));
    command(0x00 | (x & 0x0F));
    command(0x10 | (x >> 4));
  }

  void draw5(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e) {
    data(a);
    data(b);
    data(c);
    data(d);
    data(e);
    data(0x00);
    data(0x00);
    data(0x00);
  }

  void drawChar(char c) {
    if (c >= 'a' && c <= 'z') {
      c -= 32;
    }

    switch (c) {
      case ' ': draw5(0x00,0x00,0x00,0x00,0x00); break;
      case '+': draw5(0x08,0x08,0x3E,0x08,0x08); break;
      case '-': draw5(0x08,0x08,0x08,0x08,0x08); break;
      case '.': draw5(0x00,0x60,0x60,0x00,0x00); break;
      case '/': draw5(0x20,0x10,0x08,0x04,0x02); break;
      case ':': draw5(0x00,0x36,0x36,0x00,0x00); break;
      case '0': draw5(0x3E,0x51,0x49,0x45,0x3E); break;
      case '1': draw5(0x00,0x42,0x7F,0x40,0x00); break;
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
      case 'I': draw5(0x00,0x41,0x7F,0x41,0x00); break;
      case 'J': draw5(0x20,0x40,0x41,0x3F,0x01); break;
      case 'K': draw5(0x7F,0x08,0x14,0x22,0x41); break;
      case 'L': draw5(0x7F,0x40,0x40,0x40,0x40); break;
      case 'M': draw5(0x7F,0x02,0x0C,0x02,0x7F); break;
      case 'N': draw5(0x7F,0x04,0x08,0x10,0x7F); break;
      case 'O': draw5(0x3E,0x41,0x41,0x41,0x3E); break;
      case 'P': draw5(0x7F,0x09,0x09,0x09,0x06); break;
      case 'Q': draw5(0x3E,0x41,0x51,0x21,0x5E); break;
      case 'R': draw5(0x7F,0x09,0x19,0x29,0x46); break;
      case 'S': draw5(0x46,0x49,0x49,0x49,0x31); break;
      case 'T': draw5(0x01,0x01,0x7F,0x01,0x01); break;
      case 'U': draw5(0x3F,0x40,0x40,0x40,0x3F); break;
      case 'V': draw5(0x1F,0x20,0x40,0x20,0x1F); break;
      case 'W': draw5(0x3F,0x40,0x38,0x40,0x3F); break;
      case 'X': draw5(0x63,0x14,0x08,0x14,0x63); break;
      case 'Y': draw5(0x07,0x08,0x70,0x08,0x07); break;
      case 'Z': draw5(0x61,0x51,0x49,0x45,0x43); break;
      default: draw5(0x00,0x00,0x00,0x00,0x00); break;
    }
  }
};

TinySSD1306 oled;

// -------------------------- PID ----------------------------
struct PID {
  float kp;
  float ki;
  float kd;
  float integralLimit;
  float outputLimit;
  float integral;
  float lastError;
  bool first;

  PID(float p, float i, float d, float iLimit, float oLimit)
      : kp(p), ki(i), kd(d), integralLimit(iLimit), outputLimit(oLimit),
        integral(0.0f), lastError(0.0f), first(true) {}

  void reset() {
    integral = 0.0f;
    lastError = 0.0f;
    first = true;
  }

  float update(float error, float dt) {
    if (dt <= 0.0f) {
      dt = 0.001f;
    }
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit);

    float derivative = first ? 0.0f : (error - lastError) / dt;
    first = false;
    lastError = error;

    return constrain(kp * error + ki * integral + kd * derivative,
                     -outputLimit, outputLimit);
  }
};

// ------------------------- Helpers -------------------------
float normalizeDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

float angleErrorDeg(float target, float current) {
  return normalizeDeg(target - current);
}

float yawDeg() {
  mpu.update();
  return normalizeDeg(mpu.getAngleZ());
}

long atomicLeftTicks() {
  noInterrupts();
  long value = leftTicks;
  interrupts();
  return value;
}

long atomicRightTicks() {
  noInterrupts();
  long value = rightTicks;
  interrupts();
  return value;
}

void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

float ticksToMm(long ticks) {
  return ticks * MM_PER_COUNT;
}

float ticksToAbsMm(long ticks) {
  return fabs(ticksToMm(ticks));
}

int applyMinPwm(float pwm, int minPwm, int maxPwm) {
  int out = (int)round(constrain(pwm, -maxPwm, maxPwm));
  if (out > 0 && out < minPwm) out = minPwm;
  if (out < 0 && out > -minPwm) out = -minPwm;
  return out;
}

void setMotorRaw(uint8_t pwmPin, uint8_t dirPin, int command) {
  command = constrain(command, -255, 255);
  digitalWrite(dirPin, command >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(command));
}

void setMotorCommand(uint8_t motorNumber, uint8_t pwmPin, uint8_t dirPin, float command) {
  if (motorNumber == 2) {
    command = -command;
  }
  setMotorRaw(pwmPin, dirPin, (int)round(command));
}

void setWheelPwm(float motor1Command, float motor2Command) {
  setMotorCommand(1, LEFT_PWM, LEFT_DIR, motor1Command * LEFT_PWM_TRIM);
  setMotorCommand(2, RIGHT_PWM, RIGHT_DIR, motor2Command * RIGHT_PWM_TRIM);
}

void stopMotors() {
  setWheelPwm(0, 0);
}

void leftEncoderISR() {
  leftTicks += digitalRead(LEFT_ENC_B) ? 1 : -1;
}

void rightEncoderISR() {
  rightTicks += digitalRead(RIGHT_ENC_B) ? 1 : -1;
}

void setAllLidarsOff() {
  pinMode(LIDAR_LEFT_XSHUT, OUTPUT);
  pinMode(LIDAR_FRONT_XSHUT, OUTPUT);
  pinMode(LIDAR_RIGHT_XSHUT, OUTPUT);
  digitalWrite(LIDAR_LEFT_XSHUT, LOW);
  digitalWrite(LIDAR_FRONT_XSHUT, LOW);
  digitalWrite(LIDAR_RIGHT_XSHUT, LOW);
  delay(20);
}

bool startLidar(VL6180X &sensor, uint8_t xshutPin, uint8_t newAddress, const __FlashStringHelper *name) {
  digitalWrite(xshutPin, HIGH);
  delay(50);
  sensor.setTimeout(80);
  sensor.init();
  sensor.configureDefault();
  sensor.setAddress(newAddress);
  delay(10);

  Serial.print(name);
  Serial.print(F(" VL6180X @0x"));
  Serial.println(newAddress, HEX);
  return true;
}

uint16_t readLidarMm(VL6180X &sensor, bool ok) {
  if (!ok) {
    return 9999;
  }
  uint16_t mm = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) {
    return 9998;
  }
  return mm;
}

uint16_t readFrontMm() {
  lastFrontMm = readLidarMm(lidarFront, lidarFrontOk);
  return lastFrontMm;
}

void readAllLidars() {
  lastLeftMm = readLidarMm(lidarLeft, lidarLeftOk);
  lastFrontMm = readLidarMm(lidarFront, lidarFrontOk);
  lastRightMm = readLidarMm(lidarRight, lidarRightOk);
  lastLidarAllMs = millis();
}

void drawFloatRow(uint8_t row, const char *label, float value) {
  char number[9];
  char line[17];
  dtostrf(value, 7, 1, number);
  snprintf(line, sizeof(line), "%s:%s DEG", label, number);
  oled.drawLine(row, line);
}

void drawMmRow(uint8_t row, const char *label, uint16_t mm) {
  char line[17];
  snprintf(line, sizeof(line), "%s:%4u MM", label, mm);
  oled.drawLine(row, line);
}

void showTelemetry(const char *status) {
  mpu.update();
  unsigned long now = millis();
  if (now - lastTelemetryMs < 250UL) {
    return;
  }
  lastTelemetryMs = now;

  readAllLidars();

  oled.drawLine(0, status);
  drawFloatRow(1, "X", imuOk ? mpu.getAngleX() : 0.0f);
  drawFloatRow(2, "Y", imuOk ? mpu.getAngleY() : 0.0f);
  drawFloatRow(3, "Z", imuOk ? mpu.getAngleZ() : 0.0f);
  drawMmRow(5, "L", lastLeftMm);
  drawMmRow(6, "F", lastFrontMm);
  drawMmRow(7, "R", lastRightMm);
}

void beginHardware(const char *title) {
  Serial.begin(115200);
  delay(200);
  Serial.println(title);

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

  bool oledOk = oled.begin();
  oled.drawLine(0, oledOk ? "OLED OK" : "OLED CHECK");
  delay(300);

  setAllLidarsOff();

  byte mpuStatus = mpu.begin();
  imuOk = (mpuStatus == 0);
  Serial.print(F("MPU6050 status: "));
  Serial.println(mpuStatus);
  oled.drawLine(1, imuOk ? "IMU OK" : "IMU FAIL");
  if (imuOk) {
    oled.drawLine(2, "KEEP STILL");
    delay(1000);
    mpu.calcOffsets(true, true);
  }

  lidarLeftOk = startLidar(lidarLeft, LIDAR_LEFT_XSHUT, ADDR_LEFT, F("Left"));
  lidarFrontOk = startLidar(lidarFront, LIDAR_FRONT_XSHUT, ADDR_FRONT, F("Front"));
  lidarRightOk = startLidar(lidarRight, LIDAR_RIGHT_XSHUT, ADDR_RIGHT, F("Right"));

  readAllLidars();
  resetEncoders();
  oled.clear();
  showTelemetry(title);
}

void settle(unsigned long ms, const char *status) {
  stopMotors();
  unsigned long start = millis();
  while (millis() - start < ms) {
    showTelemetry(status);
    delay(10);
  }
}

bool driveDistanceMm(float targetMm, float targetYaw, unsigned long timeoutMs, const char *status) {
  const long leftStart = atomicLeftTicks();
  const long rightStart = atomicRightTicks();

  PID yawPid(0.55f, 0.00f, 0.015f, 25.0f, 14.0f);

  uint8_t stable = 0;
  unsigned long startMs = millis();
  unsigned long lastMs = millis();

  while (millis() - startMs < timeoutMs) {
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    float leftMm = ticksToAbsMm(atomicLeftTicks() - leftStart);
    float rightMm = ticksToAbsMm(atomicRightTicks() - rightStart);
    float travelledMm = 0.5f * (leftMm + rightMm);
    float distError = targetMm - travelledMm;
    float yawError = angleErrorDeg(targetYaw, yawDeg());

    if (fabs(yawError) > 35.0f) {
      stopMotors();
      oled.drawLine(0, "DRIVE YAW FAIL");
      return false;
    }

    if (fabs(distError) < 4.0f && fabs(yawError) < 4.0f) {
      stopMotors();
      stable++;
      if (stable >= 8) {
        return true;
      }
    } else {
      stable = 0;

      float baseMagnitude = 58.0f;
      if (distError < 220.0f) {
        baseMagnitude = 45.0f;
      }
      if (distError < 70.0f) {
        baseMagnitude = 36.0f;
      }
      if (distError <= 0.0f) {
        baseMagnitude = 0.0f;
      }

      float base = FORWARD_COMMAND_SIGN * baseMagnitude;
      float corr = DRIVE_YAW_CORRECTION_SIGN * yawPid.update(yawError, dt);
      corr = constrain(corr, -14.0f, 14.0f);

      // Translation and heading correction are deliberately separated. The
      // previous encoder-sync term could fight the IMU and grow into a drift.
      setWheelPwm(base + corr, base - corr);
    }

    showTelemetry(status);
    delay(10);
  }

  stopMotors();
  return false;
}

bool turnToYaw(float targetYaw, unsigned long timeoutMs, const char *status) {
  PID turnPid(2.70f, 0.015f, 0.18f, 60.0f, 120.0f);
  uint8_t stable = 0;
  unsigned long startMs = millis();
  unsigned long lastMs = millis();

  while (millis() - startMs < timeoutMs) {
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    float error = angleErrorDeg(targetYaw, yawDeg());
    if (fabs(error) < 2.0f) {
      stopMotors();
      stable++;
      if (stable >= 8) {
        return true;
      }
    } else {
      stable = 0;
      float turn = turnPid.update(error, dt);
      turn = applyMinPwm(turn, 42, 120);
      setWheelPwm(turn, -turn);
    }

    showTelemetry(status);
    delay(10);
  }

  stopMotors();
  return false;
}

bool turnRelativeDeg(float deltaDeg, unsigned long timeoutMs, const char *status) {
  float targetYaw = normalizeDeg(yawDeg() + deltaDeg);
  return turnToYaw(targetYaw, timeoutMs, status);
}

}  // namespace wk8
