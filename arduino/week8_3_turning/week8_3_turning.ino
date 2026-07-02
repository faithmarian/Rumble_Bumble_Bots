/*
  MTRN3100 Week 8 Task 3.3 - Turning.

  Standalone sketch. No custom .hpp library.

  Sequence:
    1. Start, turn 90 degrees clockwise, stop.
    2. Wait while demonstrator lifts/rotates the robot.
    3. When a yaw disturbance is detected, wait briefly for placement, then
       rotate back to the same setpoint.

  Current physical LiDAR mapping on this robot:
    Left  = A0
    Front = A2
    Right = A1
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <math.h>

// ------------------------- Pins -------------------------
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

// ---------------------- Turning Tune --------------------
const float FIRST_TURN_DEG = -90.0f;  // clockwise/right
const unsigned long TURN_TIMEOUT_MS = 10000UL;
const unsigned long WAIT_FOR_DISTURB_MS = 25000UL;

// If the first 90 degree turn goes the wrong way, flip this between +1 and -1.
const float TURN_MOTOR_SIGN = 1.0f;

const float TURN_KP = 1.55f;
const float TURN_KD = 0.035f;
const float TURN_ERROR_LIMIT_DEG = 90.0f;
const float TURN_DEADBAND_DEG = 0.7f;
const int TURN_PWM_MAX = 95;
const int TURN_PWM_NEAR = 52;
const int TURN_PWM_FINE = 34;
const int TURN_PWM_MIN = 28;
const float TURN_FILTER_ALPHA = 0.22f;
const float TURN_SLEW_PWM_PER_SEC = 120.0f;
const float TURN_DONE_TOL_DEG = 2.0f;
const unsigned long TURN_STABLE_MS = 450UL;

// ------------------------- Devices -----------------------
MPU6050 mpu(Wire);
VL6180X lidarLeft;
VL6180X lidarFront;
VL6180X lidarRight;

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
      case 'J': draw5(0x20,0x40,0x41,0x3F,0x01); break;
      case 'K': draw5(0x7F,0x08,0x14,0x22,0x41); break;
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

// ----------------------- Helpers -------------------------
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

void drawTelemetry(const char *status, float error, float turnCmd) {
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
  dtostrf(error, 5, 1, num);
  snprintf(line, sizeof(line), "E:%s T:%3d", num, (int)turnCmd);
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
      if (now - stableSince >= TURN_STABLE_MS) {
        return true;
      }
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
      if (targetTurn > filteredTurn + maxStep) {
        filteredTurn += maxStep;
      } else if (targetTurn < filteredTurn - maxStep) {
        filteredTurn -= maxStep;
      } else {
        filteredTurn = targetTurn;
      }

      setWheelPwm(filteredTurn, -filteredTurn);
      drawTelemetry(status, error, filteredTurn);
    }
    delay(10);
  }

  stopMotors();
  return false;
}

bool waitForExternalRotation(float targetYaw) {
  unsigned long startMs = millis();
  while (millis() - startMs < WAIT_FOR_DISTURB_MS) {
    float error = angleError(targetYaw, yawDeg());
    if (fabs(error) > 15.0f) {
      settle(1200, "T3 PUT DOWN");
      return true;
    }
    stopMotors();
    drawTelemetry("T3 WAIT TURN", error, 0);
    delay(20);
  }
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

  Wire.begin();
  Wire.setClock(100000);
  oled.begin();
  oled.drawLine(0, "T3 TURNING");

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

  settle(1200, "T3 READY");
}

// -------------------------- Loop -------------------------
void loop() {
  static bool hasRun = false;
  static bool taskOk = false;

  if (!hasRun) {
    hasRun = true;
    float targetYaw = normalizeDeg(yawDeg() + FIRST_TURN_DEG);
    bool s1 = turnToYaw(targetYaw, TURN_TIMEOUT_MS, "T3 RIGHT 90");
    settle(800, s1 ? "T3 S1 OK" : "T3 S1 FAIL");

    bool moved = false;
    bool s2 = false;
    if (s1) moved = waitForExternalRotation(targetYaw);
    if (s1 && moved) {
      s2 = turnToYaw(targetYaw, TURN_TIMEOUT_MS, "T3 RETURN");
      settle(800, s2 ? "T3 S2 OK" : "T3 S2 FAIL");
    }
    taskOk = s1 && moved && s2;
  }

  stopMotors();
  drawTelemetry(taskOk ? "T3 DONE" : "T3 CHECK", 0, 0);
  delay(20);
}
