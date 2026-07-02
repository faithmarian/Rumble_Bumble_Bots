/*
  MTRN3100 Week 8 Task 3.2 - Driving and stopping.

  Standalone sketch. No custom .hpp library.

  Goal:
    Use the physical front VL6180X to keep the robot at 100 mm from a wall.
    The sketch keeps responding after the required two wall moves, so you can
    test by moving the wall back and forward repeatedly.

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

// ---------------------- Assessment ----------------------
const float TARGET_WALL_MM = 100.0f;
const float WALL_TOL_MM = 5.0f;
const float WALL_MOVED_BACK_THRESHOLD_MM = 135.0f;
const float WALL_MOVED_FORWARD_THRESHOLD_MM = 75.0f;
const unsigned long STABLE_HOLD_MS = 900UL;

// -------------------- Robot Control ---------------------
// Positive equal motor commands drove backward on this chassis, so forward is negative.
const float FORWARD_SIGN = -1.0f;

// Distance controller: positive distance error means "too far from wall", so move forward.
const float WALL_DIST_KP = 0.92f;
const float WALL_DIST_KD = 0.035f;
const int WALL_PWM_MIN = 26;
const int WALL_PWM_MAX = 68;
const int WALL_PWM_FINE = 34;
const float WALL_RAMP_PWM_PER_SEC = 42.0f;

// Smooth heading hold copied from the first task's stable behavior.
const float YAW_MOTOR_SIGN = 1.0f;
const float WALL_YAW_KP = 0.62f;
const int WALL_YAW_CORR_MAX = 9;
const float WALL_CORR_FILTER_ALPHA = 0.12f;
const float WALL_CORR_SLEW_PWM_PER_SEC = 18.0f;
const float WALL_YAW_DEADBAND_DEG = 0.8f;
const float WALL_YAW_ERROR_LIMIT_DEG = 14.0f;

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
unsigned long lastSerialMs = 0;

// ------------------------- OLED --------------------------
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

// ----------------------- Helpers -------------------------
void leftEncoderISR() {
  leftTicks += digitalRead(LEFT_ENC_B) ? 1 : -1;
}

void rightEncoderISR() {
  rightTicks += digitalRead(RIGHT_ENC_B) ? 1 : -1;
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

void drawTelemetry(const char *status, float error, float base, float corr) {
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
  snprintf(line, sizeof(line), "E:%s B:%3d", num, (int)base);
  oled.drawLine(4, line);
  snprintf(line, sizeof(line), "C:%3d F:%4u", (int)corr, f);
  oled.drawLine(5, line);
  snprintf(line, sizeof(line), "L:%4u R:%4u", l, r);
  oled.drawLine(7, line);
}

// ------------------- Wall Hold State ---------------------
enum Stage {
  STAGE_APPROACH = 0,
  STAGE_WAIT_BACK,
  STAGE_FOLLOW_BACK,
  STAGE_WAIT_FORWARD,
  STAGE_FOLLOW_FORWARD,
  STAGE_CONTINUOUS_HOLD
};

Stage stage = STAGE_APPROACH;
unsigned long stableSince = 0;
float targetYaw = 0.0f;
float lastDistanceError = 0.0f;
float currentBasePwm = 0.0f;
float filteredCorr = 0.0f;
unsigned long lastControlMs = 0;

const char *stageName() {
  switch (stage) {
    case STAGE_APPROACH: return "T2 APPROACH";
    case STAGE_WAIT_BACK: return "T2 WAIT BACK";
    case STAGE_FOLLOW_BACK: return "T2 FOLLOW BK";
    case STAGE_WAIT_FORWARD: return "T2 WAIT FWD";
    case STAGE_FOLLOW_FORWARD: return "T2 FOLLOW FW";
    case STAGE_CONTINUOUS_HOLD: return "T2 HOLD";
  }
  return "T2";
}

bool stableAtTarget(float error, float yawError) {
  return fabs(error) <= WALL_TOL_MM && fabs(yawError) < 8.0f;
}

void updateStage(uint16_t front, float error, float yawError) {
  unsigned long now = millis();

  if (stage == STAGE_WAIT_BACK && front > WALL_MOVED_BACK_THRESHOLD_MM) {
    stage = STAGE_FOLLOW_BACK;
    stableSince = 0;
  } else if (stage == STAGE_WAIT_FORWARD && front < WALL_MOVED_FORWARD_THRESHOLD_MM) {
    stage = STAGE_FOLLOW_FORWARD;
    stableSince = 0;
  }

  bool canAdvance =
    (stage == STAGE_APPROACH || stage == STAGE_FOLLOW_BACK || stage == STAGE_FOLLOW_FORWARD);

  if (canAdvance && stableAtTarget(error, yawError)) {
    if (stableSince == 0) stableSince = now;
    if (now - stableSince >= STABLE_HOLD_MS) {
      if (stage == STAGE_APPROACH) {
        stage = STAGE_WAIT_BACK;
      } else if (stage == STAGE_FOLLOW_BACK) {
        stage = STAGE_WAIT_FORWARD;
      } else if (stage == STAGE_FOLLOW_FORWARD) {
        stage = STAGE_CONTINUOUS_HOLD;
      }
      stableSince = 0;
    }
  } else if (!stableAtTarget(error, yawError)) {
    stableSince = 0;
  }
}

void controlWall() {
  unsigned long now = millis();
  float dt = (now - lastControlMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastControlMs = now;

  uint16_t front = readLidar(lidarFront, lidarFrontOk);
  if (front >= 9990) {
    stopMotors();
    currentBasePwm = 0.0f;
    filteredCorr = 0.0f;
    drawTelemetry("T2 NO FRONT", 0, 0, 0);
    return;
  }

  float error = constrain((float)front - TARGET_WALL_MM, -80.0f, 120.0f);
  float yawError = angleError(targetYaw, yawDeg());
  updateStage(front, error, yawError);

  float derivative = (error - lastDistanceError) / max(dt, 0.001f);
  lastDistanceError = error;

  float targetBasePwm = WALL_DIST_KP * error + WALL_DIST_KD * derivative;
  targetBasePwm = constrain(targetBasePwm, -WALL_PWM_MAX, WALL_PWM_MAX);

  if (fabs(error) <= WALL_TOL_MM) {
    targetBasePwm = 0.0f;
  } else if (fabs(error) < 25.0f) {
    targetBasePwm = constrain(targetBasePwm, -WALL_PWM_FINE, WALL_PWM_FINE);
  }

  if (targetBasePwm > 0.0f && targetBasePwm < WALL_PWM_MIN && fabs(error) > WALL_TOL_MM) {
    targetBasePwm = WALL_PWM_MIN;
  } else if (targetBasePwm < 0.0f && targetBasePwm > -WALL_PWM_MIN && fabs(error) > WALL_TOL_MM) {
    targetBasePwm = -WALL_PWM_MIN;
  }

  float maxBaseStep = WALL_RAMP_PWM_PER_SEC * dt;
  if (targetBasePwm > currentBasePwm + maxBaseStep) {
    currentBasePwm += maxBaseStep;
  } else if (targetBasePwm < currentBasePwm - maxBaseStep) {
    currentBasePwm -= maxBaseStep;
  } else {
    currentBasePwm = targetBasePwm;
  }

  float yawForControl = yawError;
  if (fabs(yawForControl) < WALL_YAW_DEADBAND_DEG) yawForControl = 0.0f;
  yawForControl = constrain(yawForControl, -WALL_YAW_ERROR_LIMIT_DEG, WALL_YAW_ERROR_LIMIT_DEG);

  float corrRaw = YAW_MOTOR_SIGN * WALL_YAW_KP * yawForControl;
  float dynamicCorrMax = min((float)WALL_YAW_CORR_MAX, fabs(currentBasePwm) * 0.20f);
  corrRaw = constrain(corrRaw, -dynamicCorrMax, dynamicCorrMax);

  float targetCorr = filteredCorr + WALL_CORR_FILTER_ALPHA * (corrRaw - filteredCorr);
  float maxCorrStep = WALL_CORR_SLEW_PWM_PER_SEC * dt;
  if (targetCorr > filteredCorr + maxCorrStep) {
    filteredCorr += maxCorrStep;
  } else if (targetCorr < filteredCorr - maxCorrStep) {
    filteredCorr -= maxCorrStep;
  } else {
    filteredCorr = targetCorr;
  }
  float corr = constrain(filteredCorr, -dynamicCorrMax, dynamicCorrMax);

  float base = FORWARD_SIGN * currentBasePwm;
  setWheelPwm(base + corr, base - corr);

  drawTelemetry(stageName(), error, currentBasePwm, corr);

  if (now - lastSerialMs > 150) {
    lastSerialMs = now;
    Serial.print(F("stage="));
    Serial.print(stageName());
    Serial.print(F(" front="));
    Serial.print(front);
    Serial.print(F(" err="));
    Serial.print(error, 1);
    Serial.print(F(" base="));
    Serial.print(currentBasePwm, 1);
    Serial.print(F(" yawErr="));
    Serial.print(yawError, 1);
    Serial.print(F(" corr="));
    Serial.println(corr, 1);
  }
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
  oled.drawLine(0, "T2 LIDAR");

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

  targetYaw = yawDeg();
  lastControlMs = millis();
  delay(1200);
  oled.clear();
}

// -------------------------- Loop -------------------------
void loop() {
  controlWall();
  delay(10);
}
