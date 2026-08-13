#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <math.h>
#include <string.h>

// Change only these values on marking day.
const int8_t START_ROW = 8;
const int8_t START_COLUMN = 2;
const uint8_t START_HEADING = 0;  // 0=N, 1=E, 2=S, 3=W
const int8_t GOAL_ROW = 4;
const int8_t GOAL_COLUMN = 4;
const uint8_t GOAL_HEADING = 1;   // Final robot heading at the goal

const uint8_t ROWS = 9;
const uint8_t COLUMNS = 9;
const uint8_t CELL_COUNT = ROWS * COLUMNS;
const uint8_t NAVIGABLE_CELL_COUNT = CELL_COUNT - 12;
const int8_t DR[4] = {-1, 0, 1, 0};
const int8_t DC[4] = {0, 1, 0, -1};
const char HEADING_NAME[4] = {'N', 'E', 'S', 'W'};
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

// Week 8 Task 4 motion values.
const float CELL_MM = 180.0f;
const float ROBOT_SIZE_MM = 76.0f;
const float WALL_GAP_MM = (CELL_MM - ROBOT_SIZE_MM) / 2.0f;
const float WHEEL_DIAMETER_MM = 44.0f;
const float ENCODER_COUNTS_PER_REV = 356.72f;
const float MM_PER_COUNT = PI * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV;
const int DRIVE_PWM = 42;
const int DRIVE_PWM_FINAL = 28;
const int DRIVE_PWM_TRIM = 22;
const float DRIVE_PWM_RAMP = 12.0f;
const float DRIVE_PWM_BRAKE = 50.0f;
const float DISTANCE_KP = 0.40f;
const float DISTANCE_KD = 0.08f;
const float YAW_KP = 0.62f;
const float YAW_KD = 0.05f;
const float ENCODER_KP = 0.04f;
const float WALL_KP = 0.15f;
const float CORRECTION_RAMP = 12.0f;
const int CORRECTION_MAX = 10;
const uint16_t SIDE_WALL_MAX_MM = 95;
const uint16_t FRONT_WALL_MAX_MM = 120;
const float WALL_HEADING_BASELINE_MM = 35.0f;
const float WALL_HEADING_GAIN = 0.50f;
const float TURN_KP = 0.65f;
const float TURN_KD = 0.12f;
const int TURN_PWM_MIN = 20;
const int TURN_PWM_MAX = 28;
const float TURN_PWM_RAMP = 36.0f;
const float TURN_TOLERANCE_DEG = 1.5f;
const unsigned long TURN_STABLE_MS = 250;
const unsigned long CELL_SETTLE_MS = 60;
const unsigned long ACTION_PAUSE_MS = 80;
const unsigned long PHASE_PAUSE_MS = 150;
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

class TinyOLED {
public:
  void begin() {
    const uint8_t init[] = {
      0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,0x20,0x02,
      0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF
    };
    memset(cache, 0xFF, sizeof(cache));
    for (uint8_t i = 0; i < sizeof(init); ++i) command(init[i]);
    for (uint8_t row = 0; row < 8; ++row) drawLine(row, "");
  }

  void drawLine(uint8_t row, const char *text) {
    char next[17] = {0};
    strncpy(next, text, 16);
    if (memcmp(cache[row], next, sizeof(next)) == 0) return;
    memcpy(cache[row], next, sizeof(next));
    uint8_t pixels[128] = {0};
    for (uint8_t column = 0; column < 16 && next[column]; ++column) {
      uint8_t glyph[5];
      getGlyph(next[column], glyph);
      for (uint8_t i = 0; i < 5; ++i) pixels[column * 8 + i] = glyph[i];
    }
    command(0xB0 | (row & 7)); command(0x00); command(0x10);
    for (uint8_t start = 0; start < 128; start += 16) {
      Wire.beginTransmission(OLED_ADDR);
      Wire.write(0x40);
      for (uint8_t i = 0; i < 16; ++i) Wire.write(pixels[start + i]);
      Wire.endTransmission();
    }
  }

private:
  char cache[8][17];

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

float travelledMm() {
  long left, right; readEncoders(left, right);
  return 0.5f * (labs(left) + labs(right)) * MM_PER_COUNT;
}

float encoderCorrection() {
  long left, right; readEncoders(left, right);
  return constrain(ENCODER_KP * ((float)labs(left) - (float)labs(right)), -3.0f, 3.0f);
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float yaw() { mpu.update(); return mpu.getAngleZ(); }

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

float wallCorrection() {
  bool left = sideWall(leftMm), right = sideWall(rightMm);
  if (left && (!right || leftMm <= rightMm)) return WALL_KP * ((float)leftMm - WALL_GAP_MM);
  if (right) return WALL_KP * (WALL_GAP_MM - (float)rightMm);
  return 0.0f;
}

uint8_t completionPercent() {
  return (uint16_t)visitedCount * 100 / NAVIGABLE_CELL_COUNT;
}

char edgeSymbol(uint8_t direction) {
  uint8_t bit = 1 << direction;
  uint8_t value = cellMap[robotRow][robotColumn];
  if (!(value & (bit << 4))) return '?';
  return value & bit ? 'X' : 'O';
}

void drawTelemetry(const char *status) {
  if (millis() - lastDisplayMs < 150) return;
  lastDisplayMs = millis();
  char line[17];
  oled.drawLine(0, status);
  snprintf(line, sizeof(line), "MAP:%2u/%2u %3uP", visitedCount, NAVIGABLE_CELL_COUNT, completionPercent());
  oled.drawLine(1, line);
  snprintf(line, sizeof(line), "POS:%d,%d H:%c", robotRow, robotColumn, HEADING_NAME[robotHeading]);
  oled.drawLine(2, line);
  snprintf(line, sizeof(line), "N:%c E:%c S:%c W:%c",
           edgeSymbol(0), edgeSymbol(1), edgeSymbol(2), edgeSymbol(3));
  oled.drawLine(3, line);
  snprintf(line, sizeof(line), "GOAL:%d,%d H:%c", GOAL_ROW, GOAL_COLUMN, HEADING_NAME[GOAL_HEADING]);
  oled.drawLine(4, line);
  snprintf(line, sizeof(line), "L:%3u F:%3u", leftMm, frontMm); oled.drawLine(5, line);
  snprintf(line, sizeof(line), "R:%3u Z:%4ld", rightMm, lround(yaw())); oled.drawLine(6, line);
  snprintf(line, sizeof(line), "PH:%c VIS:%2u", phaseCode, visitedCount); oled.drawLine(7, line);
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
  mpu.begin(); oled.drawLine(0, "KEEP STILL"); delay(1000); mpu.calcOffsets(true, true);
  startLidar(lidarLeft, LIDAR_LEFT_XSHUT, ADDR_LEFT);
  startLidar(lidarFront, LIDAR_FRONT_XSHUT, ADDR_FRONT);
  startLidar(lidarRight, LIDAR_RIGHT_XSHUT, ADDR_RIGHT);
}

bool turnTo(float target, const char *status) {
  unsigned long lastMs = millis(), stableSince = 0;
  float lastError = normalizeAngle(target - yaw()), output = 0.0f;
  while (true) {
    unsigned long now = millis();
    float dt = constrain((now - lastMs) / 1000.0f, 0.001f, 0.05f);
    lastMs = now;
    float error = normalizeAngle(target - yaw());
    if (fabs(error) <= TURN_TOLERANCE_DEG) {
      stopMotors(); output = 0.0f;
      if (stableSince == 0) stableSince = now;
      if (now - stableSince >= TURN_STABLE_MS) return true;
    } else {
      stableSince = 0;
      float derivative = normalizeAngle(error - lastError) / dt;
      float closingRate = max(0.0f, -(error * derivative) / fabs(error));
      float magnitude = TURN_KP * fabs(error) - TURN_KD * closingRate;
      float requested = 0.0f;
      if (magnitude > 0.0f) {
        magnitude = constrain(magnitude, (float)TURN_PWM_MIN, (float)TURN_PWM_MAX);
        requested = error > 0 ? magnitude : -magnitude;
      }
      if (output * requested < 0) output = 0;
      if (fabs(requested) < fabs(output)) output = requested;
      else output += constrain(requested - output, -TURN_PWM_RAMP * dt, TURN_PWM_RAMP * dt);
      if (output == 0) stopMotors(); else setWheels(output, -output);
    }
    lastError = error;
    updateLidars(80); drawTelemetry(status); delay(10);
  }
}

void turnToDirection(uint8_t direction, const char *status) {
  targetYaw = normalizeAngle(northYaw + HEADING_YAW[direction]);
  if (direction != robotHeading || fabs(normalizeAngle(targetYaw - yaw())) > TURN_TOLERANCE_DEG) {
    turnTo(targetYaw, status);
  }
  robotHeading = direction;
}

bool driveCell(const char *status) {
  resetEncoders();
  float pwm = 0.0f, correction = 0.0f;
  float driveYaw = targetYaw;
  float lastYawError = normalizeAngle(driveYaw - yaw());
  float lastDistance = 0.0f, wallReference = 0.0f, wallReferenceDistance = 0.0f;
  int8_t trackedWall = 0;
  unsigned long lastMs = millis();

  while (true) {
    unsigned long now = millis();
    float dt = max((now - lastMs) / 1000.0f, 0.001f);
    lastMs = now;
    float distance = travelledMm();
    float remaining = CELL_MM - distance;
    float speed = max(0.0f, (distance - lastDistance) / dt);
    lastDistance = distance;
    updateLidars(30);

    int8_t currentWall = 0;
    float currentWallMm = 0.0f;
    if (trackedWall > 0 && sideWall(leftMm)) { currentWall = 1; currentWallMm = leftMm; }
    else if (trackedWall < 0 && sideWall(rightMm)) { currentWall = -1; currentWallMm = rightMm; }
    else if (sideWall(leftMm) && (!sideWall(rightMm) || leftMm <= rightMm)) { currentWall = 1; currentWallMm = leftMm; }
    else if (sideWall(rightMm)) { currentWall = -1; currentWallMm = rightMm; }

    bool yawChanged = false;
    if (currentWall == 0 || fabs(currentWallMm - WALL_GAP_MM) > 8.0f) trackedWall = 0;
    else if (currentWall != trackedWall) {
      trackedWall = currentWall; wallReference = currentWallMm; wallReferenceDistance = distance;
    } else if (distance - wallReferenceDistance >= WALL_HEADING_BASELINE_MM) {
      float travel = distance - wallReferenceDistance;
      float change = trackedWall > 0 ? currentWallMm - wallReference : wallReference - currentWallMm;
      if (fabs(change) <= 8.0f) {
        float wallAngle = atan2(change, travel) * RAD_TO_DEG;
        driveYaw = normalizeAngle(driveYaw + constrain(WALL_HEADING_GAIN * wallAngle, -1.25f, 1.25f));
        yawChanged = true;
      }
      wallReference = currentWallMm; wallReferenceDistance = distance;
    }

    bool front = frontWall(frontMm);
    if ((front && frontMm <= WALL_GAP_MM) || remaining <= 0.5f) break;

    int requestedPwm = (int)round(constrain(DISTANCE_KP * remaining - DISTANCE_KD * speed,
                                             (float)DRIVE_PWM_TRIM, (float)DRIVE_PWM));
    if (remaining <= 15.0f || (front && frontMm < 65)) requestedPwm = DRIVE_PWM_TRIM;
    else if (remaining <= 60.0f || (front && frontMm < 90)) requestedPwm = DRIVE_PWM_FINAL;
    float ramp = requestedPwm < pwm ? DRIVE_PWM_BRAKE : DRIVE_PWM_RAMP;
    pwm += constrain((float)requestedPwm - pwm, -ramp * dt, ramp * dt);

    float error = normalizeAngle(driveYaw - yaw());
    float derivative = yawChanged ? 0.0f : normalizeAngle(error - lastYawError) / dt;
    lastYawError = error;
    float yawPd = fabs(error) < 0.6f ? 0.0f : YAW_KP * error + YAW_KD * derivative;
    float requestedCorrection = constrain(yawPd + encoderCorrection() + wallCorrection(),
                                           -(float)CORRECTION_MAX, (float)CORRECTION_MAX);
    correction += constrain(requestedCorrection - correction,
                            -CORRECTION_RAMP * dt, CORRECTION_RAMP * dt);
    setWheels(-pwm + correction, -pwm - correction);
    drawTelemetry(status); delay(10);
  }
  stopMotors();
  return travelledMm() >= 150.0f;
}

bool inside(int8_t row, int8_t column) {
  return row >= 0 && row < ROWS && column >= 0 && column < COLUMNS;
}

bool navigableCell(int8_t row, int8_t column) {
  if (!inside(row, column)) return false;
  uint8_t edgeRow = min(row, ROWS - 1 - row);
  uint8_t edgeColumn = min(column, COLUMNS - 1 - column);
  return edgeRow + edgeColumn >= 2;
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
  memset(dfsParent, -1, sizeof(dfsParent));
  for (uint8_t row = 0; row < ROWS; ++row) {
    for (uint8_t column = 0; column < COLUMNS; ++column) {
      if (!navigableCell(row, column)) continue;
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

void sendMap() {
  Serial.print(F("MAP,"));
  Serial.print((int)robotRow); Serial.print(','); Serial.print((int)robotColumn); Serial.print(',');
  Serial.print(robotHeading); Serial.print(','); Serial.print(visitedCount); Serial.print(',');
  Serial.print(NAVIGABLE_CELL_COUNT); Serial.print(','); Serial.print(completionPercent()); Serial.print(',');
  Serial.print(phaseCode); Serial.print(',');
  for (uint8_t row = 0; row < ROWS; ++row)
    for (uint8_t column = 0; column < COLUMNS; ++column)
      Serial.print(cellMap[row][column] & 0x0F, HEX);
  Serial.print(',');
  for (uint8_t row = 0; row < ROWS; ++row)
    for (uint8_t column = 0; column < COLUMNS; ++column)
      Serial.print((cellMap[row][column] >> 4) & 0x0F, HEX);
  Serial.print(',');
  for (uint8_t row = 0; row < ROWS; ++row)
    for (uint8_t column = 0; column < COLUMNS; ++column)
      Serial.print(wasVisited(row, column) ? '1' : '0');
  Serial.println();
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
  drawTelemetry("MAPPING");
  sendMap();
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
  sendMap();
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

bool moveOneCell(uint8_t direction, const char *status) {
  turnToDirection(direction, status);
  waitStopped(ACTION_PAUSE_MS, status);
  frontMm = stableRange(lidarFront);
  if (frontWall(frontMm)) {
    setEdge(robotRow, robotColumn, direction, true);
    sendMap();
    return false;
  }
  setEdge(robotRow, robotColumn, direction, false);
  if (!driveCell(status)) {
    motionFault = true;
    phaseCode = 'F';
    sendMap();
    return false;
  }
  robotRow += DR[direction];
  robotColumn += DC[direction];
  waitStopped(CELL_SETTLE_MS, status);
  return true;
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
    if (!moveOneCell(direction, "RETURN")) {
      motionFault = true;
      phaseCode = 'F';
      sendMap();
      return;
    }
    sendMap();
  }
  sendMap();
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

void sendShortestPath() {
  Serial.print(F("PATH,")); Serial.print(shortestLength + 1); Serial.print(',');
  int8_t row = START_ROW, column = START_COLUMN;
  Serial.print(cellId(row, column));
  for (uint8_t i = 0; i < shortestLength; ++i) {
    row += DR[shortestPath[i]]; column += DC[shortestPath[i]];
    Serial.print(','); Serial.print(cellId(row, column));
  }
  Serial.println();

  Serial.print(F("COMMANDS,"));
  uint8_t heading = START_HEADING;
  for (uint8_t i = 0; i < shortestLength; ++i) {
    uint8_t change = (shortestPath[i] + 4 - heading) & 3;
    if (change == 1) Serial.print('r');
    else if (change == 3) Serial.print('l');
    else if (change == 2) Serial.print(F("rr"));
    Serial.print('f');
    heading = shortestPath[i];
  }
  uint8_t finalChange = (GOAL_HEADING + 4 - heading) & 3;
  if (finalChange == 1) Serial.print('r');
  else if (finalChange == 3) Serial.print('l');
  else if (finalChange == 2) Serial.print(F("rr"));
  Serial.println();
}

void runAutonomousMapping() {
  initialiseMap();
  robotRow = START_ROW; robotColumn = START_COLUMN; robotHeading = START_HEADING;
  motionFault = false;
  float startYaw = round(yaw());
  northYaw = normalizeAngle(startYaw - HEADING_YAW[START_HEADING]);
  targetYaw = normalizeAngle(northYaw + HEADING_YAW[START_HEADING]);
  Serial.print(F("CONFIG,"));
  Serial.print(ROWS); Serial.print(','); Serial.print(COLUMNS); Serial.print(',');
  Serial.print((int)START_ROW); Serial.print(','); Serial.print((int)START_COLUMN); Serial.print(',');
  Serial.print(START_HEADING); Serial.print(','); Serial.print((int)GOAL_ROW); Serial.print(',');
  Serial.print((int)GOAL_COLUMN); Serial.print(','); Serial.println(GOAL_HEADING);

  exploreMaze();
  if (motionFault) return;
  phaseCode = 'R';
  turnToDirection(START_HEADING, "AT START");
  waitStopped(PHASE_PAUSE_MS, "MAP COMPLETE");

  if (!buildShortestPath()) {
    phaseCode = 'X'; sendMap();
    while (true) { stopMotors(); drawTelemetry("NO PATH"); delay(20); }
  }
  sendShortestPath();
  phaseCode = 'S'; sendMap();
  waitStopped(PHASE_PAUSE_MS, "SHORTEST RUN");
  for (uint8_t i = 0; i < shortestLength; ++i) {
    if (!moveOneCell(shortestPath[i], "SHORTEST")) {
      motionFault = true;
      phaseCode = 'F';
      sendMap();
      return;
    }
    sendMap();
  }
  turnToDirection(GOAL_HEADING, "GOAL HEADING");
  phaseCode = 'D'; sendMap();
}

bool configurationValid() {
  return navigableCell(START_ROW, START_COLUMN)
         && navigableCell(GOAL_ROW, GOAL_COLUMN)
         && START_HEADING < 4
         && GOAL_HEADING < 4;
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_ENC_A, INPUT_PULLUP); pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
  Wire.begin(); Wire.setClock(400000);
  oled.begin();
  if (!configurationValid()) {
    oled.drawLine(0, "CONFIG ERROR");
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
