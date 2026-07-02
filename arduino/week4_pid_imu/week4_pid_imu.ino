/*
  MTRN3100 Week 4 barebones movement controller.

  Sequence:
    1. Drive forward 200 mm using encoder distance PID.
    2. Hold the initial IMU yaw during the drive using yaw PID.
    3. Turn left 90 deg four times using IMU yaw PID.
    4. Turn right 90 deg four times using IMU yaw PID.

  Hardware mapping follows the UNSW MDI controller PCB and the working
  micromouse1.2 motor/encoder logic:
    Left motor  (MOT1): PWM D11, DIR D12
    Right motor (MOT2): PWM D9,  DIR D10
    Left encoder:  A D2 interrupt, B D7
    Right encoder: A D3 interrupt, B D8
    MPU6050: I2C on A4 SDA, A5 SCL

  Required Arduino library:
    MPU6050_light by rfetick
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ------------------------- Pin Map -------------------------
const uint8_t LEFT_ENC_A = 2;
const uint8_t LEFT_ENC_B = 7;
const uint8_t RIGHT_ENC_A = 3;
const uint8_t RIGHT_ENC_B = 8;

const uint8_t LEFT_PWM = 11;
const uint8_t LEFT_DIR = 12;
const uint8_t RIGHT_PWM = 9;
const uint8_t RIGHT_DIR = 10;

// ---------------------- Robot Geometry ---------------------
// These match the working micromouse1.2 code. Numerically this is equivalent
// to the lab04 16 mm / 700 CPR pair, but it preserves the tested convention.
const float WHEEL_RADIUS_MM = 32.0f;
const float AXLE_TRACK_MM = 90.0f;
const float ENCODER_COUNTS_PER_REV = 1400.0f;
const float MM_PER_COUNT = (2.0f * PI * WHEEL_RADIUS_MM) / ENCODER_COUNTS_PER_REV;

const float DRIVE_DISTANCE_MM = 200.0f;
const float TURN_ANGLE_DEG = 90.0f;

// -------------------- Direction Calibration ----------------
// This follows micromouse1.2:
//   motor1.setPWM(+pwm), motor2.setPWM(+pwm) drives straight.
//   motor2 is inverted inside setMotorCommand() because it is mounted opposite.
//   turn left:  motor1 +pwm, motor2 -pwm.
//   turn right: motor1 -pwm, motor2 +pwm.

// Fine trim if one side is mechanically stronger. Keep near 1.0.
const float LEFT_PWM_TRIM = 1.00f;
const float RIGHT_PWM_TRIM = 1.00f;

// ------------------------ PWM Limits -----------------------
const int DRIVE_PWM_MIN = 42;
const int DRIVE_PWM_MAX = 125;
const int TURN_PWM_MIN = 42;
const int TURN_PWM_MAX = 115;
const int CORRECTION_PWM_MAX = 45;

const float DRIVE_DONE_TOL_MM = 2.0f;
const float TURN_DONE_TOL_DEG = 1.5f;
const uint8_t DONE_STABLE_CYCLES = 10;

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

    float derivative = 0.0f;
    if (!first) {
      derivative = (error - lastError) / dt;
    }

    first = false;
    lastError = error;

    float output = kp * error + ki * integral + kd * derivative;
    return constrain(output, -outputLimit, outputLimit);
  }
};

// Distance error is in mm, yaw error is in deg.
PID distancePid(1.15f, 0.00f, 0.10f, 80.0f, DRIVE_PWM_MAX);
PID driveYawPid(2.30f, 0.00f, 0.10f, 45.0f, CORRECTION_PWM_MAX);
PID wheelSyncPid(0.28f, 0.00f, 0.02f, 80.0f, 18.0f);
PID turnYawPid(2.70f, 0.015f, 0.18f, 60.0f, TURN_PWM_MAX);

// ------------------------ Globals --------------------------
MPU6050 mpu(Wire);

volatile long leftTicks = 0;
volatile long rightTicks = 0;

float yawZeroDeg = 0.0f;
unsigned long lastControlMs = 0;

// ---------------------- Small Helpers ----------------------
float normalizeDeg(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle <= -180.0f) {
    angle += 360.0f;
  }
  return angle;
}

float yawDeg() {
  mpu.update();
  return normalizeDeg(mpu.getAngleZ() - yawZeroDeg);
}

float angleErrorDeg(float target, float current) {
  return normalizeDeg(target - current);
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

float ticksToMm(long ticks) {
  return ticks * MM_PER_COUNT;
}

float ticksToAbsMm(long ticks) {
  return fabs(ticksToMm(ticks));
}

int applyMinPwm(float pwm, int minPwm, int maxPwm) {
  int out = (int)round(constrain(pwm, -maxPwm, maxPwm));
  if (out > 0 && out < minPwm) {
    out = minPwm;
  } else if (out < 0 && out > -minPwm) {
    out = -minPwm;
  }
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

void settleRobot(unsigned long ms = 350) {
  stopMotors();
  unsigned long start = millis();
  while (millis() - start < ms) {
    mpu.update();
    delay(5);
  }
}

// ---------------------- Encoder ISRs -----------------------
void leftEncoderISR() {
  leftTicks += digitalRead(LEFT_ENC_B) ? 1 : -1;
}

void rightEncoderISR() {
  rightTicks += digitalRead(RIGHT_ENC_B) ? 1 : -1;
}

// --------------------- Motion Primitives -------------------
void driveDistanceMm(float targetMm) {
  const long leftStart = atomicLeftTicks();
  const long rightStart = atomicRightTicks();
  const float targetYaw = yawDeg();

  distancePid.reset();
  driveYawPid.reset();
  wheelSyncPid.reset();

  uint8_t stable = 0;
  unsigned long startMs = millis();
  unsigned long lastMs = millis();
  unsigned long lastPrintMs = 0;
  float lastBase = 0.0f;
  float lastCorr = 0.0f;

  Serial.print(F("Drive target mm: "));
  Serial.println(targetMm, 1);

  while (millis() - startMs < 8000UL) {
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    long leftDelta = atomicLeftTicks() - leftStart;
    long rightDelta = atomicRightTicks() - rightStart;
    float leftMm = ticksToAbsMm(leftDelta);
    float rightMm = ticksToAbsMm(rightDelta);
    float travelledMm = 0.5f * (leftMm + rightMm);

    float distError = targetMm - travelledMm;
    float currentYaw = yawDeg();
    float yawError = angleErrorDeg(targetYaw, currentYaw);
    float syncError = leftMm - rightMm;

    if (abs(distError) < DRIVE_DONE_TOL_MM && abs(yawError) < 3.0f) {
      stopMotors();
      lastBase = 0.0f;
      lastCorr = 0.0f;
      stable++;
      if (stable >= DONE_STABLE_CYCLES) {
        break;
      }
    } else {
      stable = 0;

      float base = distancePid.update(distError, dt);
      base = applyMinPwm(base, DRIVE_PWM_MIN, DRIVE_PWM_MAX);

      float yawCorr = driveYawPid.update(yawError, dt);
      float syncCorr = wheelSyncPid.update(syncError, dt);
      float corr = constrain(yawCorr + syncCorr, -CORRECTION_PWM_MAX, CORRECTION_PWM_MAX);
      lastBase = base;
      lastCorr = corr;

      float leftCmd = base + corr;
      float rightCmd = base - corr;

      // Avoid driving backwards at the end of the 200 mm run; coast into the target.
      if (distError < 0.0f) {
        leftCmd = 0.0f;
        rightCmd = 0.0f;
      }

      setWheelPwm(leftCmd, rightCmd);
    }

    if (now - lastPrintMs > 120UL) {
      Serial.print(F("drive mm="));
      Serial.print(travelledMm, 1);
      Serial.print(F(" err="));
      Serial.print(distError, 1);
      Serial.print(F(" yawErr="));
      Serial.print(yawError, 2);
      Serial.print(F(" L="));
      Serial.print(leftMm, 1);
      Serial.print(F(" R="));
      Serial.print(rightMm, 1);
      Serial.print(F(" pwm="));
      Serial.print(lastBase, 0);
      Serial.print(F(" corr="));
      Serial.println(lastCorr, 1);
      lastPrintMs = now;
    }

    delay(10);
  }

  stopMotors();
  Serial.print(F("Finished drive: "));
  Serial.print(0.5f * (ticksToAbsMm(atomicLeftTicks() - leftStart) +
                       ticksToAbsMm(atomicRightTicks() - rightStart)), 1);
  Serial.print(F(" mm, yaw="));
  Serial.println(yawDeg(), 2);
}

void turnRelativeDeg(float deltaDeg) {
  const float startYaw = yawDeg();
  const float targetYaw = normalizeDeg(startYaw + deltaDeg);

  turnYawPid.reset();

  uint8_t stable = 0;
  unsigned long startMs = millis();
  unsigned long lastMs = millis();
  unsigned long lastPrintMs = 0;
  float lastTurn = 0.0f;

  Serial.print(F("Turn target deg: "));
  Serial.println(deltaDeg, 1);

  while (millis() - startMs < 6000UL) {
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    float currentYaw = yawDeg();
    float error = angleErrorDeg(targetYaw, currentYaw);
    if (abs(error) < TURN_DONE_TOL_DEG) {
      stopMotors();
      lastTurn = 0.0f;
      stable++;
      if (stable >= DONE_STABLE_CYCLES) {
        break;
      }
    } else {
      stable = 0;

      float turn = turnYawPid.update(error, dt);
      turn = applyMinPwm(turn, TURN_PWM_MIN, TURN_PWM_MAX);
      lastTurn = turn;

      // Positive turn command means CCW/left, matching micromouse1.2:
      // motor1 +pwm, motor2 -pwm.
      setWheelPwm(turn, -turn);
    }

    if (now - lastPrintMs > 100UL) {
      Serial.print(F("turn yaw="));
      Serial.print(currentYaw, 2);
      Serial.print(F(" err="));
      Serial.print(error, 2);
      Serial.print(F(" pwm="));
      Serial.println(lastTurn, 0);
      lastPrintMs = now;
    }

    delay(10);
  }

  stopMotors();
  Serial.print(F("Finished turn: targetDelta="));
  Serial.print(deltaDeg, 1);
  Serial.print(F(" actualDelta="));
  Serial.print(angleErrorDeg(yawDeg(), startYaw), 2);
  Serial.print(F(" finalYaw="));
  Serial.println(yawDeg(), 2);
}

// ------------------------- Setup ---------------------------
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
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
    stopMotors();
    delay(100);
  }

  Serial.println(F("..."));
  delay(1000);
  mpu.calcOffsets(true, true);
  mpu.update();
  yawZeroDeg = mpu.getAngleZ();
  Serial.println(F("Calibration done."));

  Serial.println(F("starts in 3 seconds."));
  delay(3000);
  lastControlMs = millis();
}

// -------------------------- Loop ---------------------------
void loop() {
  Serial.println(F("=== run ==="));

  driveDistanceMm(DRIVE_DISTANCE_MM);
  settleRobot();

  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(F("Left turn "));
    Serial.print(i + 1);
    Serial.println(F("/4"));
    turnRelativeDeg(TURN_ANGLE_DEG);
    settleRobot();
  }

  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(F("Right turn "));
    Serial.print(i + 1);
    Serial.println(F("/4"));
    turnRelativeDeg(-TURN_ANGLE_DEG);
    settleRobot();
  }

  stopMotors();
  Serial.println(F("complete."));
  Serial.print(F("Final yaw relative to start: "));
  Serial.println(yawDeg(), 2);

  while (true) {
    stopMotors();
    mpu.update();
    delay(1000);
  }
}
