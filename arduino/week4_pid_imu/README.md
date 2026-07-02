# Week 4 PID + IMU Arduino Controller

Open `week4_pid_imu.ino` in Arduino IDE and upload to the Arduino Nano.

## What It Does

1. Calibrates the MPU6050 while the robot is stationary.
2. Drives forward 200 mm using encoder distance feedback.
3. Uses MPU6050 yaw feedback to keep the robot straight.
4. Performs four 90 degree left turns.
5. Performs four 90 degree right turns.

## Pin Mapping

This follows the UNSW MDI controller PCB and the working `micromouse1.2` code:

- Left motor/MOT1: PWM `D11`, DIR `D12`
- Right motor/MOT2: PWM `D9`, DIR `D10`
- Left encoder: A `D2`, B `D7`
- Right encoder: A `D3`, B `D8`
- MPU6050: SDA `A4`, SCL `A5`

## First Checks

- The motor convention now matches `micromouse1.2`: straight motion sends `motor1=+pwm`, `motor2=+pwm`.
- Right motor inversion is handled inside `setMotorCommand()` because motor 2 is physically mounted opposite.
- If straight motion spins in place, check the `if (motorNumber == 2) command = -command;` line.
- If a 200 mm command does not travel 200 mm, tune `WHEEL_RADIUS_MM` or `ENCODER_COUNTS_PER_REV`.
- If turning is the wrong direction, swap the signs in `setWheelPwm(turn, -turn)` inside `turnRelativeDeg`.

## Main Tuning Constants

- Straight distance: `DRIVE_DISTANCE_MM`
- Wheel radius: `WHEEL_RADIUS_MM`
- Axle track: `AXLE_TRACK_MM`
- Encoder CPR: `ENCODER_COUNTS_PER_REV`
- Straight PID: `distancePid`, `driveYawPid`, `wheelSyncPid`
- Turn PID: `turnYawPid`
- PWM caps: `DRIVE_PWM_MAX`, `TURN_PWM_MAX`
