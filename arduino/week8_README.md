# Week 8 Arduino Sketches

These four sketches match the Week 8 simple driving assessment tasks.

## Common Hardware Assumptions

All sketches use the course PCB schematic:

- Left motor: PWM `D11`, DIR `D12`
- Right motor: PWM `D9`, DIR `D10`
- Left encoder: A `D2`, B `D7`
- Right encoder: A `D3`, B `D8`
- MPU6050: `A4 SDA`, `A5 SCL`
- OLED SSD1306: `A4 SDA`, `A5 SCL`
- Left VL6180X XSHUT: `A0`, address changed to `0x54`
- Front VL6180X XSHUT: `A2`, address changed to `0x56`
- Right VL6180X XSHUT: `A1`, address changed to `0x55`

The OLED code does not need `U8g2`. It is driven directly using `Wire`.

Install these Arduino libraries:

- `MPU6050_light`
- `VL6180X` by Pololu

## Sketches

- `week8_1_straight_tracking`: Task 3.1, drive straight slightly past the 1 m finish line using IMU heading hold.
- `week8_2_lidar_stop`: Task 3.2, front LiDAR wall following to `100 mm`.
- `week8_3_turning`: Task 3.3, right 90 degrees, then recover after external rotation.
- `week8_4_chaining`: Task 3.4, execute the marking-day command string.

## Values To Tune First

- Straight task: `STRAIGHT_DISTANCE_MM`
- Wall task: `TARGET_WALL_MM`, `WALL_TOL_MM`, PID values inside `trackWallToTarget()`
- Turning task: PID values inside `turnToYaw()` in `week8_common/MTRNWeek8Common.hpp`
- Chaining task: `COMMAND_STRING`, `CELL_DISTANCE_MM`

## OLED Display

Every sketch displays:

- `X/Y/Z`: MPU6050 angles in degrees
- `L/F/R`: left, front, right LiDAR distances in millimetres
- Top row: current task/stage

## Suggested Test Order

1. Upload `week8_1_straight_tracking` and tune straight-line PWM/PID.
2. Upload `week8_3_turning` and tune turn PID.
3. Upload `week8_2_lidar_stop` and tune wall distance control.
4. Upload `week8_4_chaining`, set the real command string, and tune `CELL_DISTANCE_MM`.
