# Week 12 Task 4.1.2 - Maze Completion

**The race build is `week12_4_1_2_maze_completion_pid_migrated/`.** It carries
the trajectory-tracking motion core proven on the board in the Task 4.3 build,
including every field fix from its eight tuning rounds.
`week12_4_1_2_maze_completion/` is the original one-cell-at-a-time reference and
is kept unchanged.

## What the race build inherits from 4.3

- Trapezoidal profile with velocity feedforward, not a PID chasing a raw error.
- D terms read `getGyroZ()` directly. Differentiating the angle over an
  irregular loop period was the main source of the constant buzzing.
- Turns coast inside a 2 degree zone instead of chasing the last degree through
  the static-friction kick, which caused kick-overshoot-reverse-kick buzzing.
- One-layer micromouse correction, **scaled by the forward drive**. With forward
  at zero the per-wheel deadband used to turn any differential into a hard
  in-place twitch, which is what made the robot pivot and then hit a wall.
- Every range reading needs two consecutive samples that agree, so a single
  spike cannot collapse the cruise speed.
- Wall-edge odometry sync: a side wall appearing or ending means the robot is at
  a cell boundary, so the measured distance is snapped towards it. This stops
  error accumulating into an off-by-one-cell turn.
- Stall watchdogs in both the straight and the turn: back off and resume.
- Continuous static-to-kinetic deadband blend. Switching at a threshold injected
  an 8 PWM square wave.

## Marking workflow

1. Run the Task 4.1.1 notebook and select the supplied start pose and goal pose.
2. Copy the generated `const char COMMANDS[] = "...";` line.
3. Replace the matching line near the top of the Task 4.1.2 sketch.
4. Upload, place the robot at the centre of the start cell in the selected
   heading, keep it still during IMU calibration, and switch it on.

Commands use the course convention: `f` moves one 180 mm cell, `l` turns 90
degrees counter-clockwise, and `r` turns 90 degrees clockwise. The generated
turns at the end of the string set the required final heading.

## Heading rules

The sketch follows the same rules as the Task 4.3 mapping build:

- `northYaw` is captured once at startup and is never trimmed by a wall.
- Each turn aims at `northYaw` plus the exact heading for that direction, so a
  turn that finishes slightly short does not push its error into the next turn.
- Wall following trims a local copy of the heading that lives and dies inside
  one straight, so a crooked wall cannot rotate the rest of the route.

## What makes it quick

- **Consecutive `f` commands run as one straight.** `fff` is a single 540 mm run
  instead of three stop-start cells, which is where most of the lap time is won.
- **One range sensor is read per control tick** while driving. Three blocking
  reads per loop stretched the control period far too much at speed, and wall
  following is only as good as the loop feeding it.
- **Absolute headings allow a loose turn tolerance**, and turn tolerance is a
  large part of the lap time.
- A wall reached at the nominal gap is an absolute distance fix, so encoder slip
  is cleared at the end of every run that finishes facing a wall.

## Tuning for the race

Raise `DRIVE_SPEED_MAX` first, and only that.

```
100  validated on the board by the 4.3 build
120  current setting, straights here are chained and longer
140  quick, needs a charged battery - watch for weaving
```

`TURN_RATE_MAX` is the same idea for corners, currently 110 deg/s against the
100 proven on 4.3. If the robot weaves down a corridor or clips a post, put both
back to 100 before touching any gain.

`ENCODER_DISTANCE_SCALE` is 1.02, measured on this chassis. If cells land
consistently short or long, this is the number to change, not the cell size.

Watch the serial output after each run:

- `Route finished in N s` is the lap time.
- `Turn retries: 0` is what you want. Retries mean a turn timed out and had to
  be repeated, which costs about half a second each.
- `A run stopped short` means a straight ended more than 45 mm before its target,
  so something was in the way or the path is wrong.

Safety nets, none of which should trigger on a good run: turns give up after
`SETTLE_TIMEOUT_MS` and are retried once, straights give up after
`DRIVE_TIMEOUT_MS` per cell, both stall watchdogs back off and resume up to
`STALL_RECOVERY_MAX` times, and the front-wall stop is only accepted within
`FRONT_WALL_MAX_MM` of the end of a run so a stray reading cannot end a long
straight early.

Install `MPU6050_light` and Pololu `VL6180X`. The SSD1306 OLED is driven directly
through `Wire`, so no OLED library is required.
