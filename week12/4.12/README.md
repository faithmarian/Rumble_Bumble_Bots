# Week 12 Task 4.1.2 - Maze Completion (race build)

The Arduino sketch is in `week12_4_1_2_maze_completion` and contains all robot
control code in one `.ino` file.

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
- **The range sensors free-run** and one is read per control cycle, front twice
  as often as the sides. The display refreshes a single row per pass. Together
  these keep the loop near 10 ms instead of the 45 ms that blocking reads and a
  full screen refresh used to cost; wall following is only as good as the loop
  feeding it.
- **Absolute headings allow a loose turn tolerance**, and turn tolerance is a
  large part of the lap time.
- A wall reached at the nominal gap is an absolute distance fix, so encoder slip
  is cleared at the end of every run that finishes facing a wall.

## Tuning for the race

Raise `RUN_PWM_MAX` first, and only that. Everything else scales with it.

```
 42  the old one-cell-at-a-time value, very safe
 85  current setting, roughly double the speed
110  quick, needs a straight track and a charged battery
```

If the robot weaves down a corridor or clips a post, drop it back 10 and re-run
before touching any gain. Watch the serial output after each run:

- `Route finished in N s` is the lap time.
- `Turn timeouts: 0` is what you want. Several of them means `TURN_PWM_MAX` is
  too low for the tyres and surface.
- `A run stopped early` means a straight ended before three quarters of a cell,
  so something was in the way or the path is wrong.

Safety nets, none of which should ever trigger on a good run: turns give up
after `TURN_TIMEOUT_MS` and carry on, straights give up after
`RUN_TIMEOUT_BASE_MS` plus `RUN_TIMEOUT_PER_CELL_MS` per cell, a stalled wheel
gets a PWM boost until it turns, and the front-wall stop is only accepted within
`FRONT_WALL_MAX_MM` of the end of a run so a stray reading cannot end a long
straight early.

Install `MPU6050_light` and Pololu `VL6180X`. The SSD1306 OLED is driven directly
through `Wire`, so no OLED library is required.
