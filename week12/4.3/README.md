# Week 12 Task 4.3 - Autonomous Mapping

**Flash `autonomous_mapping_pid_migrated/autonomous_mapping_pid_migrated.ino`.**
That is the maintained sketch. `autonomous_mapping/autonomous_mapping.ino` is
the old Week-8-style reference and is kept unchanged.

This build is **OLED-only**: all serial output has been removed at the
user's request, so the robot needs no PC at all. (`live_map.py` is kept in
this folder for reference but does not work with this build.)

Workflow:

1. The robot maps walls using the left, front, and right VL6180X sensors.
2. A depth-first search visits every reachable cell and returns to the start.
3. A breadth-first search calculates the shortest route from start to goal.
4. The robot executes that route. The whole run is shown live on the OLED.

## Set the marking-day maze

Edit only `MAZE_LAYOUT` at the top of the sketch:

- `.` explorable cell, `#` blocked cell
- `N/E/S/W` start cell + heading (upper case, exactly one)
- `n/e/s/w` goal cell + heading (lower case, exactly one)

The twelve chamfered corner cells stay blocked no matter what the layout
says. For the real 9x9 board make every playable cell `.` so the completion
denominator is 69. Place the robot in the centre of the start cell and keep
it still during the MPU6050 calibration after power-on.

## OLED display (map + progress only)

- Left half: the full 9x9 maze as a 64x64 pixel map. Solid line = measured
  wall, dotted line = unmeasured wall, no line = measured open, centre dot
  = visited cell, solid 3x3 block = robot, hollow 3x3 block = goal,
  checkerboard = blocked cell.
- Right half: visited/total, percentage, and a progress bar. The top row
  stays blank except for `FAULT` / `NO PATH` / `DONE` - a fault is never
  silent.
- A single-cell move that fails during exploration no longer stops the
  run: the robot backs up to the cell it came from, records that edge as
  blocked, and keeps exploring.

The full map redraws when the map changes (robot stationary); during
chained runs only the robot marker cell is patched (~2 ms), so the display
never interrupts motion.

## Self-correction (no corner docking)

The old corner-dock pose reset was removed. Drift is handled continuously
instead:

- **Gyro drift**: at every mapping stop the raw gyro rate is averaged for
  ~80 ms (with outlier rejection) - that is the current bias, learned and
  subtracted from then on. The damping rate gets the same correction.
- **Encoder scale**: every dead-end reversal is a run of exactly known
  physical length (52 mm wall gap at both ends), used to re-calibrate
  mm-per-count on the fly (EMA, clamped to +-6%).
- **Longitudinal position**: re-anchored whenever the robot stops 52 mm
  from a front wall.

## Trajectory keeping (rebuilt, single layer)

The multi-stage steer system was torn out after it caused sudden in-place
pivots (forward drive clamped to zero + per-wheel deadband kick turned any
heading correction into a pivot; a single unfiltered front-lidar spike
could trigger it mid-corridor). The replacement is the classic micromouse
law, one layer, no mode memory:

- Side wall(s) visible: P on the corridor offset (0.22 PWM/mm, gain rises
  past a 12 mm knee), damped by the gyro rate. The physical corridor is
  the reference - IMU error is irrelevant while wall-following.
- No walls: P on the grid heading by gyro, same damping.
- The whole correction scales with the forward drive output, so at zero
  forward drive the correction is zero: an in-place pivot is impossible.
- All three lidars now require two consecutive agreeing readings; a
  single spike can neither steer the robot nor fake a front wall.
- A side reading under 34 mm caps speed at 45 mm/s until clear.
- Wall-edge odometry sync: whenever a side wall starts or ends, the robot
  is physically at a cell boundary, and the measured distance is snapped
  towards it (half-blend, 30 mm window, 60 mm total cap). This pins the
  longitudinal position to the real maze, so odometry error can no longer
  accumulate into an off-by-one-cell turn - which is how the virtual
  fence got breached: the fence logic itself is map-based and airtight,
  but a physically desynced robot executed a perfectly legal turn one
  cell early, straight through a fence gap that has no physical wall.
- A timed-out turn is retried once after settling instead of being
  silently accepted.
- Stall recovery: commanded-but-not-moving (drive) or output-without-
  rotation (turn) triggers stop, back off, resume - twice per move before
  faulting.
- The motor deadband feedforward blends from 18 PWM at standstill down to
  10 PWM when moving (kinetic vs static friction).

## Arduino libraries

- `MPU6050_light`
- Pololu `VL6180X`

The OLED is driven directly through `Wire`, no OLED library. Verified to
compile with `arduino:avr:nano` (88% flash, 53% RAM).
