# Rumble Bumble Bots

Team repository for the MTRN3100 micromouse project.

## Repository Layout

- `arduino/` - Arduino Nano sketches for sensor diagnostics, PID/IMU driving, and Week 8 driving tasks.
- `webots_week4/` - Webots worlds, robot protos, C# controller source, and map drawing utilities.
- `print/` - CAD and printable chassis/support files.
- `project/` - Project schematic images used by the team.

## Arduino Sketches

The Week 8 sketches share pin mappings and helper code in `arduino/week8_common/MTRNWeek8Common.hpp`.

Install these Arduino libraries before uploading:

- `MPU6050_light`
- `VL6180X` by Pololu

The OLED code uses `Wire` directly and does not require an extra OLED library.

## Webots Controllers

The Webots controllers are C# projects. Generated `bin/`, `obj/`, executable, DLL, and PDB outputs are intentionally ignored so the repository stays source-focused.

## Publish Notes

This repository intentionally excludes local Codex state, lecture/lab handouts, downloaded reference repositories, course PDFs, build outputs, logs, archives, and environment/secret files.
