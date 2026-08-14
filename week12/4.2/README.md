# Week 12 Task 4.2

## Two notebooks

`Route_Recognition_4_2_All_In_One.ipynb` is the self-contained version. Every
stage of the pipeline is written into its own commented cell, so it does not
import `continuous_planner.py` or `grid_wall_detector.py`, and the sample photo
is embedded as base64. Sending that single `.ipynb` to someone else is enough:
they open it, run all, and get a route. It takes its photo from the embedded
sample, from a local file, or straight from a camera. The camera mode opens a
preview window where `SPACE` takes the shot, `N` switches to the next camera so
an external one can be selected, and `ESC` cancels; the captured frame is saved
under `captures/` and recognised immediately.

Start and goal are set as variables, and the notebook draws the maze with every
cell labelled `row,column` (blocked corners red, the detected 5 x 5 course
yellow) so the coordinates can be read straight off the picture. This is the
main path because it needs no desktop window and works in any Jupyter frontend,
local or remote.

The click-and-drag window is still available as an optional cell, but it only
appears when a GUI build of `opencv-python` is installed rather than
`opencv-python-headless`, Jupyter runs on the same machine as the display, and
a display exists. If any one fails the window never appears and usually **does
not raise an error - it just looks like the cell has hung**. A self-contained
cell diagnoses which of the three is missing and prints the fix.

`Continuous_Planning_4_2.ipynb` is the original workflow, kept unchanged. It
launches `continuous_planner.py` as a separate process with the interactive
click-and-drag window, so it needs that window to work and is **not** the one
to hand to a teammate. The rest of this document describes that pipeline, which
both notebooks share.

Run `Continuous_Planning_4_2.ipynb` from top to bottom. The program rectifies
the complete 9 x 9 maze and compares two valid route types:

`Task 4.1 detected-wall grid path`

`Task 4.1 grid path -> Task 4.2 continuous 5 x 5 path -> Task 4.1 grid path`

- `IMAGE`: maze photograph.
- `course_top_row`, `course_left_column`: top-left cell of the 5 x 5 section.
  Leave both at `None`, which is the default, to find it in the photograph.

## Finding the 5 x 5 course

The course does not sit in the same place on every board, so its position is
measured rather than configured. Every legal 5 x 5 position is scored on three
independent pieces of evidence:

1. **The interior is empty.** An ordinary 5 x 5 block of a 9 x 9 maze contains
   many of its 40 internal wall segments; the course contains none. This is by
   far the strongest signal.
2. **The boundary is a closed ring** apart from a few portals, so most of the 20
   perimeter segments carry wall.
3. **Every cylinder lies inside it** and none lies outside.

Coverage of the wall mask is used rather than the thresholded wall decisions, so
a wall too faint to be classified still counts. Positions overlapping the twelve
chamfered corner cells are rejected outright. Cylinders are searched for across
the whole board first, since finding the course needs them, and any found
outside the winning position are then dropped.

Across the supplied photographs this separates cleanly: boards holding a course
score an interior of 0.005 to 0.06 and beat the runner up by 0.11 to 0.22, while
plain maze boards sit at 0.13 to 0.30 interior and win by 0.005 to 0.048. A
result outside those bounds is refused with the ranked candidates printed, so a
plain maze photo is reported rather than silently planned against a made-up
course. The notebook prints the top five candidates every run.

All 20 boundary segments are measured, so the opening count and positions may
change between maps.

Outside walls are also detected from the current photograph rather than loaded
from a maze template. The detector combines locally normalised dark wall bodies,
horizontal/vertical edge segments, cyan endpoint evidence, and grid alignment.
Compact dark objects are masked before wall scoring. If people or other objects
hide a large part of the maze, planning stops and asks for another overhead photo
instead of guessing the hidden walls.

The second cell opens an interactive occupancy map:

1. Drag inside a standard cell toward the robot's initial heading.
2. Drag inside another standard cell toward the required final heading.
3. The shortest reachable route type is solved, drawn, copied, and saved automatically.

A short click followed by `N`, `E`, `S`, or `W` is also supported. Every
selected and generated heading is rounded to a whole degree. Right-click or
press `R` to choose again, `C` to copy the complete serial route, `S` to save,
and Escape or the window close button to finish. The completed route remains
visible until one of those explicit close actions is used.

Start and goal use the Task 4.1 `(row,column,heading)` format. Standard maze
sections are solved with the same orientation-aware cell planner as Task 4.1.
The five-by-five cells are blocked in that grid planner, so an outside-only
route cannot cross the course accidentally. When crossing the course is useful,
the planner compares every ordered pair of detected openings. Boundary inflation
is removed only inside those openings and cylinder clearance is then reapplied.

The robot is approximated as a 100 mm circle. Detected walls use a 35 mm
planning boundary, while each 100 mm course cylinder uses its radius plus the
same 35 mm boundary:

`wall = 35 mm, cylinder = 50 + 35 = 85 mm`

The rounded segments are checked again against this clearance before commands
are written.

Inside the 5 x 5 area, every detected cylinder and the surrounding walls form a
metric occupancy map. The number of cylinders is not fixed: zero, one, five, or
more detected cylinders are all accepted. A 10 mm A* search first finds a safe
corridor, then whole-path line-of-sight simplification removes unnecessary turns
before a small whole-route search converts every segment to an integer heading
without accumulating more than 10 mm of final position error. The robot footprint and safety margin remain
inflated around every obstacle, and the course boundary is masked so the route
cannot leave through an unintended gap.

Generated files are saved in `output/`:

- `rectified_maze.png`: perspective-corrected complete maze.
- `hybrid_occupancy_map.png`: detected outside walls with safety inflation only in the 5 x 5 course.
- `hybrid_planned_route.png`: grid paths and continuous waypoints in different colours.
- `route_commands.txt`: `BEGIN`, integer `T`, integer `F`, and `END` tokens.

The notebook also prints every movement on its own line, including the turn,
resulting heading, straight distance, and predicted endpoint. At the end it
prints an Arduino-ready route block such as:

```cpp
#define GENERATED_ROUTE "BEGIN;T-90;F300;T45;F220;END;"
```

The generated command line mixes three tokens: `T` turns by a whole number of
degrees, `G180` moves one normal grid cell with lidar wall following, and `F`
moves a continuous-course distance using IMU and encoders. Paste that complete
line between the markers in the Arduino sketch and upload it. Keeping the marker
area empty preserves live Serial transfer mode.

## How the sketch stays on the planned track

The sketch keeps three separate headings so that a local correction can never
rotate the rest of the route:

- `northYaw` is the grid reference captured at `BEGIN`.
- `routeYaw` is `northYaw` plus the exact sum of every `T` token.
- The commanded heading is `routeYaw` plus a temporary lean.

Wall following and obstacle avoidance only move the temporary lean. A crooked
wall or a cylinder therefore changes how the robot drives the current segment,
but the next segment still starts from the planned heading. Every token ends
with a re-square onto `routeYaw`, which is what keeps the course entry and exit
doorways lined up.

Gyro drift is handled separately. When a wall angle survives a whole cell it is
treated as drift rather than a crooked wall, so a bounded share of it (at most
1.2 degrees per cell and 12 degrees in total) is folded into `northYaw` itself.

Position error is measured, not assumed. Each segment tracks how far the robot
has travelled along the planned line and how far it sits to one side of it. The
side error is steered out with a lookahead lean, and whatever error remains at
the end of a segment is rotated into the next segment frame instead of being
discarded. Side walls give an absolute lateral fix and a wall ahead gives an
absolute distance fix, so encoder slip is cleared every grid cell. After an
avoidance detour the robot therefore merges back onto the planned line rather
than continuing parallel to it.

For `F` movements, the robot uses a lower course speed and slows further when
the heading error exceeds six degrees or an obstacle is inside the caution zone.
After a segment, only a residual error of at least two degrees triggers a short
closed-loop realignment before the next command.

Lidar avoidance is active during `F` movements. The robot first slows and steers
away inside the caution zone, which starts at 60 mm to the side and 80 mm ahead.
At 35 mm front clearance or 18 mm side clearance, it backs up, steps toward the
clearer side, restores the planned heading, and rejoins the planned line over
the next 260 mm. Recovery is limited to four tries per segment. Close to the end
of a segment the robot only slows for something ahead, because the object there
is usually the wall the segment aims at.

Turns cannot stall. If the encoders stop moving while an output is commanded,
the drive PWM is boosted until the wheels break free, so a small commanded angle
no longer leaves the robot buzzing in one place. A turn that still cannot finish
backs off briefly and retries once, then reports the residual instead of
blocking the route forever. The same stall watchdog runs during straight
movements.

The three range sensors run in continuous mode and one of them is read per
control cycle, with the front sensor read twice as often as the sides. The
display is refreshed one row at a time. Together these keep the control loop
near 10 ms instead of the 45 ms that blocking reads and full-screen refreshes
used to cost, which is what makes the avoidance fast enough to matter.

For live transfer, upload the sketch with the marker area empty and close Serial
Monitor before the notebook opens the COM port. The Arduino waits for each
instruction to finish and replies `DONE`; `BLOCKED` stops transmission.
