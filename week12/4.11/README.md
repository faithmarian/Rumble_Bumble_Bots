# Week 12 Task 4.1.1

## Two notebooks

`Maze_Recognition_4_1_1_All_In_One.ipynb` is the self-contained version, and the
one to hand to a teammate. Every stage sits in its own commented cell, the
sample photograph is embedded as base64, and nothing outside the file is
needed: no `pics` folder, no `run_notebook.cmd`, no custom kernel. Open it, run
all, and it prints the `const char COMMANDS[] = "...";` line for
`week12_4_1_2_maze_completion.ino`.

It takes its photograph from the embedded sample, from a local file, or straight
from a camera. Camera mode opens a preview window where `SPACE` takes the shot,
`N` switches to the next camera so an external one can be selected, and `ESC`
cancels; the frame is saved under `captures/` and recognised immediately.

### Picking start and goal without a window

Start and goal are set as variables, and the notebook draws the recognised maze
with every cell labelled `row,column` (blocked corner cells in red) so reading
the coordinates off the picture is easy. This is the main path because it needs
no desktop window and works in any Jupyter frontend, local or remote.

The original click-and-drag window is still there as an optional cell, but it
only appears when three things are all true: a GUI build of `opencv-python` is
installed rather than `opencv-python-headless`, Jupyter is running on the same
machine as the display, and a display exists. If any one fails the window never
appears and usually **does not raise an error - it just looks like the cell has
hung**, which is why it is no longer the main path. A self-contained cell
diagnoses which of the three is missing and prints the fix.

Nothing is written to disk unless `SAVE_OUTPUT_DIR` is set.

Only `numpy`, `opencv-python` and `matplotlib` are required.

## Original notebook

`Path_Generation_4_1_1.ipynb` is the original workflow, kept unchanged.

Open it and select the kernel `Python (MTRN3100 Week 12)`, then run the cells
from top to bottom. It reads a photograph from `../pics`, displays the source
photograph and detected 9 x 9 wall layout, opens the start/goal selection
window, draws the route, and outputs an `f/l/r` navigation string.

Change `IMAGE_NAME` in the configuration cell to use another image from
`../pics`. If the selected file is a low-resolution duplicate, the notebook
automatically uses the matching higher-resolution photograph for wall
recognition.

Run `run_notebook.cmd` once if the kernel or packages are not installed.
