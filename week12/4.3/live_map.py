from __future__ import annotations

import argparse
import tkinter as tk
from dataclasses import dataclass, field

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    serial = None
    list_ports = None


DIRECTIONS = ((-1, 0), (0, 1), (1, 0), (0, -1))
HEADING_MARK = ("^", ">", "v", "<")
PHASES = {
    "B": "Starting",
    "M": "Mapping",
    "R": "Returned to start",
    "S": "Shortest-path run",
    "D": "Complete",
    "X": "No path",
    "F": "Motion fault",
}


@dataclass
class MapState:
    rows: int = 9
    columns: int = 9
    start: tuple[int, int, int] = (8, 2, 0)
    goal: tuple[int, int, int] = (4, 4, 1)
    robot: tuple[int, int, int] = (8, 2, 0)
    visited_count: int = 0
    total_cells: int = 69
    completion: int = 0
    phase: str = "B"
    walls: list[int] = field(default_factory=lambda: [0] * 81)
    known: list[int] = field(default_factory=lambda: [0] * 81)
    visited: list[bool] = field(default_factory=lambda: [False] * 81)
    blocked: list[bool] | None = None
    path: list[int] = field(default_factory=list)
    commands: str = ""


def parse_packet(line: str, state: MapState) -> bool:
    fields = line.strip().split(",")
    if not fields:
        return False
    if fields[0] == "CONFIG" and len(fields) in (9, 10):
        state.rows, state.columns = int(fields[1]), int(fields[2])
        state.start = (int(fields[3]), int(fields[4]), int(fields[5]))
        state.goal = (int(fields[6]), int(fields[7]), int(fields[8]))
        state.robot = state.start
        size = state.rows * state.columns
        state.walls = [0] * size
        state.known = [0] * size
        state.visited = [False] * size
        state.path = []
        state.commands = ""
        if len(fields) == 10 and len(fields[9]) == size:
            state.blocked = [value == "0" for value in fields[9]]
        return True
    if fields[0] == "POS" and len(fields) == 4:
        state.robot = (int(fields[1]), int(fields[2]), int(fields[3]))
        return True
    if fields[0] == "MAP" and len(fields) == 11:
        size = state.rows * state.columns
        if any(len(fields[index]) != size for index in (8, 9, 10)):
            return False
        state.robot = (int(fields[1]), int(fields[2]), int(fields[3]))
        state.visited_count = int(fields[4])
        state.total_cells = int(fields[5])
        state.completion = int(fields[6])
        state.phase = fields[7]
        state.walls = [int(value, 16) for value in fields[8]]
        state.known = [int(value, 16) for value in fields[9]]
        state.visited = [value == "1" for value in fields[10]]
        return True
    if fields[0] == "PATH" and len(fields) >= 2:
        count = int(fields[1])
        state.path = [int(value) for value in fields[2:2 + count]]
        return True
    if fields[0] == "COMMANDS" and len(fields) == 2:
        state.commands = fields[1]
        return True
    return False


def blocked_corner_cell(row: int, column: int, rows: int = 9, columns: int = 9) -> bool:
    edge_row = min(row, rows - 1 - row)
    edge_column = min(column, columns - 1 - column)
    return edge_row + edge_column < 2


def cell_blocked(state: MapState, row: int, column: int) -> bool:
    if state.blocked is not None:
        return state.blocked[row * state.columns + column]
    return blocked_corner_cell(row, column, state.rows, state.columns)


def ascii_map(state: MapState) -> str:
    """9x9 text map: known walls drawn, unknown walls shown as *."""
    rows, columns = state.rows, state.columns
    lines: list[str] = []
    phase = PHASES.get(state.phase, state.phase)
    lines.append(
        f"{phase}  visited {state.visited_count}/{state.total_cells}"
        f"  {state.completion}%"
    )
    for row in range(rows):
        top = ""
        mid = ""
        for column in range(columns):
            index = row * columns + column
            known = state.known[index]
            walls = state.walls[index]
            if known & 1:
                top += "+" + ("---" if walls & 1 else "   ")
            else:
                top += "+ * "
            if known & 8:
                mid += "|" if walls & 8 else " "
            else:
                mid += "*"
            if cell_blocked(state, row, column):
                mid += "###"
            elif (row, column) == state.robot[:2]:
                mid += f" {HEADING_MARK[state.robot[2]]} "
            elif (row, column) == state.start[:2]:
                mid += " S "
            elif (row, column) == state.goal[:2]:
                mid += " G "
            elif state.visited[index]:
                mid += " . "
            else:
                mid += "   "
        index = row * columns + columns - 1
        top += "+"
        if state.known[index] & 2:
            mid += "|" if state.walls[index] & 2 else " "
        else:
            mid += "*"
        lines.append(top)
        lines.append(mid)
    bottom = ""
    for column in range(columns):
        index = (rows - 1) * columns + column
        if state.known[index] & 4:
            bottom += "+" + ("---" if state.walls[index] & 4 else "   ")
        else:
            bottom += "+ * "
    bottom += "+"
    lines.append(bottom)
    return "\n".join(lines)


class MapWindow:
    CELL = 68
    MARGIN = 42
    PANEL = 150

    def __init__(self, port: str, baud: int, show_ascii: bool) -> None:
        if serial is None:
            raise SystemExit("pyserial is missing. Install it with: python -m pip install -r requirements.txt")
        self.state = MapState()
        self.serial = serial.Serial(port, baud, timeout=0)
        self.show_ascii = show_ascii
        self.last_ascii = ""
        self.root = tk.Tk()
        self.root.title("MTRN3100 Week 12 - Autonomous Mapping")
        width = self.MARGIN * 2 + self.CELL * self.state.columns
        height = self.MARGIN * 2 + self.CELL * self.state.rows + self.PANEL
        self.canvas = tk.Canvas(self.root, width=width, height=height, bg="#f4f5f2", highlightthickness=0)
        self.canvas.pack()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.port = port
        self.draw()
        self.root.after(20, self.poll)

    def poll(self) -> None:
        try:
            changed = False
            while self.serial.in_waiting:
                line = self.serial.readline().decode("ascii", errors="ignore")
                if parse_packet(line, self.state):
                    changed = True
            if changed:
                self.draw()
                if self.show_ascii:
                    text = ascii_map(self.state)
                    if text != self.last_ascii:
                        self.last_ascii = text
                        print("\n" + text, flush=True)
        except serial.SerialException as error:
            self.canvas.create_text(20, 20, anchor="nw", text=str(error), fill="#b42318")
            return
        self.root.after(20, self.poll)

    def centre(self, cell_id: int) -> tuple[float, float]:
        row, column = divmod(cell_id, self.state.columns)
        return (
            self.MARGIN + (column + 0.5) * self.CELL,
            self.MARGIN + (row + 0.5) * self.CELL,
        )

    def draw(self) -> None:
        self.canvas.delete("all")
        rows, columns = self.state.rows, self.state.columns
        for row in range(rows):
            for column in range(columns):
                index = row * columns + column
                x0 = self.MARGIN + column * self.CELL
                y0 = self.MARGIN + row * self.CELL
                if cell_blocked(self.state, row, column):
                    self.canvas.create_rectangle(
                        x0, y0, x0 + self.CELL, y0 + self.CELL,
                        fill="#343735", outline="#f04c32", width=2,
                    )
                    self.canvas.create_line(
                        x0 + 15, y0 + 15, x0 + self.CELL - 15, y0 + self.CELL - 15,
                        fill="#b9bcb9", width=6,
                    )
                    self.canvas.create_line(
                        x0 + self.CELL - 15, y0 + 15, x0 + 15, y0 + self.CELL - 15,
                        fill="#b9bcb9", width=6,
                    )
                    continue
                fill = "#dcefdc" if self.state.visited[index] else "#ffffff"
                self.canvas.create_rectangle(x0, y0, x0 + self.CELL, y0 + self.CELL, fill=fill, outline="")
                self.canvas.create_text(x0 + 6, y0 + 6, anchor="nw", text=f"{row},{column}", fill="#9aa09a", font=("Consolas", 8))
                self.draw_edges(row, column, x0, y0, index)

        if len(self.state.path) > 1:
            points = [coordinate for cell in self.state.path for coordinate in self.centre(cell)]
            self.canvas.create_line(*points, fill="#f0ad00", width=6, capstyle=tk.ROUND, joinstyle=tk.ROUND)

        start_id = self.state.start[0] * columns + self.state.start[1]
        goal_id = self.state.goal[0] * columns + self.state.goal[1]
        sx, sy = self.centre(start_id)
        gx, gy = self.centre(goal_id)
        self.canvas.create_oval(sx - 10, sy - 10, sx + 10, sy + 10, fill="#22a447", outline="")
        self.canvas.create_oval(gx - 10, gy - 10, gx + 10, gy + 10, fill="#c43bb4", outline="")
        goal_dr, goal_dc = DIRECTIONS[self.state.goal[2]]
        self.canvas.create_line(
            gx, gy, gx + goal_dc * 24, gy + goal_dr * 24,
            fill="#c43bb4", width=6, arrow=tk.LAST,
        )
        self.draw_robot()
        self.draw_panel()

    def draw_edges(self, row: int, column: int, x0: float, y0: float, index: int) -> None:
        endpoints = (
            (x0, y0, x0 + self.CELL, y0),
            (x0 + self.CELL, y0, x0 + self.CELL, y0 + self.CELL),
            (x0, y0 + self.CELL, x0 + self.CELL, y0 + self.CELL),
            (x0, y0, x0, y0 + self.CELL),
        )
        for direction, points in enumerate(endpoints):
            bit = 1 << direction
            if self.state.known[index] & bit:
                if self.state.walls[index] & bit:
                    self.canvas.create_line(*points, fill="#202421", width=5)
            else:
                # Unknown wall: a row of * markers until the robot measures it.
                ax, ay, bx, by = points
                for fraction in (0.25, 0.5, 0.75):
                    self.canvas.create_text(
                        ax + (bx - ax) * fraction, ay + (by - ay) * fraction,
                        text="*", fill="#a5aaa5", font=("Consolas", 13, "bold"),
                    )

    def draw_robot(self) -> None:
        row, column, heading = self.state.robot
        cell_id = row * self.state.columns + column
        x, y = self.centre(cell_id)
        dr, dc = DIRECTIONS[heading]
        self.canvas.create_oval(x - 13, y - 13, x + 13, y + 13, fill="#2374d8", outline="#ffffff", width=2)
        self.canvas.create_line(x, y, x + dc * 24, y + dr * 24, fill="#2374d8", width=7, arrow=tk.LAST)

    def draw_panel(self) -> None:
        top = self.MARGIN * 2 + self.CELL * self.state.rows + 18
        width = self.MARGIN * 2 + self.CELL * self.state.columns
        phase = PHASES.get(self.state.phase, self.state.phase)
        title = f"{phase}   Visited {self.state.visited_count}/{self.state.total_cells}   {self.state.completion}%"
        self.canvas.create_text(self.MARGIN, top, anchor="nw", text=title, fill="#202421", font=("Segoe UI", 16, "bold"))
        bar_y = top + 38
        self.canvas.create_rectangle(self.MARGIN, bar_y, width - self.MARGIN, bar_y + 18, fill="#d8ddd8", outline="")
        progress = (width - 2 * self.MARGIN) * max(0, min(100, self.state.completion)) / 100
        self.canvas.create_rectangle(self.MARGIN, bar_y, self.MARGIN + progress, bar_y + 18, fill="#22a447", outline="")
        details = f"Port: {self.port}    Start: {self.state.start}    Goal: {self.state.goal}"
        self.canvas.create_text(self.MARGIN, bar_y + 30, anchor="nw", text=details, fill="#59605a", font=("Consolas", 10))
        legend = "black line = wall    * = unexplored wall    green = visited"
        self.canvas.create_text(self.MARGIN, bar_y + 50, anchor="nw", text=legend, fill="#59605a", font=("Consolas", 10))
        commands = self.state.commands or "-"
        self.canvas.create_text(self.MARGIN, bar_y + 70, anchor="nw", text=f"Commands: {commands}", fill="#59605a", font=("Consolas", 10))

    def close(self) -> None:
        self.serial.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def choose_port(requested: str | None) -> str:
    if list_ports is None:
        raise SystemExit("pyserial is missing. Install it with: python -m pip install -r requirements.txt")
    if requested:
        return requested
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("No serial port found. Run: python live_map.py COM4")
    return ports[0].device


def main() -> None:
    parser = argparse.ArgumentParser(description="Live map for Week 12 Task 4.3")
    parser.add_argument("port", nargs="?", help="Arduino serial port, for example COM4")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--ascii", action="store_true",
                        help="also print the map as text in this console")
    args = parser.parse_args()
    port = choose_port(args.port)
    MapWindow(port, args.baud, args.ascii).run()


if __name__ == "__main__":
    main()
