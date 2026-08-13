from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

from grid_wall_detector import detect_grid_walls


MAZE_CELLS = 9
COURSE_CELLS = 5
CELL_MM = 180
MAZE_MM = MAZE_CELLS * CELL_MM
DISPLAY_PX = 720
PANEL_PX = 420
DIRECTIONS = ("N", "E", "S", "W")
DELTAS = ((-1, 0), (0, 1), (1, 0), (0, -1))
DIRECTION_HEADINGS = (0, -90, 180, 90)
BLOCKED_CORNERS = (
    (0, 0), (0, 1), (1, 0),
    (0, 7), (0, 8), (1, 8),
    (7, 0), (8, 0), (8, 1),
    (7, 8), (8, 7), (8, 8),
)


@dataclass(frozen=True)
class CoursePortal:
    side: str
    offset: int


@dataclass(frozen=True)
class PlannerConfig:
    robot_diameter_mm: float = 100.0
    obstacle_diameter_mm: float = 100.0
    wall_safety_mm: float = 35.0
    cylinder_safety_mm: float = 35.0
    grid_mm: int = 10
    minimum_course_segment_mm: int = 40
    maximum_segment_mm: int = 1000
    route_turn_penalty_mm: float = 120.0
    # None means "find the 5 x 5 course in the photograph". Set an integer to
    # pin it down by hand. Use resolve_course_config() before reading these.
    course_top_row: int | None = None
    course_left_column: int | None = None
    portal_open_threshold: float = 0.35
    start_heading_deg: int = 0
    goal_heading_deg: int | None = 0

    @property
    def robot_radius_mm(self) -> float:
        return self.robot_diameter_mm / 2

    @property
    def wall_clearance_mm(self) -> float:
        return self.wall_safety_mm

    @property
    def cylinder_clearance_mm(self) -> float:
        return self.obstacle_diameter_mm / 2 + self.cylinder_safety_mm


@dataclass(frozen=True)
class Motion:
    turn_deg: int
    distance_mm: int
    heading_deg: int


@dataclass
class GlobalMap:
    rectified: np.ndarray
    wall_mask: np.ndarray
    occupancy_mask: np.ndarray
    obstacles_mm: list[tuple[float, float]]
    dark_threshold: int
    horizontal_walls: np.ndarray
    vertical_walls: np.ndarray
    horizontal_scores: np.ndarray
    vertical_scores: np.ndarray
    # Where the 5 x 5 obstacle course was found in this photograph.
    course_top_row: int = 0
    course_left_column: int = 0
    course_candidates: tuple = ()


@dataclass
class PlanResult:
    map_data: GlobalMap
    config: PlannerConfig
    control_points_mm: list[tuple[float, float]]
    waypoints_mm: list[tuple[float, float]]
    motions: list[Motion]
    occupancy_image: np.ndarray
    route_image: np.ndarray
    start_heading_deg: int
    goal_heading_deg: int | None


@dataclass
class GridMaze:
    horizontal_walls: np.ndarray
    vertical_walls: np.ndarray
    blocked_cells: np.ndarray

    def can_move(self, row: int, column: int, direction: int) -> bool:
        if self.blocked_cells[row, column]:
            return False
        if direction == 0:
            wall = self.horizontal_walls[row, column]
        elif direction == 1:
            wall = self.vertical_walls[row, column + 1]
        elif direction == 2:
            wall = self.horizontal_walls[row + 1, column]
        else:
            wall = self.vertical_walls[row, column]
        next_row = row + DELTAS[direction][0]
        next_column = column + DELTAS[direction][1]
        return (
            not wall
            and 0 <= next_row < MAZE_CELLS
            and 0 <= next_column < MAZE_CELLS
            and not self.blocked_cells[next_row, next_column]
        )


@dataclass(frozen=True)
class PortalPose:
    portal: CoursePortal
    inside_cell: tuple[int, int]
    outside_cell: tuple[int, int]
    inward_direction: int

    @property
    def outward_direction(self) -> int:
        return (self.inward_direction + 2) % 4


@dataclass
class HybridPlan:
    map_data: GlobalMap
    grid_maze: GridMaze
    config: PlannerConfig
    start_pose: tuple[int, int, int]
    goal_pose: tuple[int, int, int]
    entry: PortalPose | None
    exit: PortalPose | None
    grid_before: str
    grid_after: str
    cells_before: list[tuple[int, int]]
    cells_after: list[tuple[int, int]]
    continuous: PlanResult | None
    occupancy_image: np.ndarray
    route_image: np.ndarray

    @property
    def uses_course(self) -> bool:
        return self.continuous is not None


def read_image(path: str | Path) -> np.ndarray:
    path = Path(path)
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read image: {path}")
    return image


def write_image(path: str | Path, image: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(path.suffix or ".png", image)
    if not ok:
        raise ValueError(f"Could not encode image: {path}")
    encoded.tofile(str(path))


def _order_quad(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).ravel()
    return np.array(
        [
            points[np.argmin(sums)],
            points[np.argmin(differences)],
            points[np.argmax(sums)],
            points[np.argmax(differences)],
        ],
        dtype=np.float32,
    )


def _cyan_mask(image: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (86, 55, 28), (108, 255, 255))
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )


def _largest_cluster(points: np.ndarray, radius: float) -> np.ndarray:
    unused = set(range(len(points)))
    clusters: list[list[int]] = []
    radius_squared = radius * radius
    while unused:
        seed = unused.pop()
        cluster = [seed]
        queue = [seed]
        while queue:
            current = queue.pop()
            candidates = list(unused)
            if not candidates:
                continue
            delta = points[candidates] - points[current]
            nearby = [
                candidates[index]
                for index, value in enumerate(delta)
                if float(value @ value) <= radius_squared
            ]
            for index in nearby:
                unused.remove(index)
                cluster.append(index)
                queue.append(index)
        clusters.append(cluster)
    return points[max(clusters, key=len)]


def _maze_quad(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(1.0, 1800.0 / max(height, width))
    small = cv2.resize(image, None, fx=scale, fy=scale) if scale < 1 else image.copy()
    cyan = _cyan_mask(small)
    count, _, stats, centroids = cv2.connectedComponentsWithStats(cyan)
    image_area = small.shape[0] * small.shape[1]
    points = np.array(
        [
            centroids[index]
            for index in range(1, count)
            if max(4, int(image_area * 0.000002))
            <= stats[index, cv2.CC_STAT_AREA]
            <= max(300, int(image_area * 0.004))
        ],
        dtype=np.float32,
    )
    if len(points) >= 20:
        cluster = _largest_cluster(points, 0.13 * min(small.shape[:2]))
        if len(cluster) >= 15:
            return _order_quad(cv2.boxPoints(cv2.minAreaRect(cluster)))

    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (7, 7), 0), 45, 130)
    edges = cv2.morphologyEx(
        edges,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)),
        iterations=2,
    )
    candidates: list[tuple[float, np.ndarray]] = []
    for contour in cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        area = cv2.contourArea(contour)
        if area < image_area * 0.12:
            continue
        hull = cv2.convexHull(contour)
        perimeter = cv2.arcLength(hull, True)
        for epsilon in (0.015, 0.025, 0.04):
            polygon = cv2.approxPolyDP(hull, epsilon * perimeter, True)
            if len(polygon) == 4 and cv2.isContourConvex(polygon):
                candidates.append((area, _order_quad(polygon[:, 0, :])))
                break
    if not candidates:
        raise RuntimeError("Maze frame was not detected. Retake the photo from above.")
    return max(candidates, key=lambda item: item[0])[1]


def _warp(image: np.ndarray, quad: np.ndarray, size: int) -> np.ndarray:
    destination = np.array(
        [[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(_order_quad(quad), destination)
    return cv2.warpPerspective(image, matrix, (size, size))


def _fit_lattice(projection: np.ndarray, line_count: int) -> tuple[float, float]:
    projection = cv2.GaussianBlur(
        projection.astype(np.float32).reshape(1, -1), (0, 0), 4
    ).ravel()
    projection /= float(projection.max() + 1e-6)
    length = len(projection)
    ideal_step = length / (line_count - 1)
    best = (-math.inf, 0.0, ideal_step)
    for step in np.arange(ideal_step * 0.74, ideal_step * 1.10, 0.5):
        maximum_origin = length - 1 - step * (line_count - 1)
        if maximum_origin < 0:
            continue
        for origin in np.arange(0.0, maximum_origin + 0.25, 0.5):
            score = 0.08 * step * (line_count - 1) / length
            for position in origin + step * np.arange(line_count):
                centre = int(round(position))
                score += float(
                    projection[max(0, centre - 4):min(length, centre + 5)].max()
                )
            if score > best[0]:
                best = (score, float(origin), float(step))
    return best[1], best[2]


def rectify_maze(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(1.0, 1800.0 / max(height, width))
    small = cv2.resize(image, None, fx=scale, fy=scale) if scale < 1 else image.copy()
    coarse = _warp(small, _maze_quad(small), 1000)
    cyan = _cyan_mask(coarse)
    x_origin, x_step = _fit_lattice(cyan.sum(axis=0), MAZE_CELLS + 1)
    y_origin, y_step = _fit_lattice(cyan.sum(axis=1), MAZE_CELLS + 1)
    grid_quad = np.array(
        [
            [x_origin, y_origin],
            [x_origin + MAZE_CELLS * x_step, y_origin],
            [x_origin + MAZE_CELLS * x_step, y_origin + MAZE_CELLS * y_step],
            [x_origin, y_origin + MAZE_CELLS * y_step],
        ],
        dtype=np.float32,
    )
    return _warp(coarse, grid_quad, MAZE_MM)


def _detect_cylinders(dark: np.ndarray, config: PlannerConfig) -> list[tuple[float, float]]:
    """Find fixed-size cylinders even when their silhouettes touch a wall.

    When the course position is still unknown the whole board is searched,
    because locating the course needs the cylinders. That is safe: the filter
    below demands a dark blob at least 70 mm across in both directions, and a
    12 mm maze wall never produces one. The chamfered corners are the one
    exception, so they are excluded explicitly.
    """
    distance = cv2.distanceTransform(dark, cv2.DIST_L2, 5)
    core = np.zeros_like(dark)
    inset = round(config.wall_safety_mm)
    known = config.course_top_row is not None and config.course_left_column is not None
    if known:
        top = config.course_top_row * CELL_MM + inset
        left = config.course_left_column * CELL_MM + inset
        bottom = (config.course_top_row + COURSE_CELLS) * CELL_MM - inset
        right = (config.course_left_column + COURSE_CELLS) * CELL_MM - inset
    else:
        top = left = inset
        bottom = right = MAZE_MM - inset
    minimum_radius = 0.30 * config.obstacle_diameter_mm
    core[top:bottom, left:right] = (distance[top:bottom, left:right] >= minimum_radius) * 255
    if not known:
        # The cut-off corner cells are large dark areas and would otherwise
        # register as cylinders.
        for row, column in BLOCKED_CORNERS:
            core[row * CELL_MM:(row + 1) * CELL_MM,
                 column * CELL_MM:(column + 1) * CELL_MM] = 0

    obstacles: list[tuple[float, float]] = []
    count, labels, stats, _ = cv2.connectedComponentsWithStats(core)
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] < 200:
            continue
        ys, xs = np.where(labels == label)
        peak = int(np.argmax(distance[ys, xs]))
        radius = distance[ys[peak], xs[peak]]
        if 0.35 * config.obstacle_diameter_mm <= radius <= 0.70 * config.obstacle_diameter_mm:
            obstacles.append((float(xs[peak]), float(ys[peak])))
    return sorted(obstacles, key=lambda point: (point[1], point[0]))


def extract_global_map(rectified: np.ndarray, config: PlannerConfig) -> GlobalMap:
    hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
    gray_raw = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    dark_fraction = float(np.mean(gray_raw < 80))
    coloured_fraction = float(np.mean(hsv[:, :, 1] > 70))
    if dark_fraction > 0.28 and coloured_fraction > 0.12:
        raise ValueError(
            "Maze is heavily occluded; take another overhead photo before planning"
        )
    gray = cv2.GaussianBlur(gray_raw, (7, 7), 0)
    otsu, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    dark_threshold = int(np.clip(0.80 * otsu, 65, 105))
    dark = (gray < dark_threshold).astype(np.uint8) * 255
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )

    obstacles = _detect_cylinders(dark, config)
    wall_source = dark.copy()
    obstacle_guard = round(config.obstacle_diameter_mm / 2 + 10)
    for x, y in obstacles:
        cv2.circle(wall_source, (round(x), round(y)), obstacle_guard, 0, -1)

    wall_mask = np.zeros_like(dark)
    contours = cv2.findContours(wall_source, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    for contour in contours:
        area = cv2.contourArea(contour)
        _, _, width, height = cv2.boundingRect(contour)
        diameter = max(width, height)
        if area >= 280 and (diameter >= 65 or area >= 1400):
            cv2.drawContours(wall_mask, [contour], -1, 255, -1)

    # Recover reflective wall bodies that are lighter than the contour threshold.
    wall_body = cv2.inRange(gray, 0, min(130, dark_threshold + 30))
    for x, y in obstacles:
        cv2.circle(wall_body, (round(x), round(y)), obstacle_guard, 0, -1)
    wall_body = cv2.morphologyEx(
        wall_body,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)),
    )
    horizontal = cv2.morphologyEx(
        wall_body,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (55, 9)),
    )
    vertical = cv2.morphologyEx(
        wall_body,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (9, 55)),
    )
    wall_mask = cv2.bitwise_or(
        wall_mask,
        cv2.bitwise_or(horizontal, vertical),
    )

    for row, column in BLOCKED_CORNERS:
        cv2.rectangle(
            wall_mask,
            (column * CELL_MM, row * CELL_MM),
            ((column + 1) * CELL_MM - 1, (row + 1) * CELL_MM - 1),
            255,
            -1,
        )
    cv2.rectangle(wall_mask, (0, 0), (MAZE_MM - 1, MAZE_MM - 1), 255, 3)

    horizontal_walls, vertical_walls, horizontal_scores, vertical_scores = (
        detect_grid_walls(
            rectified,
            MAZE_CELLS,
            MAZE_CELLS,
            tuple(
                (x, y, 0.42 * CELL_MM)
                for x, y in obstacles
            ),
        )
    )

    # The course position is decided here, from the walls and the cylinders,
    # because everything downstream needs it. Cylinders found outside the
    # course are dropped: on this board a cylinder only exists inside it.
    partial = GlobalMap(
        rectified, wall_mask, wall_mask, obstacles, dark_threshold,
        horizontal_walls, vertical_walls, horizontal_scores, vertical_scores,
    )
    if config.course_top_row is not None and config.course_left_column is not None:
        top_row = config.course_top_row
        left_column = config.course_left_column
        candidates: tuple = ()
    else:
        ranked = rank_course_positions(partial, limit=5)
        best = detect_course_position(partial)
        top_row, left_column = best.top_row, best.left_column
        candidates = tuple(ranked)

    obstacles = [
        (x, y)
        for x, y in obstacles
        if left_column * CELL_MM <= x < (left_column + COURSE_CELLS) * CELL_MM
        and top_row * CELL_MM <= y < (top_row + COURSE_CELLS) * CELL_MM
    ]

    wall_radius = int(round(config.wall_clearance_mm))
    occupancy = cv2.dilate(
        wall_mask,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * wall_radius + 1, 2 * wall_radius + 1),
        ),
    )
    cylinder_radius = int(round(config.cylinder_clearance_mm))
    for x, y in obstacles:
        cv2.circle(occupancy, (round(x), round(y)), cylinder_radius, 255, -1)

    return GlobalMap(
        rectified,
        wall_mask,
        occupancy,
        obstacles,
        dark_threshold,
        horizontal_walls,
        vertical_walls,
        horizontal_scores,
        vertical_scores,
        top_row,
        left_column,
        candidates,
    )


def _edge_coverage(
    wall_mask: np.ndarray,
    horizontal: bool,
    grid_line: int,
    segment: int,
) -> float:
    line = grid_line * CELL_MM
    start = segment * CELL_MM + 25
    end = (segment + 1) * CELL_MM - 25
    half_width = 50
    if horizontal:
        region = wall_mask[
            max(0, line - half_width):min(MAZE_MM, line + half_width + 1),
            start:end,
        ]
        return float(np.mean(np.any(region > 0, axis=0)))
    region = wall_mask[
        start:end,
        max(0, line - half_width):min(MAZE_MM, line + half_width + 1),
    ]
    return float(np.mean(np.any(region > 0, axis=1)))


def _grid_coverage(wall_mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Wall pixel coverage of every grid segment in the board.

    Returned as horizontal[line_row, segment_column] and
    vertical[line_column, segment_row], both in 0..1. Measuring coverage once
    for all 180 segments keeps the course search to pure array indexing.
    """
    horizontal = np.zeros((MAZE_CELLS + 1, MAZE_CELLS), dtype=np.float32)
    vertical = np.zeros((MAZE_CELLS + 1, MAZE_CELLS), dtype=np.float32)
    for line in range(MAZE_CELLS + 1):
        for segment in range(MAZE_CELLS):
            horizontal[line, segment] = _edge_coverage(wall_mask, True, line, segment)
            vertical[line, segment] = _edge_coverage(wall_mask, False, line, segment)
    return horizontal, vertical


@dataclass(frozen=True)
class CourseCandidate:
    top_row: int
    left_column: int
    score: float
    interior_wall: float      # 0 is a clean open area, 1 is a normal maze block
    boundary_wall: float      # 1 is fully enclosed, lower means more openings
    cylinders_inside: int
    cylinders_total: int

    @property
    def position(self) -> tuple[int, int]:
        return (self.top_row, self.left_column)


def _course_overlaps_blocked_corner(top: int, left: int) -> bool:
    return any(
        top <= row < top + COURSE_CELLS and left <= column < left + COURSE_CELLS
        for row, column in BLOCKED_CORNERS
    )


def rank_course_positions(
    map_data: GlobalMap,
    limit: int = 5,
) -> list[CourseCandidate]:
    """Score every legal 5 x 5 position and return the best ones first.

    Three independent pieces of evidence separate the obstacle course from an
    ordinary block of the maze:

    1. Its interior is empty. A normal 5 x 5 block of a 9 x 9 maze contains
       many of the 40 internal wall segments; the course contains none. This
       is by far the strongest signal.
    2. Its boundary is a closed ring apart from a few portals, so most of the
       20 perimeter segments carry wall.
    3. Every detected cylinder lies inside it, and none lies outside.

    Coverage of the wall mask is used rather than the thresholded wall
    decisions, so a wall that was too faint to be classified still counts.
    """
    horizontal, vertical = _grid_coverage(map_data.wall_mask)
    obstacles = map_data.obstacles_mm
    span = MAZE_CELLS - COURSE_CELLS

    candidates: list[CourseCandidate] = []
    for top in range(span + 1):
        for left in range(span + 1):
            if _course_overlaps_blocked_corner(top, left):
                continue

            # Interior: 4 internal lines each way, 5 segments per line.
            interior = float(np.mean([
                float(np.mean(horizontal[top + 1:top + COURSE_CELLS,
                                         left:left + COURSE_CELLS])),
                float(np.mean(vertical[left + 1:left + COURSE_CELLS,
                                       top:top + COURSE_CELLS])),
            ]))

            # Perimeter: the four sides of the ring.
            boundary = float(np.mean(np.concatenate([
                horizontal[top, left:left + COURSE_CELLS],
                horizontal[top + COURSE_CELLS, left:left + COURSE_CELLS],
                vertical[left, top:top + COURSE_CELLS],
                vertical[left + COURSE_CELLS, top:top + COURSE_CELLS],
            ])))

            inside = sum(
                1
                for x, y in obstacles
                if left * CELL_MM <= x < (left + COURSE_CELLS) * CELL_MM
                and top * CELL_MM <= y < (top + COURSE_CELLS) * CELL_MM
            )
            if obstacles:
                cylinder_term = inside / len(obstacles)
            else:
                cylinder_term = 0.5      # no evidence either way

            score = (
                2.5 * (1.0 - interior)
                + 1.0 * boundary
                + 1.5 * cylinder_term
            ) / 5.0
            candidates.append(CourseCandidate(
                top, left, score, interior, boundary, inside, len(obstacles),
            ))

    candidates.sort(key=lambda item: item.score, reverse=True)
    return candidates[:limit] if limit else candidates


# A real course leaves almost nothing inside and wins by a clear margin.
# Measured over the supplied photographs: boards holding a course score an
# interior of 0.005 to 0.06 and beat the runner up by 0.11 to 0.22, while
# plain maze boards sit at 0.13 to 0.30 interior and 0.005 to 0.048 margin.
COURSE_MAX_INTERIOR = 0.22
COURSE_MIN_MARGIN = 0.05


def detect_course_position(map_data: GlobalMap) -> CourseCandidate:
    ranked = rank_course_positions(map_data, limit=5)
    if not ranked:
        raise RuntimeError("No legal 5 x 5 course position exists on this board")
    best = ranked[0]
    margin = best.score - ranked[1].score if len(ranked) > 1 else 1.0

    if best.interior_wall > COURSE_MAX_INTERIOR or margin < COURSE_MIN_MARGIN:
        table = "\n".join(
            f"    {c.position}  score {c.score:.3f}  interior {c.interior_wall:.3f}"
            f"  boundary {c.boundary_wall:.3f}  cylinders {c.cylinders_inside}"
            for c in ranked
        )
        raise RuntimeError(
            "No 5 x 5 obstacle course was recognised in this photograph.\n"
            f"  best guess {best.position}, interior wall {best.interior_wall:.3f} "
            f"(want below {COURSE_MAX_INTERIOR}), margin {margin:.3f} "
            f"(want above {COURSE_MIN_MARGIN}).\n"
            "  A course interior is empty; every candidate here still has walls\n"
            "  in it, so this is most likely a plain maze photograph.\n"
            "  Ranked candidates:\n" + table + "\n"
            "  If the course really is there, set course_top_row and\n"
            "  course_left_column by hand to override the search."
        )
    return best


def resolve_course_config(
    map_data: GlobalMap,
    config: PlannerConfig,
) -> PlannerConfig:
    """Fills in the course position when it was left as None.

    extract_global_map already decided this, so no image work is repeated.
    """
    if config.course_top_row is not None and config.course_left_column is not None:
        return config
    return replace(
        config,
        course_top_row=map_data.course_top_row,
        course_left_column=map_data.course_left_column,
    )


def detect_course_portals(
    map_data: GlobalMap,
    config: PlannerConfig,
) -> tuple[CoursePortal, ...]:
    config = resolve_course_config(map_data, config)
    top, left = config.course_top_row, config.course_left_column
    edges = (
        ("N", True, top, left, map_data.horizontal_walls[top, left:left + 5]),
        ("E", False, left + 5, top, map_data.vertical_walls[top:top + 5, left + 5]),
        ("S", True, top + 5, left, map_data.horizontal_walls[top + 5, left:left + 5]),
        ("W", False, left, top, map_data.vertical_walls[top:top + 5, left]),
    )
    portals: list[CoursePortal] = []
    for side, horizontal, line, first_segment, walls in edges:
        for offset in range(5):
            coverage = _edge_coverage(
                map_data.wall_mask,
                horizontal,
                line,
                first_segment + offset,
            )
            if not walls[offset] and coverage < config.portal_open_threshold:
                portals.append(CoursePortal(side, offset))
    return tuple(portals)


def extract_grid_maze(map_data: GlobalMap, config: PlannerConfig) -> GridMaze:
    config = resolve_course_config(map_data, config)
    horizontal = map_data.horizontal_walls.copy()
    vertical = map_data.vertical_walls.copy()

    blocked = np.zeros((MAZE_CELLS, MAZE_CELLS), dtype=bool)
    for row, column in BLOCKED_CORNERS:
        blocked[row, column] = True
    top, left = config.course_top_row, config.course_left_column
    blocked[top:top + 5, left:left + 5] = True
    return GridMaze(horizontal, vertical, blocked)


def _portal_pose(portal: CoursePortal, config: PlannerConfig) -> PortalPose:
    if portal.side not in {"N", "E", "S", "W"} or not 0 <= portal.offset < 5:
        raise ValueError(f"Invalid course portal: {portal}")
    top, left = config.course_top_row, config.course_left_column
    if portal.side == "N":
        inside = (top, left + portal.offset)
        outside = (top - 1, left + portal.offset)
        inward = 2
    elif portal.side == "E":
        inside = (top + portal.offset, left + 4)
        outside = (top + portal.offset, left + 5)
        inward = 3
    elif portal.side == "S":
        inside = (top + 4, left + portal.offset)
        outside = (top + 5, left + portal.offset)
        inward = 0
    else:
        inside = (top + portal.offset, left)
        outside = (top + portal.offset, left - 1)
        inward = 1
    if not all(0 <= value < MAZE_CELLS for cell in (inside, outside) for value in cell):
        raise ValueError(f"Course portal lies outside the maze: {portal}")
    return PortalPose(portal, inside, outside, inward)


def _cell_centre(cell: tuple[int, int]) -> tuple[float, float]:
    return (
        (cell[1] + 0.5) * CELL_MM,
        (cell[0] + 0.5) * CELL_MM,
    )


def plan_grid_commands(
    maze: GridMaze,
    start: tuple[int, int, int],
    goal: tuple[int, int, int],
) -> tuple[str, list[tuple[int, int]]]:
    if maze.blocked_cells[start[0], start[1]] or maze.blocked_cells[goal[0], goal[1]]:
        return "", []
    queue = [(0, 0, 0, start)]
    costs = {start: (0, 0)}
    previous: dict[tuple[int, int, int], tuple[tuple[int, int, int], str]] = {}
    sequence = 0
    while queue:
        forwards, turns, _, state = heapq.heappop(queue)
        if costs.get(state) != (forwards, turns):
            continue
        if state == goal:
            break
        row, column, direction = state
        transitions = [
            ("l", (row, column, (direction - 1) % 4), (forwards, turns + 1)),
            ("r", (row, column, (direction + 1) % 4), (forwards, turns + 1)),
        ]
        if maze.can_move(row, column, direction):
            delta_row, delta_column = DELTAS[direction]
            transitions.append(
                (
                    "f",
                    (row + delta_row, column + delta_column, direction),
                    (forwards + 1, turns),
                )
            )
        for command, next_state, next_cost in transitions:
            if next_cost >= costs.get(next_state, (math.inf, math.inf)):
                continue
            costs[next_state] = next_cost
            previous[next_state] = (state, command)
            sequence += 1
            heapq.heappush(queue, (*next_cost, sequence, next_state))
    if goal not in costs:
        return "", []

    commands: list[str] = []
    state = goal
    while state != start:
        state, command = previous[state]
        commands.append(command)
    commands.reverse()
    row, column, direction = start
    cells = [(row, column)]
    for command in commands:
        if command == "l":
            direction = (direction - 1) % 4
        elif command == "r":
            direction = (direction + 1) % 4
        else:
            delta_row, delta_column = DELTAS[direction]
            row += delta_row
            column += delta_column
            cells.append((row, column))
    return "".join(commands), cells


def _course_map(map_data: GlobalMap, config: PlannerConfig) -> GlobalMap:
    top = config.course_top_row * CELL_MM
    left = config.course_left_column * CELL_MM
    bottom = top + COURSE_CELLS * CELL_MM
    right = left + COURSE_CELLS * CELL_MM
    occupancy = np.full_like(map_data.occupancy_mask, 255)
    occupancy[top:bottom, left:right] = map_data.occupancy_mask[top:bottom, left:right]

    half_width = int(round(config.wall_clearance_mm))
    depth = int(round(0.65 * CELL_MM))
    for portal in detect_course_portals(map_data, config):
        pose = _portal_pose(portal, config)
        centre_x, centre_y = (round(value) for value in _cell_centre(pose.inside_cell))
        if portal.side == "N":
            corners = (centre_x - half_width, top, centre_x + half_width, top + depth)
        elif portal.side == "S":
            corners = (centre_x - half_width, bottom - depth, centre_x + half_width, bottom)
        elif portal.side == "W":
            corners = (left, centre_y - half_width, left + depth, centre_y + half_width)
        else:
            corners = (right - depth, centre_y - half_width, right, centre_y + half_width)
        cv2.rectangle(occupancy, corners[:2], corners[2:], 0, -1)

    # Portal carving removes boundary-wall inflation only; cylinders stay protected.
    cylinder_radius = int(round(config.cylinder_clearance_mm))
    for x, y in map_data.obstacles_mm:
        cv2.circle(occupancy, (round(x), round(y)), cylinder_radius, 255, -1)
    return GlobalMap(
        map_data.rectified,
        map_data.wall_mask,
        occupancy,
        map_data.obstacles_mm,
        map_data.dark_threshold,
        map_data.horizontal_walls,
        map_data.vertical_walls,
        map_data.horizontal_scores,
        map_data.vertical_scores,
    )


def _normalise_angle(angle: int | float) -> int:
    return int((round(angle) + 180) % 360 - 180)


def _map_heading(start: tuple[float, float], end: tuple[float, float]) -> int:
    dx, dy = end[0] - start[0], end[1] - start[1]
    return _normalise_angle(math.degrees(math.atan2(-dx, -dy)))


def _endpoint(start: tuple[float, float], heading: int, distance: int) -> tuple[float, float]:
    radians = math.radians(heading)
    return start[0] - distance * math.sin(radians), start[1] - distance * math.cos(radians)


def _point_clear(point: tuple[float, float], occupancy: np.ndarray) -> bool:
    x, y = point
    if not (0 <= x < MAZE_MM and 0 <= y < MAZE_MM):
        return False
    column = int(np.clip(round(x), 0, occupancy.shape[1] - 1))
    row = int(np.clip(round(y), 0, occupancy.shape[0] - 1))
    return occupancy[row, column] == 0


def _segment_clear(
    start: tuple[float, float],
    end: tuple[float, float],
    occupancy: np.ndarray,
) -> bool:
    if not _point_clear(start, occupancy) or not _point_clear(end, occupancy):
        return False
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(2, int(math.ceil(distance / 3.0)) + 1)
    x = np.clip(
        np.rint(np.linspace(start[0], end[0], count)),
        0,
        occupancy.shape[1] - 1,
    ).astype(np.int32)
    y = np.clip(
        np.rint(np.linspace(start[1], end[1], count)),
        0,
        occupancy.shape[0] - 1,
    ).astype(np.int32)
    return not np.any(occupancy[y, x])


def _nearest_node(
    point: tuple[float, float],
    occupancy: np.ndarray,
    step: int,
) -> tuple[int, int]:
    maximum = (MAZE_MM - 1) // step
    target = (
        int(np.clip(round(point[0] / step), 0, maximum)),
        int(np.clip(round(point[1] / step), 0, maximum)),
    )
    for radius in range(13):
        ring = [
            (target[0] + dx, target[1] + dy)
            for dx in range(-radius, radius + 1)
            for dy in range(-radius, radius + 1)
            if max(abs(dx), abs(dy)) == radius
        ]
        ring.sort(key=lambda node: math.hypot(node[0] - target[0], node[1] - target[1]))
        for node in ring:
            if not (0 <= node[0] <= maximum and 0 <= node[1] <= maximum):
                continue
            if _point_clear((node[0] * step, node[1] * step), occupancy):
                return node
    raise ValueError(f"Point {point} is not connected to planning grid")


def _astar(
    start_mm: tuple[float, float],
    goal_mm: tuple[float, float],
    occupancy: np.ndarray,
    config: PlannerConfig,
) -> list[tuple[float, float]]:
    step = config.grid_mm
    start = _nearest_node(start_mm, occupancy, step)
    goal = _nearest_node(goal_mm, occupancy, step)
    moves = (
        (-1, -1), (0, -1), (1, -1),
        (-1, 0),             (1, 0),
        (-1, 1),  (0, 1),   (1, 1),
    )
    maximum = (MAZE_MM - 1) // step
    queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
    cost = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    visited: set[tuple[int, int]] = set()

    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            break
        current_mm = (current[0] * step, current[1] * step)
        for dx, dy in moves:
            neighbour = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbour[0] <= maximum and 0 <= neighbour[1] <= maximum):
                continue
            neighbour_mm = (neighbour[0] * step, neighbour[1] * step)
            if not _segment_clear(current_mm, neighbour_mm, occupancy):
                continue
            new_cost = current_cost + step * math.hypot(dx, dy)
            if new_cost >= cost.get(neighbour, math.inf):
                continue
            cost[neighbour] = new_cost
            parent[neighbour] = current
            heuristic = step * math.hypot(goal[0] - neighbour[0], goal[1] - neighbour[1])
            heapq.heappush(queue, (new_cost + heuristic, new_cost, neighbour))

    if goal not in cost:
        raise RuntimeError("No collision-free global route exists between these poses")
    nodes = [goal]
    while nodes[-1] != start:
        nodes.append(parent[nodes[-1]])
    nodes.reverse()
    points = [(node[0] * step, node[1] * step) for node in nodes]
    points[0] = start_mm
    points[-1] = goal_mm
    return points


def _smooth_path(
    path: Sequence[tuple[float, float]],
    occupancy: np.ndarray,
    minimum_distance: float,
) -> list[tuple[float, float]]:
    costs: list[tuple[int, float] | None] = [None] * len(path)
    previous = [-1] * len(path)
    costs[0] = (0, 0.0)
    for end in range(1, len(path)):
        for start in range(end):
            if costs[start] is None:
                continue
            distance = math.dist(path[start], path[end])
            if distance < minimum_distance:
                continue
            if not _segment_clear(path[start], path[end], occupancy):
                continue
            candidate = (costs[start][0] + 1, costs[start][1] + distance)
            if costs[end] is None or candidate < costs[end]:
                costs[end] = candidate
                previous[end] = start
    if costs[-1] is None:
        raise RuntimeError("No executable straight-segment route exists")
    indices = [len(path) - 1]
    while indices[-1] != 0:
        indices.append(previous[indices[-1]])
    return [path[index] for index in reversed(indices)]


def _split_long_segments(
    points: Sequence[tuple[float, float]],
    maximum_distance: int,
) -> list[tuple[float, float]]:
    split = [points[0]]
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        parts = max(1, int(math.ceil(distance / maximum_distance)))
        for part in range(1, parts + 1):
            fraction = part / parts
            split.append(
                (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
            )
    return split


def _quantise_path(
    points: Sequence[tuple[float, float]],
    occupancy: np.ndarray,
    config: PlannerConfig,
) -> tuple[list[tuple[float, float]], list[Motion]]:
    targets = _split_long_segments(points, config.maximum_segment_mm)
    states = [(0.0, targets[0], [], [], [targets[0]])]
    for index, target in enumerate(targets[1:], 1):
        candidates = []
        for score, actual, headings, distances, actual_points in states:
            ideal_heading = _map_heading(actual, target)
            ideal_distance = max(1, int(round(math.dist(actual, target))))
            for heading_offset in sorted(range(-8, 9), key=abs):
                heading = _normalise_angle(ideal_heading + heading_offset)
                for distance_offset in sorted(range(-10, 11), key=abs):
                    distance = max(1, ideal_distance + distance_offset)
                    end = _endpoint(actual, heading, distance)
                    if not _segment_clear(actual, end, occupancy):
                        continue
                    error = math.dist(end, target)
                    candidates.append(
                        (
                            score + error + 0.05 * abs(heading_offset),
                            end,
                            headings + [heading],
                            distances + [distance],
                            actual_points + [end],
                        )
                    )
        if not candidates:
            raise RuntimeError("Whole-degree rounding removed the required clearance")
        final_segment = index == len(targets) - 1
        candidates.sort(
            key=lambda state: (
                math.dist(state[1], target) if final_segment else state[0],
                state[0],
            )
        )
        states = []
        seen = set()
        for state in candidates:
            key = (round(state[1][0]), round(state[1][1]), state[2][-1])
            if key in seen:
                continue
            seen.add(key)
            states.append(state)
            if len(states) == 40:
                break

    _, _, headings, distances, actual_points = states[0]

    motions: list[Motion] = []
    previous_heading = _normalise_angle(config.start_heading_deg)
    for heading, distance in zip(headings, distances):
        motions.append(Motion(_normalise_angle(heading - previous_heading), distance, heading))
        previous_heading = heading
    if config.goal_heading_deg is not None:
        final_heading = _normalise_angle(config.goal_heading_deg)
        final_turn = _normalise_angle(final_heading - previous_heading)
        if final_turn:
            motions.append(Motion(final_turn, 0, final_heading))
    return actual_points, motions


def _plan_on_map(
    map_data: GlobalMap,
    start_mm: tuple[float, float],
    goal_mm: tuple[float, float],
    config: PlannerConfig,
    render: bool = True,
) -> PlanResult:
    if not _point_clear(start_mm, map_data.occupancy_mask):
        raise ValueError("Start pose is inside a wall or safety area")
    if not _point_clear(goal_mm, map_data.occupancy_mask):
        raise ValueError("Goal pose is inside a wall or safety area")
    grid_path = _astar(start_mm, goal_mm, map_data.occupancy_mask, config)
    path = _smooth_path(
        grid_path,
        map_data.occupancy_mask,
        config.minimum_course_segment_mm,
    )
    waypoints, motions = _quantise_path(path, map_data.occupancy_mask, config)
    if render:
        occupancy_image = draw_occupancy(map_data, config)
        route_image = draw_route(map_data, waypoints, motions, config)
    else:
        occupancy_image = np.empty((0, 0, 3), dtype=np.uint8)
        route_image = np.empty((0, 0, 3), dtype=np.uint8)
    result = PlanResult(
        map_data,
        config,
        [start_mm, goal_mm],
        waypoints,
        motions,
        occupancy_image,
        route_image,
        _normalise_angle(config.start_heading_deg),
        None if config.goal_heading_deg is None else _normalise_angle(config.goal_heading_deg),
    )
    validate_result(result)
    return result


def plan_hybrid_on_map(
    map_data: GlobalMap,
    start_pose: tuple[int, int, int],
    goal_pose: tuple[int, int, int],
    config: PlannerConfig,
    course_cache: dict[tuple[CoursePortal, CoursePortal], PlanResult | None] | None = None,
) -> HybridPlan:
    config = resolve_course_config(map_data, config)
    grid_maze = extract_grid_maze(map_data, config)
    for name, pose in (("Start", start_pose), ("Goal", goal_pose)):
        row, column, direction = pose
        if not (0 <= row < MAZE_CELLS and 0 <= column < MAZE_CELLS and 0 <= direction < 4):
            raise ValueError(f"{name} pose is outside the 9 x 9 maze")
        if grid_maze.blocked_cells[row, column]:
            raise ValueError(f"{name} must be a standard maze cell outside the 5 x 5 course")

    detected = detect_course_portals(map_data, config)
    portals = [_portal_pose(portal, config) for portal in detected]
    candidates: list[tuple[tuple[float, int, float], HybridPlan]] = []
    cache = {} if course_cache is None else course_cache

    direct, direct_cells = plan_grid_commands(grid_maze, start_pose, goal_pose)
    direct_distance = math.inf
    direct_score = math.inf
    if direct_cells:
        direct_plan = HybridPlan(
            map_data,
            grid_maze,
            config,
            start_pose,
            goal_pose,
            None,
            None,
            direct,
            "",
            direct_cells,
            [],
            None,
            np.empty((0, 0, 3), dtype=np.uint8),
            np.empty((0, 0, 3), dtype=np.uint8),
        )
        direct_distance = direct.count("f") * CELL_MM
        direct_turns = direct.count("l") + direct.count("r")
        direct_score = direct_distance + config.route_turn_penalty_mm * direct_turns
        candidates.append(((direct_score, direct_turns, direct_distance), direct_plan))

    continuous_map = _course_map(map_data, config)
    for entry in portals:
        for exit_portal in portals:
            if entry == exit_portal:
                continue
            before_goal = (*entry.outside_cell, entry.inward_direction)
            after_start = (*exit_portal.outside_cell, exit_portal.outward_direction)
            before, cells_before = plan_grid_commands(grid_maze, start_pose, before_goal)
            after, cells_after = plan_grid_commands(grid_maze, after_start, goal_pose)
            if not cells_before or not cells_after:
                continue
            grid_distance = (before.count("f") + after.count("f") + 2) * CELL_MM
            grid_turns = (
                before.count("l") + before.count("r")
                + after.count("l") + after.count("r")
            )
            course_minimum = math.dist(
                _cell_centre(entry.inside_cell),
                _cell_centre(exit_portal.inside_cell),
            )
            optimistic = (
                grid_distance + course_minimum
                + config.route_turn_penalty_mm * grid_turns
            )
            if direct_score <= optimistic:
                continue
            selected = replace(
                config,
                start_heading_deg=DIRECTION_HEADINGS[entry.inward_direction],
                goal_heading_deg=DIRECTION_HEADINGS[exit_portal.outward_direction],
            )
            key = (entry.portal, exit_portal.portal)
            if key not in cache:
                try:
                    cache[key] = _plan_on_map(
                        continuous_map,
                        _cell_centre(entry.inside_cell),
                        _cell_centre(exit_portal.inside_cell),
                        selected,
                        render=False,
                    )
                except (RuntimeError, ValueError, AssertionError):
                    cache[key] = None
            continuous = cache[key]
            if continuous is None:
                continue
            distance = grid_distance + validate_result(continuous)["path_length_mm"]
            turns = grid_turns + sum(
                motion.turn_deg != 0 for motion in continuous.motions
            )
            score = distance + config.route_turn_penalty_mm * turns
            plan = HybridPlan(
                map_data,
                grid_maze,
                config,
                start_pose,
                goal_pose,
                entry,
                exit_portal,
                before,
                after,
                cells_before,
                cells_after,
                continuous,
                np.empty((0, 0, 3), dtype=np.uint8),
                np.empty((0, 0, 3), dtype=np.uint8),
            )
            candidates.append(((score, turns, distance), plan))
    if not candidates:
        raise RuntimeError(
            "No route exists through the detected outside walls or the 5 x 5 course"
        )
    result = min(candidates, key=lambda item: item[0])[1]
    result.occupancy_image = draw_hybrid_occupancy(result)
    result.route_image = draw_hybrid_route(result)
    validate_hybrid_result(result)
    return result


def plan_hybrid_route(
    image_path: str | Path,
    start_pose: tuple[int, int, int],
    goal_pose: tuple[int, int, int],
    config: PlannerConfig = PlannerConfig(),
) -> HybridPlan:
    rectified = rectify_maze(read_image(image_path))
    return plan_hybrid_on_map(
        extract_global_map(rectified, config), start_pose, goal_pose, config
    )


def plan_route(
    image_path: str | Path,
    start_mm: tuple[float, float],
    goal_mm: tuple[float, float],
    config: PlannerConfig = PlannerConfig(),
) -> PlanResult:
    rectified = rectify_maze(read_image(image_path))
    return _plan_on_map(extract_global_map(rectified, config), start_mm, goal_mm, config)


def _draw_pose(
    image: np.ndarray,
    point: tuple[float, float],
    heading: int,
    colour: tuple[int, int, int],
    label: str,
    scale: float = 1.0,
) -> None:
    start = (int(round(point[0] * scale)), int(round(point[1] * scale)))
    tip = _endpoint(point, heading, 65.0 / scale)
    end = (int(round(tip[0] * scale)), int(round(tip[1] * scale)))
    thickness = max(3, int(round(5 * scale)))
    cv2.circle(image, start, max(6, int(round(10 * scale))), colour, -1, cv2.LINE_AA)
    cv2.arrowedLine(image, start, end, colour, thickness, cv2.LINE_AA, tipLength=0.30)
    cv2.putText(
        image,
        label,
        (start[0] + 12, start[1] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        max(0.42, 0.65 * scale),
        colour,
        max(1, int(round(2 * scale))),
        cv2.LINE_AA,
    )


def _draw_grid(image: np.ndarray) -> None:
    for index in range(MAZE_CELLS + 1):
        coordinate = min(MAZE_MM - 1, index * CELL_MM)
        cv2.line(image, (coordinate, 0), (coordinate, MAZE_MM - 1), (125, 130, 135), 2)
        cv2.line(image, (0, coordinate), (MAZE_MM - 1, coordinate), (125, 130, 135), 2)


def draw_occupancy(map_data: GlobalMap, config: PlannerConfig) -> np.ndarray:
    view = map_data.rectified.copy()
    overlay = view.copy()
    overlay[map_data.occupancy_mask > 0] = (0, 150, 255)
    view = cv2.addWeighted(view, 0.58, overlay, 0.42, 0)
    wall_overlay = view.copy()
    wall_overlay[map_data.wall_mask > 0] = (0, 30, 230)
    view = cv2.addWeighted(view, 0.70, wall_overlay, 0.30, 0)
    for index, (x, y) in enumerate(map_data.obstacles_mm, 1):
        centre = (round(x), round(y))
        cv2.circle(view, centre, round(config.obstacle_diameter_mm / 2), (0, 0, 255), 5)
        cv2.putText(
            view, f"O{index}", (centre[0] + 12, centre[1] - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA,
        )
    _draw_grid(view)
    return view


def draw_route(
    map_data: GlobalMap,
    points: Sequence[tuple[float, float]],
    motions: Sequence[Motion],
    config: PlannerConfig,
) -> np.ndarray:
    view = draw_occupancy(map_data, config)
    pixel_points = [(round(x), round(y)) for x, y in points]
    driven = [motion for motion in motions if motion.distance_mm > 0]
    for index, (start, end, motion) in enumerate(
        zip(pixel_points, pixel_points[1:], driven), 1
    ):
        cv2.arrowedLine(view, start, end, (255, 70, 30), 7, cv2.LINE_AA, tipLength=0.04)
        midpoint = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
        cv2.putText(
            view,
            f"{index}: {motion.heading_deg}deg/{motion.distance_mm}mm",
            midpoint,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 70, 30),
            2,
            cv2.LINE_AA,
        )
    for index, point in enumerate(pixel_points[1:-1], 1):
        cv2.circle(view, point, 8, (255, 255, 0), -1, cv2.LINE_AA)
        cv2.putText(
            view, str(index), (point[0] + 10, point[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA,
        )
    _draw_pose(view, points[0], config.start_heading_deg, (40, 220, 40), "S")
    final_heading = (
        config.goal_heading_deg
        if config.goal_heading_deg is not None
        else motions[-1].heading_deg if motions else config.start_heading_deg
    )
    _draw_pose(view, points[-1], final_heading, (220, 40, 220), "G")
    return view


def _course_cell(row: int, column: int, config: PlannerConfig) -> bool:
    return (
        config.course_top_row <= row < config.course_top_row + 5
        and config.course_left_column <= column < config.course_left_column + 5
    )


def _draw_detected_grid_walls(
    image: np.ndarray,
    maze: GridMaze,
    config: PlannerConfig,
) -> None:
    colour = (20, 35, 235)
    for row in range(MAZE_CELLS + 1):
        for column in range(MAZE_CELLS):
            adjacent = ((row - 1, column), (row, column))
            if any(_course_cell(r, c, config) for r, c in adjacent):
                continue
            if maze.horizontal_walls[row, column]:
                y = min(row * CELL_MM, MAZE_MM - 1)
                cv2.line(
                    image,
                    (column * CELL_MM, y),
                    ((column + 1) * CELL_MM, y),
                    colour,
                    8,
                    cv2.LINE_AA,
                )
    for row in range(MAZE_CELLS):
        for column in range(MAZE_CELLS + 1):
            adjacent = ((row, column - 1), (row, column))
            if any(_course_cell(r, c, config) for r, c in adjacent):
                continue
            if maze.vertical_walls[row, column]:
                x = min(column * CELL_MM, MAZE_MM - 1)
                cv2.line(
                    image,
                    (x, row * CELL_MM),
                    (x, (row + 1) * CELL_MM),
                    colour,
                    8,
                    cv2.LINE_AA,
                )


def _draw_course_boundary(
    image: np.ndarray,
    portals: Sequence[CoursePortal],
    config: PlannerConfig,
) -> None:
    top = config.course_top_row * CELL_MM
    left = config.course_left_column * CELL_MM
    bottom, right = top + 5 * CELL_MM, left + 5 * CELL_MM
    openings = {(portal.side, portal.offset) for portal in portals}
    for side in DIRECTIONS:
        for offset in range(5):
            if side == "N":
                start = (left + offset * CELL_MM, top)
                end = (left + (offset + 1) * CELL_MM, top)
            elif side == "E":
                start = (right, top + offset * CELL_MM)
                end = (right, top + (offset + 1) * CELL_MM)
            elif side == "S":
                start = (left + offset * CELL_MM, bottom)
                end = (left + (offset + 1) * CELL_MM, bottom)
            else:
                start = (left, top + offset * CELL_MM)
                end = (left, top + (offset + 1) * CELL_MM)
            if (side, offset) not in openings:
                cv2.line(image, start, end, (0, 215, 255), 8, cv2.LINE_AA)
                continue
            centre = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
            cv2.circle(image, centre, 13, (40, 220, 40), 4, cv2.LINE_AA)
            cv2.putText(
                image, f"{side}{offset}", (centre[0] + 15, centre[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (40, 220, 40), 2, cv2.LINE_AA,
            )


def _hybrid_map_view(
    map_data: GlobalMap,
    maze: GridMaze,
    config: PlannerConfig,
) -> np.ndarray:
    view = map_data.rectified.copy()
    top = config.course_top_row * CELL_MM
    left = config.course_left_column * CELL_MM
    bottom, right = top + 5 * CELL_MM, left + 5 * CELL_MM
    course = view[top:bottom, left:right]
    overlay = course.copy()
    course_occupancy = _course_map(map_data, config).occupancy_mask
    occupied = course_occupancy[top:bottom, left:right] > 0
    overlay[occupied] = (0, 150, 255)
    view[top:bottom, left:right] = cv2.addWeighted(course, 0.60, overlay, 0.40, 0)
    _draw_grid(view)
    _draw_detected_grid_walls(view, maze, config)
    _draw_course_boundary(view, detect_course_portals(map_data, config), config)
    for index, (x, y) in enumerate(map_data.obstacles_mm, 1):
        if not (left <= x <= right and top <= y <= bottom):
            continue
        centre = (round(x), round(y))
        cv2.circle(view, centre, round(config.obstacle_diameter_mm / 2), (0, 0, 255), 5)
        cv2.putText(
            view, f"O{index}", (centre[0] + 12, centre[1] - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA,
        )
    return view


def draw_hybrid_occupancy(result: HybridPlan) -> np.ndarray:
    view = _hybrid_map_view(result.map_data, result.grid_maze, result.config)
    if result.uses_course:
        assert result.entry is not None and result.exit is not None
        for label, portal, colour in (
            ("P1", result.entry, (40, 220, 40)),
            ("P2", result.exit, (220, 40, 220)),
        ):
            point = tuple(round(value) for value in _cell_centre(portal.inside_cell))
            cv2.circle(view, point, 18, colour, 5, cv2.LINE_AA)
            cv2.putText(
                view, label, (point[0] + 22, point[1] - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour, 2, cv2.LINE_AA,
            )
    return view


def _draw_cell_path(
    image: np.ndarray,
    cells: Sequence[tuple[int, int]],
    colour: tuple[int, int, int],
) -> None:
    points = [tuple(round(value) for value in _cell_centre(cell)) for cell in cells]
    for start, end in zip(points, points[1:]):
        cv2.arrowedLine(image, start, end, colour, 9, cv2.LINE_AA, tipLength=0.10)


def draw_hybrid_route(result: HybridPlan) -> np.ndarray:
    view = draw_hybrid_occupancy(result)
    _draw_cell_path(view, result.cells_before, (40, 210, 40))
    _draw_cell_path(view, result.cells_after, (220, 170, 30))

    if not result.uses_course:
        _draw_pose(
            view, _cell_centre(result.start_pose[:2]),
            DIRECTION_HEADINGS[result.start_pose[2]], (40, 220, 40), "S",
        )
        _draw_pose(
            view, _cell_centre(result.goal_pose[:2]),
            DIRECTION_HEADINGS[result.goal_pose[2]], (220, 40, 220), "G",
        )
        cv2.putText(
            view, "GRID WALL ROUTE", (25, 55), cv2.FONT_HERSHEY_SIMPLEX,
            0.78, (40, 210, 40), 2, cv2.LINE_AA,
        )
        return view

    assert result.entry is not None and result.exit is not None
    assert result.continuous is not None

    entry_outside = tuple(round(value) for value in _cell_centre(result.entry.outside_cell))
    entry_inside = tuple(round(value) for value in _cell_centre(result.entry.inside_cell))
    exit_inside = tuple(round(value) for value in _cell_centre(result.exit.inside_cell))
    exit_outside = tuple(round(value) for value in _cell_centre(result.exit.outside_cell))
    cv2.arrowedLine(view, entry_outside, entry_inside, (40, 210, 40), 9, cv2.LINE_AA, tipLength=0.10)

    points = [tuple(round(value) for value in point) for point in result.continuous.waypoints_mm]
    for start, end in zip(points, points[1:]):
        cv2.arrowedLine(view, start, end, (255, 70, 30), 9, cv2.LINE_AA, tipLength=0.05)
    for index, point in enumerate(points[1:-1], 1):
        cv2.circle(view, point, 8, (255, 255, 0), -1, cv2.LINE_AA)
        cv2.putText(
            view, str(index), (point[0] + 10, point[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA,
        )
    cv2.arrowedLine(view, exit_inside, exit_outside, (220, 170, 30), 9, cv2.LINE_AA, tipLength=0.10)

    _draw_pose(
        view,
        _cell_centre(result.start_pose[:2]),
        DIRECTION_HEADINGS[result.start_pose[2]],
        (40, 220, 40),
        "S",
    )
    _draw_pose(
        view,
        _cell_centre(result.goal_pose[:2]),
        DIRECTION_HEADINGS[result.goal_pose[2]],
        (220, 40, 220),
        "G",
    )
    cv2.putText(view, "GRID", entry_outside, cv2.FONT_HERSHEY_SIMPLEX, 0.65, (40, 210, 40), 2)
    course_label = (result.config.course_left_column * CELL_MM + 25,
                    result.config.course_top_row * CELL_MM + 45)
    cv2.putText(view, "CONTINUOUS 5x5", course_label, cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (255, 70, 30), 2, cv2.LINE_AA)
    cv2.putText(view, "GRID", exit_outside, cv2.FONT_HERSHEY_SIMPLEX, 0.65, (220, 170, 30), 2)
    return view


def navigation_steps(result: PlanResult) -> list[str]:
    x, y = result.control_points_mm[0]
    lines = [
        f"START  x={x:.1f} mm, y={y:.1f} mm, heading={result.start_heading_deg:+d} deg"
    ]
    action = 1
    segment = 0
    for motion in result.motions:
        if motion.turn_deg:
            lines.append(
                f"STEP {action:02d}  TURN  {motion.turn_deg:+d} deg"
                f" -> heading {motion.heading_deg:+d} deg"
            )
            action += 1
        if motion.distance_mm:
            segment += 1
            x, y = result.waypoints_mm[segment]
            lines.append(
                f"STEP {action:02d}  DRIVE {motion.distance_mm:d} mm"
                f" -> x={x:.1f} mm, y={y:.1f} mm"
            )
            action += 1
    final_heading = (
        result.goal_heading_deg
        if result.goal_heading_deg is not None
        else result.motions[-1].heading_deg if result.motions else result.start_heading_deg
    )
    lines.append(f"GOAL   x={x:.1f} mm, y={y:.1f} mm, heading={final_heading:+d} deg")
    return lines


def validate_result(result: PlanResult) -> dict[str, float]:
    if len(result.waypoints_mm) < 2:
        raise AssertionError("Route has no straight segment")
    driven = [motion for motion in result.motions if motion.distance_mm > 0]
    if len(driven) != len(result.waypoints_mm) - 1:
        raise AssertionError("Motion count does not match waypoint count")
    path_length = 0.0
    for start, end, motion in zip(result.waypoints_mm, result.waypoints_mm[1:], driven):
        if not isinstance(motion.turn_deg, int) or not isinstance(motion.heading_deg, int):
            raise AssertionError("A turn or heading is not an integer")
        if not 1 <= motion.distance_mm <= result.config.maximum_segment_mm + 2:
            raise AssertionError("A straight segment is outside Arduino limits")
        if not _segment_clear(start, end, result.map_data.occupancy_mask):
            raise AssertionError("Rounded route crosses an occupied safety area")
        path_length += math.hypot(end[0] - start[0], end[1] - start[1])
    goal = result.control_points_mm[-1]
    predicted = result.waypoints_mm[-1]
    goal_error = math.dist(goal, predicted)
    if goal_error > 10.0:
        raise AssertionError("Integer-heading route finishes more than 10 mm from the goal")
    return {
        "path_length_mm": path_length,
        "goal_error_mm": goal_error,
    }


def _grid_tokens(commands: str) -> list[str]:
    tokens: list[str] = []
    for command in commands:
        if command == "f":
            tokens.append(f"G{CELL_MM};")
        elif command == "l":
            tokens.append("T90;")
        elif command == "r":
            tokens.append("T-90;")
        else:
            raise ValueError(f"Unknown grid command: {command!r}")
    return tokens


def hybrid_route_tokens(result: HybridPlan) -> list[str]:
    if not result.uses_course:
        return ["BEGIN;", *_grid_tokens(result.grid_before), "END;"]
    assert result.continuous is not None
    tokens = ["BEGIN;", *_grid_tokens(result.grid_before), f"G{CELL_MM};"]
    for motion in result.continuous.motions:
        if motion.turn_deg:
            tokens.append(f"T{motion.turn_deg};")
        if motion.distance_mm:
            tokens.append(f"F{motion.distance_mm};")
    tokens.extend((f"G{CELL_MM};", *_grid_tokens(result.grid_after), "END;"))
    return tokens


def hybrid_arduino_declaration(result: HybridPlan) -> str:
    return f'#define GENERATED_ROUTE "{"".join(hybrid_route_tokens(result))}"'


def _grid_pose_text(pose: tuple[int, int, int]) -> str:
    return f"({pose[0]},{pose[1]},{DIRECTIONS[pose[2]]})"


def validate_hybrid_result(result: HybridPlan) -> dict[str, float]:
    if not result.cells_before or result.cells_before[0] != result.start_pose[:2]:
        raise AssertionError("GRID path does not start at the selected pose")
    if not result.uses_course:
        if result.cells_before[-1] != result.goal_pose[:2]:
            raise AssertionError("GRID path does not reach the selected goal")
        forwards = result.grid_before.count("f")
        return {
            "path_length_mm": float(forwards * CELL_MM),
            "goal_error_mm": 0.0,
            "grid_cells": float(forwards),
        }

    assert result.entry is not None and result.exit is not None
    assert result.continuous is not None
    metrics = validate_result(result.continuous)
    if result.cells_before[-1] != result.entry.outside_cell:
        raise AssertionError("GRID-before path does not reach the course entrance")
    if result.cells_after[0] != result.exit.outside_cell:
        raise AssertionError("GRID-after path does not start at the course exit")
    if result.cells_after[-1] != result.goal_pose[:2]:
        raise AssertionError("GRID-after path does not reach the selected goal")
    top = result.config.course_top_row * CELL_MM
    left = result.config.course_left_column * CELL_MM
    for x, y in result.continuous.waypoints_mm:
        if not (
            left <= x <= left + COURSE_CELLS * CELL_MM
            and top <= y <= top + COURSE_CELLS * CELL_MM
        ):
            raise AssertionError("A continuous waypoint left the 5 x 5 course")
    metrics["grid_cells"] = float(
        result.grid_before.count("f") + result.grid_after.count("f") + 2
    )
    return metrics


def hybrid_navigation_steps(result: HybridPlan) -> list[str]:
    if not result.uses_course:
        return [
            f"START {_grid_pose_text(result.start_pose)}",
            f"GRID WALL ROUTE: {result.grid_before or '(already at goal pose)'}",
            f"GOAL {_grid_pose_text(result.goal_pose)}",
        ]
    assert result.entry is not None and result.exit is not None
    assert result.continuous is not None
    lines = [
        f"START {_grid_pose_text(result.start_pose)}",
        f"GRID BEFORE: {result.grid_before or '(already at entrance)'}",
        f"ENTER: G{CELL_MM} to cell {result.entry.inside_cell}",
        "CONTINUOUS 5x5:",
        *[f"  {line}" for line in navigation_steps(result.continuous)],
        f"EXIT: G{CELL_MM} to cell {result.exit.outside_cell}",
        f"GRID AFTER: {result.grid_after or '(already at goal)'}",
        f"GOAL {_grid_pose_text(result.goal_pose)}",
    ]
    return lines


def save_hybrid_result(result: HybridPlan, output_dir: str | Path) -> Path:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    write_image(output / "rectified_maze.png", result.map_data.rectified)
    write_image(output / "hybrid_occupancy_map.png", result.occupancy_image)
    write_image(output / "hybrid_planned_route.png", result.route_image)
    route_file = output / "route_commands.txt"
    entry_text = exit_text = "not used"
    if result.uses_course:
        assert result.entry is not None and result.exit is not None
        entry_text = (
            f"{result.entry.portal.side}{result.entry.portal.offset}, "
            f"inside {result.entry.inside_cell}"
        )
        exit_text = (
            f"{result.exit.portal.side}{result.exit.portal.offset}, "
            f"inside {result.exit.inside_cell}"
        )
    lines = [
        f"START = {_grid_pose_text(result.start_pose)}",
        f"GOAL = {_grid_pose_text(result.goal_pose)}",
        f"MODE = {'GRID + CONTINUOUS 5x5' if result.uses_course else 'GRID WALL ROUTE'}",
        f"COURSE = top-left ({result.config.course_top_row}, {result.config.course_left_column}), size 5 x 5",
        "DETECTED PORTALS = " + ", ".join(
            f"{portal.side}{portal.offset}"
            for portal in detect_course_portals(result.map_data, result.config)
        ),
        f"ENTRY = {entry_text}",
        f"EXIT = {exit_text}",
        "",
        "# Complete navigation",
        *hybrid_navigation_steps(result),
        "",
        "# Paste between the GENERATED ROUTE markers in the Arduino sketch",
        hybrid_arduino_declaration(result),
        "",
        "# G = normal 180 mm grid cell with wall following",
        "# F = continuous-course straight distance without wall following",
        " ".join(hybrid_route_tokens(result)),
    ]
    route_file.write_text("\n".join(lines) + "\n", encoding="ascii")
    return route_file


def print_hybrid_result(result: HybridPlan) -> None:
    metrics = validate_hybrid_result(result)
    print(f"Start: {_grid_pose_text(result.start_pose)}")
    print(f"Goal:  {_grid_pose_text(result.goal_pose)}")
    print(f"Mode:  {'GRID + CONTINUOUS 5x5' if result.uses_course else 'GRID WALL ROUTE'}")
    print("Detected portals: " + ", ".join(
        f"{portal.side}{portal.offset}"
        for portal in detect_course_portals(result.map_data, result.config)
    ))
    if result.uses_course:
        assert result.entry is not None and result.exit is not None
        print(f"Course entry: {result.entry.portal.side}{result.entry.portal.offset}")
        print(f"Course exit:  {result.exit.portal.side}{result.exit.portal.offset}")
    print(f"GRID cells: {int(metrics['grid_cells'])}")
    if result.uses_course:
        print(f"Continuous path: {metrics['path_length_mm']:.1f} mm")
        print(f"Continuous goal error: {metrics['goal_error_mm']:.1f} mm")
    print("\n".join(hybrid_navigation_steps(result)))
    print("\nPaste into Arduino:")
    print(hybrid_arduino_declaration(result))


class ContinuousNavigatorUI:
    def __init__(
        self,
        image_path: str | Path,
        config: PlannerConfig = PlannerConfig(),
        output_dir: str | Path = "output",
    ) -> None:
        self.source = Path(image_path)
        self.config = config
        self.output_dir = Path(output_dir)
        self.original = read_image(self.source)
        self.map_data = extract_global_map(rectify_maze(self.original), config)
        self.grid_maze = extract_grid_maze(self.map_data, config)
        self.portals = detect_course_portals(self.map_data, config)
        self.course_cache: dict[
            tuple[CoursePortal, CoursePortal], PlanResult | None
        ] = {}
        self.occupancy = _hybrid_map_view(self.map_data, self.grid_maze, config)
        self.start: tuple[int, int, int] | None = None
        self.goal: tuple[int, int, int] | None = None
        self.pending_cell: tuple[int, int] | None = None
        self.drag: tuple[int, int, tuple[int, int]] | None = None
        self.result: HybridPlan | None = None
        self.solving = False
        self.status = "Set START in a standard maze cell"
        self.window = "Week 12 - Hybrid grid and continuous navigation"
        self.clipboard_root = None

    @staticmethod
    def _pose_text(pose: tuple[int, int, int] | None) -> str:
        if pose is None:
            return "-"
        return _grid_pose_text(pose)

    @staticmethod
    def _put(
        image: np.ndarray,
        text: str,
        x: int,
        y: int,
        scale: float = 0.50,
        colour: tuple[int, int, int] = (220, 225, 230),
        thickness: int = 1,
    ) -> None:
        cv2.putText(
            image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
            scale, colour, thickness, cv2.LINE_AA,
        )

    @staticmethod
    def _screen_cell(x: int, y: int) -> tuple[int, int]:
        cell_px = DISPLAY_PX / MAZE_CELLS
        return (
            int(np.clip(y // cell_px, 0, MAZE_CELLS - 1)),
            int(np.clip(x // cell_px, 0, MAZE_CELLS - 1)),
        )

    def assign_pose(self, cell: tuple[int, int], direction: int) -> None:
        self.pending_cell = None
        if self.grid_maze.blocked_cells[cell]:
            self.status = "Choose a standard cell outside the 5 x 5 course"
            return
        pose = (cell[0], cell[1], direction % 4)
        if self.start is None:
            self.start = pose
            self.status = "Set GOAL in a standard maze cell"
        elif self.goal is None:
            self.goal = pose
            self.solve()
        else:
            self.start = pose
            self.goal = None
            self.result = None
            self.status = "New START set; now set GOAL"

    def solve(self) -> None:
        if self.solving:
            return
        assert self.start is not None and self.goal is not None
        self.solving = True
        try:
            try:
                result = plan_hybrid_on_map(
                    self.map_data,
                    self.start,
                    self.goal,
                    self.config,
                    self.course_cache,
                )
            except (RuntimeError, ValueError, AssertionError) as error:
                self.goal = None
                self.result = None
                self.status = f"No route: {error}"
                print(self.status)
                return
            route_file = save_hybrid_result(result, self.output_dir)
            self.result = result
            mode = "Hybrid" if result.uses_course else "Outside GRID"
            self.status = f"{mode} ROUTE READY - Esc closes the window"
            print_hybrid_result(result)
            print(f"Saved commands: {route_file.resolve()}", flush=True)
            self.copy_commands(result)
        finally:
            self.solving = False

    def reset(self) -> None:
        self.start = None
        self.goal = None
        self.pending_cell = None
        self.drag = None
        self.result = None
        self.status = "Set START in a standard maze cell"

    def copy_commands(self, result: HybridPlan | None = None) -> None:
        selected = self.result if result is None else result
        if selected is None:
            return
        try:
            if self.clipboard_root is None:
                import tkinter as tk

                self.clipboard_root = tk.Tk()
                self.clipboard_root.withdraw()
            text = hybrid_arduino_declaration(selected)
            self.clipboard_root.clipboard_clear()
            self.clipboard_root.clipboard_append(text)
            self.clipboard_root.update()
        except Exception:
            self.clipboard_root = None

    def save(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        write_image(self.output_dir / "rectified_maze.png", self.map_data.rectified)
        write_image(self.output_dir / "hybrid_occupancy_map.png", self.occupancy)
        if self.result is not None:
            save_hybrid_result(self.result, self.output_dir)

    def mouse(self, event: int, x: int, y: int, _flags: int, _parameter) -> None:
        if self.solving:
            return
        if event == cv2.EVENT_RBUTTONUP:
            self.reset()
            return
        if event == cv2.EVENT_LBUTTONDOWN and 0 <= x < DISPLAY_PX and 0 <= y < DISPLAY_PX:
            self.drag = (x, y, self._screen_cell(x, y))
            return
        if event != cv2.EVENT_LBUTTONUP or self.drag is None:
            return
        start_x, start_y, cell = self.drag
        self.drag = None
        dx, dy = x - start_x, y - start_y
        if math.hypot(dx, dy) < 14:
            self.pending_cell = cell
            self.status = "Press N, E, S or W"
            return
        if abs(dx) >= abs(dy):
            direction = 1 if dx > 0 else 3
        else:
            direction = 2 if dy > 0 else 0
        self.assign_pose(cell, direction)

    @staticmethod
    def _wrapped_tokens(tokens: Sequence[str], width: int = 34) -> list[str]:
        lines: list[str] = []
        line = ""
        for token in tokens:
            candidate = token if not line else f"{line} {token}"
            if len(candidate) > width:
                lines.append(line)
                line = token
            else:
                line = candidate
        if line:
            lines.append(line)
        return lines

    def render(self) -> np.ndarray:
        source = self.result.route_image if self.result is not None else self.occupancy
        view = cv2.resize(source, (DISPLAY_PX, DISPLAY_PX), interpolation=cv2.INTER_AREA)
        scale = DISPLAY_PX / MAZE_MM
        if self.result is None and self.start is not None:
            _draw_pose(
                view,
                _cell_centre(self.start[:2]),
                DIRECTION_HEADINGS[self.start[2]],
                (40, 220, 40),
                "S",
                scale,
            )
        if self.pending_cell is not None:
            centre = tuple(round(value * scale) for value in _cell_centre(self.pending_cell))
            cv2.circle(view, centre, 11, (0, 210, 255), 3, cv2.LINE_AA)
        for row, column in BLOCKED_CORNERS:
            x0, y0 = round(column * CELL_MM * scale), round(row * CELL_MM * scale)
            x1, y1 = round((column + 1) * CELL_MM * scale), round((row + 1) * CELL_MM * scale)
            cv2.line(view, (x0 + 16, y0 + 16), (x1 - 16, y1 - 16), (185, 190, 195), 4)
            cv2.line(view, (x1 - 16, y0 + 16), (x0 + 16, y1 - 16), (185, 190, 195), 4)

        panel = np.full((DISPLAY_PX, PANEL_PX, 3), (29, 32, 36), dtype=np.uint8)
        self._put(panel, "WEEK 12 HYBRID NAVIGATION", 20, 38, 0.64, (245, 245, 245), 2)
        mode_text = "DETECTED WALL GRID + CONTINUOUS 5x5"
        if self.result is not None and not self.result.uses_course:
            mode_text = "DETECTED WALL GRID ROUTE"
        self._put(panel, mode_text, 20, 72, 0.43, (175, 185, 195))
        self._put(panel, f"Start: {self._pose_text(self.start)}", 20, 108, 0.48, (70, 220, 90), 2)
        self._put(panel, f"Goal:  {self._pose_text(self.goal)}", 20, 140, 0.48, (220, 90, 220), 2)
        self._put(
            panel,
            f"Cylinders: {len(self.map_data.obstacles_mm)}",
            20,
            174,
            0.46,
            (175, 185, 195),
        )
        portal_names = ",".join(
            f"{portal.side}{portal.offset}" for portal in self.portals
        ) or "none"
        portal_text = f"Open({len(self.portals)}): {portal_names}"
        self._put(panel, portal_text, 190, 174, 0.42, (175, 185, 195))
        self._put(panel, "Complete Arduino route", 20, 214, 0.54, (70, 205, 245), 2)

        tokens = ["-"]
        if self.result is not None:
            tokens = hybrid_route_tokens(self.result)
        lines = self._wrapped_tokens(tokens)
        y = 246
        for line in lines[:13]:
            self._put(panel, line, 20, y, 0.46, (245, 215, 85))
            y += 25
        if len(lines) > 13:
            self._put(panel, f"+ {len(lines) - 13} more lines in route_commands.txt", 20, y, 0.40)

        self._put(panel, "Drag in START cell, then GOAL cell", 20, 626, 0.44, (185, 192, 202))
        self._put(panel, "Click only: N/E/S/W    R: reset", 20, 653, 0.42, (185, 192, 202))
        self._put(panel, "C: copy code    S: save    Esc: close", 20, 678, 0.42, (185, 192, 202))
        self._put(panel, self.status[:50], 20, 707, 0.41, (90, 205, 245))
        return np.hstack((view, panel))

    def run(self) -> HybridPlan | None:
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window, self.mouse)
        preview = self.original.copy()
        maximum = max(preview.shape[:2])
        if maximum > 900:
            preview = cv2.resize(
                preview, None, fx=900 / maximum, fy=900 / maximum,
                interpolation=cv2.INTER_AREA,
            )
        cv2.imshow("Week 12 - Original photo input", preview)
        self.save()
        cardinal = {
            ord("n"): 0, ord("N"): 0,
            ord("e"): 1, ord("E"): 1,
            ord("s"): 2, ord("S"): 2,
            ord("w"): 3, ord("W"): 3,
        }
        while True:
            cv2.imshow(self.window, self.render())
            key = cv2.waitKey(20) & 0xFF
            if cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == 27:
                break
            if self.pending_cell is not None and key in cardinal:
                self.assign_pose(self.pending_cell, cardinal[key])
            elif key in (ord("r"), ord("R")):
                self.reset()
            elif key in (ord("c"), ord("C")):
                self.copy_commands()
            elif key in (ord("s"), ord("S")):
                self.save()
        cv2.destroyAllWindows()
        if self.clipboard_root is not None:
            self.clipboard_root.destroy()
            self.clipboard_root = None
        return self.result


def send_to_arduino(
    port: str,
    result: HybridPlan,
    baudrate: int = 115200,
    ready_timeout_s: float = 20.0,
) -> None:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("Install pyserial in the Week 12 environment first") from error

    with serial.Serial(port, baudrate, timeout=0.2) as board:
        deadline = time.time() + ready_timeout_s
        while time.time() < deadline:
            if board.readline().decode(errors="ignore").strip() == "READY":
                break
        else:
            raise TimeoutError("Arduino did not report READY")

        for token in hybrid_route_tokens(result):
            board.write((token + "\n").encode("ascii"))
            board.flush()
            expected = "FINISHED" if token == "END;" else "DONE"
            while True:
                line = board.readline().decode(errors="ignore").strip()
                if line == expected:
                    break
                if line in {"BLOCKED", "ERROR"}:
                    raise RuntimeError(f"Arduino stopped: {line}")


def _main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Week 12 Task 4.2 planner")
    parser.add_argument("image", type=Path)
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--course-row", type=int, default=1)
    parser.add_argument("--course-column", type=int, default=3)
    args = parser.parse_args()
    config = PlannerConfig(
        course_top_row=args.course_row,
        course_left_column=args.course_column,
    )
    ContinuousNavigatorUI(args.image, config, args.output).run()


if __name__ == "__main__":
    _main()
