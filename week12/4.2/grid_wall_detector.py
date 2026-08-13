from __future__ import annotations

import math

import cv2
import numpy as np


WALL_SCORE = 0.85


def _cyan_mask(image: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (86, 55, 28), (108, 255, 255))
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )


def _odd(value: float) -> int:
    size = max(3, int(round(value)))
    return size if size % 2 else size + 1


def _component_score(
    mask: np.ndarray,
    vertical: bool,
    segment: int,
    grid_line: int,
    rows: int,
    columns: int,
) -> float:
    height, width = mask.shape
    cell_x, cell_y = width / columns, height / rows
    if vertical:
        y0, y1 = round((segment + 0.08) * cell_y), round((segment + 0.92) * cell_y)
        x0 = max(0, round(grid_line * cell_x - 0.40 * cell_x))
        x1 = min(width, round(grid_line * cell_x + 0.40 * cell_x) + 1)
        along, across, target = cell_y, cell_x, grid_line * cell_x
    else:
        x0, x1 = round((segment + 0.08) * cell_x), round((segment + 0.92) * cell_x)
        y0 = max(0, round(grid_line * cell_y - 0.40 * cell_y))
        y1 = min(height, round(grid_line * cell_y + 0.40 * cell_y) + 1)
        along, across, target = cell_x, cell_y, grid_line * cell_y

    crop = mask[y0:y1, x0:x1]
    count, labels, stats, _ = cv2.connectedComponentsWithStats(crop)
    projection = np.zeros(crop.shape[0] if vertical else crop.shape[1], dtype=bool)
    for label in range(1, count):
        x, y, component_width, component_height, area = stats[label]
        length = component_height if vertical else component_width
        thickness = component_width if vertical else component_height
        if area < cell_x * cell_y * 0.002:
            continue
        if thickness < max(4, round(0.025 * across)):
            continue
        if length < 0.65 * along and length < 1.35 * thickness:
            continue
        axis_edges = (
            (x0 + x, x0 + x + component_width - 1)
            if vertical
            else (y0 + y, y0 + y + component_height - 1)
        )
        if min(abs(value - target) for value in axis_edges) > 0.30 * across:
            continue
        projection |= np.any(labels == label, axis=1 if vertical else 0)

    occupied = np.flatnonzero(projection)
    if not occupied.size:
        return 0.0
    coverage = occupied.size / along
    span = (occupied[-1] - occupied[0] + 1) / along
    if coverage >= 0.65 and span >= 0.65:
        return 1.0
    return 0.70 if coverage >= 0.45 and span >= 0.45 else 0.0


def _score_grid(mask: np.ndarray, vertical: bool, rows: int, columns: int) -> np.ndarray:
    shape = (rows, columns + 1) if vertical else (rows + 1, columns)
    scores = np.zeros(shape, dtype=np.float32)
    for row in range(shape[0]):
        for column in range(shape[1]):
            segment, line = (row, column) if vertical else (column, row)
            scores[row, column] = _component_score(
                mask, vertical, segment, line, rows, columns
            )
    return scores


def _directional_mask(
    gray: np.ndarray,
    cyan_guard: np.ndarray,
    limit: int,
    vertical: bool,
    cell: float,
) -> np.ndarray:
    mask = cv2.inRange(gray, 0, limit)
    mask[cyan_guard > 0] = 0
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
    )
    length = _odd(0.29 * cell)
    kernel = (3, length) if vertical else (length, 3)
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, kernel),
    )


def _local_directional_mask(
    gray: np.ndarray,
    guard: np.ndarray,
    vertical: bool,
    cell: float,
) -> np.ndarray:
    """Extract wall bodies by local contrast when global exposure is uneven."""
    background = cv2.GaussianBlur(gray, (_odd(0.65 * cell), _odd(0.65 * cell)), 0)
    contrast = cv2.subtract(background, gray)
    mask = ((contrast >= 24) & (gray <= 180)).astype(np.uint8) * 255
    mask[guard > 0] = 0
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
    )
    length = _odd(0.27 * cell)
    kernel = (3, length) if vertical else (length, 3)
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, kernel),
    )


def _compact_object_guard(gray: np.ndarray, cell: float, otsu: float) -> np.ndarray:
    """Mask compact dark objects so their silhouettes cannot become grid walls."""
    dark = cv2.inRange(gray, 0, min(105, round(0.78 * otsu)))
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    guard = np.zeros_like(gray)
    contours = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, width, height = cv2.boundingRect(contour)
        diameter = max(width, height)
        if not 0.20 * cell <= diameter <= 0.82 * cell:
            continue
        if min(width, height) < 0.55 * diameter or area < 0.020 * cell * cell:
            continue
        (centre_x, centre_y), radius = cv2.minEnclosingCircle(contour)
        if area / max(math.pi * radius * radius, 1.0) < 0.38:
            continue
        cv2.circle(
            guard,
            (round(centre_x), round(centre_y)),
            round(radius + 0.10 * cell),
            255,
            -1,
        )
    return guard


def _line_contrast(
    gray: np.ndarray,
    line: tuple[int, int, int, int],
    horizontal: bool,
    otsu: float,
    cell: float,
) -> float:
    x1, y1, x2, y2 = line
    line_mask = np.zeros_like(gray)
    cv2.line(line_mask, (x1, y1), (x2, y2), 255, max(3, round(0.05 * cell)))
    pixels = gray[line_mask > 0]
    if not pixels.size:
        return 0.0
    line_value = float(np.percentile(pixels, 25))
    padding = max(8, round(0.14 * cell))
    crop = gray[
        max(0, min(y1, y2) - padding):min(gray.shape[0], max(y1, y2) + padding + 1),
        max(0, min(x1, x2) - padding):min(gray.shape[1], max(x1, x2) + padding + 1),
    ]
    contrast = float(np.percentile(crop, 75)) - line_value
    value_limit = min(148.0, otsu + 10.0)
    strict_limit = min(108.0, 0.76 * otsu)
    side_values = []
    for offset in (-max(8, round(0.12 * cell)), max(8, round(0.12 * cell))):
        side = np.zeros_like(gray)
        shift_x, shift_y = (0, offset) if horizontal else (offset, 0)
        cv2.line(
            side,
            (x1 + shift_x, y1 + shift_y),
            (x2 + shift_x, y2 + shift_y),
            255,
            max(3, round(0.05 * cell)),
        )
        values = gray[side > 0]
        side_values.append(float(np.percentile(values, 60)) if values.size else 255.0)
    if min(side_values) < strict_limit + 12.0:
        return 0.0
    if line_value >= value_limit or (
        line_value >= strict_limit and contrast < 0.23 * otsu
    ):
        return 0.0
    darkness = (value_limit - line_value) / max(30.0, 0.40 * otsu)
    contrast_score = contrast / max(35.0, 0.43 * otsu)
    return float(np.clip(max(darkness, contrast_score), 0.0, 1.0))


def _add_thin_lines(
    edges: np.ndarray,
    gray: np.ndarray,
    horizontal_scores: np.ndarray,
    vertical_scores: np.ndarray,
    rows: int,
    columns: int,
    otsu: float,
) -> None:
    cell_x, cell_y = gray.shape[1] / columns, gray.shape[0] / rows
    cell = min(cell_x, cell_y)
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=max(18, round(0.25 * cell)),
        minLineLength=max(32, round(0.42 * cell)),
        maxLineGap=max(7, round(0.10 * cell)),
    )
    if lines is None:
        return
    h_support = np.zeros_like(horizontal_scores)
    v_support = np.zeros_like(vertical_scores)
    for x1, y1, x2, y2 in lines[:, 0]:
        dx, dy = float(x2 - x1), float(y2 - y1)
        horizontal = abs(dx) >= 0.42 * cell_x and abs(dy) <= max(5.0, 0.12 * abs(dx))
        vertical = abs(dy) >= 0.42 * cell_y and abs(dx) <= max(5.0, 0.12 * abs(dy))
        if not horizontal and not vertical:
            continue
        confidence = _line_contrast(
            gray, (int(x1), int(y1), int(x2), int(y2)), horizontal, otsu, cell
        )
        if confidence < 0.55:
            continue
        if horizontal:
            axis, step, along = 0.5 * (y1 + y2), cell_y, cell_x
            line_count, segment_count = rows, columns
            start, end, support = *sorted((x1, x2)), h_support
        else:
            axis, step, along = 0.5 * (x1 + x2), cell_x, cell_y
            line_count, segment_count = columns, rows
            start, end, support = *sorted((y1, y2)), v_support
        grid_line = round(axis / step)
        if abs(axis - grid_line * step) > 0.18 * step or not 0 <= grid_line <= line_count:
            continue
        for segment in range(segment_count):
            overlap = max(
                0.0,
                min(end, (segment + 1) * along) - max(start, segment * along),
            )
            if overlap < 0.38 * along:
                continue
            index = (grid_line, segment) if horizontal else (segment, grid_line)
            support[index] += overlap * confidence / (0.55 * along)
    np.maximum(horizontal_scores, np.clip(h_support, 0, 1), out=horizontal_scores)
    np.maximum(vertical_scores, np.clip(v_support, 0, 1), out=vertical_scores)


def _add_endpoint_walls(
    edges: np.ndarray,
    gray: np.ndarray,
    cyan: np.ndarray,
    scores: np.ndarray,
    rows: int,
    columns: int,
    otsu: float,
    vertical: bool,
) -> None:
    cell_x, cell_y = gray.shape[1] / columns, gray.shape[0] / rows
    cell = min(cell_x, cell_y)
    radius = max(12, round(0.28 * cell))

    def cyan_pixels(x: int, y: int) -> int:
        return int(
            np.count_nonzero(
                cyan[
                    max(0, y - radius):min(gray.shape[0], y + radius + 1),
                    max(0, x - radius):min(gray.shape[1], x + radius + 1),
                ]
            )
        )

    line_count = columns if vertical else rows
    segment_count = rows if vertical else columns
    for grid_line in range(1, line_count):
        axis = round(grid_line * (cell_x if vertical else cell_y))
        for segment in range(segment_count):
            index = (segment, grid_line) if vertical else (grid_line, segment)
            if scores[index] >= WALL_SCORE:
                continue
            if vertical:
                x0, x1 = max(0, round(axis - 0.24 * cell_x)), min(
                    gray.shape[1], round(axis + 0.24 * cell_x) + 1
                )
                y0, y1 = round(segment * cell_y), round((segment + 1) * cell_y)
                endpoints = ((axis, y0), (axis, y1))
            else:
                x0, x1 = round(segment * cell_x), round((segment + 1) * cell_x)
                y0, y1 = max(0, round(axis - 0.24 * cell_y)), min(
                    gray.shape[0], round(axis + 0.24 * cell_y) + 1
                )
                endpoints = ((x0, axis), (x1, axis))
            if min(cyan_pixels(*point) for point in endpoints) < 0.009 * cell * cell:
                continue
            lines = cv2.HoughLinesP(
                edges[y0:y1, x0:x1],
                1,
                np.pi / 180,
                threshold=max(10, round(0.12 * cell)),
                minLineLength=max(24, round(0.30 * (cell_y if vertical else cell_x))),
                maxLineGap=max(10, round(0.18 * (cell_y if vertical else cell_x))),
            )
            for x_start, y_start, x_end, y_end in (
                [] if lines is None else lines[:, 0]
            ):
                dx, dy = float(x_end - x_start), float(y_end - y_start)
                length = abs(dy) if vertical else abs(dx)
                cross = abs(dx) if vertical else abs(dy)
                along = cell_y if vertical else cell_x
                if length < 0.50 * along or cross > max(6.0, 0.12 * length):
                    continue
                line = (
                    int(x_start + x0),
                    int(y_start + y0),
                    int(x_end + x0),
                    int(y_end + y0),
                )
                measured_axis = 0.5 * (
                    (line[0] + line[2]) if vertical else (line[1] + line[3])
                )
                if abs(measured_axis - axis) > 0.18 * (cell_x if vertical else cell_y):
                    continue
                if _line_contrast(gray, line, not vertical, otsu, cell) >= 0.55:
                    scores[index] = 1.0
                    break


def _promote_parallel(scores: np.ndarray, candidates: np.ndarray, axis: int) -> None:
    strong = scores >= 0.85
    neighbour = np.zeros_like(strong)
    if axis == 1:
        neighbour[:, 1:] |= strong[:, :-1]
        neighbour[:, :-1] |= strong[:, 1:]
    else:
        neighbour[1:] |= strong[:-1]
        neighbour[:-1] |= strong[1:]
    scores[(candidates >= 0.65) & ~strong & neighbour] = 1.0


def _promote_relaxed_vertical_corners(
    vertical: np.ndarray,
    relaxed: np.ndarray,
    horizontal: np.ndarray,
) -> None:
    rows, line_count = vertical.shape
    columns = line_count - 1
    for row in range(rows):
        for column in range(line_count):
            if vertical[row, column] >= 0.85 or relaxed[row, column] < 0.85:
                continue
            top = (
                (column > 0 and horizontal[row, column - 1] >= 0.85)
                or (column < columns and horizontal[row, column] >= 0.85)
            )
            bottom = (
                (column > 0 and horizontal[row + 1, column - 1] >= 0.85)
                or (column < columns and horizontal[row + 1, column] >= 0.85)
            )
            if top and bottom:
                vertical[row, column] = 1.0


def detect_grid_walls(
    image: np.ndarray,
    rows: int = 9,
    columns: int = 9,
    ignored_circles: tuple[tuple[float, float, float], ...] = (),
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    otsu, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cyan = _cyan_mask(image)
    cell_x, cell_y = image.shape[1] / columns, image.shape[0] / rows
    cell = min(cell_x, cell_y)
    guard = cv2.bitwise_or(
        _compact_object_guard(blurred, cell, float(otsu)),
        cv2.dilate(
        cyan, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        ),
    )
    for x, y, radius in ignored_circles:
        cv2.circle(guard, (round(x), round(y)), round(radius), 255, -1)
    horizontal_mask = _directional_mask(
        blurred, guard, min(108, round(0.76 * otsu)), False, cell_x
    )
    vertical_mask = _directional_mask(
        blurred, guard, min(108, round(0.76 * otsu)), True, cell_y
    )
    relaxed_vertical = _directional_mask(
        blurred, guard, min(160, round(1.08 * otsu)), True, cell_y
    )
    horizontal = _score_grid(horizontal_mask, False, rows, columns)
    vertical = _score_grid(vertical_mask, True, rows, columns)
    relaxed_scores = _score_grid(relaxed_vertical, True, rows, columns)
    local_horizontal = _score_grid(
        _local_directional_mask(blurred, guard, False, cell_x),
        False,
        rows,
        columns,
    )
    local_vertical = _score_grid(
        _local_directional_mask(blurred, guard, True, cell_y),
        True,
        rows,
        columns,
    )
    np.maximum(horizontal, local_horizontal, out=horizontal)
    np.maximum(vertical, local_vertical, out=vertical)

    enhanced = cv2.createCLAHE(clipLimit=1.8, tileGridSize=(8, 8)).apply(gray)
    edges = cv2.Canny(
        cv2.GaussianBlur(enhanced, (3, 3), 0),
        max(35, round(0.36 * otsu)),
        max(100, round(1.02 * otsu)),
        L2gradient=True,
    )
    edges[cv2.dilate(guard, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))) > 0] = 0
    _add_thin_lines(edges, gray, horizontal, vertical, rows, columns, float(otsu))
    _add_endpoint_walls(edges, gray, cyan, horizontal, rows, columns, float(otsu), False)
    _add_endpoint_walls(edges, gray, cyan, vertical, rows, columns, float(otsu), True)
    _promote_parallel(horizontal, horizontal, 1)
    _promote_parallel(vertical, vertical, 0)
    _promote_parallel(vertical, relaxed_scores, 0)
    _promote_relaxed_vertical_corners(vertical, relaxed_scores, horizontal)

    # A 0.70 component has a continuous wall body over almost half a cell.
    # Compact dark objects have already been removed, so this is conservative
    # for walls while rejecting isolated floor marks and connector pieces.
    horizontal[horizontal >= 0.65] = 1.0
    vertical[vertical >= 0.65] = 1.0

    horizontal_walls = horizontal >= WALL_SCORE
    vertical_walls = vertical >= WALL_SCORE
    horizontal_walls[0, :] = horizontal_walls[-1, :] = True
    vertical_walls[:, 0] = vertical_walls[:, -1] = True
    return horizontal_walls, vertical_walls, horizontal, vertical
