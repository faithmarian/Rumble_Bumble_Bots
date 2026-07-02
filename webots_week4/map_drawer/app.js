(() => {
  "use strict";

  const CELL_M = 0.180;
  const WALL_HEIGHT_M = 0.150;
  const CYLINDER_RADIUS_M = 0.050;
  const BOARD_MARGIN_M = 0.105;

  const canvas = document.getElementById("mazeCanvas");
  const ctx = canvas.getContext("2d");
  const statusEl = document.getElementById("status");
  const wallCountEl = document.getElementById("wallCount");
  const cylinderCountEl = document.getElementById("cylinderCount");
  const dotCountEl = document.getElementById("dotCount");
  const mapNameEl = document.getElementById("mapName");
  const gridRowsEl = document.getElementById("gridRows");
  const gridColsEl = document.getElementById("gridCols");
  const applyGridBtn = document.getElementById("applyGridBtn");
  const wallThicknessEl = document.getElementById("wallThickness");
  const abstractTextEl = document.getElementById("abstractText");
  const wbtTextEl = document.getElementById("wbtText");
  const protoTextEl = document.getElementById("protoText");

  let gridRows = 9;
  let gridCols = 9;
  let mode = "draw";
  let drag = null;
  let clickTimer = null;
  let suppressClick = false;

  const state = {
    walls: [],
    cylinders: [],
    dots: [],
    start: { row: 2, col: 7 },
    goal: { row: 7, col: 5 },
    history: [],
  };

  const board = {
    x: 72,
    y: 72,
    width: 756,
    height: 756,
    maxSize: 756,
    step: 94.5,
    dotRadius: 4.5,
  };

  function clampInt(value, min, max, fallback) {
    const parsed = Number.parseInt(value, 10);
    if (!Number.isFinite(parsed)) return fallback;
    return Math.max(min, Math.min(max, parsed));
  }

  function defaultStart() {
    return { row: 0, col: 0 };
  }

  function defaultGoal() {
    return { row: gridRows - 1, col: gridCols - 1 };
  }

  function clampCell(cell) {
    return {
      row: Math.max(0, Math.min(gridRows - 1, Number(cell?.row || 0))),
      col: Math.max(0, Math.min(gridCols - 1, Number(cell?.col || 0))),
    };
  }

  function layoutBoard() {
    const rowSpan = Math.max(1, gridRows);
    const colSpan = Math.max(1, gridCols);
    board.step = board.maxSize / Math.max(rowSpan, colSpan);
    board.width = colSpan * board.step;
    board.height = rowSpan * board.step;
    board.x = (canvas.width - board.width) / 2;
    board.y = (canvas.height - board.height) / 2;
    board.dotRadius = Math.max(3.5, Math.min(5.5, board.step * 0.045));
  }

  function wallThicknessM() {
    return Math.max(0.006, Math.min(0.040, Number(wallThicknessEl.value || 12) / 1000));
  }

  function setStatus(text) {
    statusEl.textContent = text;
  }

  function cloneData() {
    return JSON.stringify({
      gridRows,
      gridCols,
      walls: state.walls,
      cylinders: state.cylinders,
      dots: state.dots,
      start: state.start,
      goal: state.goal,
    });
  }

  function pushHistory() {
    state.history.push(cloneData());
    if (state.history.length > 80) state.history.shift();
  }

  function restore(snapshot) {
    const data = JSON.parse(snapshot);
    gridRows = clampInt(data.gridRows, 2, 30, gridRows);
    gridCols = clampInt(data.gridCols, 2, 30, gridCols);
    gridRowsEl.value = String(gridRows);
    gridColsEl.value = String(gridCols);
    state.walls = data.walls || [];
    state.cylinders = data.cylinders || [];
    state.dots = data.dots || [];
    state.start = clampCell(data.start || defaultStart());
    state.goal = clampCell(data.goal || defaultGoal());
    update();
  }

  function gridToCanvas(row, col) {
    return {
      x: board.x + col * board.step,
      y: board.y + row * board.step,
    };
  }

  function cellToCanvas(row, col) {
    return {
      x: board.x + (col + 0.5) * board.step,
      y: board.y + (row + 0.5) * board.step,
    };
  }

  function gridToWorld(row, col) {
    return {
      x: (col - gridCols / 2) * CELL_M,
      y: (gridRows / 2 - row) * CELL_M,
    };
  }

  function cellToWorld(row, col) {
    return {
      x: (col + 0.5 - gridCols / 2) * CELL_M,
      y: (gridRows / 2 - row - 0.5) * CELL_M,
    };
  }

  function canvasToPoint(event) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    return {
      x: (event.clientX - rect.left) * scaleX,
      y: (event.clientY - rect.top) * scaleY,
    };
  }

  function nearestGrid(point, maxDistance = 32) {
    let best = null;
    let bestDistance = Infinity;
    for (let row = 0; row <= gridRows; row += 1) {
      for (let col = 0; col <= gridCols; col += 1) {
        const p = gridToCanvas(row, col);
        const distance = Math.hypot(point.x - p.x, point.y - p.y);
        if (distance < bestDistance) {
          bestDistance = distance;
          best = { row, col, distance };
        }
      }
    }
    return bestDistance <= maxDistance ? best : null;
  }

  function nearestCell(point, maxDistance = 42) {
    let best = null;
    let bestDistance = Infinity;
    for (let row = 0; row < gridRows; row += 1) {
      for (let col = 0; col < gridCols; col += 1) {
        const p = cellToCanvas(row, col);
        const distance = Math.hypot(point.x - p.x, point.y - p.y);
        if (distance < bestDistance) {
          bestDistance = distance;
          best = { row, col, distance };
        }
      }
    }
    return bestDistance <= maxDistance ? best : null;
  }

  function normalizeWall(a, b) {
    if (!a || !b) return null;
    const dRow = b.row - a.row;
    const dCol = b.col - a.col;
    if (Math.abs(dRow) >= Math.abs(dCol)) {
      b = { row: b.row, col: a.col };
    } else {
      b = { row: a.row, col: b.col };
    }
    if (a.row === b.row && a.col === b.col) return null;
    if (a.row === b.row) {
      const col1 = Math.min(a.col, b.col);
      const col2 = Math.max(a.col, b.col);
      return { o: "H", row: a.row, col1, col2 };
    }
    const row1 = Math.min(a.row, b.row);
    const row2 = Math.max(a.row, b.row);
    return { o: "V", col: a.col, row1, row2 };
  }

  function wallKey(wall) {
    return wall.o === "H"
      ? `H:${wall.row}:${wall.col1}:${wall.col2}`
      : `V:${wall.col}:${wall.row1}:${wall.row2}`;
  }

  function compactWalls(walls) {
    const byLine = new Map();
    for (const wall of walls) {
      if (wall.o === "H") {
        const key = `H:${wall.row}`;
        const list = byLine.get(key) || [];
        list.push([wall.col1, wall.col2]);
        byLine.set(key, list);
      } else {
        const key = `V:${wall.col}`;
        const list = byLine.get(key) || [];
        list.push([wall.row1, wall.row2]);
        byLine.set(key, list);
      }
    }
    const result = [];
    for (const [key, ranges] of byLine.entries()) {
      ranges.sort((a, b) => a[0] - b[0]);
      const merged = [];
      for (const range of ranges) {
        const last = merged[merged.length - 1];
        if (!last || range[0] > last[1]) {
          merged.push([...range]);
        } else {
          last[1] = Math.max(last[1], range[1]);
        }
      }
      const [o, indexText] = key.split(":");
      const index = Number(indexText);
      for (const [a, b] of merged) {
        if (b <= a) continue;
        if (o === "H") result.push({ o: "H", row: index, col1: a, col2: b });
        else result.push({ o: "V", col: index, row1: a, row2: b });
      }
    }
    return result.sort((a, b) => wallKey(a).localeCompare(wallKey(b)));
  }

  function addWall(wall) {
    if (!wall) return;
    pushHistory();
    state.walls.push(wall);
    state.walls = compactWalls(state.walls);
    setStatus(`Wall ${wallKey(wall)}`);
    update();
  }

  function sameCell(a, b) {
    return a && b && a.row === b.row && a.col === b.col;
  }

  function toggleCylinder(cell) {
    pushHistory();
    const index = state.cylinders.findIndex((item) => sameCell(item, cell));
    if (index >= 0) {
      state.cylinders.splice(index, 1);
      setStatus(`Cylinder removed (${cell.row}, ${cell.col})`);
    } else {
      state.cylinders.push({ row: cell.row, col: cell.col });
      setStatus(`Cylinder added (${cell.row}, ${cell.col})`);
    }
    update();
  }

  function toggleDot(cell) {
    pushHistory();
    const index = state.dots.findIndex((item) => sameCell(item, cell));
    if (index >= 0) {
      state.dots.splice(index, 1);
      setStatus(`Black dot removed (${cell.row}, ${cell.col})`);
    } else {
      state.dots.push({ row: cell.row, col: cell.col });
      setStatus(`Black dot added (${cell.row}, ${cell.col})`);
    }
    update();
  }

  function eraseAt(cell) {
    pushHistory();
    const before = cloneData();
    state.cylinders = state.cylinders.filter((item) => !sameCell(item, cell));
    state.dots = state.dots.filter((item) => !sameCell(item, cell));
    state.walls = state.walls.filter((wall) => {
      if (wall.o === "H") {
        return !(wall.row === cell.row && cell.col >= wall.col1 && cell.col <= wall.col2);
      }
      return !(wall.col === cell.col && cell.row >= wall.row1 && cell.row <= wall.row2);
    });
    if (cloneData() === before) {
      state.history.pop();
      setStatus("Nothing nearby");
    } else {
      setStatus(`Erased near (${cell.row}, ${cell.col})`);
    }
    update();
  }

  function setPoint(kind, cell) {
    pushHistory();
    state[kind] = { row: cell.row, col: cell.col };
    setStatus(`${kind === "start" ? "Start" : "Goal"} cell = (${cell.row}, ${cell.col})`);
    update();
  }

  function applyGridSize() {
    const nextRows = clampInt(gridRowsEl.value, 2, 30, gridRows);
    const nextCols = clampInt(gridColsEl.value, 2, 30, gridCols);
    gridRowsEl.value = String(nextRows);
    gridColsEl.value = String(nextCols);

    if (nextRows === gridRows && nextCols === gridCols) {
      setStatus(`Grid already ${gridRows}x${gridCols}`);
      return;
    }

    pushHistory();
    gridRows = nextRows;
    gridCols = nextCols;
    state.walls = [];
    state.cylinders = [];
    state.dots = [];
    state.start = defaultStart();
    state.goal = defaultGoal();
    drag = null;
    suppressClick = false;
    setStatus(`Grid set to ${gridRows} rows x ${gridCols} columns`);
    update();
  }

  function drawBoard() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#9aa3ab";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const inset = Math.max(42, board.step * 0.55);
    const left = board.x - inset;
    const top = board.y - inset;
    const right = board.x + board.width + inset;
    const bottom = board.y + board.height + inset;
    const cut = Math.min(150, Math.max(36, Math.min(right - left, bottom - top) * 0.18));
    const polygon = [
      [left + cut, top],
      [right - cut, top],
      [right, top + cut],
      [right, bottom - cut],
      [right - cut, bottom],
      [left + cut, bottom],
      [left, bottom - cut],
      [left, top + cut],
    ];
    ctx.beginPath();
    polygon.forEach(([x, y], index) => {
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.fillStyle = "#050505";
    ctx.fill();
    ctx.strokeStyle = "#45484c";
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.fillStyle = "#777d82";
    for (let row = 0; row <= gridRows; row += 1) {
      for (let col = 0; col <= gridCols; col += 1) {
        const p = gridToCanvas(row, col);
        ctx.beginPath();
        ctx.arc(p.x, p.y, board.dotRadius, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }

  function drawWall(wall, preview = false) {
    const thicknessPx = preview ? 10 : 14;
    ctx.strokeStyle = preview ? "rgba(100, 180, 255, 0.78)" : "#f3f0e6";
    ctx.lineWidth = thicknessPx;
    ctx.lineCap = "square";
    ctx.lineJoin = "miter";
    ctx.beginPath();
    if (wall.o === "H") {
      const a = gridToCanvas(wall.row, wall.col1);
      const b = gridToCanvas(wall.row, wall.col2);
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
    } else {
      const a = gridToCanvas(wall.row1, wall.col);
      const b = gridToCanvas(wall.row2, wall.col);
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
    }
    ctx.stroke();
  }

  function drawCylinder(item) {
    const p = gridToCanvas(item.row, item.col);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 25, 0, Math.PI * 2);
    ctx.fillStyle = "#050505";
    ctx.fill();
    ctx.strokeStyle = "#f3f0e6";
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  function drawDot(item) {
    const p = gridToCanvas(item.row, item.col);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
    ctx.fillStyle = "#000";
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  function drawMarker(cell, color, label) {
    const p = cellToCanvas(cell.row, cell.col);
    ctx.fillStyle = color;
    ctx.fillRect(p.x - 27, p.y - 27, 54, 54);
    ctx.fillStyle = "#fff";
    ctx.font = "bold 15px Segoe UI, Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, p.x, p.y);
  }

  function draw() {
    drawBoard();
    state.walls.forEach((wall) => drawWall(wall));
    if (drag && drag.preview) drawWall(drag.preview, true);
    state.cylinders.forEach(drawCylinder);
    state.dots.forEach(drawDot);
    drawMarker(state.start, "#24c83a", "S");
    drawMarker(state.goal, "#e73737", "G");
  }

  function protoHeader(protoName) {
    return [
      "#VRML_SIM R2025a utf8",
      "",
      `PROTO ${protoName} [`,
      `  field SFString name "${protoName}"`,
      "] {",
      "  Solid {",
      "    name IS name",
      "    children [",
    ].join("\n");
  }

  function boxSolid(name, x, y, z, sx, sy, sz, color, material = "") {
    const contact = material ? `\n        contactMaterial "${material}"` : "";
    return [
      "      Solid {",
      `        translation ${fmt(x)} ${fmt(y)} ${fmt(z)}${contact}`,
      "        children [ Shape {",
      `          appearance PBRAppearance { baseColor ${color} roughness 0.76 metalness 0 }`,
      `          geometry Box { size ${fmt(sx)} ${fmt(sy)} ${fmt(sz)} }`,
      "        } ]",
      `        name "${name}"`,
      `        boundingObject Box { size ${fmt(sx)} ${fmt(sy)} ${fmt(sz)} }`,
      "      }",
    ].join("\n");
  }

  function visualBox(name, x, y, z, sx, sy, sz, color) {
    return [
      "      Pose {",
      `        translation ${fmt(x)} ${fmt(y)} ${fmt(z)}`,
      "        children [ Shape {",
      `          appearance PBRAppearance { baseColor ${color} roughness 0.76 metalness 0 }`,
      `          geometry Box { size ${fmt(sx)} ${fmt(sy)} ${fmt(sz)} }`,
      "        } ]",
      `        # ${name}`,
      "      }",
    ].join("\n");
  }

  function cylinderSolid(name, x, y, z, radius, height, color) {
    return [
      "      Solid {",
      `        translation ${fmt(x)} ${fmt(y)} ${fmt(z)}`,
      "        children [ Shape {",
      `          appearance PBRAppearance { baseColor ${color} roughness 0.78 metalness 0 }`,
      `          geometry Cylinder { height ${fmt(height)} radius ${fmt(radius)} subdivision 36 }`,
      "        } ]",
      `        name "${name}"`,
      `        boundingObject Cylinder { height ${fmt(height)} radius ${fmt(radius)} subdivision 36 }`,
      "      }",
    ].join("\n");
  }

  function fmt(number) {
    return Number(number).toFixed(5).replace(/0+$/, "0").replace(/\.0$/, ".0");
  }

  function safeName(raw) {
    const cleaned = String(raw || "drawn_micromouse_map")
      .trim()
      .replace(/[^a-zA-Z0-9_ -]/g, "_")
      .replace(/\s+/g, "_");
    return cleaned || "drawn_micromouse_map";
  }

  function wallToWorld(wall) {
    const thickness = wallThicknessM();
    if (wall.o === "H") {
      const a = gridToWorld(wall.row, wall.col1);
      const b = gridToWorld(wall.row, wall.col2);
      return {
        x: (a.x + b.x) / 2,
        y: a.y,
        sx: Math.abs(b.x - a.x) + thickness,
        sy: thickness,
      };
    }
    const a = gridToWorld(wall.row1, wall.col);
    const b = gridToWorld(wall.row2, wall.col);
    return {
      x: a.x,
      y: (a.y + b.y) / 2,
      sx: thickness,
      sy: Math.abs(b.y - a.y) + thickness,
    };
  }

  function generateProto() {
    const protoName = pascalName(safeName(mapNameEl.value));
    const lines = [protoHeader(protoName)];
    const boardSizeX = gridCols * CELL_M + BOARD_MARGIN_M * 2;
    const boardSizeY = gridRows * CELL_M + BOARD_MARGIN_M * 2;

    lines.push(boxSolid("black floor", 0, 0, -0.002, boardSizeX, boardSizeY, 0.004, "0.015 0.015 0.015", "maze_floor"));

    for (let row = 0; row <= gridRows; row += 1) {
      for (let col = 0; col <= gridCols; col += 1) {
        const p = gridToWorld(row, col);
        lines.push(cylinderSolid(`grid dot ${row} ${col}`, p.x, p.y, 0.004, 0.004, 0.001, "0.32 0.32 0.32"));
      }
    }

    state.walls.forEach((wall, index) => {
      const w = wallToWorld(wall);
      lines.push(boxSolid(`white wall ${String(index + 1).padStart(3, "0")}`, w.x, w.y, WALL_HEIGHT_M / 2, w.sx, w.sy, WALL_HEIGHT_M, "0.96 0.95 0.90"));
    });

    state.cylinders.forEach((item, index) => {
      const p = gridToWorld(item.row, item.col);
      lines.push(cylinderSolid(`cylindrical obstacle ${String(index + 1).padStart(3, "0")}`, p.x, p.y, WALL_HEIGHT_M / 2, CYLINDER_RADIUS_M, WALL_HEIGHT_M, "0.005 0.005 0.005"));
    });

    state.dots.forEach((item, index) => {
      const p = gridToWorld(item.row, item.col);
      lines.push(cylinderSolid(`black point ${String(index + 1).padStart(3, "0")}`, p.x, p.y, 0.007, 0.018, 0.006, "0.0 0.0 0.0"));
    });

    const s = cellToWorld(state.start.row, state.start.col);
    const g = cellToWorld(state.goal.row, state.goal.col);
    lines.push(visualBox("start marker", s.x, s.y, 0.003, 0.10, 0.10, 0.004, "0.05 0.75 0.12"));
    lines.push(visualBox("goal marker", g.x, g.y, 0.003, 0.10, 0.10, 0.004, "0.85 0.05 0.05"));
    lines.push("    ]", "  }", "}");
    return { name: protoName, content: lines.join("\n") + "\n" };
  }

  function pascalName(name) {
    const parts = name.split(/[_ -]+/).filter(Boolean);
    const joined = parts.map((part) => part.charAt(0).toUpperCase() + part.slice(1)).join("");
    return /^[A-Z]/.test(joined) ? joined : `Map${joined || "Generated"}`;
  }

  function generateWbt(protoName, protoFileName) {
    const s = cellToWorld(state.start.row, state.start.col);
    const config = `GRID ${gridRows} ${gridCols}; START ${state.start.row} ${state.start.col}; GOAL ${state.goal.row} ${state.goal.col}; HEADING E`;
    return [
      "#VRML_SIM R2025a utf8",
      "",
      `EXTERNPROTO "./${protoFileName}"`,
      'EXTERNPROTO "../../protos/MTRN3100KitMouse.proto"',
      "",
      "WorldInfo {",
      '  title "Drawn MTRN3100 Micromouse Map"',
      "  basicTimeStep 32",
      "  contactProperties [",
      "    ContactProperties { material1 \"drive_wheel\" material2 \"maze_floor\" coulombFriction [ 1.0 ] bounce 0 }",
      "    ContactProperties { material1 \"caster_ball\" material2 \"maze_floor\" coulombFriction [ 0.2 ] bounce 0 }",
      "    ContactProperties { material1 \"chassis\" material2 \"maze_floor\" coulombFriction [ 0.6 ] bounce 0 }",
      "  ]",
      "}",
      "Viewpoint {",
      "  orientation 0 1 0 0",
      "  position 0 0 2.35",
      "}",
      "Background { skyColor [ 0.55 0.58 0.62 ] }",
      "DirectionalLight { direction -0.4 -0.6 -1 intensity 1.8 }",
      `${protoName} { name "${safeName(mapNameEl.value)}" }`,
      "MTRN3100KitMouse {",
      `  translation ${fmt(s.x)} ${fmt(s.y)} 0`,
      "  rotation 0 0 1 0",
      '  name "MTRN3100 kit mouse drawn map"',
      '  controller "autonomous_maze_runner"',
      `  customData "${config}"`,
      "}",
      "",
    ].join("\n");
  }

  function generateAbstract() {
    const lines = [];
    lines.push(`# ${safeName(mapNameEl.value)}`);
    lines.push(`GRID ${gridRows} ${gridCols}`);
    lines.push(`GRID_POINTS ${gridRows + 1} ${gridCols + 1}`);
    lines.push("CELL_MM 180");
    lines.push(`WALL_THICKNESS_MM ${Number(wallThicknessEl.value || 12)}`);
    lines.push("WALL_HEIGHT_MM 150");
    lines.push("CYLINDER_DIAMETER_MM 100");
    lines.push(`START ${state.start.row} ${state.start.col}`);
    lines.push(`GOAL ${state.goal.row} ${state.goal.col}`);
    for (const wall of state.walls) {
      if (wall.o === "H") lines.push(`WALL H row=${wall.row} col=${wall.col1}..${wall.col2}`);
      else lines.push(`WALL V col=${wall.col} row=${wall.row1}..${wall.row2}`);
    }
    for (const item of state.cylinders) lines.push(`CYLINDER row=${item.row} col=${item.col}`);
    for (const item of state.dots) lines.push(`BLACK_DOT row=${item.row} col=${item.col}`);
    const compact = [
      ...state.walls.map((wall) => wall.o === "H" ? `H${wall.row},${wall.col1}-${wall.col2}` : `V${wall.col},${wall.row1}-${wall.row2}`),
      ...state.cylinders.map((item) => `C${item.row},${item.col}`),
      ...state.dots.map((item) => `P${item.row},${item.col}`),
    ].join(";");
    lines.push(`STRING ${compact}`);
    return lines.join("\n") + "\n";
  }

  function currentFiles() {
    const base = safeName(mapNameEl.value);
    const proto = generateProto();
    const protoFile = `${proto.name}.proto`;
    const wbtFile = `${base}.wbt`;
    return {
      [`${base}.txt`]: generateAbstract(),
      [`${base}.json`]: JSON.stringify({
        grid: { rows: gridRows, cols: gridCols },
        grid_points: { rows: gridRows + 1, cols: gridCols + 1 },
        cell_mm: 180,
        wall_thickness_mm: Number(wallThicknessEl.value || 12),
        wall_height_mm: 150,
        cylinder_diameter_mm: 100,
        start: state.start,
        goal: state.goal,
        walls: state.walls,
        cylinders: state.cylinders,
        dots: state.dots,
      }, null, 2),
      [protoFile]: proto.content,
      [wbtFile]: generateWbt(proto.name, protoFile),
    };
  }

  function updateOutputs() {
    const files = currentFiles();
    const base = safeName(mapNameEl.value);
    abstractTextEl.value = files[`${base}.txt`];
    const protoEntry = Object.entries(files).find(([name]) => name.endsWith(".proto"));
    protoTextEl.value = protoEntry ? protoEntry[1] : "";
    wbtTextEl.value = files[`${base}.wbt`];
  }

  function updateCounts() {
    wallCountEl.textContent = String(state.walls.length);
    cylinderCountEl.textContent = String(state.cylinders.length);
    dotCountEl.textContent = String(state.dots.length);
  }

  function update() {
    layoutBoard();
    updateCounts();
    updateOutputs();
    draw();
  }

  function downloadText(name, text) {
    const blob = new Blob([text], { type: "text/plain;charset=utf-8" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  }

  function downloadZipless() {
    const files = currentFiles();
    for (const [name, content] of Object.entries(files)) {
      downloadText(name, content);
    }
    setStatus("Downloaded generated files");
  }

  async function saveToGenerated() {
    const files = currentFiles();
    try {
      const response = await fetch("/save", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ files }),
      });
      const payload = await response.json();
      if (!payload.ok) throw new Error(payload.error || "Save failed");
      setStatus(`Saved: ${payload.written.join(", ")}`);
    } catch (error) {
      setStatus("Run server.js to save into generated; downloaded instead");
      downloadZipless();
    }
  }

  canvas.addEventListener("pointerdown", (event) => {
    const point = canvasToPoint(event);
    const cell = nearestGrid(point);
    if (!cell) return;
    if (mode === "draw") {
      drag = { start: cell, current: cell, preview: null };
      suppressClick = false;
      canvas.setPointerCapture(event.pointerId);
    }
  });

  canvas.addEventListener("pointermove", (event) => {
    if (!drag) return;
    const point = canvasToPoint(event);
    const cell = nearestGrid(point, 900);
    if (!cell) return;
    const preview = normalizeWall(drag.start, cell);
    drag.current = cell;
    drag.preview = preview;
    suppressClick = !!preview;
    draw();
  });

  canvas.addEventListener("pointerup", (event) => {
    if (!drag) return;
    const finalWall = drag.preview;
    drag = null;
    try {
      canvas.releasePointerCapture(event.pointerId);
    } catch {
      /* pointer may already be released */
    }
    if (finalWall) addWall(finalWall);
    draw();
  });

  canvas.addEventListener("click", (event) => {
    const point = canvasToPoint(event);
    const target = mode === "start" || mode === "goal" ? nearestCell(point) : nearestGrid(point);
    if (!target) return;
    if (suppressClick) {
      suppressClick = false;
      return;
    }
    if (event.detail === 1) {
      if (mode === "erase") eraseAt(target);
      else if (mode === "start") setPoint("start", target);
      else if (mode === "goal") setPoint("goal", target);
    } else if (event.detail === 2) {
      clearTimeout(clickTimer);
      clickTimer = setTimeout(() => {
        if (mode === "draw") toggleCylinder(target);
      }, 220);
    } else if (event.detail >= 3) {
      clearTimeout(clickTimer);
      if (mode === "draw") toggleDot(target);
    }
  });

  document.querySelectorAll(".tool").forEach((button) => {
    button.addEventListener("click", () => {
      mode = button.dataset.mode;
      document.querySelectorAll(".tool").forEach((item) => item.classList.toggle("active", item === button));
      setStatus(`Mode: ${button.textContent}`);
    });
  });

  document.querySelectorAll(".tab").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".tab").forEach((item) => item.classList.toggle("active", item === button));
      document.querySelectorAll(".output").forEach((item) => item.classList.toggle("active", item.id === button.dataset.tab));
    });
  });

  document.getElementById("undoBtn").addEventListener("click", () => {
    const snapshot = state.history.pop();
    if (!snapshot) {
      setStatus("Nothing to undo");
      return;
    }
    restore(snapshot);
    setStatus("Undo");
  });

  document.getElementById("clearBtn").addEventListener("click", () => {
    pushHistory();
    state.walls = [];
    state.cylinders = [];
    state.dots = [];
    setStatus("Cleared");
    update();
  });

  document.getElementById("exportBtn").addEventListener("click", () => {
    updateOutputs();
    setStatus("Generated text");
  });

  document.getElementById("copyBtn").addEventListener("click", async () => {
    const active = document.querySelector(".output.active");
    await navigator.clipboard.writeText(active.value);
    setStatus("Copied");
  });

  document.getElementById("downloadBtn").addEventListener("click", downloadZipless);
  document.getElementById("saveBtn").addEventListener("click", saveToGenerated);
  applyGridBtn.addEventListener("click", applyGridSize);
  gridRowsEl.addEventListener("keydown", (event) => {
    if (event.key === "Enter") applyGridSize();
  });
  gridColsEl.addEventListener("keydown", (event) => {
    if (event.key === "Enter") applyGridSize();
  });
  mapNameEl.addEventListener("input", updateOutputs);
  wallThicknessEl.addEventListener("input", update);

  update();
})();
