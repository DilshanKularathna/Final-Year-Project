# MoD Discovery — Codebase Analysis

> Generated during Phase A of the Map of Dynamics implementation.

## Simulator Overview

| Property | Value |
|---|---|
| **Simulator** | Custom Python / Pygame grid simulation |
| **Language** | Python 3 |
| **Entry point** | `Sim/main.py` |
| **FPS / tick rate** | 60 FPS (fixed via `pygame.time.Clock`) |
| **Window** | 1280 × 720 px |
| **Grid** | 32 columns × 18 rows, 40 px/cell |
| **Units** | Pixels (no metric scale existed before MoD) |
| **Robots** | 4 × Smorphi-style holonomic (`SmorphiRobot` in `robot.py`) |
| **Humans** | 2 × scripted random-walk workers (`HumanWorker` in `human.py`) |

## Coordinate Frame

- **Pixel space**: `(x, y)` — `x` grows right, `y` grows down. Origin at top-left of window.
- **Grid space**: `(col, row)` — `col` grows right, `row` grows down.
- Conversion: `pixel_to_grid(px, py) → (col, row)` and `cell_center(col, row) → (px, py)` in `environment.py`.
- **MoD grid indices**: `[row, col]` (NumPy convention) — matches `warehouse.grid[row][col]`.

## Static Map

- **Location**: `Warehouse.grid` — a Python 2D list `grid[row][col]`.
- **Cell types**: `CELL_AISLE=0`, `CELL_SHELF=1`, `CELL_PICKUP=2`, `CELL_DROPOFF=3`.
- **Layout**: 4 horizontal shelf bands at rows (3–4), (7–8), (11–12), (15–16). Each band has 3-column blocks separated by 2-column gaps. Left column (1) is PICKUP, right column (30) is DROPOFF.
- **No file-based map**: built procedurally in `Warehouse.__init__()`. MoD wraps it via `StaticMap`.

## Robot Model

- **Class**: `SmorphiRobot` in `robot.py`.
- **Pose access**: `robot.x`, `robot.y` (floats, pixel coords). `robot.angle` (heading in radians).
- **Effective position**: `robot.position` property includes lane-keeping offset.
- **Grid cell**: `robot.grid_cell` property → `(col, row)`.
- **States**: IDLE, ALLOCATING, PICKING, MOVING, COMPLETED, YIELDING, REROUTING.
- **No lidar model**: proxemic checks use Euclidean distance only.

## Human Model

- **Class**: `HumanWorker` in `human.py`.
- **Pose access**: `human.x`, `human.y` (floats, pixel coords). `human.vx`, `human.vy` (velocity px/frame).
- **Movement**: A*-based random walk between walkable AISLE cells. Pauses at waypoints for ~120 frames.
- **Ground-truth accessible**: `human.position` → `(x, y)`.

## Coordinator

- **Class**: `Coordinator` in `coordinator.py`.
- **Frame counter**: `self.frame` — integer, incremented each tick. Used for task allocation timing only.
- **Update order**: humans → coordinator → robots (enforced in `main.py`).

## Render Loop

In `main.py`:
1. `warehouse.draw(screen)` — background, shelves, zones, grid lines
2. `_draw_proxemic_zones(screen, h)` per human — translucent circles
3. `h.draw(screen)` per human — body + intent vector
4. `robot.draw(screen)` per robot — body + path + state tag
5. `_draw_hud(screen, clock, robots, humans, frame)` — telemetry overlay

MoD overlay should be drawn **after** `warehouse.draw()` and **before** humans/robots.

## Hook Points for MoD

1. **Static map** → `warehouse.grid` (wrap with `StaticMap`)
2. **Robot poses** → `robot.x`, `robot.y`, `robot.angle` each tick
3. **Human ground-truth positions** → `human.x`, `human.y` each tick
4. **Render** → insert overlay draw call between `warehouse.draw()` and human/robot draws
5. **Simulation time** → derive from frame counter: `sim_time_s = frame / FPS`
6. **Event handling** → add keyboard cases in the existing event loop

## Files Modified by MoD

| File | Change |
|---|---|
| `Sim/config.py` | Add `MAP_OF_DYNAMICS` dict, `PIXELS_PER_METER`, `SIM_PHASE` |
| `Sim/main.py` | Add sim_time, MoD init/update/render, keyboard toggles |
| `Sim/mod/` (new package) | All MoD modules |
