# Human-Aware Multi-Robot Warehouse Simulation

A Pygame-based simulation of a **multi-agent warehouse system** where 4 holonomic robots (Smorphi-style) coordinate pick-and-drop tasks while safely navigating around 2 human workers using proxemic safety zones, prioritized MAPF (Multi-Agent Path Finding), and left-hand lane-keeping rules.

---

## Architecture Overview

```
main.py                    ← Entry point & game loop
  ├── config.py            ← Global configuration (single source of truth)
  ├── environment.py       ← Warehouse grid, layout & pathfinding (A*, ARA*, WHCA*)
  ├── human.py             ← Human worker agent (wandering + intent prediction)
  ├── robot.py             ← Smorphi robot agent (state machine + lane keeping)
  └── coordinator.py       ← Central brain (priority, proxemics, task allocation)
```

---

## Module Breakdown

---

### 1. `config.py` — Global Configuration (Single Source of Truth)

This is the **central parameter store**. Every tunable value lives here so all other modules import from one place. Changing a value here changes behaviour globally.

#### What it defines:

| Category | Key Parameters | Values |
|---|---|---|
| **Display** | Window size, FPS | 1280×720 px, 60 FPS |
| **Grid** | Cell size, grid dimensions | 40×40 px per cell → 32 columns × 18 rows |
| **Robots** | Count, radius, speeds | 4 robots, 14px radius, 2.0 px/frame nominal speed, 0.0 yield speed, 1.5 reroute speed |
| **Humans** | Count, radius, speed, intent | 2 humans, 10px radius, 1.2 px/frame, 100px intent vector, 120-frame wander pause |
| **Proxemics** | Safety radii, cone angle | Inner = 60px (hard stop), Outer = 130px (reroute eval), 30° half-angle intent cone |
| **Lane Keeping** | Lateral offset | 7px left of heading direction |
| **WHCA\*** | Reservation horizon, collision margin | 60 time-step window, 2px physical pushback buffer |

#### State Constants:

The module defines 7 robot states as string constants and maps each to a distinct colour:

| State | Colour | Meaning |
|---|---|---|
| `IDLE` | Green | Waiting for a task |
| `ALLOCATING` | Gold | Task just assigned, 1-second planning pause |
| `PICKING` | Amber-Gold | At pickup location, simulating item grab |
| `MOVING` | Blue | Following waypoint path |
| `COMPLETED` | Emerald Green | At dropoff location, simulating item release |
| `YIELDING` | Red | Full stop — safety yield |
| `REROUTING` | Purple | Taking alternate path at reduced speed |

---

### 2. `environment.py` — Warehouse Grid Layout & Navigation Graph

This module defines the **physical world** — the warehouse layout, obstacle placement, and all pathfinding algorithms.

#### Construction (`__init__`):

1. Creates an **18×32 2D array**, all cells initially set to `AISLE` (walkable)
2. Calls `_place_shelves()` — places **4 horizontal bands** of shelf blocks at rows (3–4), (7–8), (11–12), and (15–16). Each band contains blocks of 3 columns with 2-column gaps between them, leaving a 4-cell margin on left and right sides for robot circulation
3. Calls `_place_zones()` — **column 1** becomes `PICKUP` (green) and **column 30** becomes `DROPOFF` (amber) for rows 2 through 15
4. Pre-computes all walkable waypoints and lists of pickup/dropoff cell coordinates for task allocation

#### Cell Types:

| Constant | Value | Description |
|---|---|---|
| `CELL_AISLE` | 0 | Walkable corridor |
| `CELL_SHELF` | 1 | Static obstacle (rack) |
| `CELL_PICKUP` | 2 | Pick-up zone (left edge) |
| `CELL_DROPOFF` | 3 | Drop-off zone (right edge) |

#### Helper Functions:

- `is_walkable(col, row)` — checks if a grid cell is not a shelf
- `cell_center(col, row)` — returns the pixel centre (x, y) of a grid cell
- `pixel_to_grid(px, py)` — converts pixel coordinates to grid (col, row)
- `random_walkable_pixel()` — returns a random pixel position on a walkable cell
- `get_neighbors(col, row)` — yields walkable 4-connected neighbours (up, down, left, right)

#### Pathfinding Algorithms:

##### A\* (Standard Grid-Based)

Standard A\* search with **Manhattan distance heuristic** on a 4-connected grid. Accepts an optional `blocked_cells` set for dynamic obstacle avoidance (e.g., cells occupied by humans). Returns a list of pixel waypoints from start to goal, or an empty list if no path exists.

**How it works:**
1. Converts pixel start/goal to grid coordinates
2. Uses a priority queue ordered by `f(n) = g(n) + h(n)`
3. Expands neighbours, skipping blocked cells
4. Reconstructs the path by backtracking through `came_from` pointers
5. Converts grid cells back to pixel centres

##### ARA\* (Anytime Repairing A\*)

An anytime variant that trades optimality for speed using a **heuristic inflation factor** (`epsilon`).

**How it works:**
1. Starts with `epsilon = 2.5`, meaning `f(n) = g(n) + 2.5 · h(n)`
2. The inflated heuristic aggressively guides search toward the goal, finding a *suboptimal but fast* path first
3. If no path is found, reduces epsilon by 0.5 (toward 1.0) and re-searches, progressively improving path quality
4. At `epsilon = 1.0`, the search is equivalent to standard A\* (optimal)
5. If ARA\* fails entirely, falls back to standard A\*

**Why ARA\*?** In a real-time simulation at 60 FPS, getting *any valid path quickly* is more important than waiting for the mathematically optimal one. ARA\* provides this "good enough, fast" guarantee.

##### WHCA\* (Windowed Hierarchical Cooperative A\*)

Searches in **3D space-time** `(col, row, time)`. Each state includes a discrete timestep, enabling robots to plan trajectories that avoid collisions not just in space but also in time.

**How it works:**
1. State space: each node is `(col, row, t)` — a cell at a specific timestep
2. At each timestep `t`, a robot can either **move** to a 4-connected neighbour or **wait in place**
3. **Vertex collision check**: Is cell `(c, r)` already reserved by another robot at time `t`? If yes, skip
4. **Edge-swap collision check**: Are two robots swapping positions at time `t`? (Robot A moves to B's cell while B simultaneously moves to A's cell). If yes, skip
5. Moving costs 1.0; waiting costs 1.2 (slightly penalized to encourage movement over unnecessary idling)
6. Search is bounded by `max_t` (default 80 timesteps) to prevent infinite exploration

#### Rendering (`draw`):

- Fills the background with near-black
- Draws shelf cells with a **3D bevel effect** (inner lighter rectangle)
- Colours pickup zones green and dropoff zones amber
- Overlays subtle grid lines
- Renders "PICK" and "DROP" labels on the respective zones

---

### 3. `human.py` — Human Worker Agent

Simulates **unpredictable human pedestrians** that wander through warehouse aisles. Robots must proactively avoid these workers.

#### Movement Behaviour:

**Target Selection (`_pick_new_target`):**
- Picks a random walkable cell at least 3 cells away from current position
- Plans an A\* path to the chosen destination
- Tries up to 50 times to find a valid destination with a path of length > 1
- Falls back to a simple straight-line path if no A\* path is found

**Per-Frame Update (`update`):**
1. **If paused** (`pause_timer > 0`): stays still, velocity set to zero, counts down each frame. When timer hits 0, picks a new target
2. **If moving**: steers toward the next waypoint on the A\* path at `HUMAN_SPEED = 1.2 px/frame`
3. **Waypoint arrival**: When within `SPEED × 2` distance of the current waypoint, snaps to it and advances to the next waypoint in the path
4. **Final waypoint reached**: Pauses for `120 ± random(-30, 60)` frames before picking a new target (simulates the human doing work at a shelf)

#### Predictive Intent System:

This is the key feature that enables **proactive** rather than purely reactive robot avoidance:

- **`intent_endpoint()`**: Projects a **100px vector** in the direction of current velocity (or last known heading if stationary). This tells the coordinator *where the human is heading*
- **`position` property**: Returns current `(x, y)` tuple — used by the coordinator for distance calculations
- **`velocity` property**: Returns current `(vx, vy)` tuple — used for intent prediction

#### Rendering (`draw`):

1. **Intent vector**: A blue line from the human's body to the intent endpoint, with a small arrowhead at the tip
2. **Body circle**: Filled circle in cyan (H1) or mint (H2), with white outline
3. **Label**: "H1" or "H2" rendered above the body

---

### 4. `robot.py` — Smorphi-Style Holonomic Robot Agent

The robot's **internal state machine, movement logic, lane keeping, and localization**.

#### State Machine (7 States):

| State | Speed | Duration | What happens |
|---|---|---|---|
| `IDLE` | 0 | Until task assigned | Waiting for coordinator to assign a task |
| `ALLOCATING` | 0 | 60 frames (1 sec) | Task just assigned — 1-second planning pause |
| `MOVING` | 2.0 px/frame | Until destination | Following A\*/ARA\* waypoint path |
| `PICKING` | 0 | 60 frames (1 sec) | Arrived at pickup — simulating item grab |
| `YIELDING` | 0 | Until safe | Full stop — human or higher-priority robot nearby |
| `REROUTING` | 1.5 px/frame | Until clear | Taking alternate path at reduced speed |
| `COMPLETED` | 0 | 60 frames (1 sec) | Arrived at dropoff — simulating item release |

#### Complete Task Lifecycle:

```
IDLE
  │  Coordinator assigns pickup + dropoff paths
  ▼
ALLOCATING (1-second pause, gold colour)
  │  Timer expires
  ▼
MOVING (follows pickup path, phase = "PICKUP", blue colour)
  │  Arrives at pickup location
  ▼
PICKING (1-second pause, amber-gold colour)
  │  Timer expires, switches to dropoff path
  ▼
MOVING (follows dropoff path, phase = "DROPOFF", blue colour)
  │  Arrives at dropoff location
  ▼
COMPLETED (1-second pause, emerald green colour)
  │  Timer expires, clear_path() called
  ▼
IDLE (ready for next task)
```

At any point during `MOVING`, the coordinator may interrupt with `YIELDING` (human/robot too close) or `REROUTING` (alternate path needed). Once the threat clears, the robot resumes `MOVING`.

#### Left-Hand Lane Keeping:

When moving, the robot computes a **perpendicular vector pointing left** of its heading direction:

```
left_nx = -sin(angle)
left_ny =  cos(angle)
offset  = (left_nx × 7px, left_ny × 7px)
```

This shifts the robot's effective position **7 pixels to the left side of the aisle**, mimicking real-world traffic rules. Opposing robots traveling in opposite directions will naturally stay on their respective sides, preventing head-on collisions in narrow aisles.

#### Markov Localization:

`_update_markov_localization()` tracks the belief state `(belief_col, belief_row)` — currently a simplified version that snaps to the current grid cell. The `belief_variance` (initialized to 4.0) provides the framework for tracking position uncertainty, which could be expanded for more realistic sensor noise simulation.

#### Key Methods:

- `assign_task(pickup_path, dropoff_path, label)` — sets up the two-phase path and begins `ALLOCATING`
- `clear_path()` — resets all navigation state and returns to `IDLE`
- `set_state(new_state, dwell_frames)` — transitions state and sets speed accordingly
- `distance_to(px, py)` — Euclidean distance from robot's effective position to a point
- `remaining_waypoints()` — returns the un-traveled portion of the current path

#### Rendering (`draw`):

1. **Path preview**: Thin coloured line showing remaining waypoints (with lane offset applied)
2. **Body circle**: Filled circle coloured by current state, with white outline
3. **Heading tick**: White line from centre to the edge of the circle in the direction of travel
4. **ID label**: "R1"–"R4" in black text centred on the body
5. **State tag**: Current state name rendered below the body in the state's colour

---

### 5. `coordinator.py` — Central Decision-Making Brain (Prioritized MAPF Engine)

The **fleet orchestrator** that enforces safety rules, priority-based right-of-way, and task allocation every frame.

#### Per-Frame Update Loop (`update()`):

For each robot, the coordinator evaluates **4 rules in strict priority order**:

##### Rule 1 — Task Allocation (IDLE robots only)

Every 30 frames (~0.5 seconds), if a robot is `IDLE`:
1. Picks a **random pickup cell** and a **random dropoff cell** from the warehouse
2. Plans two ARA\* paths: `current position → pickup` and `pickup → dropoff`
3. If both paths are valid, assigns the task to the robot (triggering `ALLOCATING` state)

##### Rule 2 — Human Proxemic Safety (Highest Priority)

Checks distance from the robot to **every human worker**:
- If any human is within `PROXEMIC_INNER_RADIUS = 60px` → **immediate YIELD** (full stop)
- This is the **highest priority rule** — human safety overrides everything
- The robot remains yielded until no human is within the inner zone

##### Rule 3 — Inter-Robot Priority Avoidance

Robots have **fixed priority**: `R1 > R2 > R3 > R4` (lower ID = higher priority).

- If a **lower-priority robot** (higher ID) is within **36px** of a higher-priority robot, it yields
- The 36px threshold = `2 × ROBOT_RADIUS (14) + 8px margin`
- **Higher-priority robots never yield to lower-priority ones** — only the junior robot stops

##### Rule 4 — Safe Distance Resumption

If a robot is currently in `YIELDING` state but **no human and no higher-priority robot** is nearby anymore → resume `MOVING`

#### Important Design Decisions:

- Robots in `ALLOCATING` or `COMPLETED` states are **skipped entirely** — they're already paused for their 1-second dwell and shouldn't be interrupted by proxemic checks
- The `_log()` method prints structured telemetry to the console showing state transitions with reasons (this is what you see in the terminal output)

---

### 6. `main.py` — Simulation Entry Point & Game Loop

The **orchestrator module** that initializes Pygame, creates all agents, and runs the simulation loop.

#### Initialization Sequence:

1. Creates a **Pygame window** (1280×720) with the title "Human-Aware Multi-Robot Warehouse Simulation"
2. Builds the `Warehouse` (grid, shelves, zones, waypoints)
3. Spawns **2 `HumanWorker`** instances (H1 and H2) at random walkable positions
4. Spawns **4 `SmorphiRobot`** instances (R1–R4) in the **top area** (rows 1–2) on random walkable cells. Tries up to 100 times per robot to find a valid spawn position
5. Creates the `Coordinator` linking the warehouse, all robots, and all humans
6. Prints a startup banner to the console

#### Game Loop (60 FPS):

Each frame executes in this exact order:

```
1. EVENTS    →  Check for window close or ESC key press
2. UPDATE    →  Humans move first → Coordinator evaluates → Robots execute
3. RENDER    →  Warehouse → Proxemic zones → Humans → Robots → HUD
4. DISPLAY   →  Flip the frame buffer and tick the clock
```

**Update order matters**: Humans move first so the coordinator has their latest positions before making safety decisions. The coordinator then sets robot states (yielding, resuming, etc.). Finally, robots execute their movement based on the updated state.

#### Proxemic Zone Rendering (`_draw_proxemic_zones`):

Draws **two translucent circles** around each human using alpha-blended surfaces:
- **Outer circle** (130px radius): Translucent orange — the awareness/reroute evaluation zone
- **Inner circle** (60px radius): Translucent red — the hard-stop safety zone

#### HUD Overlay (`_draw_hud`):

Displays real-time telemetry at the top-left:
- **Line 1**: FPS, frame count, fleet size, MAPF strategy label
- **Per robot**: ID, current state (coloured), pixel position, current task label
- **Per human**: ID, pixel position, velocity vector

---

## Running the Simulation

```bash
cd Sim
python main.py
```

Press **ESC** or close the window to exit.
