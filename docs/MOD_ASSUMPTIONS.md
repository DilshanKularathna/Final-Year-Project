# Map of Dynamics (MoD) — Assumptions & Engineering Decisions

This document records all engineering assumptions, conventions, mathematical formulations, and default parameters adopted for the Map of Dynamics (MoD) Risk Overlay implementation in this repository.

---

## 1. Simulation & Environment Architecture

| Item | Assumption / Choice | Rationale & Code Reference |
|---|---|---|
| **Simulator Type** | Custom 2D Grid / Pygame Warehouse Simulator | Native simulation in `Sim/` at 60 FPS. |
| **Simulation Time** | `sim_time_s = frame / FPS` (continuous float seconds) | **Rule 6**: All MoD timestamps and decay calculations must use simulation time, never wall-clock time. Tracked in `main.py` and `run_mod_demo.py`. |
| **Grid Resolution** | 32 columns × 18 rows, cell size = 40×40 px | Exactly matches `environment.Warehouse` grid layout. |
| **Unit Scale** | `PIXELS_PER_METER = 80.0` px/m (1 cell = 0.5 m) | Allows metric specifications in config while keeping Pygame rendering in pixels. Centralised in `config.py` and `StaticMap.meters_to_pixels()`. |
| **Coordinate Conventions** | `(x, y)` in pixels, `x` right, `y` down. Grid indexing: `(row, col) = (int(y // 40), int(x // 40))` | Single source of truth in `StaticMap.world_to_cell(x, y)` and `StaticMap.cell_to_world(row, col)`. All MoD layers use shape `(rows, cols)` = `(18, 32)`. |
| **Walkable vs Obstacle Mask** | Static obstacles = shelves (`CELL_SHELF = 1`). Walkable human space = aisles (`CELL_AISLE = 0`). | Shelf cells are masked out of the risk layer (`risk[obstacle_mask] = 0.0`) so robots do not falsely treat walls/shelves as dynamic human hazards. |

---

## 2. Risk Equation & Layer Parameters

The core risk equation implemented in `Sim/mod/mod_core.py` is:

$$R_{ij}(H) = \alpha \cdot P_{ij}(H) + \beta \cdot \exp\left(-\lambda \cdot \Delta t_{ij}\right)$$

### Parameter Constraints & Defaults

| Symbol | Parameter Key | Default | Valid Range | Physical Interpretation |
|---|---|---|---|---|
| $\alpha$ | `MAP_OF_DYNAMICS["alpha"]` | `0.6` | $\alpha > 0$ | Weight of long-term habitual human presence. |
| $\beta$ | `MAP_OF_DYNAMICS["beta"]` | `0.3` | $\beta > 0$ | Weight of short-term recency (recent presence). |
| $\alpha + \beta$ | — | `0.9` | $\alpha + \beta < 1.0$ | Guarantees $R_{ij} < 1.0$ and reserves 0.1 headroom for future hazards (e.g. robot density, blind corners). Enforced at startup. |
| $T_{1/2}$ | `MAP_OF_DYNAMICS["half_life_s"]` | `20.0` s | $T_{1/2} > 0$ | Time for recency risk to halve after human leaves a cell. |
| $\lambda$ | `cfg.lambd` | $\frac{\ln(2)}{20.0} \approx 0.03466$ s$^{-1}$ | $\lambda > 0$ | Exponential decay rate derived from half-life. |
| $W$ | `MAP_OF_DYNAMICS["window_s"]` | `2.0` s | $W > 0$ | Duration of discrete observation window for Bayesian counting. |
| $a$ | `MAP_OF_DYNAMICS["prior_a"]` | `0.5` | $a > 0$ | Beta/Laplace prior pseudo-count for human presence. |
| $b$ | `MAP_OF_DYNAMICS["prior_b"]` | `9.5` | $b > 0$ | Beta/Laplace prior pseudo-count for human absence. |
| $P_0$ | $a / (a + b)$ | `0.05` | $(0, 1)$ | Prior probability for completely unobserved cells. Prevents false "safe" learning. |
| $r_h$ | `MAP_OF_DYNAMICS["human_radius_m"]` | `0.35` m (28 px) | $r_h > 0$ | Spatial disc stamped around detected human position. |
| $f_{\text{risk}}$ | `MAP_OF_DYNAMICS["risk_update_hz"]` | `5.0` Hz | $> 0$ | Frequency of risk layer matrix recomputation (every 0.2 sim-seconds). |
| $\tau$ | `MAP_OF_DYNAMICS["forgetting_tau_s"]` | `None` (off) | $\tau > 0$ or `None` | Optional exponential forgetting time constant $\exp(-W/\tau)$ applied to historic counts. |

---

## 3. Observation, Sensing & Multi-Robot Fusion

1. **Ray-Cast Visibility Masking (`VisibilityEstimator`):**
   - Rays cast from each robot's $(x, y)$ heading across FOV (default 360° omnidirectional, 180 rays, 5.0 m range).
   - Ray traversal uses Bresenham line algorithm on the static grid.
   - Rays terminate at the first obstacle (shelf) or max range.
   - **Crucial Rule:** Only cells along the line-of-sight *before* the obstacle are marked visible. Cells behind shelves are occluded. Unseen cells are **never** updated with negative observations.

2. **Multi-Robot OR-Fusion:**
   - All robots in the fleet integrate observations into the shared `MapOfDynamics` instance during each window.
   - `_win_observed` is the elementwise boolean OR of visible masks from all robots.
   - `_win_human` is the elementwise boolean OR of human detection masks from all robots.
   - **Preventing Double Counting:** If Robot 1 and Robot 2 simultaneously observe cell $(i, j)$ containing a human, `_win_observed[i, j]` and `_win_human[i, j]` are each set to `True` once. At window closure, counts increment by exactly `+1.0`.

3. **Never-Seen Cells and Numerical Stability:**
   - Cells where humans have never been detected have `last_seen[i, j] = NaN`.
   - Recency is explicitly evaluated using NumPy boolean masking:
     $$\text{recency}_{ij} = \begin{cases} 0.0 & \text{if } \text{isnan}(\text{last\_seen}_{ij}) \\ \exp\left(-\lambda \cdot \max(t - \text{last\_seen}_{ij}, \varepsilon)\right) & \text{otherwise} \end{cases}$$
   - No `NaN` or `Inf` can ever enter the risk computation.

---

## 4. Open Questions & Future Extensions

1. **Planner Integration Hook (`planner_hook.py`):**
   - Implemented behind `MAP_OF_DYNAMICS["planner_hook_enabled"] = False`.
   - Exposes `risk_at(x_px, y_px)` and `edge_cost(length, risk, k_risk = 2.0)`.
   - When enabled in future phases, the A* planner in `Warehouse.find_path()` can add risk penalties to edge traversals.
2. **Dynamic / Directional Flow:**
   - Detected velocity $(v_x, v_y)$ is captured in `HumanDetection` dataclass and ready for future CLiFF-map / STeF-map extensions.
3. **Time-of-Day Cyclic Shifts:**
   - The modular design allows maintaining separate $P_{ij}^{(k)}$ tables per operational shift or hourly bucket.
