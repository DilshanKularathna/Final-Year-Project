# Map of Dynamics (MoD) — Risk Overlay Guide

The Map of Dynamics (MoD) adds a human-aware spatial perception and risk mapping overlay to the multi-robot warehouse simulation.

---

## 1. Quick Start Commands

### Run the Interactive Simulation with MoD Overlay
```bash
cd Sim
python main.py
```
*(Ensure `MAP_OF_DYNAMICS["enabled"] = True` in `Sim/config.py`)*

### Interactive Keyboard Controls
When the simulation window is active:
| Key | Action |
|:---:|:---|
| **`1`** | Display **Combined Risk Layer** ($R_{ij} = \alpha P_{ij} + \beta \text{recency}_{ij}$) [Default] |
| **`2`** | Display **Long-Term Habit Probability Layer** ($P_{ij}$) |
| **`3`** | Display **Short-Term Recency Layer** ($\exp(-\lambda \Delta t_{ij})$) |
| **`4`** | Display **Sensor Observation Coverage Layer** |
| **`M`** | Toggle MoD Overlay **ON / OFF** |
| **`S`** | Export an immediate PNG snapshot to `outputs/mod_snapshots/` |
| **`ESC`** | Exit simulation |

### Run the Automated Unit & Acceptance Tests
```bash
cd Sim
python -m pytest tests/test_mod.py -v
```

### Run the Demonstration Experiment (Headless & Fast)
```bash
python experiments/run_mod_demo.py --sim-minutes 10.0
```

---

## 2. How the Risk Equation Maps to Code

The core risk model is defined by:

$$R_{ij}(H) = \alpha \cdot P_{ij}(H) + \beta \cdot \exp\left(-\lambda \cdot \Delta t_{ij}\right)$$

Below is the direct mapping from the mathematical formulation to the implementation in [`Sim/mod/mod_core.py`](file:///Sim/mod/mod_core.py):

### 1. Habitual Presence Probability $P_{ij}(H)$
- **Equation**:
  $$P_{ij} = \frac{\text{hum\_count}_{ij} + a}{\text{obs\_count}_{ij} + a + b}$$
  with Beta pseudo-counts $a = 0.5$, $b = 9.5$ (prior $P_0 = 0.05$).
- **Function**: `MapOfDynamics._close_window(t)` in [`Sim/mod/mod_core.py`](file:///Sim/mod/mod_core.py#L205-L227).
- **Explanation**: At the end of each observation window ($W = 2.0$ s), the accumulator arrays `_win_observed` and `_win_human` (boolean OR-fused across all robots) increment `obs_count` and `hum_count`. Laplace-smoothed probability is re-evaluated using NumPy array vectorization.

### 2. Time Since Last Sighting $\Delta t_{ij}$
- **Equation**:
  $$\Delta t_{ij} = \max(t_{\text{now}} - \text{last\_seen}_{ij}, \varepsilon), \quad \varepsilon = 10^{-6}$$
- **Function**: `MapOfDynamics._recompute_risk(t)` in [`Sim/mod/mod_core.py`](file:///Sim/mod/mod_core.py#L231-L253).
- **Timestamp Tracking**: Whenever a robot detects a human in a cell, `integrate()` stamps the cell with current simulation time:
  ```python
  self.last_seen[human_cells] = t
  ```

### 3. Short-Term Recency Term
- **Equation**:
  $$\text{recency}_{ij} = \begin{cases} 0.0 & \text{if cell was never observed with a human} \\ \exp\left(-\lambda \cdot \Delta t_{ij}\right) & \text{otherwise} \end{cases}$$
  where $\lambda = \frac{\ln(2)}{T_{\text{half}}}$ with $T_{\text{half}} = 20.0$ s.
- **Function**: `MapOfDynamics._recompute_risk(t)` in [`Sim/mod/mod_core.py`](file:///Sim/mod/mod_core.py#L242-L245).
- **Implementation**:
  ```python
  never_seen = np.isnan(self.last_seen)
  dt = np.where(never_seen, 0.0, np.maximum(t - self.last_seen, eps))
  self.recency = np.where(never_seen, 0.0, np.exp(-lambd * dt)).astype(np.float32)
  ```

### 4. Combined Risk $R_{ij}(H)$
- **Equation**:
  $$R_{ij} = \alpha \cdot P_{ij} + \beta \cdot \text{recency}_{ij}, \quad \text{with } R_{ij}[\text{obstacles}] = 0.0$$
- **Function**: `MapOfDynamics._recompute_risk(t)` in [`Sim/mod/mod_core.py`](file:///Sim/mod/mod_core.py#L248-L253).
- **Implementation**:
  ```python
  self.risk = (self.cfg.alpha * self.prob + self.cfg.beta * self.recency).astype(np.float32)
  self.risk[self._obstacle_mask] = 0.0
  ```

---

## 3. Architecture & Module Structure

```
Sim/
├── mod/
│   ├── __init__.py         # Package exports (MapOfDynamics, MoDConfig, StaticMap)
│   ├── static_map.py       # Static occupancy grid wrapper, pixel<->grid conversions, save/load
│   ├── visibility.py       # Bresenham ray-casting visibility estimator with wall occlusion
│   ├── detection.py        # Human detector (Mode A: ground-truth, Mode B: dynamic points)
│   ├── mod_core.py         # MoD engine: Bayesian counting, recency decay, vectorised risk equation
│   ├── overlay.py          # Pygame heatmap overlay, HUD, FOV wedges, keyboard controls, snapshots
│   └── planner_hook.py     # Optional risk-aware edge cost helper for path planners
├── tests/
│   └── test_mod.py         # Pytest test suite covering all 9 acceptance criteria
├── config.py               # Central config with MAP_OF_DYNAMICS dictionary and feature flag
├── main.py                 # Simulation entry point with non-intrusive MoD hooks
experiments/
└── run_mod_demo.py         # Headless 10-minute 3-aisle controlled demonstration experiment
docs/
├── MOD_DISCOVERY.md        # Initial repository discovery notes
├── MOD_ASSUMPTIONS.md      # Detailed assumptions, conventions, and default parameters
└── MOD_README.md           # This document
```

---

## 4. Acceptance Criteria Verification

All 9 acceptance criteria specified in the engineering requirements are verified by automated tests in `Sim/tests/test_mod.py`:

| # | Criterion | Test Function | Result |
|---|---|---|:---:|
| 1 | **Bounds & Validation** | `test_bounds_valid_config`, `test_invalid_configs_raise` | **PASSED** |
| 2 | **Exponential Decay** | `test_recency_decay` ($\approx 1.0$ at $t_0$, $\approx 0.5$ at $t_0 + T_{1/2}$) | **PASSED** |
| 3 | **Never-Seen Cells** | `test_never_seen_cells` (recency $= 0.0$, no NaN/Inf) | **PASSED** |
| 4 | **Visibility & Occlusion** | `test_visibility_occlusion` (wall blocks rays behind obstacle) | **PASSED** |
| 5 | **No False "Safe" Learning** | `test_no_false_safe_learning` (unobserved cells maintain $P = P_0$) | **PASSED** |
| 6 | **Multi-Robot OR-Fusion** | `test_multi_robot_fusion` (same cell/window increments by 1, not 2) | **PASSED** |
| 7 | **Habit Learning** | `test_habit_learning` ($P(A) > P(B) > P(C)$ from activity frequency) | **PASSED** |
| 8 | **State Persistence** | `test_persistence` (`save()` / `load()` reproduces all layers exactly) | **PASSED** |
| 9 | **Feature Flag Off** | `test_feature_flag_off` (sim operates identically with MoD disabled) | **PASSED** |
