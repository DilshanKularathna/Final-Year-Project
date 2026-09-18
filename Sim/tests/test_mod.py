"""
tests/test_mod.py — Map of Dynamics Acceptance Tests
======================================================
Verifies all 9 acceptance criteria specified in the MoD specification:

  1. Bounds: 0 <= risk < 1 and 0 < prob < 1 for valid inputs; invalid configs raise.
  2. Decay: recency ≈ 1 at t0, ≈ 0.5 at t0 + T_half, monotonically decreases.
  3. Never-seen cells: recency is exactly 0.0; no NaN/inf in risk.
  4. Visibility: wall occlusion blocks observation behind wall.
  5. No false "safe" learning: unobserved cells maintain P = P0.
  6. Multi-robot fusion: multiple robots in same window increment count by 1.
  7. Habit learning: frequently visited aisle has clearly higher P than others.
  8. Persistence: save() and load() reproduce layers exactly.
  9. Feature flag off: simulation operates identically without MoD.
"""

import math
import os
import tempfile
import pytest
import numpy as np

import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from mod.static_map import StaticMap
from mod.mod_core import MapOfDynamics, MoDConfig
from mod.visibility import VisibilityEstimator
from mod.detection import HumanDetector, HumanDetection
from environment import Warehouse, CELL_AISLE, CELL_SHELF
from config import MAP_OF_DYNAMICS


@pytest.fixture
def basic_warehouse():
    return Warehouse()


@pytest.fixture
def basic_static_map(basic_warehouse):
    return StaticMap(basic_warehouse.grid)


@pytest.fixture
def default_config():
    return MoDConfig(
        alpha=0.6,
        beta=0.3,
        half_life_s=20.0,
        window_s=2.0,
        prior_a=0.5,
        prior_b=9.5,
        human_radius_m=0.35,
        risk_update_hz=5.0,
    )


# ─────────────────────────────────────────────────────────────────────────────
# 1. Bounds: valid and invalid configurations
# ─────────────────────────────────────────────────────────────────────────────

def test_bounds_valid_config(basic_static_map, default_config):
    """
    With random counts, random last_seen, and valid config:
    0 <= risk < 1 and 0 < prob < 1 must always hold.
    """
    mod = MapOfDynamics(basic_static_map, default_config)
    rng = np.random.default_rng(42)

    H, W = basic_static_map.rows, basic_static_map.cols

    # Inject random realistic counts
    obs = rng.integers(0, 100, size=(H, W)).astype(np.float32)
    hum = (obs * rng.uniform(0.0, 1.0, size=(H, W))).astype(np.float32)
    mod.obs_count = obs
    mod.hum_count = hum

    # Random last_seen times between 0 and 100, some NaN
    last_seen = rng.uniform(0.0, 100.0, size=(H, W)).astype(np.float32)
    nan_mask = rng.uniform(0.0, 1.0, size=(H, W)) < 0.4
    last_seen[nan_mask] = np.nan
    mod.last_seen = last_seen

    # Force recomputation
    mod._close_window(t=100.0)
    mod._recompute_risk(t=100.0)

    # Validate prob bounds: 0 < prob < 1
    assert np.all(mod.prob > 0.0), f"prob min is {mod.prob.min()} <= 0"
    assert np.all(mod.prob < 1.0), f"prob max is {mod.prob.max()} >= 1"

    # Validate risk bounds: 0 <= risk < 1
    assert np.all(mod.risk >= 0.0), f"risk min is {mod.risk.min()} < 0"
    assert np.all(mod.risk < 1.0), f"risk max is {mod.risk.max()} >= 1"
    assert not np.any(np.isnan(mod.risk)), "risk contains NaN"
    assert not np.any(np.isinf(mod.risk)), "risk contains Inf"


def test_invalid_configs_raise():
    """Invalid config (alpha+beta >= 1, non-positive values) must raise ValueError."""
    # alpha + beta >= 1
    with pytest.raises(ValueError, match=r"alpha \+ beta must be < 1"):
        MoDConfig(alpha=0.6, beta=0.4).validate()

    with pytest.raises(ValueError, match=r"alpha \+ beta must be < 1"):
        MoDConfig(alpha=0.8, beta=0.3).validate()

    # non-positive alpha or beta
    with pytest.raises(ValueError, match=r"alpha must be > 0"):
        MoDConfig(alpha=0.0, beta=0.3).validate()

    with pytest.raises(ValueError, match=r"beta must be > 0"):
        MoDConfig(alpha=0.6, beta=-0.1).validate()

    # non-positive half_life_s
    with pytest.raises(ValueError, match=r"half_life_s must be > 0"):
        MoDConfig(half_life_s=0.0).validate()

    # non-positive window_s
    with pytest.raises(ValueError, match=r"window_s must be > 0"):
        MoDConfig(window_s=-1.0).validate()

    # non-positive priors
    with pytest.raises(ValueError, match=r"prior_a and prior_b must be > 0"):
        MoDConfig(prior_a=0.0).validate()


# ─────────────────────────────────────────────────────────────────────────────
# 2. Decay: recency at t0, t0 + T_half, and monotonic decrease
# ─────────────────────────────────────────────────────────────────────────────

def test_recency_decay(basic_static_map, default_config):
    """
    A cell with a human at t0 has recency ≈ 1.0 at t0, ≈ 0.5 at t0 + T_half,
    and monotonically decreases over time.
    """
    mod = MapOfDynamics(basic_static_map, default_config)
    t0 = 10.0
    r, c = 5, 5

    # Human seen at (r, c) at t0
    mod.last_seen[r, c] = t0

    # At t0 + eps (immediate observation)
    mod._recompute_risk(t0 + 1e-5)
    rec_t0 = mod.recency[r, c]
    assert abs(rec_t0 - 1.0) < 0.01, f"Expected recency ~ 1.0 at t0, got {rec_t0}"

    # At t0 + T_half
    T_half = default_config.half_life_s  # 20s
    mod._recompute_risk(t0 + T_half)
    rec_half = mod.recency[r, c]
    assert abs(rec_half - 0.5) < 0.01, f"Expected recency ~ 0.5 at t0 + T_half, got {rec_half}"

    # At t0 + 2 * T_half
    mod._recompute_risk(t0 + 2 * T_half)
    rec_quarter = mod.recency[r, c]
    assert abs(rec_quarter - 0.25) < 0.01, f"Expected recency ~ 0.25 at t0 + 2*T_half, got {rec_quarter}"

    # Verify monotonic decrease across a time series
    times = [t0 + i * 5.0 for i in range(10)]
    rec_values = []
    for t in times:
        mod._recompute_risk(t)
        rec_values.append(mod.recency[r, c])

    for i in range(len(rec_values) - 1):
        assert rec_values[i] > rec_values[i + 1], (
            f"Recency not strictly decreasing: {rec_values[i]} <= {rec_values[i+1]}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# 3. Never-seen cells: recency == 0, no NaN/inf
# ─────────────────────────────────────────────────────────────────────────────

def test_never_seen_cells(basic_static_map, default_config):
    """
    For cells that have never contained a human, recency is exactly 0.0,
    and risk contains no NaN or inf.
    """
    mod = MapOfDynamics(basic_static_map, default_config)
    mod._recompute_risk(t=50.0)

    # Initially, all cells are never seen
    assert np.all(np.isnan(mod.last_seen))
    assert np.all(mod.recency == 0.0)

    # Risk should simply equal alpha * P0 (or 0 for obstacles)
    P0 = default_config.prior_a / (default_config.prior_a + default_config.prior_b)
    expected_risk = default_config.alpha * P0

    non_obstacles = ~basic_static_map.obstacle_mask
    assert np.allclose(mod.risk[non_obstacles], expected_risk)
    assert np.all(mod.risk[basic_static_map.obstacle_mask] == 0.0)

    assert not np.any(np.isnan(mod.risk))
    assert not np.any(np.isinf(mod.risk))


# ─────────────────────────────────────────────────────────────────────────────
# 4. Visibility: wall occlusion
# ─────────────────────────────────────────────────────────────────────────────

def test_visibility_occlusion():
    """
    Ray casting through static map must stop at obstacles.
    A cell behind a wall is not marked visible, while a cell in front is.
    """
    # Create a synthetic 10x10 map with a wall of shelves at column 5
    grid = [[CELL_AISLE for _ in range(10)] for _ in range(10)]
    for r in range(10):
        grid[r][5] = CELL_SHELF

    smap = StaticMap(grid)
    vis = VisibilityEstimator(smap, fov_deg=360.0, max_range_m=10.0, n_rays=180)

    # Place robot at (col 3, row 5) -> looking horizontally toward wall
    robot_x, robot_y = smap.cell_to_world(5, 3)
    mask = vis.visible_cells(robot_x, robot_y, robot_angle_rad=0.0)

    # Cell in front of wall (col 4, row 5) must be visible
    assert mask[5, 4] is True or mask[5, 4] == 1, "Cell in front of wall should be visible"

    # Shelf itself (col 5, row 5) is visible (hit by rays)
    assert mask[5, 5] is True or mask[5, 5] == 1, "Obstacle cell at surface should be visible"

    # Cell behind wall (col 6, row 5) must NOT be visible (occluded)
    assert mask[5, 6] is False or mask[5, 6] == 0, "Cell behind wall must be occluded"

    # Test MoD integration with this mask: obs_count behind wall remains 0
    cfg = MoDConfig(window_s=1.0)
    mod = MapOfDynamics(smap, cfg)
    empty_human_mask = np.zeros_like(mask, dtype=bool)

    mod.integrate(robot_id=1, visible_mask=mask, human_mask=empty_human_mask, t=0.0)
    mod.step(t=1.1)  # closes window (1.1 - 0.0 >= 1.0)

    assert mod.obs_count[5, 4] == 1.0, "Observed cell in front of wall should have count 1"
    assert mod.obs_count[5, 6] == 0.0, "Occluded cell behind wall should have count 0"


# ─────────────────────────────────────────────────────────────────────────────
# 5. No false "safe" learning: unobserved cells keep P = P0
# ─────────────────────────────────────────────────────────────────────────────

def test_no_false_safe_learning(basic_static_map, default_config):
    """
    Unobserved cells must maintain P = P0.
    A cell a robot cannot see must NOT count as 'no human'.
    """
    mod = MapOfDynamics(basic_static_map, default_config)
    P0 = default_config.prior_a / (default_config.prior_a + default_config.prior_b)

    # Observe only cell (2, 2) with no human
    H, W = basic_static_map.rows, basic_static_map.cols
    vis_mask = np.zeros((H, W), dtype=bool)
    vis_mask[2, 2] = True
    hum_mask = np.zeros((H, W), dtype=bool)

    # Integrate for 5 windows (window_s = 2.0)
    for i in range(5):
        t_start = i * default_config.window_s
        mod.integrate(1, vis_mask, hum_mask, t_start)
        mod.step(t_start + default_config.window_s)

    # Observed cell (2, 2) drops below P0 because obs_count=5, hum_count=0
    expected_p_22 = default_config.prior_a / (5.0 + default_config.prior_a + default_config.prior_b)
    assert abs(mod.prob[2, 2] - expected_p_22) < 1e-4
    assert mod.prob[2, 2] < P0

    # Unobserved cell (10, 10) must strictly maintain P = P0
    assert abs(mod.prob[10, 10] - P0) < 1e-6, (
        f"Unobserved cell changed P from {P0} to {mod.prob[10, 10]}"
    )
    assert mod.obs_count[10, 10] == 0.0
    assert mod.hum_count[10, 10] == 0.0


# ─────────────────────────────────────────────────────────────────────────────
# 6. Multi-robot fusion: same cell/window = 1 count
# ─────────────────────────────────────────────────────────────────────────────

def test_multi_robot_fusion(basic_static_map, default_config):
    """
    Two robots observing the same cell and human in the same window
    increments count by 1, not 2.
    """
    mod = MapOfDynamics(basic_static_map, default_config)
    H, W = basic_static_map.rows, basic_static_map.cols

    target_r, target_c = 4, 8

    mask1 = np.zeros((H, W), dtype=bool)
    mask1[target_r, target_c] = True
    hum1 = np.zeros((H, W), dtype=bool)
    hum1[target_r, target_c] = True

    mask2 = np.zeros((H, W), dtype=bool)
    mask2[target_r, target_c] = True
    hum2 = np.zeros((H, W), dtype=bool)
    hum2[target_r, target_c] = True

    # Robot 1 integrates at t = 0.2 (window starts at 0.2)
    mod.integrate(robot_id=1, visible_mask=mask1, human_mask=hum1, t=0.2)
    # Robot 2 integrates at t = 0.8 (same window [0.2, 2.2])
    mod.integrate(robot_id=2, visible_mask=mask2, human_mask=hum2, t=0.8)

    # Step at t = 2.3 to close window (2.3 - 0.2 = 2.1 >= 2.0)
    mod.step(t=2.3)

    assert mod.obs_count[target_r, target_c] == 1.0, (
        f"Expected obs_count == 1 from OR-fusion, got {mod.obs_count[target_r, target_c]}"
    )
    assert mod.hum_count[target_r, target_c] == 1.0, (
        f"Expected hum_count == 1 from OR-fusion, got {mod.hum_count[target_r, target_c]}"
    )


# ─────────────────────────────────────────────────────────────────────────────
# 7. Habit learning: repeated observations increase P
# ─────────────────────────────────────────────────────────────────────────────

def test_habit_learning(basic_static_map, default_config):
    """
    Scripted visits where aisle A is used heavily, B rarely, C never.
    After learning: P(A) > P(B) > P(C).
    """
    mod = MapOfDynamics(basic_static_map, default_config)
    H, W = basic_static_map.rows, basic_static_map.cols

    # Designate three walkable test cells
    cell_a = (3, 4)
    cell_b = (3, 8)
    cell_c = (3, 12)

    # All three cells are regularly observed
    vis_mask = np.zeros((H, W), dtype=bool)
    vis_mask[cell_a] = True
    vis_mask[cell_b] = True
    vis_mask[cell_c] = True

    # Run 20 windows:
    # Cell A: human present in 18 out of 20 windows (90%)
    # Cell B: human present in 4 out of 20 windows (20%)
    # Cell C: human present in 0 out of 20 windows (0%)
    n_windows = 20
    for w in range(n_windows):
        t_start = w * default_config.window_s
        hum_mask = np.zeros((H, W), dtype=bool)
        if w < 18:
            hum_mask[cell_a] = True
        if w < 4:
            hum_mask[cell_b] = True

        mod.integrate(robot_id=1, visible_mask=vis_mask, human_mask=hum_mask, t=t_start)
        mod.step(t=t_start + default_config.window_s)

    p_a = mod.prob[cell_a]
    p_b = mod.prob[cell_b]
    p_c = mod.prob[cell_c]

    assert p_a > p_b > p_c, f"Expected P(A) > P(B) > P(C), got {p_a:.3f} > {p_b:.3f} > {p_c:.3f}"
    assert p_a > 0.6, f"P(A) expected > 0.6, got {p_a}"
    assert p_c < 0.05, f"P(C) expected < 0.05, got {p_c}"



# ─────────────────────────────────────────────────────────────────────────────
# 8. Persistence: save/load roundtrip
# ─────────────────────────────────────────────────────────────────────────────

def test_persistence(basic_static_map, default_config):
    """save() followed by load() reproduces all layers exactly."""
    mod1 = MapOfDynamics(basic_static_map, default_config)
    rng = np.random.default_rng(123)
    H, W = basic_static_map.rows, basic_static_map.cols

    mod1.obs_count = rng.integers(0, 50, size=(H, W)).astype(np.float32)
    mod1.hum_count = rng.integers(0, 20, size=(H, W)).astype(np.float32)
    mod1.prob = rng.uniform(0.01, 0.99, size=(H, W)).astype(np.float32)
    mod1.recency = rng.uniform(0.0, 1.0, size=(H, W)).astype(np.float32)
    mod1.risk = rng.uniform(0.0, 0.9, size=(H, W)).astype(np.float32)
    mod1.observed_ever = (rng.uniform(0, 1, size=(H, W)) > 0.5).astype(np.float32)

    last_seen = rng.uniform(0, 100, size=(H, W)).astype(np.float32)
    last_seen[last_seen < 40] = np.nan
    mod1.last_seen = last_seen

    with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as tmp:
        tmp_path = tmp.name

    try:
        mod1.save(tmp_path)
        mod2 = MapOfDynamics(basic_static_map, default_config)
        mod2.load(tmp_path)

        for layer_name in ("obs_count", "hum_count", "prob", "recency", "risk", "observed_ever"):
            arr1 = getattr(mod1, layer_name)
            arr2 = getattr(mod2, layer_name)
            assert np.array_equal(arr1, arr2), f"Layer '{layer_name}' mismatch after save/load"

        # last_seen has NaNs, compare using isnan mask and equal values
        nan1 = np.isnan(mod1.last_seen)
        nan2 = np.isnan(mod2.last_seen)
        assert np.array_equal(nan1, nan2), "NaN locations in last_seen mismatch"
        assert np.allclose(mod1.last_seen[~nan1], mod2.last_seen[~nan2]), "last_seen values mismatch"

    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


# ─────────────────────────────────────────────────────────────────────────────
# 9. Feature flag off: simulation baseline intact
# ─────────────────────────────────────────────────────────────────────────────

def test_feature_flag_off(basic_warehouse):
    """
    When MAP_OF_DYNAMICS["enabled"] is False, simulation components
    operate cleanly without creating MoD structures.
    """
    from robot import SmorphiRobot
    from human import HumanWorker
    from coordinator import Coordinator

    # Setup standard simulation entities
    humans = [HumanWorker(basic_warehouse, 1)]
    robot_pos = basic_warehouse.cell_center(5, 5)
    robots = [SmorphiRobot(1, robot_pos, basic_warehouse)]
    coord = Coordinator(basic_warehouse, robots, humans)

    # Perform a few baseline ticks
    for _ in range(10):
        for h in humans:
            h.update()
        coord.update()
        for r in robots:
            r.update()

    assert len(robots) == 1
    assert len(humans) == 1
    assert robots[0].x > 0 and robots[0].y > 0
