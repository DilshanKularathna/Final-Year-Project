"""
mod/mod_core.py — Map of Dynamics Core Engine
===============================================
Implements the MoD grid layers, window-based observation counting with
Bayesian smoothing, recency decay, and the risk equation:

    R_ij = α · P_ij + β · exp(-λ · Δt_ij)

All computation is vectorised with NumPy.  No Python loops over cells.
"""

import math
import os
from dataclasses import dataclass, field
from typing import Optional, Dict

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class MoDConfig:
    """
    Configuration for the Map of Dynamics engine.
    All parameters loaded from config.MAP_OF_DYNAMICS.
    """
    alpha: float = 0.6
    beta: float = 0.3
    half_life_s: float = 20.0
    window_s: float = 2.0
    prior_a: float = 0.5
    prior_b: float = 9.5
    human_radius_m: float = 0.35
    forgetting_tau_s: Optional[float] = None
    risk_update_hz: float = 5.0

    def __post_init__(self):
        """Mandatory startup validation."""
        self.validate()
        self.lambd = math.log(2.0) / self.half_life_s  # λ = ln(2) / T_half

    def validate(self):
        """
        Assert all constraints.  Called once at startup.
        Raises ValueError with a clear message on violation.
        """
        if self.alpha <= 0:
            raise ValueError(f"MoD config: alpha must be > 0, got {self.alpha}")
        if self.beta <= 0:
            raise ValueError(f"MoD config: beta must be > 0, got {self.beta}")
        if self.half_life_s <= 0:
            raise ValueError(f"MoD config: half_life_s must be > 0, got {self.half_life_s}")
        if self.alpha + self.beta >= 1.0:
            raise ValueError(
                f"MoD config: alpha + beta must be < 1, "
                f"got {self.alpha} + {self.beta} = {self.alpha + self.beta}"
            )
        if self.window_s <= 0:
            raise ValueError(f"MoD config: window_s must be > 0, got {self.window_s}")
        if self.prior_a <= 0 or self.prior_b <= 0:
            raise ValueError(
                f"MoD config: prior_a and prior_b must be > 0, "
                f"got a={self.prior_a}, b={self.prior_b}"
            )
        if self.risk_update_hz <= 0:
            raise ValueError(f"MoD config: risk_update_hz must be > 0, got {self.risk_update_hz}")

    @classmethod
    def from_dict(cls, d: dict) -> "MoDConfig":
        """Create from the MAP_OF_DYNAMICS config dictionary."""
        cfg = cls(
            alpha=d.get("alpha", 0.6),
            beta=d.get("beta", 0.3),
            half_life_s=d.get("half_life_s", 20.0),
            window_s=d.get("window_s", 2.0),
            prior_a=d.get("prior_a", 0.5),
            prior_b=d.get("prior_b", 9.5),
            human_radius_m=d.get("human_radius_m", 0.35),
            forgetting_tau_s=d.get("forgetting_tau_s", None),
            risk_update_hz=d.get("risk_update_hz", 5.0),
        )
        cfg.validate()
        return cfg


# ─────────────────────────────────────────────────────────────────────────────
# Map of Dynamics Engine
# ─────────────────────────────────────────────────────────────────────────────

class MapOfDynamics:
    """
    Maintains MoD grid layers and computes the risk map.

    Layers (all float32, shape (rows, cols)):
        obs_count    — total observation windows in which this cell was observed
        hum_count    — total observation windows in which a human was detected
        prob         — P_ij = (hum_count + a) / (obs_count + a + b)
        last_seen    — simulation time of last human detection (NaN = never)
        recency      — exp(-λ · Δt) or 0 for never-seen cells
        risk         — α · prob + β · recency
        observed_ever — 1.0 for cells that have been observed at least once

    Integration protocol:
        1. Each tick, for each robot: call ``integrate(robot_id, visible_mask, human_mask, t)``
        2. Each tick: call ``step(t)`` to handle window transitions and risk recomputation.
    """

    def __init__(self, static_map, cfg: MoDConfig):
        """
        Parameters
        ----------
        static_map : mod.StaticMap
        cfg : MoDConfig
        """
        self.smap = static_map
        self.cfg = cfg
        H, W = static_map.rows, static_map.cols
        self._shape = (H, W)

        # ── Persistent layers ────────────────────────────────────────────
        self.obs_count = np.zeros((H, W), dtype=np.float32)
        self.hum_count = np.zeros((H, W), dtype=np.float32)
        self.prob = np.full((H, W), cfg.prior_a / (cfg.prior_a + cfg.prior_b),
                            dtype=np.float32)
        self.last_seen = np.full((H, W), np.nan, dtype=np.float32)
        self.recency = np.zeros((H, W), dtype=np.float32)
        self.risk = np.zeros((H, W), dtype=np.float32)
        self.observed_ever = np.zeros((H, W), dtype=np.float32)

        # ── Current-window accumulators (OR-fused across robots) ─────────
        self._win_observed = np.zeros((H, W), dtype=bool)
        self._win_human = np.zeros((H, W), dtype=bool)
        self._window_start = None  # set on first integrate() call

        # ── Timing ───────────────────────────────────────────────────────
        self._risk_interval = 1.0 / cfg.risk_update_hz
        self._last_risk_update = -1e9  # force immediate first update

        # ── Static obstacle mask ─────────────────────────────────────────
        self._obstacle_mask = static_map.obstacle_mask

    # ─────────────────────────────────────────────────────────────────────
    # Integration: per-robot, per-tick
    # ─────────────────────────────────────────────────────────────────────
    def integrate(self, robot_id, visible_mask, human_mask, t):
        """
        Accumulate a robot's observations into the current window.

        Uses boolean OR so that multiple robots seeing the same cell
        in the same window count as ONE observation, not multiple.

        Parameters
        ----------
        robot_id : int
        visible_mask : np.ndarray (H, W) bool — cells this robot observed
        human_mask   : np.ndarray (H, W) bool — cells where a human was detected
        t : float — current simulation time in seconds
        """
        if self._window_start is None:
            self._window_start = t

        # OR-fuse into current window
        np.logical_or(self._win_observed, visible_mask, out=self._win_observed)
        np.logical_or(self._win_human, human_mask, out=self._win_human)

        # Update last_seen for cells where a human is detected NOW
        human_cells = human_mask & visible_mask
        if np.any(human_cells):
            self.last_seen[human_cells] = t
            self.observed_ever[human_cells] = 1.0

        # Mark newly observed cells
        self.observed_ever[visible_mask] = np.maximum(
            self.observed_ever[visible_mask], 1.0)

    # ─────────────────────────────────────────────────────────────────────
    # Step: called once per tick after all robots have integrated
    # ─────────────────────────────────────────────────────────────────────
    def step(self, t):
        """
        Close the observation window if enough time has elapsed, update
        counts and P.  Recompute recency and risk at the configured rate.

        Parameters
        ----------
        t : float — current simulation time in seconds
        """
        if self._window_start is None:
            self._window_start = t

        # ── Close window? ────────────────────────────────────────────────
        if t - self._window_start >= self.cfg.window_s:
            self._close_window(t)

        # ── Recompute risk at configured rate ────────────────────────────
        if t - self._last_risk_update >= self._risk_interval:
            self._recompute_risk(t)
            self._last_risk_update = t

    # ─────────────────────────────────────────────────────────────────────
    # Window close: update counts, P
    # ─────────────────────────────────────────────────────────────────────
    def _close_window(self, t):
        """Process the finished window and reset accumulators."""
        # Optional forgetting: decay old counts before adding new
        if self.cfg.forgetting_tau_s is not None and self.cfg.forgetting_tau_s > 0:
            decay = math.exp(-self.cfg.window_s / self.cfg.forgetting_tau_s)
            self.obs_count *= decay
            self.hum_count *= decay

        # Increment counts from this window
        self.obs_count += self._win_observed.astype(np.float32)
        self.hum_count += self._win_human.astype(np.float32)

        # Recompute P with Laplace smoothing
        # P = (hum_count + a) / (obs_count + a + b)
        a = self.cfg.prior_a
        b = self.cfg.prior_b
        self.prob = (self.hum_count + a) / (self.obs_count + a + b)

        # Reset window accumulators
        self._win_observed[:] = False
        self._win_human[:] = False
        self._window_start = t

    # ─────────────────────────────────────────────────────────────────────
    # Risk recomputation (vectorised)
    # ─────────────────────────────────────────────────────────────────────
    def _recompute_risk(self, t):
        """
        Compute:
            recency = where(isnan(last_seen), 0, exp(-λ · max(t - last_seen, ε)))
            risk = α · prob + β · recency
            risk[obstacle] = 0
        """
        eps = 1e-6
        lambd = self.cfg.lambd

        # Recency (vectorised, no loops)
        never_seen = np.isnan(self.last_seen)
        dt = np.where(never_seen, 0.0, np.maximum(t - self.last_seen, eps))
        self.recency = np.where(never_seen, 0.0,
                                np.exp(-lambd * dt)).astype(np.float32)

        # Risk
        self.risk = (self.cfg.alpha * self.prob +
                     self.cfg.beta * self.recency).astype(np.float32)

        # Zero out static obstacles
        self.risk[self._obstacle_mask] = 0.0

    # ─────────────────────────────────────────────────────────────────────
    # Public queries
    # ─────────────────────────────────────────────────────────────────────
    def layers(self) -> Dict[str, np.ndarray]:
        """Return all layers as a dictionary."""
        return {
            "risk": self.risk,
            "prob": self.prob,
            "recency": self.recency,
            "obs_count": self.obs_count,
            "hum_count": self.hum_count,
            "last_seen": self.last_seen,
            "observed_ever": self.observed_ever,
        }

    def risk_at_cell(self, row, col):
        """Return risk value at a grid cell."""
        if 0 <= row < self._shape[0] and 0 <= col < self._shape[1]:
            return float(self.risk[row, col])
        return 0.0

    def risk_at(self, x_px, y_px):
        """Return risk value at a pixel position."""
        row, col = self.smap.world_to_cell(x_px, y_px)
        return self.risk_at_cell(row, col)

    def coverage_fraction(self):
        """Fraction of non-obstacle cells that have been observed at least once."""
        non_obstacle = ~self._obstacle_mask
        total = non_obstacle.sum()
        if total == 0:
            return 0.0
        observed = (self.observed_ever[non_obstacle] > 0).sum()
        return float(observed / total)

    # ─────────────────────────────────────────────────────────────────────
    # Persistence
    # ─────────────────────────────────────────────────────────────────────
    def save(self, path):
        """Save all layers to a .npz file for reuse across runs."""
        os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
        np.savez_compressed(
            path,
            obs_count=self.obs_count,
            hum_count=self.hum_count,
            prob=self.prob,
            last_seen=self.last_seen,
            recency=self.recency,
            risk=self.risk,
            observed_ever=self.observed_ever,
        )

    def load(self, path):
        """Load layers from a .npz file."""
        data = np.load(path, allow_pickle=False)
        for key in ("obs_count", "hum_count", "prob", "last_seen",
                     "recency", "risk", "observed_ever"):
            arr = data[key]
            if arr.shape == self._shape:
                setattr(self, key, arr.astype(np.float32))
            else:
                raise ValueError(
                    f"Layer '{key}' shape {arr.shape} does not match "
                    f"static map shape {self._shape}"
                )
