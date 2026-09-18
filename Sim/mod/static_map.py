"""
mod/static_map.py — Static Occupancy Grid Wrapper
===================================================
Wraps the Warehouse grid as a NumPy-backed static map with metadata
(resolution, origin, dimensions) and centralised coordinate conversions.

All MoD modules use ``world_to_cell`` and ``cell_to_world`` from this
class so that coordinate transformations are defined exactly once.
"""

import numpy as np
import os

from config import CELL_SIZE, GRID_COLS, GRID_ROWS, PIXELS_PER_METER

# Cell type constants (mirrored from environment.py to avoid circular import)
_CELL_SHELF = 1


class StaticMap:
    """
    Read-only NumPy wrapper around the warehouse occupancy grid.

    Attributes
    ----------
    grid : np.ndarray, shape (rows, cols), dtype uint8
        Cell types: 0=AISLE, 1=SHELF, 2=PICKUP, 3=DROPOFF.
    obstacle_mask : np.ndarray, shape (rows, cols), dtype bool
        True for cells that are static obstacles (shelves).
    rows, cols : int
        Grid dimensions.
    cell_size_px : int
        Pixel size of each cell.
    resolution_m : float
        Metric size of each cell (cell_size_px / PIXELS_PER_METER).
    origin_px : (float, float)
        Pixel position of grid origin (top-left corner), always (0, 0).
    """

    def __init__(self, warehouse_grid):
        """
        Parameters
        ----------
        warehouse_grid : list[list[int]]
            The ``Warehouse.grid`` 2-D list from environment.py.
        """
        self.rows = len(warehouse_grid)
        self.cols = len(warehouse_grid[0]) if self.rows > 0 else 0
        self.cell_size_px = CELL_SIZE
        self.resolution_m = CELL_SIZE / PIXELS_PER_METER
        self.origin_px = (0.0, 0.0)

        # Convert to NumPy
        self.grid = np.array(warehouse_grid, dtype=np.uint8)
        self.obstacle_mask = (self.grid == _CELL_SHELF)

        # Walkable mask: cells a human can actually occupy (aisles).
        # Pickup/dropoff zones are work zones; humans wander on aisles only.
        self.walkable_mask = (self.grid == 0)  # CELL_AISLE

    # ─────────────────────────────────────────────────────────────────────
    # Coordinate conversions  (SINGLE SOURCE OF TRUTH)
    # ─────────────────────────────────────────────────────────────────────
    def world_to_cell(self, x_px, y_px):
        """
        Convert pixel coordinates to grid (row, col).

        Parameters
        ----------
        x_px, y_px : float
            Pixel position (x grows right, y grows down).

        Returns
        -------
        (row, col) : (int, int)
            Grid indices clamped to valid range.
        """
        col = int(x_px // self.cell_size_px)
        row = int(y_px // self.cell_size_px)
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        return row, col

    def cell_to_world(self, row, col):
        """
        Convert grid (row, col) to pixel centre (x, y).

        Returns
        -------
        (x_px, y_px) : (float, float)
        """
        x_px = col * self.cell_size_px + self.cell_size_px / 2.0
        y_px = row * self.cell_size_px + self.cell_size_px / 2.0
        return x_px, y_px

    def is_obstacle(self, row, col):
        """Check if (row, col) is a static obstacle (shelf)."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.obstacle_mask[row, col]
        return True  # out-of-bounds treated as obstacle

    def meters_to_cells(self, metres):
        """Convert a metric distance to number of grid cells (float)."""
        return metres * PIXELS_PER_METER / self.cell_size_px

    def meters_to_pixels(self, metres):
        """Convert metres to pixels."""
        return metres * PIXELS_PER_METER

    # ─────────────────────────────────────────────────────────────────────
    # Persistence
    # ─────────────────────────────────────────────────────────────────────
    def save(self, path):
        """Save the static map to a .npz file."""
        os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
        np.savez_compressed(
            path,
            grid=self.grid,
            cell_size_px=np.array([self.cell_size_px]),
            resolution_m=np.array([self.resolution_m]),
            origin_px=np.array(self.origin_px),
        )

    @classmethod
    def load(cls, path):
        """Load a static map from a .npz file."""
        data = np.load(path)
        grid_list = data["grid"].tolist()
        smap = cls(grid_list)
        return smap
