"""
mod/visibility.py — Ray-Cast Visibility Estimator
===================================================
Computes which grid cells are visible to a robot at a given pose,
using Bresenham ray-casting through the static occupancy grid.

Only cells along each ray *up to* (but not behind) the first obstacle
or the sensor's max range are marked as visible.  This prevents the
MoD from updating cells the robot cannot actually see.
"""

import math
import numpy as np


class VisibilityEstimator:
    """
    Compute a boolean visibility mask for a robot pose.

    Parameters
    ----------
    static_map : mod.StaticMap
        The frozen static occupancy grid.
    fov_deg : float
        Field-of-view in degrees (360 = omnidirectional).
    max_range_m : float
        Maximum sensor range in metres.
    n_rays : int
        Number of rays cast over the FOV.
    """

    def __init__(self, static_map, fov_deg=360.0, max_range_m=5.0, n_rays=180):
        self.smap = static_map
        self.fov_rad = math.radians(fov_deg)
        self.max_range_px = static_map.meters_to_pixels(max_range_m)
        self.max_range_cells = max_range_m * static_map.cell_size_px / (static_map.cell_size_px * (static_map.resolution_m or 1.0))
        self.n_rays = max(n_rays, 4)

        # Pre-compute ray angle offsets relative to heading
        half_fov = self.fov_rad / 2.0
        self._ray_offsets = np.linspace(-half_fov, half_fov, self.n_rays, endpoint=True)

    def visible_cells(self, robot_x_px, robot_y_px, robot_angle_rad=0.0):
        """
        Compute visibility mask from a robot pose.

        Parameters
        ----------
        robot_x_px, robot_y_px : float
            Robot position in pixel coordinates.
        robot_angle_rad : float
            Robot heading in radians (0 = right, π/2 = down).

        Returns
        -------
        mask : np.ndarray, shape (rows, cols), dtype bool
            True for cells visible to this robot.
        """
        rows, cols = self.smap.rows, self.smap.cols
        mask = np.zeros((rows, cols), dtype=bool)
        obstacle = self.smap.obstacle_mask

        # Robot's grid position
        r0, c0 = self.smap.world_to_cell(robot_x_px, robot_y_px)
        mask[r0, c0] = True  # robot's own cell is always visible

        # Bounding box in grid to limit ray endpoints
        range_cells = int(math.ceil(self.max_range_px / self.smap.cell_size_px))
        r_min = max(0, r0 - range_cells)
        r_max = min(rows - 1, r0 + range_cells)
        c_min = max(0, c0 - range_cells)
        c_max = min(cols - 1, c0 + range_cells)

        # Cast rays
        for offset in self._ray_offsets:
            angle = robot_angle_rad + offset

            # Endpoint of this ray at max range (in pixels)
            ex_px = robot_x_px + self.max_range_px * math.cos(angle)
            ey_px = robot_y_px + self.max_range_px * math.sin(angle)

            # Convert to grid
            er, ec = self.smap.world_to_cell(ex_px, ey_px)

            # Clamp to bounding box
            er = max(r_min, min(er, r_max))
            ec = max(c_min, min(ec, c_max))

            # Bresenham from (r0, c0) to (er, ec)
            self._bresenham_mark(mask, obstacle, r0, c0, er, ec,
                                 rows, cols)

        return mask

    @staticmethod
    def _bresenham_mark(mask, obstacle, r0, c0, r1, c1, rows, cols):
        """
        Walk from (r0, c0) to (r1, c1) using Bresenham's line algorithm.
        Mark each cell as visible.  Stop at the first obstacle cell
        (mark the obstacle itself as visible but nothing behind it).
        """
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dc - dr

        r, c = r0, c0
        while True:
            if 0 <= r < rows and 0 <= c < cols:
                mask[r, c] = True
                if obstacle[r, c] and (r != r0 or c != c0):
                    break  # hit obstacle — stop ray
            else:
                break  # out of bounds

            if r == r1 and c == c1:
                break

            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr
