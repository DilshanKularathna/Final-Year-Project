"""
mod/detection.py — Human Detection Module
===========================================
Provides two detection modes:

  Mode A (``ground_truth``, default):
      Uses ground-truth human positions from the simulation.  A human is
      detected if within max range, inside FOV, and line-of-sight is not
      blocked by the static map.

  Mode B (``lidar_dynamic_points``):
      Treats free-cell lidar hits as dynamic objects, clusters them, and
      filters by size to produce human candidates.

Both modes output a **human_mask** — a boolean grid marking cells occupied
by detected humans — AND-ed with the robot's visibility mask so that only
genuinely observed cells are flagged.
"""

import math
import random as _random
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class HumanDetection:
    """A single detected human in world (pixel) coordinates."""
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 0.0
    confidence: float = 1.0


class HumanDetector:
    """
    Detect humans from a robot's perspective.

    Parameters
    ----------
    static_map : mod.StaticMap
    mode : str
        ``"ground_truth"`` or ``"lidar_dynamic_points"``.
    fov_deg : float
        Sensor field of view in degrees.
    max_range_m : float
        Maximum detection range in metres.
    human_radius_m : float
        Radius of the disc stamped at each detection (metres).
    detection_prob : float
        Probability of detecting a visible human (Mode A only).
    pos_noise_m : float
        Gaussian noise σ applied to detected position (metres).
    """

    def __init__(self, static_map, mode="ground_truth",
                 fov_deg=360.0, max_range_m=5.0,
                 human_radius_m=0.35,
                 detection_prob=1.0, pos_noise_m=0.0):
        self.smap = static_map
        self.mode = mode
        self.fov_rad = math.radians(fov_deg)
        self.max_range_px = static_map.meters_to_pixels(max_range_m)
        self.human_radius_px = static_map.meters_to_pixels(human_radius_m)
        self.human_radius_cells = static_map.meters_to_cells(human_radius_m)
        self.detection_prob = detection_prob
        self.pos_noise_px = static_map.meters_to_pixels(pos_noise_m)

    # ─────────────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────────────
    def detect(self, robot_x, robot_y, robot_angle, humans, t,
               visible_mask=None):
        """
        Detect humans and return (detections, human_mask).

        Parameters
        ----------
        robot_x, robot_y : float
            Robot pose in pixel coords.
        robot_angle : float
            Robot heading (radians).
        humans : list
            List of HumanWorker objects with ``.x``, ``.y``, ``.vx``, ``.vy``.
        t : float
            Current simulation time (seconds).
        visible_mask : np.ndarray or None
            Boolean visibility mask from VisibilityEstimator.

        Returns
        -------
        detections : list[HumanDetection]
        human_mask : np.ndarray, shape (rows, cols), dtype bool
            Cells containing a detected human (already AND-ed with visible_mask).
        """
        if self.mode == "ground_truth":
            return self._detect_ground_truth(
                robot_x, robot_y, robot_angle, humans, visible_mask)
        else:
            return self._detect_lidar_dynamic(
                robot_x, robot_y, robot_angle, humans, visible_mask)

    # ─────────────────────────────────────────────────────────────────────
    # Mode A: Ground Truth
    # ─────────────────────────────────────────────────────────────────────
    def _detect_ground_truth(self, rx, ry, r_angle, humans, visible_mask):
        rows, cols = self.smap.rows, self.smap.cols
        detections = []
        human_mask = np.zeros((rows, cols), dtype=bool)

        for h in humans:
            hx, hy = h.x, h.y
            dist = math.hypot(hx - rx, hy - ry)

            # Range check
            if dist > self.max_range_px:
                continue

            # FOV check (skip if 360°)
            if self.fov_rad < 2 * math.pi - 0.01:
                angle_to_human = math.atan2(hy - ry, hx - rx)
                diff = abs((angle_to_human - r_angle + math.pi) % (2 * math.pi) - math.pi)
                if diff > self.fov_rad / 2.0:
                    continue

            # Line-of-sight check
            if not self._has_line_of_sight(rx, ry, hx, hy):
                continue

            # Detection probability
            if self.detection_prob < 1.0 and _random.random() > self.detection_prob:
                continue

            # Apply noise
            det_x, det_y = hx, hy
            if self.pos_noise_px > 0:
                det_x += _random.gauss(0, self.pos_noise_px)
                det_y += _random.gauss(0, self.pos_noise_px)

            detections.append(HumanDetection(
                x=det_x, y=det_y,
                vx=getattr(h, 'vx', 0.0),
                vy=getattr(h, 'vy', 0.0),
                radius=self.human_radius_px,
                confidence=1.0,
            ))

            # Stamp disc onto human_mask
            self._stamp_disc(human_mask, det_x, det_y, self.human_radius_cells)

        # AND with visibility mask
        if visible_mask is not None:
            human_mask &= visible_mask

        return detections, human_mask

    # ─────────────────────────────────────────────────────────────────────
    # Mode B: Lidar Dynamic Points (simplified for Pygame sim)
    # ─────────────────────────────────────────────────────────────────────
    def _detect_lidar_dynamic(self, rx, ry, r_angle, humans, visible_mask):
        """
        Detect dynamic points: for each human that is visible and in range,
        treat the cells at the human's position that are *free* in the static
        map as dynamic hits.  Cluster them and filter by size.

        In this simplified Pygame sim (no actual lidar returns), we simulate
        dynamic-point detection by checking if a human's occupied cells are
        free in the static map.
        """
        rows, cols = self.smap.rows, self.smap.cols
        detections = []
        human_mask = np.zeros((rows, cols), dtype=bool)

        for h in humans:
            hx, hy = h.x, h.y
            dist = math.hypot(hx - rx, hy - ry)
            if dist > self.max_range_px:
                continue

            if not self._has_line_of_sight(rx, ry, hx, hy):
                continue

            # Check if human's cell is free in static map (dynamic point)
            hr, hc = self.smap.world_to_cell(hx, hy)
            if not self.smap.obstacle_mask[hr, hc]:
                det_x, det_y = hx, hy
                if self.pos_noise_px > 0:
                    det_x += _random.gauss(0, self.pos_noise_px)
                    det_y += _random.gauss(0, self.pos_noise_px)

                detections.append(HumanDetection(
                    x=det_x, y=det_y,
                    vx=getattr(h, 'vx', 0.0),
                    vy=getattr(h, 'vy', 0.0),
                    radius=self.human_radius_px,
                    confidence=0.8,
                ))
                self._stamp_disc(human_mask, det_x, det_y, self.human_radius_cells)

        if visible_mask is not None:
            human_mask &= visible_mask

        return detections, human_mask

    # ─────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────
    def _has_line_of_sight(self, x0, y0, x1, y1):
        """
        Check if the line from (x0,y0) to (x1,y1) in pixel coords is
        unobstructed by static obstacles.  Uses Bresenham.
        """
        r0, c0 = self.smap.world_to_cell(x0, y0)
        r1, c1 = self.smap.world_to_cell(x1, y1)
        obstacle = self.smap.obstacle_mask

        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dc - dr
        r, c = r0, c0

        while True:
            if 0 <= r < self.smap.rows and 0 <= c < self.smap.cols:
                if obstacle[r, c] and (r != r0 or c != c0):
                    return False  # blocked
            else:
                return False  # out of bounds

            if r == r1 and c == c1:
                return True

            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr

        return True

    def _stamp_disc(self, mask, cx_px, cy_px, radius_cells):
        """
        Mark a disc of ``radius_cells`` in the boolean mask, centred at
        pixel position (cx_px, cy_px).
        """
        cr, cc = self.smap.world_to_cell(cx_px, cy_px)
        r_int = max(1, int(math.ceil(radius_cells)))

        r_min = max(0, cr - r_int)
        r_max = min(self.smap.rows - 1, cr + r_int)
        c_min = max(0, cc - r_int)
        c_max = min(self.smap.cols - 1, cc + r_int)

        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                if (r - cr) ** 2 + (c - cc) ** 2 <= r_int ** 2:
                    mask[r, c] = True
