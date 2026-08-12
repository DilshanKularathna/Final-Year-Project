"""
human.py — Human Worker Agent
==============================
Simulates a single human warehouse worker who wanders through aisles,
pauses at waypoints, and exposes a predictive intent vector that the
coordinator uses for proactive robot avoidance.

The worker picks random walkable destinations on the grid and walks
toward them along a simple straight-line path (no A* — humans don't
follow optimal paths).  When arriving at a waypoint, the worker
pauses for a configurable number of frames before selecting the next
destination.
"""

import math
import random
import pygame
from config import (
    HUMAN_RADIUS, HUMAN_SPEED, HUMAN_INTENT_LENGTH,
    HUMAN_WANDER_PAUSE,
    COLOR_HUMAN_1_BODY, COLOR_HUMAN_2_BODY, COLOR_HUMAN_BODY, COLOR_HUMAN_INTENT, COLOR_TEXT,
    CELL_SIZE,
)


class HumanWorker:
    """
    Represents a human pedestrian in the warehouse.

    Attributes
    ----------
    x, y       : float   — current pixel position.
    vx, vy     : float   — current velocity components (px/frame).
    angle      : float   — heading in radians (0 = right, pi/2 = down).
    target     : (x, y)  — current movement target pixel position.
    pause_timer: int     — countdown frames while idling at a waypoint.
    """

    def __init__(self, warehouse, human_id=1):
        """
        Parameters
        ----------
        warehouse : environment.Warehouse
        human_id  : int (1 or 2)
        """
        self.id = human_id
        self.warehouse = warehouse
        self.color = COLOR_HUMAN_1_BODY if human_id == 1 else COLOR_HUMAN_2_BODY

        # Spawn on a random walkable cell
        sx, sy = warehouse.random_walkable_pixel()
        self.x  = float(sx)
        self.y  = float(sy)
        self.vx = 0.0
        self.vy = 0.0
        self.angle = 0.0

        self.target      = None
        self.path        = []
        self.path_index  = 0
        self.pause_timer = 0
        self._pick_new_target()

    # ─────────────────────────────────────────────────────────────────────
    # Target selection
    # ─────────────────────────────────────────────────────────────────────
    def _pick_new_target(self):
        """Choose a new random walkable destination and plan an A* path."""
        for _ in range(50):
            tx, ty = self.warehouse.random_walkable_pixel()
            dist = math.hypot(tx - self.x, ty - self.y)
            if dist > CELL_SIZE * 3:
                path = self.warehouse.find_path(
                    (self.x, self.y), (float(tx), float(ty)))
                if path and len(path) > 1:
                    self.path = path
                    self.path_index = 0
                    self.target = (float(tx), float(ty))
                    return
        # fallback
        tx, ty = self.warehouse.random_walkable_pixel()
        self.target = (float(tx), float(ty))
        self.path = [(self.x, self.y), (float(tx), float(ty))]
        self.path_index = 0

    # ─────────────────────────────────────────────────────────────────────
    # Per-frame update
    # ─────────────────────────────────────────────────────────────────────
    def update(self):
        """Advance the human worker by one simulation tick."""
        # ── idling at waypoint ────────────────────────────────────────────
        if self.pause_timer > 0:
            self.vx = 0.0
            self.vy = 0.0
            self.pause_timer -= 1
            if self.pause_timer == 0:
                self._pick_new_target()
            return

        if self.target is None or self.path_index >= len(self.path):
            self._pick_new_target()
            return

        # ── steer toward next waypoint on the A* path ────────────────────
        tx, ty = self.path[self.path_index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        # ── reached current waypoint? advance to next ────────────────────
        if dist < HUMAN_SPEED * 2:
            self.x, self.y = tx, ty
            self.path_index += 1
            if self.path_index >= len(self.path):
                self.vx = 0.0
                self.vy = 0.0
                self.pause_timer = HUMAN_WANDER_PAUSE + random.randint(-30, 60)
            return

        # ── move toward current waypoint ─────────────────────────────────
        self.angle = math.atan2(dy, dx)
        self.vx = HUMAN_SPEED * (dx / dist)
        self.vy = HUMAN_SPEED * (dy / dist)
        self.x += self.vx
        self.y += self.vy

    # ─────────────────────────────────────────────────────────────────────
    # Predictive intent helpers  (used by coordinator)
    # ─────────────────────────────────────────────────────────────────────
    @property
    def position(self):
        """Return current position as a tuple."""
        return (self.x, self.y)

    @property
    def velocity(self):
        """Return current velocity as a tuple."""
        return (self.vx, self.vy)

    def intent_endpoint(self):
        """
        Return the pixel (x, y) at the tip of the predictive intent vector.
        If the worker is stationary, the intent just points in the last
        known heading direction.
        """
        speed = math.hypot(self.vx, self.vy)
        if speed > 0.01:
            ix = self.x + (self.vx / speed) * HUMAN_INTENT_LENGTH
            iy = self.y + (self.vy / speed) * HUMAN_INTENT_LENGTH
        else:
            ix = self.x + math.cos(self.angle) * HUMAN_INTENT_LENGTH
            iy = self.y + math.sin(self.angle) * HUMAN_INTENT_LENGTH
        return (ix, iy)

    # ─────────────────────────────────────────────────────────────────────
    # Rendering
    # ─────────────────────────────────────────────────────────────────────
    def draw(self, surface):
        """Draw the human worker and intent vector."""
        ix, iy = self.intent_endpoint()

        # ── intent vector line ────────────────────────────────────────────
        pygame.draw.line(surface, COLOR_HUMAN_INTENT,
                         (int(self.x), int(self.y)),
                         (int(ix), int(iy)), 2)

        # ── small arrowhead at tip ────────────────────────────────────────
        arrow_len = 8
        angle = math.atan2(iy - self.y, ix - self.x)
        left  = (ix - arrow_len * math.cos(angle - 0.4),
                 iy - arrow_len * math.sin(angle - 0.4))
        right = (ix - arrow_len * math.cos(angle + 0.4),
                 iy - arrow_len * math.sin(angle + 0.4))
        pygame.draw.polygon(surface, COLOR_HUMAN_INTENT,
                            [(int(ix), int(iy)),
                             (int(left[0]), int(left[1])),
                             (int(right[0]), int(right[1]))])

        # ── body circle ──────────────────────────────────────────────────
        pygame.draw.circle(surface, self.color,
                           (int(self.x), int(self.y)), HUMAN_RADIUS)
        pygame.draw.circle(surface, (255, 255, 255),
                           (int(self.x), int(self.y)), HUMAN_RADIUS, 2)

        # ── label ────────────────────────────────────────────────────────
        font = pygame.font.SysFont("consolas", 11, bold=True)
        lbl  = font.render(f"H{self.id}", True, COLOR_TEXT)
        surface.blit(lbl, (int(self.x) - lbl.get_width() // 2,
                           int(self.y) - HUMAN_RADIUS - 14))
