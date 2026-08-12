"""
robot.py — Smorphi-Style Holonomic Robot Agent
================================================
Manages holonomic movement, waypoint path tracking, state machine
(IDLE, ALLOCATING, PICKING, MOVING, YIELDING, REROUTING, COMPLETED), left-hand lane
keeping offset, mandatory 1-second dwell pauses, and Markov localization tracking.
"""

import math
import random
import pygame
from config import (
    ROBOT_RADIUS, HUMAN_RADIUS, ROBOT_SPEED, ROBOT_YIELD_SPEED, ROBOT_REROUTE_SPEED,
    STATE_IDLE, STATE_ALLOCATING, STATE_PICKING, STATE_MOVING, STATE_COMPLETED, STATE_YIELDING, STATE_REROUTING,
    STATE_COLORS, COLOR_TEXT, CELL_SIZE, GRID_COLS, GRID_ROWS, LANE_KEEP_OFFSET,
)


class SmorphiRobot:
    """
    Holonomic robot executing A*/ARA* waypoint paths with left-hand lane-keeping
    spatial offset, 1-second task dwell pauses (ALLOCATING, PICKING & COMPLETED),
    and Markov Localization state tracking.
    """

    def __init__(self, robot_id, start_pos, warehouse):
        self.id = robot_id
        self.warehouse = warehouse
        self.x, self.y = float(start_pos[0]), float(start_pos[1])

        # state machine & 1-second dwell timer (60 frames)
        self.state = STATE_IDLE
        self.prev_state = STATE_IDLE
        self.dwell_timer = 0
        self.phase = "IDLE"  # "IDLE", "PICKUP", "DROPOFF"

        # navigation
        self.pickup_path = []
        self.dropoff_path = []
        self.path = []
        self.path_index = 0
        self.goal = None
        self.task_label = ""

        # movement & left-hand lane offset
        self.speed = 0.0
        self.angle = 0.0
        self.lane_offset_x = 0.0
        self.lane_offset_y = 0.0

        # Markov Localization Belief
        self.belief_col, self.belief_row = self.grid_cell
        self.belief_variance = 4.0

    # ── State transitions ────────────────────────────────────────────────
    def set_state(self, new_state, dwell_frames=0):
        self.prev_state = self.state
        self.state = new_state
        self.dwell_timer = dwell_frames

        if new_state in (STATE_IDLE, STATE_YIELDING, STATE_ALLOCATING, STATE_PICKING, STATE_COMPLETED):
            self.speed = 0.0
        elif new_state == STATE_MOVING:
            self.speed = ROBOT_SPEED
        elif new_state == STATE_REROUTING:
            self.speed = ROBOT_REROUTE_SPEED

    def assign_task(self, pickup_path, dropoff_path, task_label="TASK"):
        if not pickup_path or not dropoff_path:
            return
        self.pickup_path = pickup_path
        self.dropoff_path = dropoff_path
        self.path = pickup_path
        self.path_index = 0
        self.phase = "PICKUP"
        self.goal = dropoff_path[-1]
        self.task_label = task_label
        # Start with 1-second ALLOCATING pause (60 frames)
        self.set_state(STATE_ALLOCATING, dwell_frames=60)

    def assign_path(self, path, task_label="TASK"):
        """Fallback for single path assignment."""
        mid = len(path) // 2
        self.assign_task(path[:mid+1], path[mid:], task_label)

    def clear_path(self):
        self.pickup_path = []
        self.dropoff_path = []
        self.path = []
        self.path_index = 0
        self.phase = "IDLE"
        self.goal = None
        self.task_label = ""
        self.set_state(STATE_IDLE)

    # ── Markov Localization Update ───────────────────────────────────────
    def _update_markov_localization(self, move_x, move_y):
        col, row = self.grid_cell
        self.belief_col = col
        self.belief_row = row

    # ── Per-frame update ─────────────────────────────────────────────────
    def update(self):
        # 1. Handle 1-second Dwell Timers for ALLOCATING, PICKING, and COMPLETED states
        if self.dwell_timer > 0:
            self.dwell_timer -= 1
            if self.dwell_timer == 0:
                if self.state == STATE_ALLOCATING:
                    self.set_state(STATE_MOVING)
                elif self.state == STATE_PICKING:
                    # Finished 1s PICKING pause -> start moving along Dropoff path
                    self.path = self.dropoff_path
                    self.path_index = 0
                    self.phase = "DROPOFF"
                    self.set_state(STATE_MOVING)
                elif self.state == STATE_COMPLETED:
                    # Finished 1s COMPLETED pause -> reset to IDLE
                    self.clear_path()
            return

        if self.state in (STATE_IDLE, STATE_YIELDING) or self.speed == 0.0:
            self._update_markov_localization(0, 0)
            return

        # 2. Check if current phase path is finished
        if len(self.path) > 0 and self.path_index >= len(self.path):
            if self.phase == "PICKUP":
                # Arrived at Pickup location -> 1-second Gold PICKING pause (60 frames)
                self.set_state(STATE_PICKING, dwell_frames=60)
            elif self.phase == "DROPOFF":
                # Arrived at Dropoff location -> 1-second Emerald Green COMPLETED pause (60 frames)
                self.set_state(STATE_COMPLETED, dwell_frames=60)
            return

        if not self.path:
            return

        tx, ty = self.path[self.path_index]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)

        if dist < self.speed * 1.5:
            self.x, self.y = tx, ty
            self.path_index += 1
        else:
            self.angle = math.atan2(dy, dx)

            # Left-hand Lane Keeping Rule:
            # Perpendicular vector pointing left of forward heading: (-sin(angle), cos(angle))
            left_nx = -math.sin(self.angle)
            left_ny = math.cos(self.angle)
            self.lane_offset_x = left_nx * LANE_KEEP_OFFSET
            self.lane_offset_y = left_ny * LANE_KEEP_OFFSET

            move_x = self.speed * (dx / dist)
            move_y = self.speed * (dy / dist)

            self.x += move_x
            self.y += move_y

            self._update_markov_localization(move_x, move_y)

    # ── Queries ──────────────────────────────────────────────────────────
    @property
    def position(self):
        """Effective center position considering left-hand lane offset."""
        return (self.x + self.lane_offset_x, self.y + self.lane_offset_y)

    @property
    def grid_cell(self):
        px, py = self.position
        return self.warehouse.pixel_to_grid(px, py)

    def distance_to(self, px, py):
        rx, ry = self.position
        return math.hypot(rx - px, ry - py)

    def remaining_waypoints(self):
        return self.path[self.path_index:]

    # ── Rendering ────────────────────────────────────────────────────────
    def draw(self, surface):
        color = STATE_COLORS.get(self.state, (200, 200, 200))
        cx = int(self.x + self.lane_offset_x)
        cy = int(self.y + self.lane_offset_y)

        # Path preview
        rem = self.remaining_waypoints()
        if len(rem) > 1:
            pts = [(int(p[0] + self.lane_offset_x), int(p[1] + self.lane_offset_y)) for p in rem]
            pygame.draw.lines(surface, color, False, pts, 1)

        # Body
        pygame.draw.circle(surface, color, (cx, cy), ROBOT_RADIUS)
        # Heading tick
        hx = cx + int(ROBOT_RADIUS * math.cos(self.angle))
        hy = cy + int(ROBOT_RADIUS * math.sin(self.angle))
        pygame.draw.line(surface, (255, 255, 255), (cx, cy), (hx, hy), 2)
        # Outline
        pygame.draw.circle(surface, (255, 255, 255), (cx, cy), ROBOT_RADIUS, 2)

        # ID label
        font = pygame.font.SysFont("consolas", 11, bold=True)
        lbl = font.render(f"R{self.id}", True, (0, 0, 0))
        surface.blit(lbl, (cx - lbl.get_width() // 2, cy - lbl.get_height() // 2))

        # State tag (ALLOCATING, PICKING, MOVING, YIELDING, COMPLETED)
        tfont = pygame.font.SysFont("consolas", 9, bold=True)
        tag = tfont.render(self.state, True, color)
        surface.blit(tag, (cx - tag.get_width() // 2, cy + ROBOT_RADIUS + 3))
