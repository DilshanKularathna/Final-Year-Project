"""
environment.py — Warehouse Grid Layout & Navigation Graph
==========================================================
Defines the physical warehouse: shelves (obstacles), walkable aisles,
pick-up / drop-off zones, and a waypoint graph that robots and humans
use for pathfinding.

The warehouse is stored as a 2-D array of cell types and exposes helpers
for rendering, collision queries, and A* pathfinding on the waypoint graph.
"""

import heapq
import random
import pygame
from config import (
    CELL_SIZE, GRID_COLS, GRID_ROWS,
    WINDOW_WIDTH, WINDOW_HEIGHT,
    COLOR_BACKGROUND, COLOR_GRID_LINE, COLOR_SHELF,
    COLOR_AISLE, COLOR_PICKUP_ZONE, COLOR_DROPOFF_ZONE,
    COLOR_TEXT_DIM,
)

# ─────────────────────────────────────────────────────────────────────────────
# Cell type constants
# ─────────────────────────────────────────────────────────────────────────────
CELL_AISLE   = 0   # walkable
CELL_SHELF   = 1   # static obstacle
CELL_PICKUP  = 2   # pick-up zone
CELL_DROPOFF = 3   # drop-off zone


class Warehouse:
    """
    Manages the 2-D grid map of the warehouse, navigation waypoints,
    and provides pathfinding (A*) over walkable cells.
    """

    def __init__(self):
        # ── build empty grid (all aisles by default) ──────────────────────
        self.grid = [[CELL_AISLE for _ in range(GRID_COLS)]
                     for _ in range(GRID_ROWS)]

        # ── place shelves ─────────────────────────────────────────────────
        self._place_shelves()

        # ── designate pick-up and drop-off zones ─────────────────────────
        self._place_zones()

        # ── pre-compute walkable waypoints list ──────────────────────────
        self.waypoints = self._compute_waypoints()

        # ── pre-compute pick-up and drop-off waypoints for task alloc ────
        self.pickup_points  = [(c, r) for r in range(GRID_ROWS)
                               for c in range(GRID_COLS)
                               if self.grid[r][c] == CELL_PICKUP]
        self.dropoff_points = [(c, r) for r in range(GRID_ROWS)
                               for c in range(GRID_COLS)
                               if self.grid[r][c] == CELL_DROPOFF]

    # ─────────────────────────────────────────────────────────────────────
    # Layout builders (private)
    # ─────────────────────────────────────────────────────────────────────
    def _place_shelves(self):
        """
        Create a realistic rack layout:
        - 4 rows of double-wide shelf blocks with aisle gaps between them.
        - Leave a 2-cell border on every side so robots/humans can circulate.
        """
        # Shelf row bands (row indices, inclusive)
        shelf_bands = [
            (3, 4),
            (7, 8),
            (11, 12),
            (15, 16),
        ]
        # Shelf column spans — blocks of 3 cols with 2-col gaps
        col_start = 4
        col_end   = GRID_COLS - 4
        for band_top, band_bot in shelf_bands:
            if band_bot >= GRID_ROWS:
                continue
            col = col_start
            while col + 2 <= col_end:
                for dr in range(band_top, band_bot + 1):
                    for dc in range(col, min(col + 3, col_end)):
                        self.grid[dr][dc] = CELL_SHELF
                col += 5  # 3 shelf + 2 gap

    def _place_zones(self):
        """
        Designate left-edge cells as PICKUP, right-edge cells as DROPOFF.
        """
        for r in range(2, GRID_ROWS - 2):
            # left-most column walkable area → pick-up
            self.grid[r][1] = CELL_PICKUP
            # right-most column walkable area → drop-off
            self.grid[r][GRID_COLS - 2] = CELL_DROPOFF

    # ─────────────────────────────────────────────────────────────────────
    # Waypoint / pathfinding helpers
    # ─────────────────────────────────────────────────────────────────────
    def _compute_waypoints(self):
        """Return a list of (col, row) tuples for every walkable cell."""
        pts = []
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if self.grid[r][c] != CELL_SHELF:
                    pts.append((c, r))
        return pts

    def is_walkable(self, col, row):
        """Check whether a grid cell is walkable (not a shelf)."""
        if 0 <= col < GRID_COLS and 0 <= row < GRID_ROWS:
            return self.grid[row][col] != CELL_SHELF
        return False

    def cell_center(self, col, row):
        """Return the pixel centre (x, y) of a grid cell."""
        return (col * CELL_SIZE + CELL_SIZE // 2,
                row * CELL_SIZE + CELL_SIZE // 2)

    def pixel_to_grid(self, px, py):
        """Convert pixel coordinates to grid (col, row)."""
        return int(px // CELL_SIZE), int(py // CELL_SIZE)

    def random_walkable_pixel(self):
        """Return a random pixel position on a walkable cell."""
        wp = random.choice(self.waypoints)
        return self.cell_center(*wp)

    def get_neighbors(self, col, row):
        """Yield walkable 4-connected neighbours of (col, row)."""
        for dc, dr in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nc, nr = col + dc, row + dr
            if self.is_walkable(nc, nr):
                yield (nc, nr)

    # ─────────────────────────────────────────────────────────────────────
    # A* pathfinding (grid-based, 4-connected)
    # ─────────────────────────────────────────────────────────────────────
    def find_path(self, start_pixel, goal_pixel, blocked_cells=None):
        """
        A* search from start_pixel to goal_pixel.

        Parameters
        ----------
        start_pixel : (x, y) in pixels
        goal_pixel  : (x, y) in pixels
        blocked_cells : set of (col, row) to treat as temporarily blocked
                        (used for dynamic rerouting around humans).

        Returns
        -------
        list of (px, py) pixel waypoints from start to goal, or empty list
        if no path exists.
        """
        sc, sr = self.pixel_to_grid(*start_pixel)
        gc, gr = self.pixel_to_grid(*goal_pixel)

        if blocked_cells is None:
            blocked_cells = set()

        # heuristic — Manhattan distance
        def h(c, r):
            return abs(c - gc) + abs(r - gr)

        open_set = []
        heapq.heappush(open_set, (h(sc, sr), 0, sc, sr))
        came_from = {}
        g_score = {(sc, sr): 0}

        while open_set:
            _f, cost, cc, cr = heapq.heappop(open_set)

            if (cc, cr) == (gc, gr):
                # reconstruct
                path = []
                node = (gc, gr)
                while node in came_from:
                    path.append(self.cell_center(*node))
                    node = came_from[node]
                path.append(self.cell_center(sc, sr))
                path.reverse()
                return path

            for nc, nr in self.get_neighbors(cc, cr):
                if (nc, nr) in blocked_cells:
                    continue
                tentative = cost + 1
                if tentative < g_score.get((nc, nr), float('inf')):
                    g_score[(nc, nr)] = tentative
                    came_from[(nc, nr)] = (cc, cr)
                    heapq.heappush(open_set, (tentative + h(nc, nr),
                                              tentative, nc, nr))

        return []  # no path found

    # ─────────────────────────────────────────────────────────────────────
    # ARA* (Anytime Repairing A*) Pathfinding
    # ─────────────────────────────────────────────────────────────────────
    def find_path_ara(self, start_pixel, goal_pixel, blocked_cells=None, epsilon=2.5):
        """
        Anytime Repairing A* (ARA*) search.
        Uses heuristic inflation factor (epsilon >= 1.0) to compute an initial
        suboptimal path fast, then repairs it toward optimal as epsilon -> 1.0.

        Parameters
        ----------
        start_pixel   : (x, y) in pixels
        goal_pixel    : (x, y) in pixels
        blocked_cells : set of (col, row)
        epsilon       : float (heuristic inflation factor)

        Returns
        -------
        list of (px, py) waypoints
        """
        sc, sr = self.pixel_to_grid(*start_pixel)
        gc, gr = self.pixel_to_grid(*goal_pixel)

        if blocked_cells is None:
            blocked_cells = set()

        def h(c, r):
            return abs(c - gc) + abs(r - gr)

        # Inflated cost evaluation function f(n) = g(n) + epsilon * h(n)
        open_set = []
        heapq.heappush(open_set, (epsilon * h(sc, sr), 0, sc, sr))
        came_from = {}
        g_score = {(sc, sr): 0}
        closed_set = set()

        best_path = []

        # Anytime search loop: try initial inflated search, then refine if needed
        curr_eps = epsilon
        while curr_eps >= 1.0:
            while open_set:
                _f, cost, cc, cr = heapq.heappop(open_set)

                if (cc, cr) in closed_set:
                    continue
                closed_set.add((cc, cr))

                if (cc, cr) == (gc, gr):
                    path = []
                    node = (gc, gr)
                    while node in came_from:
                        path.append(self.cell_center(*node))
                        node = came_from[node]
                    path.append(self.cell_center(sc, sr))
                    path.reverse()
                    best_path = path
                    break

                for nc, nr in self.get_neighbors(cc, cr):
                    if (nc, nr) in blocked_cells:
                        continue
                    tentative = cost + 1
                    if tentative < g_score.get((nc, nr), float('inf')):
                        g_score[(nc, nr)] = tentative
                        came_from[(nc, nr)] = (cc, cr)
                        f_val = tentative + curr_eps * h(nc, nr)
                        heapq.heappush(open_set, (f_val, tentative, nc, nr))

            if best_path or curr_eps == 1.0:
                break
            # Reduce inflation factor to repair path toward optimal
            curr_eps = max(1.0, curr_eps - 0.5)

        return best_path if best_path else self.find_path(start_pixel, goal_pixel, blocked_cells)

    # ─────────────────────────────────────────────────────────────────────
    # WHCA* Space-Time Pathfinding (3D State Space: col, row, t)
    # ─────────────────────────────────────────────────────────────────────
    def find_space_time_path(self, start_pixel, goal_pixel, reservation_table, start_t=0, max_t=80):
        """
        WHCA* Space-Time A* Search over 3D state space (col, row, t).
        Evaluates dynamic collisions and edge swap conflicts against reservation_table.
        Allows waiting in place (c, r) at t+1 if current cell is safe.
        """
        sc, sr = self.pixel_to_grid(*start_pixel)
        gc, gr = self.pixel_to_grid(*goal_pixel)

        def h(c, r):
            return abs(c - gc) + abs(r - gr)

        # State in open_set: (f_score, g_cost, col, row, t)
        open_set = []
        heapq.heappush(open_set, (h(sc, sr), 0, sc, sr, start_t))
        came_from = {}
        g_score = {(sc, sr, start_t): 0}

        while open_set:
            _f, cost, cc, cr, ct = heapq.heappop(open_set)

            if (cc, cr) == (gc, gr):
                # Reconstruct space-time path
                path = []
                curr = (cc, cr, ct)
                while curr in came_from:
                    c, r, t = curr
                    path.append((c, r, t, self.cell_center(c, r)))
                    curr = came_from[curr]
                path.append((sc, sr, start_t, self.cell_center(sc, sr)))
                path.reverse()
                return path

            if ct >= start_t + max_t:
                continue

            # Candidate next actions at time t+1: 4-way move OR wait in place
            candidates = list(self.get_neighbors(cc, cr)) + [(cc, cr)]
            for nc, nr in candidates:
                nt = ct + 1
                # 1. Vertex collision check: Is cell (nc, nr) reserved at time nt?
                if (nc, nr, nt) in reservation_table:
                    continue
                # 2. Swap edge collision check: Did another robot move (nc, nr) -> (cc, cr) at nt?
                if (nc, nr, ct) in reservation_table and (cc, cr, nt) in reservation_table:
                    if reservation_table[(nc, nr, ct)] == reservation_table[(cc, cr, nt)]:
                        continue

                tentative = cost + (1.0 if (nc, nr) != (cc, cr) else 1.2)  # slight cost for waiting
                if tentative < g_score.get((nc, nr, nt), float('inf')):
                    g_score[(nc, nr, nt)] = tentative
                    came_from[(nc, nr, nt)] = (cc, cr, ct)
                    f_val = tentative + h(nc, nr)
                    heapq.heappush(open_set, (f_val, tentative, nc, nr, nt))

        return []

    # ─────────────────────────────────────────────────────────────────────
    # Rendering
    # ─────────────────────────────────────────────────────────────────────
    def draw(self, surface):
        """Render the warehouse floor, shelves, zones, and grid lines."""
        surface.fill(COLOR_BACKGROUND)

        # ── draw cells ────────────────────────────────────────────────────
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE,
                                   CELL_SIZE, CELL_SIZE)
                cell = self.grid[r][c]
                if cell == CELL_SHELF:
                    pygame.draw.rect(surface, COLOR_SHELF, rect)
                    # inner bevel for 3-D look
                    inner = rect.inflate(-4, -4)
                    pygame.draw.rect(surface, (90, 90, 105), inner)
                elif cell == CELL_PICKUP:
                    pygame.draw.rect(surface, COLOR_PICKUP_ZONE, rect)
                elif cell == CELL_DROPOFF:
                    pygame.draw.rect(surface, COLOR_DROPOFF_ZONE, rect)
                # aisles stay as background colour

        # ── grid lines ────────────────────────────────────────────────────
        for c in range(GRID_COLS + 1):
            pygame.draw.line(surface, COLOR_GRID_LINE,
                             (c * CELL_SIZE, 0),
                             (c * CELL_SIZE, WINDOW_HEIGHT))
        for r in range(GRID_ROWS + 1):
            pygame.draw.line(surface, COLOR_GRID_LINE,
                             (0, r * CELL_SIZE),
                             (WINDOW_WIDTH, r * CELL_SIZE))

        # ── zone labels ──────────────────────────────────────────────────
        font = pygame.font.SysFont("consolas", 11)
        lbl_p = font.render("PICK", True, COLOR_TEXT_DIM)
        lbl_d = font.render("DROP", True, COLOR_TEXT_DIM)
        surface.blit(lbl_p, (1 * CELL_SIZE + 2, 1 * CELL_SIZE + 2))
        surface.blit(lbl_d, ((GRID_COLS - 2) * CELL_SIZE + 2,
                             1 * CELL_SIZE + 2))
