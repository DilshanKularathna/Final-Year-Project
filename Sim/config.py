"""
config.py — Global Configuration Parameters
=============================================
Central configuration module for the Human-Aware Multi-Robot Warehouse Simulation.
All tunable simulation parameters live here so every other module imports from
a single source of truth.  Swap values here to change behaviour globally.
"""

# ─────────────────────────────────────────────────────────────────────────────
# Display / Window
# ─────────────────────────────────────────────────────────────────────────────
WINDOW_WIDTH  = 1280          # pixels
WINDOW_HEIGHT = 720           # pixels
FPS           = 60            # target frames-per-second for the main loop

# ─────────────────────────────────────────────────────────────────────────────
# Grid Layout
# ─────────────────────────────────────────────────────────────────────────────
CELL_SIZE     = 40            # pixels per grid cell (square)
GRID_COLS     = WINDOW_WIDTH  // CELL_SIZE   # 32 columns
GRID_ROWS     = WINDOW_HEIGHT // CELL_SIZE   # 18 rows

# ─────────────────────────────────────────────────────────────────────────────
# Robot Parameters (Smorphi-style holonomic)
# ─────────────────────────────────────────────────────────────────────────────
ROBOT_RADIUS       = 14       # visual radius in pixels
ROBOT_SPEED        = 2.0      # nominal movement speed (px / frame)
ROBOT_YIELD_SPEED  = 0.0      # speed while yielding (full stop)
ROBOT_REROUTE_SPEED = 1.5     # speed while taking alternate route
NUM_ROBOTS         = 4        # 4 robots in fleet
NUM_HUMANS         = 2        # 2 human workers in warehouse
LANE_KEEP_OFFSET   = 7.0      # lateral offset in pixels (left side of heading)
WHCA_WINDOW             = 60     # Space-Time reservation horizon window (time steps)
PHYSICAL_COLLISION_MARGIN = 2.0   # Hard physical circle pushback buffer (px)

# ─────────────────────────────────────────────────────────────────────────────
# Human Worker Parameters
# ─────────────────────────────────────────────────────────────────────────────
HUMAN_RADIUS        = 10      # visual radius in pixels
HUMAN_SPEED         = 1.2     # normal walking speed (px / frame)
HUMAN_INTENT_LENGTH = 100     # length of the predictive intent vector line (px)
HUMAN_WANDER_PAUSE  = 120     # frames to idle at a waypoint before moving again

# ─────────────────────────────────────────────────────────────────────────────
# Proxemic / Safety Zones (distances in pixels)
# ─────────────────────────────────────────────────────────────────────────────
PROXEMIC_INNER_RADIUS = 60    # inner safety circle — hard stop / yield
PROXEMIC_OUTER_RADIUS = 130   # outer predictive zone — begin re-routing evaluation
INTENT_CONE_HALF_ANGLE = 30   # degrees — half-angle of the predictive intent cone

# ─────────────────────────────────────────────────────────────────────────────
# Colour Palette  (R, G, B)
# ─────────────────────────────────────────────────────────────────────────────
COLOR_BACKGROUND   = (18,  18,  24)      # near-black workspace
COLOR_GRID_LINE    = (30,  30,  40)      # subtle grid
COLOR_SHELF        = (70,  70,  85)      # warehouse rack / shelf
COLOR_AISLE        = (25,  25,  35)      # walkable aisle (slightly lighter than bg)
COLOR_PICKUP_ZONE  = (46, 139, 87)       # sea-green pickup zone
COLOR_DROPOFF_ZONE = (178, 102, 34)      # amber drop-off zone

COLOR_HUMAN_1_BODY = (0,  200, 255)      # cyan human 1
COLOR_HUMAN_2_BODY = (0,  255, 170)      # mint human 2
COLOR_HUMAN_BODY   = (0,  200, 255)      # default human marker
COLOR_HUMAN_INTENT = (0,  120, 200)      # intent vector line

COLOR_ROBOT_IDLE       = (100, 220, 100)  # green — idle
COLOR_ROBOT_ALLOCATING = (255, 215,   0)  # gold — allocating task (1s pause)
COLOR_ROBOT_PICKING    = (255, 180,   0)  # amber-gold — picking item (1s pause)
COLOR_ROBOT_MOVING     = (52,  152, 219)  # blue — moving along left lane
COLOR_ROBOT_COMPLETED  = (46,  204, 113)  # emerald green — task done (1s pause)
COLOR_ROBOT_YIELDING   = (231,  76,  60)  # red — yielding/stop in place
COLOR_ROBOT_REROUTING  = (155,  89, 182)  # purple — rerouting

COLOR_PROXEMIC_INNER  = (255,  60,  60, 50)   # translucent red
COLOR_PROXEMIC_OUTER  = (255, 180,  30, 30)   # translucent orange

COLOR_TEXT            = (210, 210, 220)
COLOR_TEXT_DIM        = (120, 120, 140)

# ─────────────────────────────────────────────────────────────────────────────
# Robot State Enum-like Constants
# ─────────────────────────────────────────────────────────────────────────────
STATE_IDLE       = "IDLE"
STATE_ALLOCATING = "ALLOCATING"
STATE_PICKING    = "PICKING"
STATE_MOVING     = "MOVING"
STATE_COMPLETED  = "COMPLETED"
STATE_YIELDING   = "YIELDING"
STATE_REROUTING  = "REROUTING"

# Map state → colour for convenience
STATE_COLORS = {
    STATE_IDLE:       COLOR_ROBOT_IDLE,
    STATE_ALLOCATING: COLOR_ROBOT_ALLOCATING,
    STATE_PICKING:    COLOR_ROBOT_PICKING,
    STATE_MOVING:     COLOR_ROBOT_MOVING,
    STATE_COMPLETED:  COLOR_ROBOT_COMPLETED,
    STATE_YIELDING:   COLOR_ROBOT_YIELDING,
    STATE_REROUTING:  COLOR_ROBOT_REROUTING,
}
