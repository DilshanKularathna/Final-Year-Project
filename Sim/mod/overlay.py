"""
mod/overlay.py — MoD Heatmap Overlay for Pygame
=================================================
Renders the risk / probability / recency / coverage layers as a
semi-transparent heatmap on top of the warehouse view.

Keyboard controls:
    1  — risk layer (default)
    2  — long-term probability P
    3  — recency
    4  — observation coverage
    M  — toggle MoD overlay on / off
    S  — export PNG snapshot
"""

import math
import os
import pygame
import numpy as np

from config import CELL_SIZE, GRID_COLS, GRID_ROWS


# ─────────────────────────────────────────────────────────────────────────────
# Colour helpers
# ─────────────────────────────────────────────────────────────────────────────

def _risk_color(v):
    """
    Map a value in [0, 1] to an RGB colour on a green→yellow→red ramp.
    Returns (R, G, B) tuple.
    """
    v = max(0.0, min(1.0, v))
    if v < 0.5:
        t = v / 0.5
        r = int(255 * t)
        g = 255
    else:
        t = (v - 0.5) / 0.5
        r = 255
        g = int(255 * (1 - t))
    return (r, g, 0)


def _build_colormap_lut(n=256):
    """Pre-build a 256-entry LUT for the green→yellow→red ramp."""
    lut = np.zeros((n, 3), dtype=np.uint8)
    for i in range(n):
        lut[i] = _risk_color(i / (n - 1))
    return lut


_COLORMAP_LUT = _build_colormap_lut()


class MoDOverlay:
    """
    Pygame overlay renderer for MoD layers.

    Parameters
    ----------
    mod : MapOfDynamics
    static_map : StaticMap
    cfg_dict : dict
        The MAP_OF_DYNAMICS config dictionary.
    """

    def __init__(self, mod, static_map, cfg_dict):
        self.mod = mod
        self.smap = static_map
        self.cfg_dict = cfg_dict

        # State
        self.visible = True          # M key toggles
        self.active_layer = "risk"   # 1/2/3/4 keys
        self._layer_names = ["risk", "prob", "recency", "observed_ever"]
        self._layer_labels = {
            "risk": "Risk R(H)",
            "prob": "Probability P(H)",
            "recency": "Recency exp(-λΔt)",
            "observed_ever": "Observation Coverage",
        }

        # Snapshot
        self._snapshot_dir = cfg_dict.get("snapshot_dir", "outputs/mod_snapshots")
        self._snapshot_interval = cfg_dict.get("snapshot_interval_s", 30.0)
        self._last_snapshot_t = -1e9
        self._snapshot_count = 0

        # Pre-create overlay surface
        self._overlay_surf = pygame.Surface(
            (GRID_COLS * CELL_SIZE, GRID_ROWS * CELL_SIZE), pygame.SRCALPHA)

        # Font (lazy init)
        self._font = None
        self._font_small = None

    def _ensure_fonts(self):
        if self._font is None:
            self._font = pygame.font.SysFont("consolas", 12, bold=True)
            self._font_small = pygame.font.SysFont("consolas", 10)

    # ─────────────────────────────────────────────────────────────────────
    # Keyboard handling
    # ─────────────────────────────────────────────────────────────────────
    def handle_key(self, key, t, screen=None):
        """
        Handle MoD-related key presses.
        Returns True if the key was consumed.
        """
        if key == pygame.K_m:
            self.visible = not self.visible
            return True
        elif key == pygame.K_1:
            self.active_layer = "risk"
            return True
        elif key == pygame.K_2:
            self.active_layer = "prob"
            return True
        elif key == pygame.K_3:
            self.active_layer = "recency"
            return True
        elif key == pygame.K_4:
            self.active_layer = "observed_ever"
            return True
        elif key == pygame.K_s:
            self.export_snapshot(t, screen)
            return True
        return False

    # ─────────────────────────────────────────────────────────────────────
    # Drawing
    # ─────────────────────────────────────────────────────────────────────
    def draw(self, screen, t, robots=None, humans=None, detections_by_robot=None):
        """
        Draw the MoD overlay on the Pygame screen.

        Call this AFTER warehouse.draw() and BEFORE drawing humans/robots.
        """
        if not self.visible:
            return

        self._ensure_fonts()
        self._draw_heatmap(screen)
        self._draw_hud(screen, t, robots, humans)
        self._draw_legend(screen)

        # Draw FOV wedges for robots
        if robots:
            for robot in robots:
                self._draw_fov_wedge(screen, robot)

        # Draw detections vs true positions
        if humans:
            for h in humans:
                # True position — white cross
                self._draw_marker(screen, int(h.x), int(h.y),
                                  (255, 255, 255), cross=True, size=6)
        if detections_by_robot:
            for robot_id, dets in detections_by_robot.items():
                for d in dets:
                    # Detected position — magenta diamond
                    self._draw_marker(screen, int(d.x), int(d.y),
                                      (255, 0, 255), cross=False, size=5)

        # Auto-snapshot
        if (self._snapshot_interval > 0 and
                t - self._last_snapshot_t >= self._snapshot_interval):
            self.export_snapshot(t, screen)

    def _draw_heatmap(self, screen):
        """Render the active layer as a cell-based heatmap overlay."""
        layer = getattr(self.mod, self.active_layer, self.mod.risk)
        observed_ever = self.mod.observed_ever
        obstacle_mask = self.smap.obstacle_mask

        # Determine scaling
        if self.active_layer == "risk":
            vmin, vmax = 0.0, 1.0
        elif self.active_layer == "prob":
            # Robust scaling: use 99th percentile for prob
            non_zero = layer[layer > 0]
            if len(non_zero) > 0:
                vmax = max(float(np.percentile(non_zero, 99)), 0.01)
            else:
                vmax = 0.1
            vmin = 0.0
        elif self.active_layer == "recency":
            vmin, vmax = 0.0, 1.0
        else:  # observed_ever / coverage
            vmin, vmax = 0.0, 1.0

        self._overlay_surf.fill((0, 0, 0, 0))

        for r in range(self.smap.rows):
            for c in range(self.smap.cols):
                if obstacle_mask[r, c]:
                    continue  # skip obstacles

                val = float(layer[r, c])
                is_observed = observed_ever[r, c] > 0

                rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE,
                                   CELL_SIZE, CELL_SIZE)

                if not is_observed and self.active_layer != "observed_ever":
                    # Never observed — subtle grey tint
                    pygame.draw.rect(self._overlay_surf,
                                     (80, 80, 100, 40), rect)
                    # Hatch pattern (diagonal lines)
                    for offset in range(0, CELL_SIZE * 2, 8):
                        x0 = c * CELL_SIZE
                        y0 = r * CELL_SIZE + offset
                        x1 = c * CELL_SIZE + min(offset, CELL_SIZE)
                        y1 = r * CELL_SIZE
                        if (y0 < (r + 1) * CELL_SIZE and
                                x1 <= (c + 1) * CELL_SIZE):
                            pygame.draw.line(self._overlay_surf,
                                             (100, 100, 120, 25),
                                             (x0, min(y0, (r + 1) * CELL_SIZE - 1)),
                                             (min(x1, (c + 1) * CELL_SIZE - 1), y1), 1)
                else:
                    # Normalise and look up colour
                    norm = (val - vmin) / (vmax - vmin) if vmax > vmin else 0.0
                    norm = max(0.0, min(1.0, norm))
                    idx = int(norm * 255)
                    cr, cg, cb = int(_COLORMAP_LUT[idx, 0]), \
                                 int(_COLORMAP_LUT[idx, 1]), \
                                 int(_COLORMAP_LUT[idx, 2])

                    # Alpha proportional to value (minimum 20 for visibility)
                    alpha = int(20 + norm * 150)
                    pygame.draw.rect(self._overlay_surf,
                                     (cr, cg, cb, alpha), rect)

        screen.blit(self._overlay_surf, (0, 0))

    def _draw_hud(self, screen, t, robots, humans):
        """Draw MoD HUD text at the bottom-left."""
        cfg = self.mod.cfg
        lines = [
            f"MoD: {self._layer_labels.get(self.active_layer, self.active_layer)}",
            f"Sim Time: {t:.1f}s  |  α={cfg.alpha}  β={cfg.beta}  "
            f"λ={cfg.lambd:.4f} (T½={cfg.half_life_s}s)",
            f"Robots: {len(robots) if robots else 0}  "
            f"Humans: {len(humans) if humans else 0}  "
            f"Coverage: {self.mod.coverage_fraction() * 100:.1f}%",
        ]

        y = screen.get_height() - len(lines) * 16 - 8
        for line in lines:
            lbl = self._font.render(line, True, (220, 220, 240))
            # Dark background for readability
            bg = pygame.Surface((lbl.get_width() + 8, lbl.get_height() + 2),
                                pygame.SRCALPHA)
            bg.fill((10, 10, 20, 180))
            screen.blit(bg, (4, y - 1))
            screen.blit(lbl, (8, y))
            y += 16

    def _draw_legend(self, screen):
        """Draw a vertical colour bar legend on the right side."""
        bar_w, bar_h = 16, 150
        x0 = screen.get_width() - bar_w - 30
        y0 = screen.get_height() - bar_h - 60

        # Draw colour bar
        for i in range(bar_h):
            norm = 1.0 - i / bar_h  # top = high, bottom = low
            idx = int(norm * 255)
            cr, cg, cb = int(_COLORMAP_LUT[idx, 0]), \
                         int(_COLORMAP_LUT[idx, 1]), \
                         int(_COLORMAP_LUT[idx, 2])
            pygame.draw.line(screen, (cr, cg, cb),
                             (x0, y0 + i), (x0 + bar_w, y0 + i))

        # Border
        pygame.draw.rect(screen, (180, 180, 200),
                         (x0 - 1, y0 - 1, bar_w + 2, bar_h + 2), 1)

        # Labels
        self._ensure_fonts()

        if self.active_layer == "prob":
            non_zero = self.mod.prob[self.mod.prob > 0]
            vmax = float(np.percentile(non_zero, 99)) if len(non_zero) > 0 else 0.1
            top_lbl = f"{vmax:.3f}"
        else:
            top_lbl = "1.0"
        bot_lbl = "0.0"

        t_surf = self._font_small.render(top_lbl, True, (200, 200, 220))
        b_surf = self._font_small.render(bot_lbl, True, (200, 200, 220))
        screen.blit(t_surf, (x0 - t_surf.get_width() - 4, y0 - 2))
        screen.blit(b_surf, (x0 - b_surf.get_width() - 4, y0 + bar_h - 10))

        # Layer name
        name_surf = self._font_small.render(
            self._layer_labels.get(self.active_layer, ""), True, (200, 200, 220))
        screen.blit(name_surf, (x0 - name_surf.get_width() // 2,
                                y0 + bar_h + 6))

    def _draw_fov_wedge(self, screen, robot):
        """Draw a subtle FOV arc around a robot."""
        fov_deg = self.cfg_dict.get("sensor_fov_deg", 360.0)
        max_range_px = self.smap.meters_to_pixels(
            self.cfg_dict.get("sensor_max_range_m", 5.0))
        cx = int(robot.x + getattr(robot, 'lane_offset_x', 0))
        cy = int(robot.y + getattr(robot, 'lane_offset_y', 0))

        if fov_deg >= 359:
            # Full circle — draw subtle ring
            pygame.draw.circle(screen, (100, 200, 255, 40),
                               (cx, cy), int(max_range_px), 1)
        else:
            # Wedge
            half_fov = math.radians(fov_deg / 2.0)
            angle = getattr(robot, 'angle', 0.0)
            start = math.degrees(angle - half_fov)
            end = math.degrees(angle + half_fov)
            r = int(max_range_px)
            rect = pygame.Rect(cx - r, cy - r, 2 * r, 2 * r)
            try:
                pygame.draw.arc(screen, (100, 200, 255),
                                rect, -math.radians(end), -math.radians(start), 1)
            except Exception:
                pass  # arc can fail for extreme angles

    def _draw_marker(self, screen, x, y, color, cross=True, size=5):
        """Draw a small marker (cross or diamond) at (x, y)."""
        if cross:
            pygame.draw.line(screen, color, (x - size, y), (x + size, y), 2)
            pygame.draw.line(screen, color, (x, y - size), (x, y + size), 2)
        else:
            pts = [(x, y - size), (x + size, y), (x, y + size), (x - size, y)]
            pygame.draw.polygon(screen, color, pts, 2)

    # ─────────────────────────────────────────────────────────────────────
    # Snapshot export
    # ─────────────────────────────────────────────────────────────────────
    def export_snapshot(self, t, screen=None):
        """Save a PNG screenshot of the current display."""
        if screen is None:
            return

        os.makedirs(self._snapshot_dir, exist_ok=True)
        filename = os.path.join(
            self._snapshot_dir,
            f"mod_snapshot_{self._snapshot_count:04d}_t{t:.1f}s.png"
        )
        pygame.image.save(screen, filename)
        self._snapshot_count += 1
        self._last_snapshot_t = t
        print(f"[MoD] Snapshot saved: {filename}", flush=True)
