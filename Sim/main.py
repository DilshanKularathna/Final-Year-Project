"""
main.py — Simulation Entry Point
==================================
Initializes Pygame, spawns the warehouse / robots / human / coordinator,
runs the game loop (tick → update → render → telemetry), and handles
quit events.

MoD Integration: When MAP_OF_DYNAMICS["enabled"] is True, the main loop
also runs the visibility estimator, human detector, and MoD engine each
tick, and renders the risk overlay.
"""

import sys
import random
import pygame
from config import (
    WINDOW_WIDTH, WINDOW_HEIGHT, FPS, CELL_SIZE,
    NUM_ROBOTS, ROBOT_RADIUS,
    PROXEMIC_INNER_RADIUS, PROXEMIC_OUTER_RADIUS,
    COLOR_PROXEMIC_INNER, COLOR_PROXEMIC_OUTER,
    COLOR_TEXT, COLOR_TEXT_DIM,
    MAP_OF_DYNAMICS,
)
from environment import Warehouse
from human import HumanWorker
from robot import SmorphiRobot
from coordinator import Coordinator


def _draw_proxemic_zones(surface, human):
    """Draw translucent inner / outer proxemic circles around the human."""
    hx, hy = int(human.x), int(human.y)

    # Outer zone (orange, translucent)
    outer_surf = pygame.Surface((PROXEMIC_OUTER_RADIUS * 2,
                                 PROXEMIC_OUTER_RADIUS * 2),
                                pygame.SRCALPHA)
    pygame.draw.circle(outer_surf,
                       COLOR_PROXEMIC_OUTER,
                       (PROXEMIC_OUTER_RADIUS, PROXEMIC_OUTER_RADIUS),
                       PROXEMIC_OUTER_RADIUS)
    surface.blit(outer_surf,
                 (hx - PROXEMIC_OUTER_RADIUS, hy - PROXEMIC_OUTER_RADIUS))

    # Inner zone (red, translucent)
    inner_surf = pygame.Surface((PROXEMIC_INNER_RADIUS * 2,
                                 PROXEMIC_INNER_RADIUS * 2),
                                pygame.SRCALPHA)
    pygame.draw.circle(inner_surf,
                       COLOR_PROXEMIC_INNER,
                       (PROXEMIC_INNER_RADIUS, PROXEMIC_INNER_RADIUS),
                       PROXEMIC_INNER_RADIUS)
    surface.blit(inner_surf,
                 (hx - PROXEMIC_INNER_RADIUS, hy - PROXEMIC_INNER_RADIUS))


def _draw_hud(surface, clock, robots, humans, frame):
    """Render a small heads-up display with FPS, fleet state colors, and task telemetry."""
    font = pygame.font.SysFont("consolas", 12, bold=True)
    y = 6
    fps_lbl = font.render(f"FPS: {clock.get_fps():.0f} | Frame: {frame} | Fleet: {len(robots)} Robots, {len(humans)} Humans | MAPF: Prioritized & Left-Lane",
                          True, COLOR_TEXT_DIM)
    surface.blit(fps_lbl, (6, y))
    y += 16

    for r in robots:
        info = (f"R{r.id} [{r.state:>10s}]  "
                f"pos=({r.position[0]:.0f},{r.position[1]:.0f})  "
                f"task={r.task_label}")
        color = pygame.Color(*(__import__('config').STATE_COLORS.get(
            r.state, (200, 200, 200))))
        lbl = font.render(info, True, color)
        surface.blit(lbl, (6, y))
        y += 14

    for h in humans:
        h_info = (f"HUMAN H{h.id} pos=({h.x:.0f},{h.y:.0f})  "
                  f"vel=({h.vx:.1f},{h.vy:.1f})")
        lbl = font.render(h_info, True, h.color)
        surface.blit(lbl, (6, y))
        y += 14


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption(
        "Human-Aware Multi-Robot Warehouse Simulation — 4 Robots, 2 Humans")
    clock = pygame.time.Clock()

    # ── Create world ─────────────────────────────────────────────────────
    warehouse = Warehouse()
    num_humans = getattr(__import__('config'), 'NUM_HUMANS', 2)
    humans = [HumanWorker(warehouse, i + 1) for i in range(num_humans)]

    # Spawn robots on walkable cells along the bottom-centre area
    robots = []
    num_robots = getattr(__import__('config'), 'NUM_ROBOTS', 4)
    for i in range(num_robots):
        for _ in range(100):
            col = random.randint(3, __import__('config').GRID_COLS - 4)
            row = random.randint(1, 2)
            if warehouse.is_walkable(col, row):
                pos = warehouse.cell_center(col, row)
                robots.append(SmorphiRobot(i + 1, pos, warehouse))
                break

    coordinator = Coordinator(warehouse, robots, humans)

    # ── MoD initialisation (feature-flagged) ─────────────────────────────
    mod_enabled = MAP_OF_DYNAMICS.get("enabled", False)
    mod_engine = None
    mod_overlay = None
    mod_visibility = None
    mod_detector = None
    detections_by_robot = {}

    if mod_enabled:
        from mod.static_map import StaticMap
        from mod.mod_core import MapOfDynamics, MoDConfig
        from mod.visibility import VisibilityEstimator
        from mod.detection import HumanDetector
        from mod.overlay import MoDOverlay

        static_map = StaticMap(warehouse.grid)

        mod_cfg = MoDConfig.from_dict(MAP_OF_DYNAMICS)
        mod_engine = MapOfDynamics(static_map, mod_cfg)

        mod_visibility = VisibilityEstimator(
            static_map,
            fov_deg=MAP_OF_DYNAMICS.get("sensor_fov_deg", 360.0),
            max_range_m=MAP_OF_DYNAMICS.get("sensor_max_range_m", 5.0),
            n_rays=MAP_OF_DYNAMICS.get("sensor_n_rays", 180),
        )

        mod_detector = HumanDetector(
            static_map,
            mode=MAP_OF_DYNAMICS.get("detection_mode", "ground_truth"),
            fov_deg=MAP_OF_DYNAMICS.get("sensor_fov_deg", 360.0),
            max_range_m=MAP_OF_DYNAMICS.get("sensor_max_range_m", 5.0),
            human_radius_m=MAP_OF_DYNAMICS.get("human_radius_m", 0.35),
            detection_prob=MAP_OF_DYNAMICS.get("detection_prob", 1.0),
            pos_noise_m=MAP_OF_DYNAMICS.get("pos_noise_m", 0.0),
        )

        mod_overlay = MoDOverlay(mod_engine, static_map, MAP_OF_DYNAMICS)

        print("[MoD] Map of Dynamics ENABLED  "
              f"(α={mod_cfg.alpha}, β={mod_cfg.beta}, "
              f"λ={mod_cfg.lambd:.4f}, T½={mod_cfg.half_life_s}s)",
              flush=True)
    else:
        print("[MoD] Map of Dynamics DISABLED", flush=True)

    print("=" * 65)
    print("  WAREHOUSE SIMULATION STARTED")
    print(f"  Robots: {len(robots)}  |  Humans: {len(humans)}  |  Grid: {warehouse.grid.__len__()}x"
          f"{len(warehouse.grid[0])}  |  FPS target: {FPS}")
    print("=" * 65)

    # ── Main loop ────────────────────────────────────────────────────────
    frame = 0
    sim_time_s = 0.0  # simulation time in seconds
    dt = 1.0 / FPS    # time step per frame
    running = True

    while running:
        # ── events ───────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                # MoD keyboard controls
                elif mod_overlay is not None:
                    mod_overlay.handle_key(event.key, sim_time_s, screen)

        # ── update ───────────────────────────────────────────────────────
        for h in humans:
            h.update()
        coordinator.update()
        for robot in robots:
            robot.update()

        # ── MoD update (feature-flagged) ─────────────────────────────────
        if mod_enabled and mod_engine is not None:
            detections_by_robot = {}
            for robot in robots:
                # Robot pose (with optional noise)
                rx, ry = robot.x, robot.y
                r_angle = robot.angle
                pose_noise = MAP_OF_DYNAMICS.get("pose_noise_m", 0.0)
                if pose_noise > 0:
                    from mod.static_map import StaticMap as _SM
                    noise_px = _SM(warehouse.grid).meters_to_pixels(pose_noise)
                    rx += random.gauss(0, noise_px)
                    ry += random.gauss(0, noise_px)

                # Visibility
                vis_mask = mod_visibility.visible_cells(rx, ry, r_angle)

                # Detection
                dets, human_mask = mod_detector.detect(
                    rx, ry, r_angle, humans, sim_time_s,
                    visible_mask=vis_mask)

                detections_by_robot[robot.id] = dets

                # Integrate into MoD
                mod_engine.integrate(robot.id, vis_mask, human_mask, sim_time_s)

            # MoD step (window close, risk recomputation)
            mod_engine.step(sim_time_s)

        # ── render ───────────────────────────────────────────────────────
        warehouse.draw(screen)

        # MoD overlay (between warehouse and agents)
        if mod_overlay is not None:
            mod_overlay.draw(screen, sim_time_s, robots, humans,
                             detections_by_robot)

        for h in humans:
            _draw_proxemic_zones(screen, h)
            h.draw(screen)
        for robot in robots:
            robot.draw(screen)
        _draw_hud(screen, clock, robots, humans, frame)

        pygame.display.flip()
        clock.tick(FPS)
        frame += 1
        sim_time_s += dt

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
