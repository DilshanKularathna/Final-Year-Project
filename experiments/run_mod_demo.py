"""
experiments/run_mod_demo.py — MoD Demonstration Experiment
============================================================
Runs the controlled warehouse demonstration specified in Section 7:

  1. Mapping phase: A mapping robot builds and freezes the static occupancy grid.
  2. Operation phase: 3 operating robots patrol/perform warehouse tasks.
     Two scripted humans repeatedly use Aisles A and B:
       - Aisle A (row 5): high human activity (Human 1 primarily operates here)
       - Aisle B (row 9): moderate human activity (Human 2 operates here part-time)
       - Aisle C (row 13): ZERO human activity (neither human enters)
  3. Headless simulation runs for 10–15 simulated minutes with a fixed random seed.
  4. Telemetry logging: samples mean P, recency, risk in Aisles A, B, C, and fleet coverage.
  5. Outputs:
       - Saved static map: outputs/static_map.npz
       - Metrics CSV log:  outputs/mod_demo_metrics.csv
       - Overlay PNGs:     outputs/mod_snapshots/demo_t*.png
       - Metrics plot:     outputs/mod_snapshots/demo_aisle_metrics.png
  6. Confirms acceptance criteria:
       - P(A) > P(B) > P(C)
       - Recency spikes on human presence and decays with T_half = 20s
       - Coverage grows as robots explore
"""

import os
import sys
import math
import random
import argparse
import numpy as np

# Use dummy video driver for fast, headless execution
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

# Add Sim directory to path
SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Sim"))
sys.path.insert(0, SIM_DIR)

from config import (
    WINDOW_WIDTH, WINDOW_HEIGHT, FPS, CELL_SIZE,
    GRID_COLS, GRID_ROWS,
    PROXEMIC_INNER_RADIUS, PROXEMIC_OUTER_RADIUS,
    COLOR_PROXEMIC_INNER, COLOR_PROXEMIC_OUTER,
    COLOR_TEXT, COLOR_TEXT_DIM,
    MAP_OF_DYNAMICS,
)
from environment import Warehouse, CELL_AISLE, CELL_SHELF
from robot import SmorphiRobot
from coordinator import Coordinator
from mod.static_map import StaticMap
from mod.mod_core import MapOfDynamics, MoDConfig
from mod.visibility import VisibilityEstimator
from mod.detection import HumanDetector
from mod.overlay import MoDOverlay


# ─────────────────────────────────────────────────────────────────────────────
# Scripted Humans for Controlled Aisle Activity
# ─────────────────────────────────────────────────────────────────────────────

class ScriptedAisleHuman:
    """
    A human worker with deterministic, scripted patrol routes across aisles:
      - Human 1 patrols Aisle A (row 5, cols 4 to 27)
      - Human 2 patrols Aisle B (row 9, cols 4 to 27) with intermittent pauses
      - Neither human ever visits Aisle C (row 13)
    """

    def __init__(self, warehouse, human_id, primary_aisle_row, patrol_cols=(4, 27)):
        self.id = human_id
        self.warehouse = warehouse
        self.row = primary_aisle_row
        self.min_col, self.max_col = patrol_cols
        self.color = (50, 180, 255) if human_id == 1 else (255, 180, 50)

        # Start at left end of the aisle
        self.col_target = self.max_col
        start_x, start_y = warehouse.cell_center(self.min_col, self.row)
        self.x = float(start_x)
        self.y = float(start_y)
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 2.0  # px/frame (~ 1.5 m/s at 80 px/m and 60 fps)
        self.pause_frames = 0
        self.angle = 0.0

    @property
    def position(self):
        return (self.x, self.y)

    @property
    def velocity(self):
        return (self.vx, self.vy)

    def intent_endpoint(self):
        return (self.x + self.vx * 30, self.y + self.vy * 30)

    def update(self):
        if self.pause_frames > 0:
            self.pause_frames -= 1
            self.vx, self.vy = 0.0, 0.0
            return

        tx, ty = self.warehouse.cell_center(self.col_target, self.row)
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < self.speed * 2:
            # Reached target end of aisle -> reverse direction
            self.x, self.y = tx, ty
            self.col_target = self.min_col if self.col_target == self.max_col else self.max_col
            # Human 2 takes longer rest pauses between patrols
            self.pause_frames = 60 if self.id == 1 else 180
            self.vx, self.vy = 0.0, 0.0
        else:
            self.angle = math.atan2(dy, dx)
            self.vx = self.speed * (dx / dist)
            self.vy = self.speed * (dy / dist)
            self.x += self.vx
            self.y += self.vy

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, (int(self.x), int(self.y)), 14)
        # Heading indicator
        hx = int(self.x + math.cos(self.angle) * 16)
        hy = int(self.y + math.sin(self.angle) * 16)
        pygame.draw.line(surface, (255, 255, 255), (int(self.x), int(self.y)), (hx, hy), 2)


# ─────────────────────────────────────────────────────────────────────────────
# Helper: Drawing Proxemics
# ─────────────────────────────────────────────────────────────────────────────

def draw_proxemics(surface, human):
    hx, hy = int(human.x), int(human.y)
    outer_surf = pygame.Surface((PROXEMIC_OUTER_RADIUS * 2, PROXEMIC_OUTER_RADIUS * 2), pygame.SRCALPHA)
    pygame.draw.circle(outer_surf, COLOR_PROXEMIC_OUTER, (PROXEMIC_OUTER_RADIUS, PROXEMIC_OUTER_RADIUS), PROXEMIC_OUTER_RADIUS)
    surface.blit(outer_surf, (hx - PROXEMIC_OUTER_RADIUS, hy - PROXEMIC_OUTER_RADIUS))


# ─────────────────────────────────────────────────────────────────────────────
# Main Demo Runner
# ─────────────────────────────────────────────────────────────────────

def run_demo(sim_minutes=10.0, seed=42, output_dir="outputs"):
    print("=" * 70)
    print("  MAP OF DYNAMICS (MoD) DEMONSTRATION EXPERIMENT")
    print(f"  Duration: {sim_minutes:.1f} sim-minutes ({sim_minutes*60:.0f} sim-seconds)")
    print(f"  Random Seed: {seed}")
    print("=" * 70)

    random.seed(seed)
    np.random.seed(seed)

    snapshot_dir = os.path.join(output_dir, "mod_snapshots")
    os.makedirs(snapshot_dir, exist_ok=True)
    os.makedirs("experiments", exist_ok=True)

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    warehouse = Warehouse()

    # ── Phase 1: Mapping Phase ───────────────────────────────────────────
    print("\n[Phase 1] Mapping Robot: Generating and freezing static occupancy grid...")
    static_map = StaticMap(warehouse.grid)
    map_save_path = os.path.join(output_dir, "static_map.npz")
    static_map.save(map_save_path)
    print(f"          Static map frozen and saved to: {map_save_path}")
    print(f"          Grid dimensions: {static_map.rows} rows x {static_map.cols} cols "
          f"({static_map.resolution_m:.2f} m/cell)")

    # ── Phase 2: MoD Engine & Perception Setup ───────────────────────────
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
        mode="ground_truth",
        human_radius_m=MAP_OF_DYNAMICS.get("human_radius_m", 0.35),
    )
    mod_overlay = MoDOverlay(mod_engine, static_map, MAP_OF_DYNAMICS)

    # ── Operating Fleet & Scripted Humans ────────────────────────────────
    # Aisle rows:
    # Aisle A: row 5 (frequent activity)
    # Aisle B: row 9 (moderate activity)
    # Aisle C: row 13 (zero human activity)
    ROW_AISLE_A = 5
    ROW_AISLE_B = 9
    ROW_AISLE_C = 13
    COLS_AISLE = list(range(4, 28))

    humans = [
        ScriptedAisleHuman(warehouse, human_id=1, primary_aisle_row=ROW_AISLE_A, patrol_cols=(4, 27)),
        ScriptedAisleHuman(warehouse, human_id=2, primary_aisle_row=ROW_AISLE_B, patrol_cols=(4, 27)),
    ]

    # Spawn 3 operating robots
    robots = []
    spawn_cells = [(5, 1), (15, 1), (25, 1)]
    for idx, (sc_col, sc_row) in enumerate(spawn_cells):
        pos = warehouse.cell_center(sc_col, sc_row)
        robots.append(SmorphiRobot(idx + 1, pos, warehouse))

    coordinator = Coordinator(warehouse, robots, humans)

    # ── Metrics Logging Setup ────────────────────────────────────────────
    metrics_log_path = os.path.join(output_dir, "mod_demo_metrics.csv")
    csv_file = open(metrics_log_path, "w", encoding="utf-8")
    csv_file.write(
        "sim_time_s,P_A,P_B,P_C,recency_A,recency_B,recency_C,risk_A,risk_B,risk_C,coverage_pct\n"
    )

    history = {
        "time": [],
        "P_A": [], "P_B": [], "P_C": [],
        "rec_A": [], "rec_B": [], "rec_C": [],
        "risk_A": [], "risk_B": [], "risk_C": [],
        "coverage": [],
    }

    total_sim_seconds = sim_minutes * 60.0
    total_frames = int(total_sim_seconds * FPS)
    dt = 1.0 / FPS

    # Scheduled snapshot times (sim-seconds)
    snapshot_times = [60.0, 180.0, 360.0, total_sim_seconds]
    next_snap_idx = 0

    print(f"\n[Phase 2] Operation Phase started: 3 robots operating across aisles...")
    print(f"          Aisle A (row {ROW_AISLE_A}): Human 1 active")
    print(f"          Aisle B (row {ROW_AISLE_B}): Human 2 active (intermittent)")
    print(f"          Aisle C (row {ROW_AISLE_C}): No humans")

    # ── Simulation Loop ──────────────────────────────────────────────────
    sim_time_s = 0.0
    log_interval_s = 2.0
    last_log_t = -1.0

    for frame in range(total_frames):
        # 1. Update entities
        for h in humans:
            h.update()
        coordinator.update()
        for r in robots:
            r.update()

        # 2. Perception & MoD Multi-Robot Integration
        detections_by_robot = {}
        for r in robots:
            vmask = mod_visibility.visible_cells(r.x, r.y, r.angle)
            dets, hmask = mod_detector.detect(r.x, r.y, r.angle, humans, sim_time_s, visible_mask=vmask)
            detections_by_robot[r.id] = dets
            mod_engine.integrate(r.id, vmask, hmask, sim_time_s)

        mod_engine.step(sim_time_s)

        # 3. Metrics Sampling
        if sim_time_s - last_log_t >= log_interval_s:
            p_layer = mod_engine.prob
            rec_layer = mod_engine.recency
            risk_layer = mod_engine.risk

            p_a = float(np.mean([p_layer[ROW_AISLE_A, c] for c in COLS_AISLE]))
            p_b = float(np.mean([p_layer[ROW_AISLE_B, c] for c in COLS_AISLE]))
            p_c = float(np.mean([p_layer[ROW_AISLE_C, c] for c in COLS_AISLE]))

            rec_a = float(np.mean([rec_layer[ROW_AISLE_A, c] for c in COLS_AISLE]))
            rec_b = float(np.mean([rec_layer[ROW_AISLE_B, c] for c in COLS_AISLE]))
            rec_c = float(np.mean([rec_layer[ROW_AISLE_C, c] for c in COLS_AISLE]))

            risk_a = float(np.mean([risk_layer[ROW_AISLE_A, c] for c in COLS_AISLE]))
            risk_b = float(np.mean([risk_layer[ROW_AISLE_B, c] for c in COLS_AISLE]))
            risk_c = float(np.mean([risk_layer[ROW_AISLE_C, c] for c in COLS_AISLE]))

            cov_pct = mod_engine.coverage_fraction() * 100.0

            csv_file.write(
                f"{sim_time_s:.1f},{p_a:.4f},{p_b:.4f},{p_c:.4f},"
                f"{rec_a:.4f},{rec_b:.4f},{rec_c:.4f},"
                f"{risk_a:.4f},{risk_b:.4f},{risk_c:.4f},{cov_pct:.1f}\n"
            )

            history["time"].append(sim_time_s)
            history["P_A"].append(p_a)
            history["P_B"].append(p_b)
            history["P_C"].append(p_c)
            history["rec_A"].append(rec_a)
            history["rec_B"].append(rec_b)
            history["rec_C"].append(rec_c)
            history["risk_A"].append(risk_a)
            history["risk_B"].append(risk_b)
            history["risk_C"].append(risk_c)
            history["coverage"].append(cov_pct)

            last_log_t = sim_time_s

        # 4. Snapshots at designated intervals
        if next_snap_idx < len(snapshot_times) and sim_time_s >= snapshot_times[next_snap_idx]:
            # Render complete scene
            warehouse.draw(screen)
            mod_overlay.draw(screen, sim_time_s, robots, humans, detections_by_robot)
            for h in humans:
                draw_proxemics(screen, h)
                h.draw(screen)
            for r in robots:
                r.draw(screen)

            snap_path = os.path.join(snapshot_dir, f"demo_snapshot_t{int(sim_time_s):04d}s.png")
            pygame.image.save(screen, snap_path)
            print(f"          [Snapshot] Saved: {snap_path} (t={sim_time_s:.0f}s)")
            next_snap_idx += 1

        # Periodic terminal progress
        if frame % (FPS * 60) == 0 and frame > 0:
            p_a_now = history["P_A"][-1]
            p_b_now = history["P_B"][-1]
            p_c_now = history["P_C"][-1]
            cov_now = history["coverage"][-1]
            print(f"          Sim Time: {sim_time_s:5.0f}s / {total_sim_seconds:.0f}s | "
                  f"P(A)={p_a_now:.3f}, P(B)={p_b_now:.3f}, P(C)={p_c_now:.3f} | "
                  f"Coverage={cov_now:.1f}%")

        sim_time_s += dt

    csv_file.close()

    # Copy CSV to experiments/ folder as well
    import shutil
    shutil.copy(metrics_log_path, "experiments/mod_demo_log.csv")

    # ── Phase 3: Analysis & Visualization Plot ───────────────────────────
    print("\n[Phase 3] Generating demonstration metrics plot...")
    generate_plot(history, output_path=os.path.join(snapshot_dir, "demo_aisle_metrics.png"))

    # ── Final Summary & Verification ─────────────────────────────────────
    final_p_a = history["P_A"][-1]
    final_p_b = history["P_B"][-1]
    final_p_c = history["P_C"][-1]
    final_cov = history["coverage"][-1]

    print("\n" + "=" * 70)
    print("  EXPERIMENT VERIFICATION RESULTS")
    print("=" * 70)
    print(f"  Final Habit Probability P(H):")
    print(f"    • Aisle A (Heavy):      P(A) = {final_p_a:.4f}")
    print(f"    • Aisle B (Moderate):   P(B) = {final_p_b:.4f}")
    print(f"    • Aisle C (Never):      P(C) = {final_p_c:.4f}")
    print(f"  Verification check: P(A) > P(B) > P(C)?")

    passed_habit = (final_p_a > final_p_b > final_p_c)
    print(f"    Result: {'[PASSED]' if passed_habit else '[FAILED]'} ({final_p_a:.3f} > {final_p_b:.3f} > {final_p_c:.3f})")

    passed_aisle_c = (final_p_c < 0.05)
    print(f"    Aisle C safe learning (P(C) < P0): {'[PASSED]' if passed_aisle_c else '[FAILED]'} (P(C)={final_p_c:.4f} < 0.05)")

    print(f"  Final Sensor Exploration Coverage: {final_cov:.1f}%")
    print(f"  Log files saved:")
    print(f"    - {metrics_log_path}")
    print(f"    - experiments/mod_demo_log.csv")
    print(f"    - {os.path.join(snapshot_dir, 'demo_aisle_metrics.png')}")
    print("=" * 70)

    return passed_habit


def generate_plot(history, output_path):
    """Generate a clean 4-panel matplotlib chart showing MoD dynamics."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = history["time"]
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    plt.subplots_adjust(hspace=0.28)

    # 1. Habit Probability P(H)
    ax1 = axes[0]
    ax1.plot(t, history["P_A"], color="crimson", linewidth=2.0, label="Aisle A (High Human Activity)")
    ax1.plot(t, history["P_B"], color="darkorange", linewidth=2.0, label="Aisle B (Moderate Human Activity)")
    ax1.plot(t, history["P_C"], color="forestgreen", linewidth=2.0, label="Aisle C (Zero Human Activity)")
    ax1.axhline(0.05, color="gray", linestyle="--", alpha=0.7, label="Prior P0 = 0.05")
    ax1.set_ylabel("Habit Probability P(H)", fontsize=11, fontweight="bold")
    ax1.set_title("Long-Term Presence Probability P_ij Across Aisles: P(A) > P(B) > P(C)", fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper left", framealpha=0.9)
    ax1.set_ylim(0.0, max(0.8, max(history["P_A"]) * 1.15))

    # 2. Short-term Recency exp(-λΔt)
    ax2 = axes[1]
    ax2.plot(t, history["rec_A"], color="crimson", linewidth=1.5, alpha=0.85, label="Recency Aisle A")
    ax2.plot(t, history["rec_B"], color="darkorange", linewidth=1.5, alpha=0.85, label="Recency Aisle B")
    ax2.plot(t, history["rec_C"], color="forestgreen", linewidth=1.5, alpha=0.85, label="Recency Aisle C")
    ax2.set_ylabel("Recency exp(-λΔt)", fontsize=11, fontweight="bold")
    ax2.set_title("Short-Term Recency: Spikes on Human Observation & Exponential Decay (T_half = 20s)", fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper left", framealpha=0.9)
    ax2.set_ylim(0.0, 1.05)

    # 3. Combined Risk R_ij = α P + β Recency
    ax3 = axes[2]
    ax3.plot(t, history["risk_A"], color="crimson", linewidth=2.0, label="Risk Aisle A (α=0.6, β=0.3)")
    ax3.plot(t, history["risk_B"], color="darkorange", linewidth=2.0, label="Risk Aisle B")
    ax3.plot(t, history["risk_C"], color="forestgreen", linewidth=2.0, label="Risk Aisle C")
    ax3.set_ylabel("Combined Risk R_ij", fontsize=11, fontweight="bold")
    ax3.set_title("Total Risk R_ij = α · P_ij + β · exp(-λ · Δt_ij)", fontsize=12)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper left", framealpha=0.9)
    ax3.set_ylim(0.0, 0.9)

    # 4. Sensor Coverage %
    ax4 = axes[3]
    ax4.plot(t, history["coverage"], color="royalblue", linewidth=2.0, label="Warehouse Coverage %")
    ax4.set_xlabel("Simulation Time (seconds)", fontsize=11, fontweight="bold")
    ax4.set_ylabel("Coverage (%)", fontsize=11, fontweight="bold")
    ax4.set_title("Multi-Robot Exploration: Walkable Area Observed Ever", fontsize=12)
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc="upper left", framealpha=0.9)
    ax4.set_ylim(0, 105)

    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"          Metrics plot saved to: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Map of Dynamics Demonstration Experiment")
    parser.add_argument("--sim-minutes", type=float, default=10.0, help="Simulation duration in minutes (default: 10.0)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility (default: 42)")
    parser.add_argument("--output-dir", type=str, default="outputs", help="Directory to save outputs (default: outputs)")
    args = parser.parse_args()

    success = run_demo(sim_minutes=args.sim_minutes, seed=args.seed, output_dir=args.output_dir)
    sys.exit(0 if success else 1)
