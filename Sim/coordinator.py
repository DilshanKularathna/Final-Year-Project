"""
coordinator.py — Central Decision-Making Brain (Prioritized MAPF Engine)
========================================================================
Manages prioritized fleet right-of-way (R1 > R2 > R3 > R4), human proxemic safety
yield enforcement, and structured task allocation with 1-second dwell timers.
"""

import math
import random
from config import (
    PROXEMIC_INNER_RADIUS, PROXEMIC_OUTER_RADIUS,
    STATE_IDLE, STATE_ALLOCATING, STATE_MOVING, STATE_COMPLETED, STATE_YIELDING, STATE_REROUTING,
    CELL_SIZE, GRID_COLS, GRID_ROWS,
)


class Coordinator:
    """
    Central Coordinator using Prioritized Planning MAPF rules.
    """

    def __init__(self, warehouse, robots, humans):
        self.warehouse = warehouse
        self.robots = robots
        self.humans = humans if isinstance(humans, list) else [humans]
        self.frame = 0

    # ── Main per-frame update loop ────────────────────────────────────────
    def update(self):
        self.frame += 1

        for robot in self.robots:
            # 1. Task allocation for IDLE robots
            if robot.state == STATE_IDLE and self.frame % 30 == 0:
                self._assign_task(robot)
                continue

            # Skip checking proxemics if robot is paused during ALLOCATING or COMPLETED phases
            if robot.state in (STATE_ALLOCATING, STATE_COMPLETED):
                continue

            # 2. Human Worker Proxemic Safety Check (Inner circle = 60px)
            human_near = False
            for human in self.humans:
                dist = robot.distance_to(*human.position)
                if dist < PROXEMIC_INNER_RADIUS:
                    human_near = True
                    if robot.state != STATE_YIELDING:
                        robot.set_state(STATE_YIELDING)
                        self._log(robot, STATE_YIELDING, f"Human H{human.id} in proxemic zone (d={dist:.0f}px)")
                    break

            if human_near:
                continue

            # 3. Inter-Robot Prioritized Avoidance (R1 > R2 > R3 > R4)
            robot_near = False
            for other in self.robots:
                if other.id == robot.id:
                    continue
                # Lower priority robot yields to higher priority robot
                if robot.id > other.id:
                    r_dist = robot.distance_to(*other.position)
                    if r_dist < 36.0:  # 2 * ROBOT_RADIUS (28) + margin (8)
                        robot_near = True
                        if robot.state != STATE_YIELDING:
                            robot.set_state(STATE_YIELDING)
                            self._log(robot, STATE_YIELDING, f"Yielding to higher-priority Robot R{other.id}")
                        break

            if robot_near:
                continue

            # 4. Safe distance resumption — resume MOVING if path is clear
            if robot.state == STATE_YIELDING and not human_near and not robot_near:
                robot.set_state(STATE_MOVING)
                self._log(robot, STATE_MOVING, "Path clear — resuming task")

    # ── Task Allocation ──────────────────────────────────────────────────
    def _assign_task(self, robot):
        if not self.warehouse.pickup_points or not self.warehouse.dropoff_points:
            return

        pickup = random.choice(self.warehouse.pickup_points)
        dropoff = random.choice(self.warehouse.dropoff_points)

        pickup_px = self.warehouse.cell_center(*pickup)
        dropoff_px = self.warehouse.cell_center(*dropoff)

        # Plan ARA* path: current pos -> pickup -> dropoff
        path_a = self.warehouse.find_path_ara(robot.position, pickup_px)
        path_b = self.warehouse.find_path_ara(pickup_px, dropoff_px)

        if path_a and path_b:
            label = f"PICK({pickup})->DROP({dropoff})"
            robot.assign_task(path_a, path_b, label)
            self._log(robot, STATE_ALLOCATING, f"Task assigned: {label} (1s ALLOCATING pause)")

    # ── Telemetry ────────────────────────────────────────────────────────
    def _log(self, robot, new_state, reason):
        print(f"[COORDINATOR] Robot R{robot.id}  "
              f"{robot.prev_state:>10s} -> {new_state:<10s} | {reason}",
              flush=True)
