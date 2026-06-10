"""Navigate mode (F-15) -- plan with A* and execute the route to a goal cell.

Route execution is a single control step per loop iteration (rotate-in-place to align,
then drive forward with proportional steering), so the loop stays responsive to the
safety ``blocked`` flag and to shutdown. Replanning policy (F-12): when blocked, wait
and retry; after ``REPLAN_RETRY_LIMIT`` retries, run a full A* from the current pose.
"""

from __future__ import annotations

import math
import time

from robot_car import config, state
from robot_car.core import path_planner as pp
from robot_car.hardware import motors

# Run status codes.
REACHED = "reached"
NO_PATH = "no_path"
ABORTED = "aborted"


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Navigator:
    def __init__(self, context):
        self.ctx = context
        self.path = []                  # current planned path, (col, row) cells
        self.path_listener = None       # callback(list_of_(col,row)) for the UI

    # -- planning ------------------------------------------------------------
    def plan(self, goal_cell):
        grid = self.ctx.planning_grid()
        start = self.ctx.current_cell()
        path = pp.plan_path(grid, start, goal_cell, self.ctx.forbidden.mask)
        if path:
            self.path = path
            self._emit_path(path)
        return path

    def _emit_path(self, path):
        if self.path_listener:
            try:
                self.path_listener(path)
            except Exception:
                pass

    # -- execution -----------------------------------------------------------
    def run(self, goal_cell, stop_event) -> str:
        if self.plan(goal_cell) is None:
            state.set_log("error", "No path to destination")
            motors.stop()
            return NO_PATH

        gx, gy = self.ctx.grid_to_world(*goal_cell)
        period = 1.0 / config.CONTROL_HZ
        idx = 1
        retry = 0
        # Stall watchdog: remember the closest we have ever been to the goal and
        # when we last made measurable progress. A goal the robot can never reach
        # (or pose jitter orbiting it) would otherwise loop here forever.
        best_dist = float("inf")
        last_progress = time.monotonic()

        while not stop_event.is_set() and not state.stop_event.is_set():
            x, y, _ = state.get_pose()
            dist = math.hypot(gx - x, gy - y)
            if dist < config.ARRIVAL_THRESHOLD:
                motors.stop()
                state.set_log("ok", "Destination reached")
                return REACHED

            now = time.monotonic()
            if dist < best_dist - config.NAV_PROGRESS_EPSILON:
                best_dist = dist
                last_progress = now
            elif now - last_progress > config.NAV_PROGRESS_TIMEOUT_S:
                motors.stop()
                state.set_log("warn", "No progress toward goal -- abandoning")
                return NO_PATH

            if state.is_blocked():
                motors.stop()
                retry += 1
                if retry >= config.REPLAN_RETRY_LIMIT:
                    state.set_log("warn", "Obstacle persists -- replanning")
                    if self.plan(goal_cell) is None:
                        return NO_PATH
                    gx, gy = self.ctx.grid_to_world(*goal_cell)
                    idx, retry = 1, 0
                stop_event.wait(config.REPLAN_WAIT_S)
                continue
            retry = 0

            idx = min(idx, len(self.path) - 1)
            if self._step_toward(self.path[idx]) == REACHED:
                idx += 1
            stop_event.wait(period)

        motors.stop()
        return ABORTED

    def _step_toward(self, cell) -> str:
        """One control step toward a waypoint cell. Returns REACHED or 'moving'."""
        tx, ty = self.ctx.grid_to_world(*cell)
        x, y, theta = state.get_pose()
        dx, dy = tx - x, ty - y
        if math.hypot(dx, dy) < config.ARRIVAL_THRESHOLD:
            return REACHED

        err = _wrap(math.atan2(dy, dx) - theta)
        if abs(err) > config.HEADING_TOLERANCE:
            turn = config.TURN_SPEED * (1.0 if err > 0 else -1.0)
            motors.set_speed(-turn, turn)            # rotate in place (+err => CCW)
        else:
            steer = max(-1.0, min(1.0, config.STEER_GAIN * err))
            left = config.DRIVE_SPEED * (1.0 - steer)
            right = config.DRIVE_SPEED * (1.0 + steer)
            motors.set_speed(left, right)
        return "moving"
