"""Mode controller -- the server-side state machine (F-17).

Owns the single "behaviour worker" thread and enforces the valid mode transitions
(web_interface_design.md). Each transition cleanly stops the running behaviour before
starting the next, so only one behaviour ever drives the motors. The web server calls
``start_idle`` / ``start_explore`` / ``navigate_to``; navigation and exploration share
listener hooks (``on_path`` / ``on_mode``) that the server wires to WebSocket emits.
"""

from __future__ import annotations

import threading

from robot_car import state
from robot_car.hardware import motors
from robot_car.modes.explore import Explorer
from robot_car.modes.idle import Idle
from robot_car.modes.navigate import Navigator, REACHED, NO_PATH


class InvalidTransition(Exception):
    pass


class ModeController:
    VALID = {
        "idle": {"explore", "navigate"},
        "explore": {"idle", "navigate"},
        "navigate": {"idle", "explore", "navigate"},
    }

    def __init__(self, context):
        self.ctx = context
        self.mode = "idle"
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.on_path = None        # callback(list_of_(col,row))
        self.on_mode = None        # callback(mode_str)

    # -- public API ----------------------------------------------------------
    def start_idle(self) -> None:
        self._switch("idle", lambda stop: Idle(self.ctx).run(stop))

    def start_explore(self, save_name: str = "explored") -> None:
        self._require_transition("explore")
        explorer = Explorer(self.ctx, save_name=save_name)
        explorer.navigator.path_listener = self.on_path
        self._switch("explore", explorer.run)

    def navigate_to(self, goal_cell) -> None:
        self._require_transition("navigate")
        if state.get_grid() is None:
            raise InvalidTransition("no map loaded")
        navigator = Navigator(self.ctx)
        navigator.path_listener = self.on_path

        def run(stop):
            status = navigator.run(goal_cell, stop)
            # On completion, settle into idle without re-entering _switch (which would
            # try to join this very thread).
            if status in (REACHED, NO_PATH) and not stop.is_set():
                self.ctx.slam.set_mapping(False)
                self._mark_mode("idle")
                Idle(self.ctx).run(stop)

        self._switch("navigate", run)

    def stop(self) -> None:
        with self._lock:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            self._thread = None
        motors.stop()

    # -- internals -----------------------------------------------------------
    def _require_transition(self, target: str) -> None:
        if target != self.mode and target not in self.VALID.get(self.mode, set()):
            raise InvalidTransition("%s -> %s is not allowed" % (self.mode, target))

    def _switch(self, name: str, target) -> None:
        with self._lock:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            self._stop = threading.Event()
            stop = self._stop
            self._mark_mode(name)
            self._thread = threading.Thread(
                target=target, args=(stop,), name="mode-%s" % name, daemon=True
            )
            self._thread.start()

    def _mark_mode(self, name: str) -> None:
        self.mode = name
        state.set_mode(name)
        if self.on_mode:
            try:
                self.on_mode(name)
            except Exception:
                pass
