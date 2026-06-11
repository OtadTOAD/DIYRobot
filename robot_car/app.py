"""Application wiring and startup sequence (F-20).

Brings the whole robot up in the right order and tears it down cleanly:

    1. select + start the hardware backend (real GPIO or simulator)
    2. start the camera daemon (capture + VO + appearance + debug stream)
    3. start the safety monitor daemon
    4. start the SLAM / localization daemon
    5. build the web server (HTTP + SocketIO) and the pose broadcaster
    6. enter the requested initial mode (idle / explore / navigate)

On SIGINT the global stop event is set, the mode worker is stopped, motors are
stopped and GPIO / the simulator are released via gpio_cleanup.
"""

from __future__ import annotations

import signal

from robot_car import config, state
from robot_car.context import RobotContext
from robot_car.controller import ModeController
from robot_car.hardware import camera, gpio_cleanup, hal, platform_detect, sensor_scheduler
from robot_car.core.safety_monitor import start_safety
from robot_car.core.watchdog import start_watchdog
from robot_car.ui.server import create_server


def build(context: RobotContext | None = None):
    """Construct the context, controller and web server (no threads started)."""
    context = context or RobotContext()
    controller = ModeController(context)
    app, socketio = create_server(context, controller)
    return context, controller, app, socketio


def run(explore: bool = False, map_name: str | None = None,
        waypoint: str | None = None) -> None:
    state.stop_event.clear()
    gpio_cleanup.register()
    print("[app] backend = %s" % platform_detect.ACTIVE_BACKEND)

    backend = hal.get_backend()
    backend.start()
    sensor_scheduler.start_scheduler()   # single owner of the ultrasonic bus

    context, controller, app, socketio = build()

    camera.start_camera()
    start_safety()
    start_watchdog()
    context.slam.start()
    app._start_background()             # pose broadcaster

    def shutdown(*_):
        state.set_log("warn", "Shutting down")
        state.stop_event.set()
        controller.stop()
        gpio_cleanup.cleanup()

    signal.signal(signal.SIGINT, lambda *a: (shutdown(), _exit()))
    signal.signal(signal.SIGTERM, lambda *a: (shutdown(), _exit()))

    # Initial mode.
    if map_name:
        context.load_map(map_name)
    if explore:
        controller.start_explore()
    elif waypoint:
        cell = context.slam.grid.world_to_grid(*context.waypoints[waypoint]) \
            if waypoint in context.waypoints else None
        if cell is not None:
            controller.navigate_to(cell)
        else:
            state.set_log("error", "Unknown waypoint '%s'" % waypoint)
            controller.start_idle()
    else:
        controller.start_idle()

    print("[app] web UI on http://%s:%d" % (config.WEB_HOST, config.WEB_PORT))
    try:
        socketio.run(app, host=config.WEB_HOST, port=config.WEB_PORT,
                     allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        shutdown()


def _exit():
    import os
    os._exit(0)
