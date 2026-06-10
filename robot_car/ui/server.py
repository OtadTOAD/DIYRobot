"""Flask + SocketIO web server (F-16, F-17).

Exposes the HTTP API and the WebSocket event stream defined in
web_interface_design.md, and wires the core threads' outputs to the browser:

  * status-log messages (state.add_log_listener)   -> 'status_log'
  * planned paths (controller.on_path)             -> 'path_update'
  * mode changes (controller.on_mode)              -> 'mode_change'
  * incremental grid changes (slam.cell_listener)  -> 'cell_update'
  * a background task pushes 'pose_update' at a fixed rate.

It also serves the annotated camera frames as an MJPEG stream at
``/camera/debug.mjpg`` so the visual-odometry / appearance pipelines are observable
from any browser, headless. SocketIO runs in ``threading`` mode so it coexists with
the hardware callback / SLAM / safety threads without monkey-patching.
"""

from __future__ import annotations

import base64
import os
import time

import cv2
from flask import Flask, Response, jsonify, request, send_from_directory
from flask_socketio import SocketIO, emit as ws_emit

from robot_car import config, state
from robot_car.core import safety_monitor
from robot_car.controller import InvalidTransition

_STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


def create_server(context, controller):
    app = Flask(__name__, static_folder=_STATIC, static_url_path="/static")
    socketio = SocketIO(app, async_mode="threading", cors_allowed_origins="*")

    grid = context.slam.grid

    # ----- wiring core -> websocket ----------------------------------------
    state.add_log_listener(
        lambda level, msg: socketio.emit("status_log", {"level": level, "msg": msg})
    )
    controller.on_path = lambda path: socketio.emit(
        "path_update", [{"x": c, "y": r} for c, r in path]
    )
    controller.on_mode = lambda mode: socketio.emit("mode_change", {"mode": mode})
    context.slam.cell_listener = lambda updates: socketio.emit(
        "cell_update", [{"x": c, "y": r, "value": v} for c, r, v in updates]
    )

    # ----- HTTP: page + map images -----------------------------------------
    @app.route("/")
    def index():
        return send_from_directory(_STATIC, "index.html")

    @app.route("/map.png")
    def map_png():
        with context.slam._lock:
            png = context.slam.grid.to_png_bytes(inflated=False)
        return Response(png, mimetype="image/png")

    @app.route("/map/export")
    def map_export():
        with context.slam._lock:
            png = context.slam.grid.to_png_bytes(inflated=True)
        return Response(png, mimetype="image/png",
                        headers={"Content-Disposition": "attachment; filename=map.png"})

    # ----- HTTP: map management --------------------------------------------
    @app.route("/map/list")
    def map_list():
        return jsonify(context.list_maps())

    @app.route("/map/load", methods=["POST"])
    def map_load():
        name = request.json.get("name")
        try:
            context.load_map(name)
        except FileNotFoundError:
            return jsonify({"error": "map not found"}), 404
        _emit_map_base(socketio.emit, context)   # broadcast: everyone sees the new map
        return jsonify({"ok": True})

    @app.route("/map/save", methods=["POST"])
    def map_save():
        context.save_map(request.json.get("name", "map"))
        return jsonify({"ok": True})

    # ----- HTTP: modes / navigation ----------------------------------------
    @app.route("/mode", methods=["POST"])
    def set_mode():
        mode = request.json.get("mode")
        try:
            if mode == "idle":
                controller.start_idle()
            elif mode == "explore":
                controller.start_explore(request.json.get("name", "explored"))
            elif mode == "navigate":
                cell = _resolve_destination(context, request.json)
                if cell is None:
                    return jsonify({"error": "no destination"}), 400
                controller.navigate_to(cell)
            else:
                return jsonify({"error": "unknown mode"}), 400
        except InvalidTransition as exc:
            return jsonify({"error": str(exc)}), 409
        return jsonify({"ok": True, "mode": controller.mode})

    @app.route("/navigate", methods=["POST"])
    def navigate():
        cell = _resolve_destination(context, request.json)
        if cell is None:
            return jsonify({"error": "no destination"}), 400
        try:
            controller.navigate_to(cell)
        except InvalidTransition as exc:
            return jsonify({"error": str(exc)}), 409
        return jsonify({"ok": True})

    # ----- HTTP: waypoints --------------------------------------------------
    @app.route("/waypoints")
    def waypoints():
        return jsonify({k: list(v) for k, v in context.waypoints.items()})

    @app.route("/waypoints/add", methods=["POST"])
    def waypoints_add():
        data = request.json
        name = data["name"]
        if "col" in data and "row" in data:
            wx, wy = context.grid_to_world(int(data["col"]), int(data["row"]))
        else:
            wx, wy, _ = state.get_pose()           # save current robot position
        context.waypoints[name] = (wx, wy)
        state.set_log("ok", "Saved waypoint '%s'" % name)
        return jsonify({"ok": True})

    @app.route("/waypoints/delete", methods=["POST"])
    def waypoints_delete():
        context.waypoints.pop(request.json.get("name"), None)
        return jsonify({"ok": True})

    # ----- HTTP: forbidden zones -------------------------------------------
    @app.route("/forbidden/add", methods=["POST"])
    def forbidden_add():
        d = request.json
        context.forbidden.add_zone(d["x1"], d["y1"], d["x2"], d["y2"])
        return jsonify({"ok": True})

    @app.route("/forbidden/clear", methods=["POST"])
    def forbidden_clear():
        context.forbidden.clear_all()
        state.set_log("info", "Cleared forbidden zones")
        return jsonify({"ok": True})

    # ----- HTTP: safety ack + camera debug ---------------------------------
    @app.route("/safety/acknowledge", methods=["POST"])
    def safety_ack():
        safety_monitor.acknowledge_drop()
        state.set_log("ok", "Drop acknowledged -- ready to resume")
        return jsonify({"ok": True})

    @app.route("/camera/debug.mjpg")
    def camera_debug():
        return Response(_mjpeg_stream(),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    # ----- WebSocket --------------------------------------------------------
    @socketio.on("connect")
    def on_connect():
        # Inside a handler, flask_socketio.emit targets just the connecting client
        # rather than broadcasting the snapshot to everyone.
        _emit_map_base(ws_emit, context)
        ws_emit("mode_change", {"mode": controller.mode})
        x, y, theta = state.get_pose()
        ws_emit("pose_update", {"x": x, "y": y, "theta": theta})

    @socketio.on("set_destination")
    def on_set_destination(data):
        try:
            controller.navigate_to((int(data["x"]), int(data["y"])))
        except InvalidTransition as exc:
            state.set_log("error", "Cannot navigate: %s" % exc)

    @socketio.on("draw_zone")
    def on_draw_zone(data):
        context.forbidden.add_zone(data["x1"], data["y1"], data["x2"], data["y2"])
        state.set_log("info", "Added forbidden zone")

    # ----- pose broadcaster -------------------------------------------------
    def pose_loop():
        while not state.stop_event.is_set():
            x, y, theta = state.get_pose()
            socketio.emit("pose_update", {"x": x, "y": y, "theta": theta})
            socketio.sleep(0.1)

    app._start_background = lambda: socketio.start_background_task(pose_loop)
    return app, socketio


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _resolve_destination(context, data):
    """Resolve a navigate request to a (col, row) cell from a click or a waypoint."""
    if data is None:
        return None
    if "waypoint" in data:
        wp = context.waypoints.get(data["waypoint"])
        if wp is None:
            return None
        return context.slam.grid.world_to_grid(wp[0], wp[1])
    if "x" in data and "y" in data:
        return int(data["x"]), int(data["y"])
    return None


def _emit_map_base(emit_fn, context):
    with context.slam._lock:
        png = context.slam.grid.to_png_bytes(inflated=False)
        meta = {
            "width": context.slam.grid.width,
            "height": context.slam.grid.height,
            "resolution": context.slam.grid.resolution,
            "origin_col": context.slam.grid.origin_col,
            "origin_row": context.slam.grid.origin_row,
        }
    b64 = base64.b64encode(png).decode("ascii")
    emit_fn("map_base", {"png": b64, **meta})


def _mjpeg_stream():
    while not state.stop_event.is_set():
        frame = state.get_debug_frame()
        if frame is None:
            frame = state.get_latest_frame()
        if frame is None:
            time.sleep(0.05)
            continue
        ok, buf = cv2.imencode(".jpg", frame)
        if ok:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                   + buf.tobytes() + b"\r\n")
        time.sleep(0.05)
