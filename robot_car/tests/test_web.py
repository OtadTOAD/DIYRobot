"""Phase H -- web server HTTP endpoints, WebSocket handshake, mode state machine."""

import pytest

from robot_car import config, state
from robot_car.app import build
from robot_car.context import RobotContext
from robot_car.core import simulator
from robot_car.hardware import hal


@pytest.fixture
def server(tmp_path):
    config.MAPS_DIR = str(tmp_path)
    hal.reset_backend()
    simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    ctx = RobotContext()
    context, controller, app, socketio = build(ctx)
    app.config["TESTING"] = True
    yield context, controller, app, socketio
    controller.stop()


def test_index_served(server):
    _, _, app, _ = server
    res = app.test_client().get("/")
    assert res.status_code == 200
    assert b"Cargo Robot" in res.data


def test_map_png_endpoint(server):
    _, _, app, _ = server
    res = app.test_client().get("/map.png")
    assert res.status_code == 200
    assert res.mimetype == "image/png"


def test_map_list_empty(server):
    _, _, app, _ = server
    res = app.test_client().get("/map/list")
    assert res.status_code == 200
    assert res.get_json() == []


def test_save_then_list_map(server):
    _, _, app, _ = server
    client = app.test_client()
    assert client.post("/map/save", json={"name": "room1"}).status_code == 200
    assert "room1" in client.get("/map/list").get_json()


def test_waypoint_add_and_list(server):
    context, _, app, _ = server
    client = app.test_client()
    client.post("/waypoints/add", json={"name": "desk", "col": 110, "row": 100})
    wps = client.get("/waypoints").get_json()
    assert "desk" in wps


def test_forbidden_add_and_clear(server):
    context, _, app, _ = server
    client = app.test_client()
    client.post("/forbidden/add", json={"x1": 10, "y1": 10, "x2": 12, "y2": 12})
    assert context.forbidden.count() == 9
    client.post("/forbidden/clear")
    assert context.forbidden.count() == 0


def test_navigate_requires_map(server):
    _, _, app, _ = server
    state.set_grid(None)
    res = app.test_client().post("/navigate", json={"x": 110, "y": 100})
    assert res.status_code == 409                  # InvalidTransition: no map loaded


def test_mode_idle_ok(server):
    _, controller, app, _ = server
    res = app.test_client().post("/mode", json={"mode": "idle"})
    assert res.status_code == 200
    assert controller.mode == "idle"


def test_websocket_connect_emits_map_base(server):
    _, _, app, socketio = server
    state.set_grid(simulator.get_world() and None)  # ensure grid published from slam grid
    client = socketio.test_client(app)
    assert client.is_connected()
    received = client.get_received()
    events = {pkt["name"] for pkt in received}
    assert "map_base" in events
    assert "mode_change" in events
    client.disconnect()
