# Web Interface — Design Decisions
## AI Cargo Robot — Discussion Notes

---

## 1. Purpose

The web interface is the primary input and monitoring solution for the robot. It runs as a Flask application on the Raspberry Pi and is accessed from any device on the same network via a browser.

Core responsibilities:
- Real-time map visualization
- Robot position and heading display
- Mode control (Idle / Explore / Navigate)
- Destination selection (click or named waypoint)
- Forbidden zone definition
- Map management (load, save, switch, export)

---

## 2. Frontend Technology

### Decision: Vanilla JS (no framework)

| Option | Pros | Cons |
|---|---|---|
| React | Clean component structure, good DX | Build toolchain needed on Pi, overkill for this scope |
| Vanilla JS | Zero build step, easy SSH debugging, simple deployment | Less structured for large UIs |

**Reason:** The dominant UI element is an HTML5 Canvas for map rendering — this is imperative drawing code regardless of framework. React's strength is managing many dynamic DOM components, but this UI has relatively few. A build toolchain on the Pi adds unnecessary complexity. A clean `index.html` + `map.js` + `ui.js` is simpler to deploy, debug, and maintain.

---

## 3. Real-Time Map Rendering

### Decision: HTML5 Canvas + WebSocket (Flask-SocketIO)

| Option | Pros | Cons |
|---|---|---|
| PNG polling over HTTP | Simple | Laggy, high bandwidth |
| Canvas + WebSocket | Truly real-time, efficient | More complex |

**Reason:** Real-time visualization is a core requirement. Polling a full PNG image every second wastes bandwidth and feels laggy. WebSocket allows the server to push lightweight JSON updates as they happen.

### Rendering Strategy

- On map load → server sends full occupancy grid as a base PNG, drawn onto canvas once
- On robot movement → server pushes pose update as small JSON `{x, y, theta}`
- On path replan → server pushes new path as list of cell coordinates
- On grid cell change (during exploration) → server pushes list of changed cells only

The browser redraws only what changed, keeping rendering fast.

### Map Layers (drawn in order)

| Layer | Content |
|---|---|
| 1 — Base grid | White = free, black = wall, grey = unknown |
| 2 — Inflated obstacles | Slightly darker shade to visualize safety margin |
| 3 — Forbidden zones | Semi-transparent red overlay |
| 4 — Planned path | Blue line from robot to goal |
| 5 — Waypoints | Named markers |
| 6 — Robot | Arrow showing position and heading |

---

## 4. Forbidden Zones

### Concept

User draws zones directly on the map canvas. Those cells are marked as permanently blocked in a separate constraint layer — not baked into the occupancy grid itself. A* treats them identically to walls during planning.

### Decision: Separate constraint layer, hard block default

**Hard block** — forbidden cells treated as walls, A* will not route through them.
**Soft penalty** — forbidden cells get high traversal cost, avoided unless no other path exists.

Default is hard block. Soft penalty available as a toggle for narrow environments where hard blocking might make some goals unreachable.

### Storage

Forbidden zones are saved as a separate file alongside the map — not written into the occupancy grid — so they can be edited or cleared without re-exploring.

### Drawing Interaction

- User clicks "Draw Zone" button → canvas enters drawing mode
- User clicks and drags to define a rectangle on the map
- Rectangle is converted from pixel coordinates to grid cell coordinates
- Sent to server via WebSocket, added to constraint layer
- Rendered immediately as red overlay on canvas
- "Clear All Zones" button resets the constraint layer

---

## 5. Mode Control

Three modes, toggled from the UI:

| Mode | Behaviour |
|---|---|
| Idle | Robot stationary, waits for commands |
| Explore | Robot autonomously builds map |
| Navigate | Robot travels to selected destination |

### Decision: Server-side state machine

The UI sends mode change requests. The server validates the transition and executes it. The browser reflects current state only — it does not manage logic.

### Valid Transitions

```
Idle      → Explore    ✅
Idle      → Navigate   ✅ (requires loaded map)
Explore   → Idle       ✅ (stops and saves partial map)
Explore   → Navigate   ✅ (uses partial map)
Navigate  → Idle       ✅ (stops immediately)
Navigate  → Explore    ✅
```

The UI disables invalid transitions (e.g. Navigate without a loaded map) and shows clear feedback.

---

## 6. Destination Selection

### Decision: Both click-to-navigate and named waypoints

**Click-to-navigate**
- User clicks a cell on the canvas
- Browser converts pixel → grid coordinates
- Sent to server, server snaps to nearest free cell if click lands on obstacle
- Server runs A* and begins navigation

**Named waypoints**
- Saved verified reachable positions with human-readable names
- Selectable from a dropdown in the control panel
- More reliable for repeated destinations

Both methods are supported. Named waypoints are preferred for reliability; click-to-navigate for flexibility.

---

## 7. Map Management

| Feature | Description |
|---|---|
| Load map | Select from saved `.map` files stored on the Pi |
| Save map | Save current exploration result with a name |
| Switch map | Load a different room map without restarting server |
| Export PNG | Download a clean image of the current map |
| Clear forbidden zones | Reset constraint layer without affecting the map |
| Rename waypoints | Edit waypoint names from the UI |

All map operations are Flask HTTP endpoints. File operations happen on the server side.

---

## 8. UI Layout

```
┌─────────────────────────────────────────────┐
│  🤖 Cargo Robot Control        [Mode Badge]  │
├────────────────────┬────────────────────────┤
│                    │  Mode:                  │
│                    │  [Idle] [Explore]       │
│                    │  [Navigate]             │
│                    │                         │
│   MAP CANVAS       │  Destination:           │
│                    │  [Click map]            │
│   occupancy grid   │  [Waypoint ▼] [Go]      │
│   robot position   │                         │
│   planned path     │  Waypoints:             │
│   forbidden zones  │  [+ Add] [🗑 Clear]      │
│   waypoints        │  • desk                 │
│                    │  • door                 │
│                    │                         │
│                    │  Forbidden Zones:       │
│                    │  [Draw] [Clear All]     │
│                    │                         │
│                    │  Maps:                  │
│                    │  [Load] [Save]          │
│                    │  [Switch] [Export PNG]  │
│                    │                         │
│                    │  Status Log:            │
│                    │  > Navigating to desk   │
│                    │  > Obstacle detected    │
└────────────────────┴────────────────────────┘
```

---

## 9. Backend API Endpoints

### HTTP Endpoints (Flask)

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | Serve main UI (`index.html`) |
| `/map/list` | GET | List available `.map` files on Pi |
| `/map/load` | POST | Load a saved map by name |
| `/map/save` | POST | Save current map with a name |
| `/map/export` | GET | Download PNG of current map |
| `/mode` | POST | Switch robot mode |
| `/navigate` | POST | Set destination and start navigation |
| `/waypoints` | GET | List all named waypoints |
| `/waypoints/add` | POST | Add or update a named waypoint |
| `/waypoints/delete` | POST | Delete a named waypoint |
| `/forbidden/add` | POST | Add a forbidden zone (grid cells) |
| `/forbidden/clear` | POST | Clear all forbidden zones |

### WebSocket Events (Flask-SocketIO)

| Event | Direction | Payload |
|---|---|---|
| `map_base` | Server → Client | Full grid as base64 PNG on load |
| `pose_update` | Server → Client | `{x, y, theta}` |
| `path_update` | Server → Client | List of `{x, y}` cell coordinates |
| `cell_update` | Server → Client | List of changed cells during exploration |
| `mode_change` | Server → Client | Current mode string |
| `status_log` | Server → Client | Status message string |
| `set_destination` | Client → Server | `{x, y}` grid coordinates from click |
| `draw_zone` | Client → Server | `{x1, y1, x2, y2}` grid rectangle |

---

## 10. File Structure

```
ui/
├── server.py          # Flask app, SocketIO handlers, API endpoints
└── static/
    ├── index.html     # Main UI shell
    ├── map.js         # Canvas rendering, WebSocket client, layer management
    └── ui.js          # Control panel logic, mode buttons, waypoint management
```

---

## 11. Decision Summary

| Component | Decision | Reason |
|---|---|---|
| Frontend | Vanilla JS | Canvas-heavy UI, zero build step, easy Pi deployment |
| Real-time transport | WebSocket (Flask-SocketIO) | Low latency, efficient delta updates |
| Map rendering | HTML5 Canvas, layered | Flexible, no extra libraries |
| Forbidden zones | Separate constraint layer, hard block | Non-destructive, editable without re-exploring |
| Destination input | Click-to-navigate + named waypoints | Flexibility and reliability |
| Mode control | Server-side state machine | Logic stays on robot, UI is display only |
| Map serving | Base PNG once, JSON deltas after | Efficient bandwidth usage |

---

*Notes from design discussion — AI Cargo Robot project*
