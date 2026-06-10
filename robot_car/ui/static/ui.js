/* Control panel logic (F-19): mode buttons, destination, waypoints, forbidden
 * zones, map management and the status log. Talks to the Flask HTTP API and to the
 * WebSocket (via window.RobotMap). */
(function () {
  const map = window.RobotMap;
  const $ = (id) => document.getElementById(id);

  async function post(url, body) {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    if (!res.ok) {
      const e = await res.json().catch(() => ({}));
      onLog("error", e.error || ("Request failed: " + url));
    }
    return res;
  }
  async function getJSON(url) { return (await fetch(url)).json(); }

  // -- status log + mode badge ---------------------------------------------
  function onLog(level, msg) {
    const log = $("log");
    const line = document.createElement("div");
    line.className = "log-" + level;
    line.textContent = "› " + msg;
    log.appendChild(line);
    log.scrollTop = log.scrollHeight;
  }
  function onMode(mode) {
    $("modeBadge").textContent = mode;
    document.querySelectorAll(".modeBtn").forEach((b) => b.classList.remove("active"));
    const id = { idle: "btnIdle", explore: "btnExplore", navigate: "btnNavigate" }[mode];
    if (id) $(id).classList.add("active");
  }
  window.RobotUI = { onLog, onMode };

  // -- mode buttons --------------------------------------------------------
  $("btnIdle").onclick = () => post("/mode", { mode: "idle" });
  $("btnExplore").onclick = () => post("/mode", { mode: "explore" });
  $("btnNavigate").onclick = () => onLog("info", "Click the map or pick a waypoint to navigate.");

  // -- destination: click map ----------------------------------------------
  let drawingZone = false, zoneStart = null;
  map.canvas.addEventListener("click", (ev) => {
    if (drawingZone) return;
    const rect = map.canvas.getBoundingClientRect();
    const cell = map.pixelToCell(ev.clientX - rect.left, ev.clientY - rect.top);
    map.socket.emit("set_destination", { x: cell.x, y: cell.y });
    onLog("info", `Navigating to cell (${cell.x}, ${cell.y})`);
  });

  // -- destination: waypoint dropdown --------------------------------------
  $("btnGo").onclick = () => {
    const name = $("waypointSelect").value;
    if (name) post("/navigate", { waypoint: name });
  };

  // -- waypoints -----------------------------------------------------------
  async function refreshWaypoints() {
    const wp = await getJSON("/waypoints");
    map.setWaypoints(wp);
    const list = $("waypointList");
    const sel = $("waypointSelect");
    list.innerHTML = "";
    sel.innerHTML = '<option value="">— waypoint —</option>';
    Object.keys(wp).forEach((name) => {
      const li = document.createElement("li");
      li.textContent = name;
      const del = document.createElement("button");
      del.textContent = "🗑";
      del.onclick = async () => { await post("/waypoints/delete", { name }); refreshWaypoints(); };
      li.appendChild(del);
      list.appendChild(li);
      const opt = document.createElement("option");
      opt.value = name; opt.textContent = name; sel.appendChild(opt);
    });
  }
  $("btnAddWp").onclick = async () => {
    const name = $("wpName").value.trim();
    if (!name) return;
    await post("/waypoints/add", { name });        // saves current robot position
    $("wpName").value = "";
    refreshWaypoints();
  };

  // -- forbidden zones -----------------------------------------------------
  $("btnDrawZone").onclick = () => {
    drawingZone = true; zoneStart = null;
    onLog("info", "Click two opposite corners to draw a forbidden zone.");
  };
  $("btnClearZones").onclick = async () => { await post("/forbidden/clear"); map.clearZones(); };
  map.canvas.addEventListener("mousedown", (ev) => {
    if (!drawingZone) return;
    const rect = map.canvas.getBoundingClientRect();
    const cell = map.pixelToCell(ev.clientX - rect.left, ev.clientY - rect.top);
    if (!zoneStart) { zoneStart = cell; return; }
    const z = { x1: zoneStart.x, y1: zoneStart.y, x2: cell.x, y2: cell.y };
    map.socket.emit("draw_zone", z);
    map.addZone(z);
    drawingZone = false; zoneStart = null;
  });

  // -- map management ------------------------------------------------------
  async function refreshMaps() {
    const maps = await getJSON("/map/list");
    const sel = $("mapSelect");
    sel.innerHTML = "";
    maps.forEach((m) => {
      const o = document.createElement("option"); o.value = m; o.textContent = m; sel.appendChild(o);
    });
  }
  $("btnLoadMap").onclick = async () => {
    const name = $("mapSelect").value;
    if (name) { await post("/map/load", { name }); refreshWaypoints(); }
  };
  $("btnSaveMap").onclick = async () => {
    const name = $("mapName").value.trim() || "map";
    await post("/map/save", { name }); refreshMaps();
  };
  $("btnExport").onclick = () => window.open("/map/export", "_blank");

  // -- safety --------------------------------------------------------------
  $("btnAck").onclick = () => post("/safety/acknowledge");

  // -- camera debug stream (start once page is up) -------------------------
  $("camImg").src = "/camera/debug.mjpg";

  // -- initial load --------------------------------------------------------
  refreshWaypoints();
  refreshMaps();
})();
