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

  // -- header: status log, mode badge, pose, connection ---------------------
  const MAX_LOG_LINES = 200;
  function onLog(level, msg) {
    const log = $("log");
    const line = document.createElement("div");
    line.className = "log-" + level;
    const t = document.createElement("time");
    t.textContent = new Date().toLocaleTimeString([], { hour12: false });
    line.appendChild(t);
    line.appendChild(document.createTextNode(msg));
    log.appendChild(line);
    while (log.childElementCount > MAX_LOG_LINES) log.removeChild(log.firstChild);
    log.scrollTop = log.scrollHeight;
  }
  function onMode(mode) {
    const badge = $("modeBadge");
    badge.textContent = mode;
    badge.className = mode;
    $("btnIdle").classList.toggle("active", mode === "idle");
    $("btnExplore").classList.toggle("active", mode === "explore");
    $("btnManual").classList.toggle("active", mode === "manual");
    setManualActive(mode === "manual");
  }
  function onPose(p) {
    const deg = ((p.theta * 180 / Math.PI) % 360).toFixed(0);
    $("poseReadout").textContent =
      `x ${p.x.toFixed(2)} m · y ${p.y.toFixed(2)} m · θ ${deg}°`;
  }
  function onConnection(up) {
    $("connDot").classList.toggle("on", up);
    onLog(up ? "ok" : "warn", up ? "Connected to robot" : "Connection lost");
  }
  window.RobotUI = { onLog, onMode, onPose, onConnection };

  // -- mode buttons ----------------------------------------------------------
  $("btnIdle").onclick = () => post("/mode", { mode: "idle" });
  $("btnExplore").onclick = () => post("/mode", { mode: "explore" });
  $("btnManual").onclick = () => post("/mode", { mode: "manual" });

  // -- manual drive (F-21): a 10 Hz keep-alive re-emits the held stick; releasing,
  //    hiding the tab, or losing focus emits an explicit zero (dead-man). -------
  let manualActive = false, manualTimer = null, manualRateMs = 100;
  let speed = 0.6;
  const pressed = { fwd: false, back: false, left: false, right: false };

  fetch("/config").then((r) => r.json()).then((c) => {
    if (c.manual_cmd_rate_hz) manualRateMs = 1000 / c.manual_cmd_rate_hz;
  }).catch(() => {});

  function manualVector() {
    const lin = (pressed.fwd ? 1 : 0) - (pressed.back ? 1 : 0);
    const ang = (pressed.left ? 1 : 0) - (pressed.right ? 1 : 0);
    return { linear: lin * speed, angular: ang * speed };
  }
  function emitManual() {
    if (manualActive) map.socket.emit("manual_cmd", manualVector());
  }
  function reflectHeld() {
    document.querySelectorAll("#dpad button").forEach((b) => {
      const d = btnDir(b);
      b.classList.toggle("held", d !== "stop" && pressed[d]);
    });
  }
  function setManualActive(on) {
    manualActive = on;
    $("manualSection").hidden = !on;
    if (on) {
      if (!manualTimer) manualTimer = setInterval(emitManual, manualRateMs);
    } else {
      clearInterval(manualTimer); manualTimer = null;
      for (const k in pressed) pressed[k] = false;
      reflectHeld();
      map.socket.emit("manual_cmd", { linear: 0, angular: 0 });
    }
  }
  function btnDir(b) {
    const lin = +b.dataset.lin, ang = +b.dataset.ang;
    if (lin === 0 && ang === 0) return "stop";
    if (lin > 0) return "fwd";
    if (lin < 0) return "back";
    return ang > 0 ? "left" : "right";
  }
  function setHeld(dir, on) {
    if (dir === "stop") { for (const k in pressed) pressed[k] = false; }
    else pressed[dir] = on;
    reflectHeld();
    emitManual();
  }

  $("manualSpeed").oninput = (e) => {
    speed = +e.target.value;
    $("manualSpeedVal").textContent = Math.round(speed * 100) + "%";
  };

  document.querySelectorAll("#dpad button").forEach((b) => {
    const dir = btnDir(b);
    b.addEventListener("pointerdown", (e) => { e.preventDefault(); setHeld(dir, true); });
    const release = (e) => { e.preventDefault(); if (dir !== "stop") setHeld(dir, false); };
    b.addEventListener("pointerup", release);
    b.addEventListener("pointerleave", release);
    b.addEventListener("pointercancel", release);
  });

  const KEYDIR = {
    w: "fwd", arrowup: "fwd", s: "back", arrowdown: "back",
    a: "left", arrowleft: "left", d: "right", arrowright: "right",
  };
  document.addEventListener("keydown", (ev) => {
    if (!manualActive || ev.repeat) return;
    const dir = KEYDIR[ev.key.toLowerCase()];
    if (dir) { ev.preventDefault(); setHeld(dir, true); }
  });
  document.addEventListener("keyup", (ev) => {
    if (!manualActive) return;
    const dir = KEYDIR[ev.key.toLowerCase()];
    if (dir) { ev.preventDefault(); setHeld(dir, false); }
  });
  // Losing focus / hiding the page must coast the robot to a stop (it stays in
  // manual mode -- the keep-alive then just re-emits the zero command).
  function stopManual() {
    for (const k in pressed) pressed[k] = false;
    reflectHeld();
    if (manualActive) map.socket.emit("manual_cmd", { linear: 0, angular: 0 });
  }
  document.addEventListener("visibilitychange", () => { if (document.hidden) stopManual(); });
  window.addEventListener("blur", stopManual);

  // -- map clicks: navigate, or forbidden-zone corners when drawing ----------
  // A single click handler dispatches both, so finishing a zone can never also
  // fire a navigation request for the same click.
  let drawingZone = false, zoneStart = null;

  function setDrawingZone(on) {
    drawingZone = on;
    zoneStart = null;
    map.setZonePreview(null);
    $("btnDrawZone").classList.toggle("active", on);
    $("btnDrawZone").textContent = on ? "Cancel Drawing" : "Draw Zone";
    $("mapHint").textContent = on
      ? "Click two opposite corners to draw a forbidden zone (Esc to cancel)."
      : "Click the map to send the robot there.";
  }

  map.canvas.addEventListener("click", (ev) => {
    const cell = map.eventToCell(ev);
    if (drawingZone) {
      if (!zoneStart) {
        zoneStart = cell;
        $("mapHint").textContent = "Now click the opposite corner.";
        return;
      }
      const z = { x1: zoneStart.x, y1: zoneStart.y, x2: cell.x, y2: cell.y };
      map.socket.emit("draw_zone", z);
      map.addZone(z);
      setDrawingZone(false);
      return;
    }
    map.socket.emit("set_destination", { x: cell.x, y: cell.y });
    map.setDestination(cell);
    onLog("info", `Navigating to cell (${cell.x}, ${cell.y})`);
  });

  map.canvas.addEventListener("mousemove", (ev) => {
    if (drawingZone && zoneStart) {
      const cell = map.eventToCell(ev);
      map.setZonePreview({ x1: zoneStart.x, y1: zoneStart.y, x2: cell.x, y2: cell.y });
    }
  });

  document.addEventListener("keydown", (ev) => {
    if (ev.key === "Escape" && drawingZone) setDrawingZone(false);
  });

  $("btnDrawZone").onclick = () => setDrawingZone(!drawingZone);
  $("btnClearZones").onclick = async () => {
    await post("/forbidden/clear");
    map.clearZones();
  };

  // -- destination: waypoint dropdown ----------------------------------------
  $("waypointSelect").onchange = () => {
    $("btnGo").disabled = !$("waypointSelect").value;
  };
  $("btnGo").onclick = () => {
    const name = $("waypointSelect").value;
    if (name) post("/navigate", { waypoint: name });
  };

  // -- waypoints --------------------------------------------------------------
  async function refreshWaypoints() {
    const wp = await getJSON("/waypoints");
    map.setWaypoints(wp);
    const list = $("waypointList");
    const sel = $("waypointSelect");
    const prev = sel.value;
    list.innerHTML = "";
    sel.innerHTML = '<option value="">— navigate to waypoint —</option>';
    const names = Object.keys(wp);
    if (!names.length) {
      const li = document.createElement("li");
      li.className = "empty";
      li.textContent = "No waypoints yet.";
      list.appendChild(li);
    }
    names.forEach((name) => {
      const li = document.createElement("li");
      li.textContent = name;
      const del = document.createElement("button");
      del.textContent = "🗑";
      del.title = "Delete waypoint";
      del.onclick = async () => { await post("/waypoints/delete", { name }); refreshWaypoints(); };
      li.appendChild(del);
      list.appendChild(li);
      const opt = document.createElement("option");
      opt.value = name; opt.textContent = name; sel.appendChild(opt);
    });
    sel.value = names.includes(prev) ? prev : "";
    $("btnGo").disabled = !sel.value;
  }
  $("btnAddWp").onclick = async () => {
    const name = $("wpName").value.trim();
    if (!name) return;
    await post("/waypoints/add", { name });        // saves current robot position
    $("wpName").value = "";
    refreshWaypoints();
  };
  $("wpName").addEventListener("keydown", (ev) => {
    if (ev.key === "Enter") $("btnAddWp").onclick();
  });

  // -- map management ----------------------------------------------------------
  async function refreshMaps() {
    const maps = await getJSON("/map/list");
    const sel = $("mapSelect");
    sel.innerHTML = "";
    if (!maps.length) {
      const o = document.createElement("option");
      o.value = ""; o.textContent = "— no saved maps —";
      sel.appendChild(o);
    }
    maps.forEach((m) => {
      const o = document.createElement("option"); o.value = m; o.textContent = m; sel.appendChild(o);
    });
    $("btnLoadMap").disabled = !sel.value;
  }
  $("btnLoadMap").onclick = async () => {
    const name = $("mapSelect").value;
    if (name) { await post("/map/load", { name }); refreshWaypoints(); }
  };
  $("btnSaveMap").onclick = async () => {
    const name = $("mapName").value.trim() || "map";
    await post("/map/save", { name });
    onLog("ok", `Map saved as '${name}'`);
    refreshMaps();
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
